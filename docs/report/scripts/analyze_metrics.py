#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rosbag2 후처리 — 토픽 갱신 주기와 발행 지연을 CSV·그래프로 낸다.

산출 지표 (03_metrics_plan.md 2장·6장):
  1. 갱신 주기   : 연속 메시지의 bag 수신 시각 간격
  2. 발행 지연   : header.stamp -> bag 수신 시각 의 차

주의 — 지표 2의 의미 범위
  header.stamp 가 센서 노출 시각인 경우에만 "센서 지연"이 된다.
  현재 설정은 그렇지 않다:
    - gscam : use_gst_timestamps=false  -> 노드가 프레임을 받은 시각
    - uwb   : 노드가 시리얼 줄을 받은 ROS 시각 (설계 문서 5장)
  따라서 이 스크립트가 내는 값은 "발행 -> 기록" 구간 지연이다.
  보고서에 쓸 때 구간을 반드시 병기한다.

사용법:
  python3 analyze_metrics.py <bag디렉터리> [-o 출력폴더] [--topics /scan /uwb_pose]
"""

import argparse
import csv
import os
import sys
from collections import OrderedDict

# ---------------------------------------------------------------- 인코딩 고정
# 한글 출력이 환경 로케일에 따라 깨지는 것을 막는다.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

NS = 1_000_000_000.0


# ---------------------------------------------------------------- 한글 폰트
def setup_matplotlib():
    """matplotlib 한글 폰트를 잡는다. 실패해도 그래프는 그린다(경고만)."""
    import matplotlib
    matplotlib.use("Agg")  # 화면 없는 Jetson에서도 저장되도록
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    # 설치 확인된 순서대로 시도 (Jetson 기본 이미지에 Noto CJK 포함)
    candidates = [
        "Noto Sans CJK KR",
        "Noto Sans CJK JP",
        "NanumGothic",
        "NanumBarunGothic",
        "Malgun Gothic",
        "AppleGothic",
    ]
    installed = {f.name for f in font_manager.fontManager.ttflist}
    chosen = next((c for c in candidates if c in installed), None)

    if chosen:
        plt.rcParams["font.family"] = chosen
        print("[정보] 한글 폰트: {}".format(chosen))
    else:
        print("[경고] 한글 폰트를 찾지 못했다. 그래프의 한글이 네모로 나올 수 있다.")
        print("       설치: sudo apt install -y fonts-noto-cjk")
        print("       설치 후 캐시 삭제: rm -rf ~/.cache/matplotlib")

    # 한글 폰트로 바꾸면 마이너스 기호가 깨진다. 유니코드 마이너스를 끈다.
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.autolayout"] = True
    return plt


# ---------------------------------------------------------------- bag 읽기
def read_bag(bag_path, wanted=None):
    """bag 을 순회하며 토픽별 (수신시각ns, header시각ns) 목록을 만든다.

    header 가 없는 메시지 타입은 header 시각을 None 으로 둔다.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        print("[오류] ROS2 파이썬 모듈을 불러오지 못했다: {}".format(e), file=sys.stderr)
        print("       source /opt/ros/humble/setup.bash 후 다시 실행할 것.", file=sys.stderr)
        sys.exit(1)

    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if wanted:
        missing = [t for t in wanted if t not in type_map]
        for m in missing:
            print("[경고] bag 에 없는 토픽을 건너뛴다: {}".format(m))
        targets = [t for t in wanted if t in type_map]
    else:
        targets = list(type_map.keys())

    if not targets:
        print("[오류] 분석할 토픽이 없다.", file=sys.stderr)
        sys.exit(1)

    print("[정보] 분석 대상 토픽: {}".format(", ".join(targets)))

    msg_classes = {}
    for t in targets:
        try:
            msg_classes[t] = get_message(type_map[t])
        except (ImportError, AttributeError, ValueError) as e:
            print("[경고] 메시지 타입을 못 불러왔다. header 없이 처리한다: {} ({})".format(t, e))
            msg_classes[t] = None

    data = OrderedDict((t, []) for t in targets)
    total = 0
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        if topic not in data:
            continue
        total += 1
        hdr_ns = None
        cls = msg_classes.get(topic)
        if cls is not None:
            try:
                msg = deserialize_message(raw, cls)
                stamp = getattr(getattr(msg, "header", None), "stamp", None)
                if stamp is not None:
                    hdr_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            except Exception as e:  # 한 줄 깨져도 전체를 멈추지 않는다
                print("[경고] 역직렬화 실패 1건 ({}): {}".format(topic, e))
        data[topic].append((recv_ns, hdr_ns))

    print("[정보] 총 {}개 메시지를 읽었다.".format(total))
    return data, type_map


# ---------------------------------------------------------------- 통계
def percentile(sorted_vals, q):
    """의존성 없이 계산하는 선형보간 백분위수. sorted_vals 는 정렬된 리스트."""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = (len(sorted_vals) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = pos - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def summarize(name, values, unit):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return None
    mean = sum(s) / n
    var = sum((v - mean) ** 2 for v in s) / n if n > 1 else 0.0
    return OrderedDict([
        ("항목", name),
        ("단위", unit),
        ("표본수", n),
        ("평균", round(mean, 6)),
        ("중앙값", round(percentile(s, 0.5), 6)),
        ("표준편차", round(var ** 0.5, 6)),
        ("최소", round(s[0], 6)),
        ("p95", round(percentile(s, 0.95), 6)),
        ("p99", round(percentile(s, 0.99), 6)),
        ("최대", round(s[-1], 6)),
    ])


def analyze_topic(topic, samples):
    """토픽 1개의 간격·지연을 계산한다."""
    if len(samples) < 2:
        print("[경고] 메시지가 2개 미만이라 간격을 못 낸다: {}".format(topic))
        return None

    samples = sorted(samples, key=lambda x: x[0])
    t0 = samples[0][0]

    rows = []
    intervals = []      # 초
    latencies = []      # 밀리초
    prev = None
    for recv_ns, hdr_ns in samples:
        elapsed = (recv_ns - t0) / NS
        interval = None if prev is None else (recv_ns - prev) / NS
        latency = None if hdr_ns is None else (recv_ns - hdr_ns) / 1e6
        rows.append(OrderedDict([
            ("경과시간_s", round(elapsed, 6)),
            ("수신시각_ns", recv_ns),
            ("header시각_ns", "" if hdr_ns is None else hdr_ns),
            ("간격_s", "" if interval is None else round(interval, 6)),
            ("발행지연_ms", "" if latency is None else round(latency, 4)),
        ]))
        if interval is not None:
            intervals.append(interval)
        if latency is not None:
            latencies.append(latency)
        prev = recv_ns

    duration = (samples[-1][0] - t0) / NS
    avg_hz = (len(samples) - 1) / duration if duration > 0 else float("nan")

    # 결손: 중앙값 간격의 2배를 넘는 구간 (03_metrics_plan.md 2.3 판정 기준)
    med = percentile(sorted(intervals), 0.5) if intervals else float("nan")
    dropouts = [v for v in intervals if med == med and v > med * 2]

    return {
        "topic": topic,
        "rows": rows,
        "count": len(samples),
        "duration_s": duration,
        "avg_hz": avg_hz,
        "median_interval_s": med,
        "dropout_count": len(dropouts),
        "dropout_max_s": max(dropouts) if dropouts else 0.0,
        "interval_stats": summarize("발행 간격", intervals, "s"),
        "latency_stats": summarize("발행 지연(발행→기록)", latencies, "ms"),
        "intervals": intervals,
        "latencies": latencies,
    }


# ---------------------------------------------------------------- 출력
def safe_name(topic):
    return topic.strip("/").replace("/", "_") or "root"


def write_csv(path, fieldnames, rows):
    # 엑셀에서 열 때 한글이 깨지지 않도록 BOM 포함 UTF-8 로 쓴다.
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_outputs(results, outdir, bag_path):
    os.makedirs(outdir, exist_ok=True)

    # 1) 토픽별 원시 CSV
    for res in results:
        p = os.path.join(outdir, "raw_{}.csv".format(safe_name(res["topic"])))
        write_csv(p, list(res["rows"][0].keys()), res["rows"])
        print("[출력] {}".format(p))

    # 2) 요약 CSV
    summary_rows = []
    for res in results:
        base = OrderedDict([
            ("토픽", res["topic"]),
            ("메시지수", res["count"]),
            ("기록길이_s", round(res["duration_s"], 3)),
            ("평균_Hz", round(res["avg_hz"], 4)),
            ("중앙값간격_s", round(res["median_interval_s"], 6)),
            ("결손횟수", res["dropout_count"]),
            ("최대결손간격_s", round(res["dropout_max_s"], 6)),
        ])
        for key, stats in (("간격", res["interval_stats"]), ("지연", res["latency_stats"])):
            if stats is None:
                continue
            for k, v in stats.items():
                if k in ("항목", "표본수"):
                    continue
                base["{}_{}".format(key, k)] = v
        summary_rows.append(base)

    # 열 순서를 일정하게 맞춘다 (토픽마다 지연 열 유무가 다를 수 있음)
    fields = []
    for r in summary_rows:
        for k in r.keys():
            if k not in fields:
                fields.append(k)
    for r in summary_rows:
        for k in fields:
            r.setdefault(k, "")

    p = os.path.join(outdir, "summary.csv")
    write_csv(p, fields, summary_rows)
    print("[출력] {}".format(p))

    # 3) 사람이 읽는 요약
    p = os.path.join(outdir, "summary.txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("# 측정 요약\n")
        f.write("bag: {}\n\n".format(os.path.abspath(bag_path)))
        f.write("주의: '발행 지연'은 header.stamp -> bag 기록 시각 구간이다.\n")
        f.write("      센서 노출부터의 지연이 아니다 (03_metrics_plan.md 6.1 참조).\n\n")
        for res in results:
            f.write("## {}\n".format(res["topic"]))
            f.write("  메시지수      : {}\n".format(res["count"]))
            f.write("  기록길이      : {:.3f} s\n".format(res["duration_s"]))
            f.write("  평균 발행률   : {:.4f} Hz\n".format(res["avg_hz"]))
            f.write("  결손(중앙값 2배 초과) : {}회, 최대 {:.4f} s\n".format(
                res["dropout_count"], res["dropout_max_s"]))
            for label, stats in (("간격(s)", res["interval_stats"]),
                                 ("지연(ms)", res["latency_stats"])):
                if stats is None:
                    f.write("  {} : 산출 불가 (header 없음)\n".format(label))
                    continue
                f.write("  {} 평균 {} / 중앙 {} / p95 {} / p99 {} / 최대 {}\n".format(
                    label, stats["평균"], stats["중앙값"],
                    stats["p95"], stats["p99"], stats["최대"]))
            f.write("\n")
    print("[출력] {}".format(p))
    return summary_rows


def draw_plots(plt, results, outdir):
    for res in results:
        topic = res["topic"]
        tag = safe_name(topic)
        rows = res["rows"]

        has_latency = bool(res["latencies"])
        nplots = 3 if has_latency else 2
        fig, axes = plt.subplots(nplots, 1, figsize=(11, 3.2 * nplots))
        if nplots == 1:
            axes = [axes]

        # (1) 시간에 따른 간격
        xs = [r["경과시간_s"] for r in rows if r["간격_s"] != ""]
        ys = [r["간격_s"] for r in rows if r["간격_s"] != ""]
        ax = axes[0]
        ax.plot(xs, ys, linewidth=0.8)
        med = res["median_interval_s"]
        ax.axhline(med, linestyle="--", linewidth=1.0,
                   label="중앙값 {:.4f} s".format(med))
        ax.axhline(med * 2, linestyle=":", linewidth=1.0,
                   label="결손 판정선 (중앙값 x2)")
        ax.set_title("{} — 발행 간격 시계열".format(topic))
        ax.set_xlabel("경과 시간 (s)")
        ax.set_ylabel("간격 (s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # (2) 간격 분포
        ax = axes[1]
        ax.hist(res["intervals"], bins=60)
        ax.set_title("{} — 발행 간격 분포 (평균 {:.3f} Hz)".format(topic, res["avg_hz"]))
        ax.set_xlabel("간격 (s)")
        ax.set_ylabel("빈도")
        ax.grid(True, alpha=0.3)

        # (3) 발행 지연
        if has_latency:
            ax = axes[2]
            lx = [r["경과시간_s"] for r in rows if r["발행지연_ms"] != ""]
            ly = [r["발행지연_ms"] for r in rows if r["발행지연_ms"] != ""]
            ax.plot(lx, ly, linewidth=0.8)
            st = res["latency_stats"]
            ax.axhline(st["p99"], linestyle=":", linewidth=1.0,
                       label="p99 {:.3f} ms".format(st["p99"]))
            ax.set_title("{} — 발행 지연 (header.stamp → bag 기록)".format(topic))
            ax.set_xlabel("경과 시간 (s)")
            ax.set_ylabel("지연 (ms)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)

        p = os.path.join(outdir, "plot_{}.png".format(tag))
        fig.savefig(p, dpi=130)
        plt.close(fig)
        print("[출력] {}".format(p))

    # 토픽 비교 막대 (2개 이상일 때만)
    if len(results) >= 2:
        fig, ax = plt.subplots(figsize=(9, 4))
        names = [r["topic"] for r in results]
        hz = [r["avg_hz"] for r in results]
        bars = ax.bar(names, hz)
        for b, v in zip(bars, hz):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                    "{:.2f}".format(v), ha="center", va="bottom", fontsize=9)
        ax.set_title("토픽별 평균 발행률")
        ax.set_ylabel("Hz")
        ax.grid(True, axis="y", alpha=0.3)
        p = os.path.join(outdir, "plot_compare_hz.png")
        fig.savefig(p, dpi=130)
        plt.close(fig)
        print("[출력] {}".format(p))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="rosbag2 → 갱신 주기·발행 지연 CSV/그래프",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예: python3 analyze_metrics.py ~/drone_ws/metrics_bags/metrics_light_solo_20260816_170000",
    )
    ap.add_argument("bag", help="rosbag2 디렉터리 경로 (metadata.yaml 이 있는 폴더)")
    ap.add_argument("-o", "--outdir", default=None,
                    help="출력 폴더. 기본은 <bag>_analysis")
    ap.add_argument("--topics", nargs="*", default=None,
                    help="분석할 토픽만 지정. 생략하면 bag 안의 전체")
    args = ap.parse_args()

    bag = os.path.abspath(os.path.expanduser(args.bag))
    if not os.path.isdir(bag):
        print("[오류] bag 디렉터리가 없다: {}".format(bag), file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(os.path.join(bag, "metadata.yaml")):
        print("[오류] metadata.yaml 이 없다. rosbag2 폴더가 맞는지 확인할 것: {}".format(bag),
              file=sys.stderr)
        sys.exit(1)

    outdir = args.outdir or (bag.rstrip("/") + "_analysis")
    print("[정보] bag   : {}".format(bag))
    print("[정보] 출력  : {}".format(outdir))

    plt = setup_matplotlib()
    data, _ = read_bag(bag, args.topics)

    results = []
    for topic, samples in data.items():
        res = analyze_topic(topic, samples)
        if res:
            results.append(res)
    if not results:
        print("[오류] 분석 결과가 비었다.", file=sys.stderr)
        sys.exit(1)

    write_outputs(results, outdir, bag)
    draw_plots(plt, results, outdir)

    # 조건 메모가 옆에 있으면 같이 복사해 둔다 (결과만 떼어 봐도 조건을 알 수 있게)
    cond = bag.rstrip("/") + "_condition.txt"
    if os.path.exists(cond):
        with open(cond, encoding="utf-8") as src:
            body = src.read()
        with open(os.path.join(outdir, "condition.txt"), "w", encoding="utf-8") as dst:
            dst.write(body)
        print("[출력] {}".format(os.path.join(outdir, "condition.txt")))

    print("\n[완료] 결과 폴더: {}".format(outdir))
    print("       보고서에 실을 때는 summary.csv 의 수치와 condition.txt 의 조건을 함께 적을 것.")


if __name__ == "__main__":
    main()
