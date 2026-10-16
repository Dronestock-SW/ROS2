"""Passive bounded UWB pipeline/CPU probe. No publishers, services or FC writes."""
import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import time


def distribution(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values:
        return {"samples": 0}
    return {"samples": len(values), "p50": values[len(values)//2],
            "p95": values[min(len(values)-1, int(len(values)*.95))], "max": values[-1]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not 5 <= args.seconds <= 300:
        parser.error("seconds must be within 5..300")
    if args.output.exists():
        parser.error("output already exists; preserve previous evidence")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus
    from sensor_msgs.msg import Range, Imu

    rclpy.init(args=[])
    node = Node("uwb_gap_receive_only_probe")
    started = time.monotonic()
    started_wall_ns, started_monotonic_ns = time.time_ns(), time.monotonic_ns()
    topics = {
        "/uwb/received": String, "/uwb/btf_decision": String,
        "/uwb/btf_status": String, "/uwb/local_odometry_status": String,
        "/uwb/btf_pose": PoseWithCovarianceStamped,
        "/mavros/state": State, "/mavros/extended_state": ExtendedState,
        "/mavros/downward_0": Range, "/mavros/imu/data": Imu,
        "/mavros/timesync_status": TimesyncStatus,
    }
    stats, reasons, statuses, invalid = {}, Counter(), {}, Counter()
    metrics = {k: deque(maxlen=24000) for k in
        ("processing_ms", "queue_age_s", "receive_age_s", "publish_age_s",
         "raw_receiver_age_s", "receiver_callback_age_s")}
    gap_events = deque(maxlen=2000)
    decision_tail = deque(maxlen=40)
    cpu = []
    subscriptions = []

    def arrival(key, now, header_ns=None):
        s = stats.setdefault(key, {"count": 0, "first_s": now-started,
                                  "last_s": None, "max_gap_s": 0., "gaps_over_250ms": 0})
        if s["last_s"] is not None:
            gap = now-started-s["last_s"]
            s["max_gap_s"] = max(s["max_gap_s"], gap)
            if gap > .25:
                s["gaps_over_250ms"] += 1
                if key in ("raw_cycle", "/uwb/btf_pose", "accepted_decision"):
                    gap_events.append({"topic": key, "end_s": now-started,
                                       "gap_s": gap, "recent_decisions": list(decision_tail)})
        s["last_s"] = now-started
        s["count"] += 1
        if header_ns is not None:
            s["last_header_age_s"] = (node.get_clock().now().nanoseconds-header_ns)/1e9

    def receive(topic, msg):
        now = time.monotonic()
        header = getattr(msg, "header", None)
        stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec if header else None
        arrival(topic, now, stamp)
        if topic == "/mavros/state":
            statuses[topic] = {"connected": msg.connected, "armed": msg.armed, "mode": msg.mode}
        elif topic == "/mavros/extended_state":
            statuses[topic] = {"landed_state": msg.landed_state}
        elif isinstance(msg, String):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError("nonobject")
                if topic == "/uwb/received":
                    if value.get("message", {}).get("type") == "uwb_raw_cycle":
                        arrival("raw_cycle", now)
                        received = value.get("host_received_ros_ns")
                        if type(received) is int:
                            metrics["raw_receiver_age_s"].append(
                                (node.get_clock().now().nanoseconds-received)/1e9)
                elif topic == "/uwb/btf_decision":
                    reasons[value.get("pose_blocked_reason") or value.get("reason", "unknown")] += 1
                    decision_tail.append({"s": now-started, "seq": value.get("seq"),
                        "reason": value.get("reason"), "published": value.get("published"),
                        "pose_blocked_reason": value.get("pose_blocked_reason")})
                    if value.get("published") is True:
                        arrival("accepted_decision", now, value.get("stamp_ns"))
                    for key in metrics:
                        if key in value:
                            metrics[key].append(value[key])
                    if type(value.get("receiver_age_ns")) is int:
                        metrics["receiver_callback_age_s"].append(value["receiver_age_ns"]/1e9)
                else:
                    s = statuses.setdefault(topic, {})
                    if "first" not in s:
                        s["first"] = value
                    s["last"] = value
            except (ValueError, TypeError, AttributeError):
                invalid[topic] += 1

    for topic, message_type in topics.items():
        subscriptions.append(node.create_subscription(message_type, topic,
            lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data))

    hz = os.sysconf("SC_CLK_TCK")
    previous_cpu = None
    previous_process = {}
    next_cpu = started
    ended = started
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if now >= next_cpu:
                parts = [int(v) for v in Path("/proc/stat").read_text().splitlines()[0].split()[1:]]
                # guest fields are already included in user/nice.
                total, idle = sum(parts[:8]), parts[3]+parts[4]
                row = {"s": now-started, "loadavg": Path("/proc/loadavg").read_text().split()[:3],
                       "cpu_busy_percent": None, "process_cpu_percent_one_core": {}}
                if previous_cpu and total > previous_cpu[0]:
                    row["cpu_busy_percent"] = 100*(1-(idle-previous_cpu[1])/(total-previous_cpu[0]))
                previous_cpu = total, idle
                for path in Path("/proc").glob("[0-9]*/stat"):
                    try:
                        text = path.read_text()
                        name = text[text.index("(")+1:text.rindex(")")]
                        if name not in ("uwb_btf_node", "uwb_node", "mavros_node"):
                            continue
                        fields = text[text.rindex(")")+2:].split()
                        ticks = int(fields[11])+int(fields[12])
                        key = path.parent.name+":"+name+":"+fields[19]
                        if key in previous_process:
                            prev_ticks, prev_at = previous_process[key]
                            row["process_cpu_percent_one_core"][key] = 100*(ticks-prev_ticks)/hz/(now-prev_at)
                        previous_process[key] = ticks, now
                    except (OSError, ValueError, IndexError):
                        pass
                cpu.append(row)
                next_cpu = now+1.
        ended = time.monotonic()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    elapsed = ended-started
    for s in stats.values():
        s["tail_age_s"] = elapsed-s["last_s"]
        span = s["last_s"]-s["first_s"]
        s["rate_hz_after_discovery"] = (s["count"]-1)/span if span > 0 else None
    report = {"scope": "passive current receive window; not historical run8 or flight proof",
        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "started_wall_ns": started_wall_ns, "started_monotonic_ns": started_monotonic_ns,
        "elapsed_s": elapsed, "fc_commands_sent": 0, "parameter_writes": 0,
        "topics": stats, "topics_not_received": sorted(set(topics)-set(stats)),
        "btf_reasons": dict(reasons), "metrics": {k: distribution(v) for k,v in metrics.items()},
        "statuses": statuses, "invalid_json": dict(invalid),
        "cpu": cpu, "gaps": list(gap_events),
        "limits": "receipt times include probe scheduling; 250ms is descriptive only; startup and tail excluded from paired gaps"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "elapsed_s": elapsed,
        "raw_cycles": stats.get("raw_cycle", {}).get("count", 0), "reasons": dict(reasons)}))


if __name__ == "__main__":
    main()
