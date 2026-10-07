"""Record one file-only UWB comparison iteration with a predeclared plan.

The simulator-pose replay remains a shadow experiment. A plan and every run
get separate immutable directories; no result is ever a PX4 flight claim.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

from drone_uwb.processing.experiments.gazebo_trial import evaluate, run
from drone_uwb.processing.experiments.h80_b import write_json
from drone_uwb.processing.runner import json_line


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _plan(path, output):
    plan = json.loads(Path(path).read_text(encoding='utf-8'))
    if plan.get('run_id') != output.name or plan.get('dataset_split') not in ('exploratory', 'tuning', 'evaluation'):
        raise ValueError('run_id_or_dataset_split_mismatch')
    if not isinstance(plan.get('hypothesis'), str) or not plan['hypothesis'].strip():
        raise ValueError('hypothesis_required')
    if not isinstance(plan.get('change'), dict) or not plan['change']:
        raise ValueError('single_change_required')
    criteria = plan.get('criteria')
    if not isinstance(criteria, dict) or criteria.get('target_m') != .07:
        raise ValueError('predeclared_7cm_target_required')
    coverage = criteria.get('min_coverage')
    if coverage is not None and (not isinstance(coverage, (int, float)) or isinstance(coverage, bool)
                                 or not math.isfinite(coverage) or not 0 < coverage <= 1):
        raise ValueError('invalid_coverage_criterion')
    intervals = plan.get('scenario_intervals', [])
    if not isinstance(intervals, list) or any(
            not isinstance(item, dict) or set(item) != {'name', 'start_s', 'end_s'}
            or not isinstance(item['name'], str) or not item['name']
            or not 0 <= item['start_s'] < item['end_s'] for item in intervals):
        raise ValueError('invalid_scenario_intervals')
    return plan


def _rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines()]


def _selected_status(metrics, plan, source):
    criteria = plan['criteria']
    if not metrics['count']:
        return 'no_outputs'
    if criteria.get('min_coverage') is None:
        return 'coverage_criterion_missing'
    if metrics['coverage'] < criteria['min_coverage']:
        return 'coverage_failed'
    if not metrics['max_under_target_on_valid']:
        return '7cm_max_failed'
    if plan['dataset_split'] != 'evaluation':
        return 'exploratory_or_tuning_only'
    if source != 'gazebo':
        return 'simulator_runtime_unverified'
    # Even sensor-fed file replay cannot establish end-to-end live delay or fusion.
    return 'live_latency_and_PX4_fusion_unverified'


def record(run_dir, plan_path, input_path, config_path, parent=None, raw_path=None, truth_path=None,
           tof_path=None, attitude_path=None, height_profile_path=None):
    """Add an append-only comparison record to an existing replay directory."""
    run_dir, plan_path = Path(run_dir), Path(plan_path)
    plan = _plan(plan_path, run_dir)
    rows = _rows(run_dir/'results.jsonl')
    if not rows:
        raise ValueError('empty_result')
    models = tuple(rows[0]['models'])
    summary = json.loads((run_dir/'summary.json').read_text(encoding='utf-8'))
    if plan.get('parent_run_id') != (Path(parent).name if parent else None):
        raise ValueError('parent_run_id_mismatch')
    checks = json.loads((run_dir/'manifest.json').read_text(encoding='utf-8'))
    if checks['input_sha256'] != _sha(input_path) or checks['config_sha256'] != _sha(config_path):
        raise ValueError('input_or_config_changed_after_replay')
    if (plan.get('input_sha256') != _sha(input_path)
            or plan.get('config_sha256') != _sha(config_path)):
        raise ValueError('predeclared_input_or_config_mismatch')
    if raw_path is not None and (plan.get('raw_sha256') != _sha(raw_path)
                                 or plan.get('truth_sha256') != _sha(truth_path)
                                 or checks['recorded_raw_sha256'] != _sha(raw_path)
                                 or checks['recorded_truth_sha256'] != _sha(truth_path)):
        raise ValueError('recorded_capture_hash_mismatch')
    sensor_paths = (tof_path, attitude_path, height_profile_path)
    if all(path is not None for path in sensor_paths) and any(
            plan.get(key) != _sha(path) or checks[key] != _sha(path)
            for key, path in zip(('tof_sha256', 'attitude_sha256', 'height_profile_sha256'), sensor_paths)):
        raise ValueError('sensor_capture_hash_mismatch')
    additions = ('plan.json', 'iteration.json', 'input_index.json', 'scenario_metrics.csv',
                 'failures.jsonl', 'comparison.md', 'iteration_outputs_sha256.json')
    if any((run_dir/name).exists() for name in additions):
        raise FileExistsError('iteration_record_already_exists')
    target = plan['criteria']['target_m']
    all_metrics = evaluate(rows, target)['models']
    statuses = {m: _selected_status(all_metrics[m], plan, summary['source']) for m in models}
    scenarios = [('overall', rows)]
    for interval in plan.get('scenario_intervals', []):
        selected = [r for r in rows if interval['start_s'] <= r['t_rel_s'] < interval['end_s']]
        scenarios.append((interval['name'], selected))
    scenario_rows = []
    for name, selected in scenarios:
        evaluated = evaluate(selected, target)['models']
        for model in models:
            value = evaluated[model]
            scenario_rows.append(dict(scenario=name, model=model, input_ticks=len(selected),
                                      output_count=value['count'], missing_count=value['missing_count'],
                                      coverage=value['coverage'], median_m=value.get('median_m'),
                                      rmse_m=value.get('rmse_m'), p95_m=value.get('p95_m'),
                                      max_m=value.get('max_m'),
                                      at_or_above_7cm_count=value['at_or_above_target_count']))
    parent_status = 'none'
    if parent:
        previous = _rows(Path(parent)/'results.jsonl')
        same_ranges = len(previous) == len(rows) and all(
            a['time_us'] == b['time_us'] and a['raw_slant_m'] == b['raw_slant_m']
            for a, b in zip(previous, rows))
        parent_status = 'same_raw' if same_ranges else 'different_raw_no_causal_pairing'
    record_data = dict(run_id=plan['run_id'], parent_run_id=plan.get('parent_run_id'),
                       dataset_split=plan['dataset_split'], source=summary['source'],
                       height_source=summary['height_source'], range_source=summary['range_source'],
                       hypothesis=plan['hypothesis'],
                       change=plan['change'], criteria=plan['criteria'], model_metrics=all_metrics,
                       model_status=statuses, parent_comparison=parent_status,
                       achieved_7cm=False,
                       achieved_reason=('pose_replay_lacks_ToF_and_end_to_end_latency_validation'
                                        if tof_path is None else
                                        'sensor_file_replay_lacks_live_latency_and_PX4_fusion_validation'))
    # The replay's own manifest is left intact; these files are iteration notes.
    shutil.copyfile(plan_path, run_dir/'plan.json')
    write_json(run_dir/'iteration.json', record_data)
    write_json(run_dir/'input_index.json', dict(input_path=str(input_path), input_sha256=_sha(input_path),
              config_path=str(config_path), config_sha256=_sha(config_path),
              plan_path=str(plan_path), plan_sha256=_sha(plan_path),
              raw_path=str(raw_path) if raw_path else None,
              raw_sha256=_sha(raw_path) if raw_path else None,
              truth_path=str(truth_path) if truth_path else None,
              truth_sha256=_sha(truth_path) if truth_path else None,
              tof_path=str(tof_path) if tof_path else None,
              tof_sha256=_sha(tof_path) if tof_path else None,
              attitude_path=str(attitude_path) if attitude_path else None,
              attitude_sha256=_sha(attitude_path) if attitude_path else None,
              height_profile_path=str(height_profile_path) if height_profile_path else None,
              height_profile_sha256=_sha(height_profile_path) if height_profile_path else None,
              parent_path=str(parent) if parent else None))
    with (run_dir/'scenario_metrics.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(scenario_rows[0]))
        writer.writeheader()
        writer.writerows(scenario_rows)
    with (run_dir/'failures.jsonl').open('x', encoding='utf-8') as stream:
        for row in rows:
            for model in models:
                value = row['models'][model]
                if not value['ok']:
                    stream.write(json_line(dict(seq=row['seq'], time_us=row['time_us'], model=model,
                                                reason=value['reason'])))
    table = ['| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |',
             '|---|---:|---:|---:|---:|---:|---|']
    for model in models:
        value = all_metrics[model]
        fmt = lambda key: f"{100*value[key]:.3f}" if key in value else '—'
        table.append(f"| {model} | {value['count']}/{len(rows)} | {fmt('max_m')} | "
                     f"{fmt('rmse_m')} | {fmt('p95_m')} | "
                     f"{value['at_or_above_target_count']} | {statuses[model]} |")
    comparison = ('# UWB 시행 비교\n'
                  '이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.\n\n'
                  f"실행 ID: `{plan['run_id']}`. 자료: `{summary['source']}`.\n"
                  f"거리 출처: `{summary['range_source']}`.\n"
                  f"높이 출처: `{summary['height_source']}`.\n"
                  f"변경 가설: {plan['hypothesis']}\n"
                  f"이전 시행과의 RAW 관계: `{parent_status}`.\n\n"
                  + '\n'.join(table) + '\n\n'
                  '유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.\n'
                  '실시간 센서 지연·PX4 융합·독립 Gazebo 비행은 검증 전이다.\n')
    (run_dir/'comparison.md').write_text(comparison, encoding='utf-8')
    write_json(run_dir/'iteration_outputs_sha256.json', {p.name: _sha(p) for p in run_dir.iterdir()
              if p.is_file() and p.name != 'iteration_outputs_sha256.json'})
    return record_data


def execute(input_path, config_path, plan_path, output, parent=None, raw_path=None, truth_path=None,
            tof_path=None, attitude_path=None, height_profile_path=None):
    output = Path(output)
    plan = _plan(plan_path, output)
    if output.exists():
        raise FileExistsError(output)
    if plan.get('parent_run_id') != (Path(parent).name if parent else None):
        raise ValueError('parent_run_id_mismatch')
    if (raw_path is None) != (truth_path is None):
        raise ValueError('recorded_raw_and_truth_required_together')
    sensor_paths = (tof_path, attitude_path, height_profile_path)
    if any(path is not None for path in sensor_paths) and not all(path is not None for path in sensor_paths):
        raise ValueError('tof_attitude_and_height_profile_required_together')
    if plan.get('input_sha256') != _sha(input_path) or plan.get('config_sha256') != _sha(config_path):
        raise ValueError('predeclared_input_or_config_mismatch')
    if raw_path is not None and (plan.get('raw_sha256') != _sha(raw_path)
                                 or plan.get('truth_sha256') != _sha(truth_path)):
        raise ValueError('predeclared_recorded_capture_mismatch')
    if all(path is not None for path in sensor_paths) and any(
            plan.get(key) != _sha(path)
            for key, path in zip(('tof_sha256', 'attitude_sha256', 'height_profile_sha256'), sensor_paths)):
        raise ValueError('predeclared_sensor_capture_mismatch')
    try:
        run(input_path, config_path, output, raw_path, truth_path,
            tof_path, attitude_path, height_profile_path)
        return record(output, plan_path, input_path, config_path, parent, raw_path, truth_path,
                      tof_path, attitude_path, height_profile_path)
    except BaseException as exc:
        output.mkdir(parents=True, exist_ok=True)
        if not (output/'plan.json').exists():
            shutil.copyfile(plan_path, output/'plan.json')
        write_json(output/'failed_attempt.json', dict(run_id=plan['run_id'],
                   error_type=type(exc).__name__, error_message=str(exc),
                   input_sha256=_sha(input_path), config_sha256=_sha(config_path),
                   raw_sha256=_sha(raw_path) if raw_path else None,
                   truth_sha256=_sha(truth_path) if truth_path else None,
                   tof_sha256=_sha(tof_path) if tof_path else None,
                   attitude_sha256=_sha(attitude_path) if attitude_path else None,
                   height_profile_sha256=_sha(height_profile_path) if height_profile_path else None,
                   plan_sha256=_sha(plan_path)))
        raise


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'config', 'plan', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--parent', type=Path)
    parser.add_argument('--raw', type=Path, help='Recorded Gazebo RAW JSONL; requires --truth')
    parser.add_argument('--truth', type=Path, help='Recorded Gazebo truth JSONL; requires --raw')
    parser.add_argument('--tof', type=Path, help='Gazebo ToF JSONL')
    parser.add_argument('--attitude', type=Path, help='Gazebo IMU JSONL')
    parser.add_argument('--height-profile', type=Path, help='Sensor geometry and validation gates')
    options = parser.parse_args(args)
    result = execute(options.input, options.config, options.plan, options.output,
                     options.parent, options.raw, options.truth,
                     options.tof, options.attitude, options.height_profile)
    print(json.dumps(dict(run_id=result['run_id'], model_status=result['model_status'],
                          achieved_7cm=result['achieved_7cm']), ensure_ascii=False))


if __name__ == '__main__':
    main()
