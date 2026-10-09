"""Read-only field checklist. Never changes approvals or issues commands."""
from .contracts import finite


def field_preflight(settings, sample, *, map_loaded, recording_ok):
    checks = []

    def check(code, title, passed):
        checks.append(dict(code=code, title=title, passed=bool(passed)))

    check('execution', '실기 명령 출력 활성', settings.execute)
    for name, title in (
        ('layout_confirmed', '앵커 배치 실측'),
        ('alignment_confirmed', '창고·PX4 좌표 방향 검증'),
        ('sensor_mount_confirmed', '센서 장착 실측'),
        ('timing_confirmed', '관측 시각 정렬 검증'),
        ('fusion_confirmed', 'ULog 센서 융합 검증'),
        ('takeoff_settings_confirmed', '이륙 설정 실측 검증'),
        ('heading_control_confirmed', '방위 제어 실측 검증'),
    ):
        check(name, title, getattr(settings, name))
    check('map', '실측 지도·SHA 연결', map_loaded)
    check('recording', '현장 기록 파일 사용 가능', recording_ok)
    check('fc', 'FC 연결·최신 상태', sample.connected and 0 <= sample.state_age_s <= settings.state_timeout_s)
    check('ground', 'DISARM·지상 상태', not sample.armed and sample.landed == 1
          and 0 <= sample.landed_age_s <= settings.state_timeout_s)
    check('mode', 'Position 또는 Hold 모드', sample.mode in ('POSCTL', 'AUTO.LOITER'))
    check('services', 'MAVROS 명령 서비스', sample.command_services_ready)
    check('estimator', 'PX4 수평 위치 추정 유효', sample.estimator_valid and 0 <= sample.estimator_age_s <= .5)
    check('pose', 'PX4 위치·현장 경계', bool(sample.xy) and settings.inside(sample.xy)
          and 0 <= sample.pose_age_s <= settings.pose_timeout_s)
    check('uwb', '최근 B_TF 수평 관측', 0 <= sample.uwb_age_s <= .25)
    check('height', '최근 장착 보정 높이 관측', 0 <= sample.height_age_s <= .2)
    check('bridge', 'UWB→PX4 실제 발행', sample.bridge_ready and sample.bridge_published > 0
          and 0 <= sample.bridge_age_s <= 2.5 and 0 <= sample.bridge_observation_age_s <= .2)
    check('transform', '관측·명령 좌표 변환 일치', sample.alignment_matches)
    check('origin', 'PX4 실제 전역 기준점', sample.origin is not None)
    fresh_params = 0 <= sample.takeoff_param_age_s <= 5
    check('height_parameter', 'FC 이륙 높이 일치', fresh_params and finite(sample.takeoff_alt_m)
          and abs(sample.takeoff_alt_m-settings.expected_mis_takeoff_alt_m) <= .01)
    check('takeoff_action', '이륙 후 Hold 설정', fresh_params and sample.takeoff_action == 0)
    check('heading_policy', '방위·높이 정책 일치', fresh_params and sample.mag_type == settings.expected_ekf2_mag_type
          and not (sample.mag_type in (0, 1) and settings.expected_mis_takeoff_alt_m < 1.6))
    check('rc', '실제 RC 입력 (SITL 제외)', not sample.rc_required or sample.rc_valid and 0 <= sample.rc_age_s <= 1)
    check('rc_mode', 'RC 전용 입력 정책 (SITL 제외)', not sample.rc_required or fresh_params and sample.rc_mode == 0)
    check('rc_switches', 'RC 모드·시동·킬 매핑과 유효 입력', not sample.rc_required or fresh_params and sample.rc_mapping_valid)
    check('rc_auto_override', 'AUTO 스틱 인계 허용', fresh_params and type(sample.rc_override) is int
          and bool(sample.rc_override & 1))
    check('battery', '배터리 연결·시작 잔량', finite(sample.battery) and .3 <= sample.battery <= 1
          and 0 <= sample.battery_age_s <= 3)
    check('motion', 'PX4 속도·방위 수신', len(sample.velocity_xy) == 2 and finite(*sample.velocity_xy,
          sample.vertical_speed_m_s, sample.yaw_deg))
    return dict(scope='observation_only_not_flight_authorization',
                checked_inputs_passed=all(row['passed'] for row in checks), checks=checks,
                blockers=[row['code'] for row in checks if not row['passed']],
                expected_takeoff_height_m=settings.expected_mis_takeoff_alt_m,
                expected_mag_type=settings.expected_ekf2_mag_type,
                requires_fresh_start=True)
