import copy

from drone_mission.hover_metrics import evaluate_window


def samples(offset=.02):
    return [dict(clock_s=i*.05,pose_xy=[offset,0.],truth_xy=[offset,0.],
        velocity_xy=[0.,0.],target_before_tick=[0.,0.],height_before_tick_m=1.7,
        snapshot=dict(altitude_enu_m=1.7,vertical_speed_m_s=0.,uwb_xy=[offset,0.]),
        armed=True,landed=2,mode='OFFBOARD') for i in range(201)]


def score(rows):return evaluate_window(rows,target_xy=[0.,0.],height_target_m=1.7)


def test_constant_bias_is_not_hidden_by_mean_subtraction():
    assert score(samples(.02))['precision_pass']
    value=score(samples(.05))
    assert value['complete'] and not value['precision_pass']
    assert 'truth_goal_xy_rms_exceeded' in value['reasons']


def test_fc_success_does_not_hide_truth_error():
    rows=samples()
    for row in rows:row['truth_xy']=[.06,0.]
    assert score(rows)['reasons']==['truth_goal_xy_rms_exceeded']


def test_missing_interval_and_two_second_hold_do_not_pass():
    assert 'short_hold' in score(samples()[:41])['reasons']
    rows=samples();del rows[90:100]
    assert 'invalid_sample_continuity' in score(rows)['reasons']


def test_moving_target_and_vertical_excursion_do_not_pass():
    rows=copy.deepcopy(samples());rows[100]['target_before_tick']=[.1,0.]
    rows[120]['snapshot']['vertical_speed_m_s']=.11
    value=score(rows)
    assert 'target_changed' in value['reasons'] and 'vz_limit_exceeded' in value['reasons']
