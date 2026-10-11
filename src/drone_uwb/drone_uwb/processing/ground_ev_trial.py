"""Bounded static EV transport experiment; never a flight readiness decision."""
import math


class GroundAuthorityLatch:
    """Fresh authority loss ends the ground session; missing data only pauses it."""
    def __init__(self):
        self.revoked = False

    def observe(self, *, fresh, armed=False, connected=True, landed=1, kill_channel=1999):
        if fresh and (armed or not connected or landed in (2,3,4) or not 1900<=kill_channel<=2100):
            self.revoked = True
        return self.revoked


def inflated_xy_covariance(values):
    """Validate the original covariance before adding the static trial allowance."""
    if len(values) != 4 or any(not math.isfinite(x) for x in values):
        raise ValueError('invalid_source_covariance')
    xx, xy, yx, yy = values
    if min(xx, yy) <= 0 or abs(xy-yx) > 1e-9 or xx*yy-xy*yx <= 0:
        raise ValueError('invalid_source_covariance')
    return (xx+.25, xy, yx, yy+.25)


def ground_trial_gate(*, connected, armed, state_age_s, landed, landed_age_s,
                      kill_channel, rc_age_s, bridge_disabled, bridge_age_s,
                      execution_disabled, flight_age_s, parameters, parameter_age_s,
                      measured_mount_frd_m):
    if not connected or armed or not 0 <= state_age_s <= 1.5:
        return 'fresh_disarmed_fc_required'
    if landed != 1 or not 0 <= landed_age_s <= 1.5:
        return 'fresh_ground_state_required'
    if not 1900 <= kill_channel <= 2100 or not 0 <= rc_age_s <= .3:
        return 'mapped_kill_switch_on_required'
    if not bridge_disabled or not 0 <= bridge_age_s <= .5:
        return 'production_bridge_must_remain_disabled'
    if not execution_disabled or not 0 <= flight_age_s <= .5:
        return 'mission_output_must_remain_disabled'
    if (not isinstance(measured_mount_frd_m, (list, tuple)) or len(measured_mount_frd_m) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1
                   for x in measured_mount_frd_m)):
        return 'measured_mount_required'
    expected=dict(EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=0., RC_MAP_KILL_SW=7)
    expected.update(zip(('EKF2_EV_POS_X','EKF2_EV_POS_Y','EKF2_EV_POS_Z'), measured_mount_frd_m))
    if not 0 <= parameter_age_s <= 3:
        return 'parameters_stale'
    for key,value in expected.items():
        actual=parameters.get(key)
        if type(actual) not in (int,float) or not math.isfinite(actual) or abs(actual-value)>.001:
            return 'parameter_mismatch:'+key
    return 'ready'
