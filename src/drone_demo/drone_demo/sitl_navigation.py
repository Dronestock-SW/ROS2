"""One MAVLink owner: live UWB observations plus a bounded SITL waypoint trial.

Start already airborne in PX4 Hold. This adapter never arms or takes off.
PX4 owns all flight control. No Gazebo truth enters navigation decisions.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

from drone_demo.mission import MissionConfig
from drone_demo.sitl_mission import PX4MissionMonitor
from drone_uwb.integration.sitl.px4_clock_tracker import PX4ClockTracker
from drone_uwb.integration.sitl.sitl_link_probe import _name, decode_px4_param
from drone_uwb.integration.sitl.sitl_observer import log_value
from drone_uwb.integration.sitl.sitl_target_contract import (
    NavigationReadiness, PX4GlobalReference, RepositionProgress, SITLTargetSettings,
    navigation_gate, reposition_fields, send_reposition_fields)


FAILSAFE_PARAMS = ('NAV_DLL_ACT', 'COM_DL_LOSS_T', 'COM_DLL_EXCEPT',
                   'COM_FAIL_ACT_T', 'COM_POS_FS_ACT')


@dataclass(frozen=True)
class NavigationPlan:
    targets_map_xy_m: tuple
    return_to_start: bool = True
    land_on_completion: bool = True
    max_leg_sim_s: float = 20.

    def __post_init__(self):
        if (not isinstance(self.targets_map_xy_m, (list, tuple))
                or not 1 <= len(self.targets_map_xy_m) <= 20):
            raise ValueError('one_to_twenty_targets_required')
        if any(not isinstance(p, (list, tuple)) or len(p) != 2
               or any(type(v) not in (int, float) or not math.isfinite(v) for v in p)
               for p in self.targets_map_xy_m):
            raise ValueError('finite_map_targets_required')
        if any(type(v) is not bool for v in (self.return_to_start, self.land_on_completion)):
            raise ValueError('boolean_navigation_options_required')
        if (type(self.max_leg_sim_s) not in (int, float)
                or not math.isfinite(self.max_leg_sim_s) or self.max_leg_sim_s <= 0):
            raise ValueError('positive_leg_timeout_required')


class SITLNavigationSession:
    def __init__(self, observer, settings, mission_config, plan, *, mode='monitor',
                 write_event=lambda event: None, host_now_ns=time.monotonic_ns):
        if observer.connection is None or observer.dialect is None:
            raise ValueError('live_mavlink_connection_required')
        if mode not in ('monitor', 'send') or mode == 'send' and observer.mode != 'send':
            raise ValueError('navigation_send_requires_observation_send_mode')
        self.observer, self.settings, self.plan = observer, settings, plan
        self.d, self.mav = observer.dialect, observer.connection.mav
        self.mode, self.write_event, self.host_now_ns = mode, write_event, host_now_ns
        self.clock = PX4ClockTracker(self.mav, host_now_ns=host_now_ns)
        self.monitor = PX4MissionMonitor(mission_config, observer.settings, self.d)
        self.messages, self.params, self.param_receipts = {}, {}, {}
        self.history = {name: deque(maxlen=64) for name in ('ODOMETRY', 'ESTIMATOR_STATUS')}
        self.origin = self.initial_reset = None
        self.progress = None
        self.phase, self.reason = 'WAITING', 'inputs_missing'
        self.targets, self.leg_index = [], 0
        self.leg_started_sim_us = None
        self.last_tick_ns = self.last_request_ns = self.last_heartbeat_ns = None
        self.streams_requested = False
        self.last_uwb_sim_us = None
        self.sent_count = self.completed_legs = 0
        self.attempted_count = 0
        self.warmup = None
        self.land_sent_host_ns = None
        self.last_good_state = None
        self.other_gcs_seen = False

    def emit(self, kind, **fields):
        clocks = {}
        for name, read in (('gazebo_time_us', lambda: self.observer.clock.now_ns()//1000),
                           ('px4_time_us', self.clock.now_us)):
            try:
                clocks[name] = read()
            except ValueError as exc:
                clocks[name], clocks[name+'_reason'] = None, str(exc)
        self.write_event(log_value(dict(type=kind, host_monotonic_ns=self.host_now_ns(),
            phase=self.phase, leg_index=self.leg_index, flight_valid=False,
            fusion_verified=False, **clocks, **fields)))

    def fresh(self, kind, limit_s):
        item = self.messages.get(kind)
        return item is not None and 0 <= (self.host_now_ns()-item[1])/1e9 <= limit_s

    def on_message(self, message):
        d, now = self.d, self.host_now_ns()
        kind = message.get_type()
        if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            if kind == 'HEARTBEAT' and message.type == d.MAV_TYPE_GCS:
                self.other_gcs_seen = True
                self.emit('other_gcs_seen', system=message.get_srcSystem(), component=message.get_srcComponent())
            return
        if kind == 'TIMESYNC':
            record = self.clock.handle(message)
            if record:
                self.emit('clock', record=record)
            return
        if kind == 'PARAM_VALUE':
            name = _name(message)
            if name in FAILSAFE_PARAMS:
                try:
                    value = decode_px4_param(name, message.param_value, message.param_type,
                        int32_type=d.MAV_PARAM_TYPE_INT32, real32_type=d.MAV_PARAM_TYPE_REAL32,
                        allowed_names=FAILSAFE_PARAMS)
                except ValueError as exc:
                    self.params.pop(name, None)
                    self.param_receipts.pop(name, None)
                    self.emit('parameter_rejected', name=name, reason=str(exc))
                else:
                    self.params[name], self.param_receipts[name] = value['value'], now
                    self.emit('parameter', record=value)
            return
        if kind == 'GPS_GLOBAL_ORIGIN':
            origin = PX4GlobalReference.from_message(message)
            if self.origin is not None and origin != self.origin:
                self.monitor.invalidate('px4_global_reference_changed', latch=True)
            else:
                self.origin = origin
        if kind == 'ODOMETRY' and message.estimator_type != d.MAV_ESTIMATOR_TYPE_AUTOPILOT:
            return
        if kind in ('ODOMETRY', 'ESTIMATOR_STATUS'):
            previous = self.messages.get(kind)
            if previous is not None:
                if message.time_usec < previous[0].time_usec:
                    self.monitor.invalidate('px4_time_reversed', latch=True)
                    return
                if message.time_usec == previous[0].time_usec:
                    return
            self.history[kind].append((message, now))
        if kind in ('HEARTBEAT', 'ODOMETRY', 'ESTIMATOR_STATUS', 'EXTENDED_SYS_STATE',
                    'GPS_GLOBAL_ORIGIN', 'POSITION_TARGET_GLOBAL_INT'):
            self.messages[kind] = (message, now)
        if kind == 'COMMAND_ACK':
            if self.progress is not None:
                self.progress.handle(message, received_host_s=now/1e9)
            self.emit('command_ack', payload=message.to_dict())

    def on_observation(self, event):
        if event.get('transmitted') is not True:
            return
        stamp = event['time_us']
        now = self.host_now_ns()
        age = max(event.get('sample_age_sim_s', math.inf), event.get('wall_residence_s', math.inf))
        if self.monitor.update_uwb(stamp*1000, now/1e9, age):
            self.last_uwb_sim_us = stamp

    def request_streams(self):
        d = self.d
        for name, rate in (('ODOMETRY', 30), ('ESTIMATOR_STATUS', 10),
                           ('POSITION_TARGET_GLOBAL_INT', 10), ('EXTENDED_SYS_STATE', 2)):
            self.mav.command_long_send(1, 1, d.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                getattr(d, 'MAVLINK_MSG_ID_'+name), 1e6/rate, 0., 0., 0., 0., 0.)
            self.emit('stream_interval_requested', message=name, rate_hz=rate)
        self.streams_requested = True

    def request_state(self):
        now, d = self.host_now_ns(), self.d
        if not self.streams_requested:
            self.request_streams()
        record = self.clock.poll()
        if record:
            self.emit('clock', record=record)
        if self.last_request_ns is None or now-self.last_request_ns >= 2_000_000_000:
            self.mav.command_long_send(1, 1, d.MAV_CMD_REQUEST_MESSAGE, 0,
                d.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0., 0., 0., 0., 0., 0.)
            for name in FAILSAFE_PARAMS:
                self.mav.param_request_read_send(1, 1, name.encode('ascii'), -1)
            self.last_request_ns = now
        if self.mode == 'send' and (self.last_heartbeat_ns is None or now-self.last_heartbeat_ns >= 1_000_000_000):
            # This process is the trial ground station; losing it must remain
            # distinguishable from losing only a UWB observation.
            self.mav.heartbeat_send(d.MAV_TYPE_GCS, d.MAV_AUTOPILOT_INVALID, 0, 0, d.MAV_STATE_ACTIVE)
            self.last_heartbeat_ns = now

    def update_pose(self):
        if not self.fresh('ODOMETRY', .2) or not self.fresh('ESTIMATOR_STATUS', .2):
            return
        try:
            clock_us = self.clock.now_us()
            # Telemetry may arrive after the last clock reply. Keep a bounded
            # history so a newer sample cannot starve every usable older one.
            odometry, odom_receipt = next(item for item in reversed(self.history['ODOMETRY'])
                                         if item[0].time_usec <= clock_us)
            estimator, est_receipt = next(item for item in reversed(self.history['ESTIMATOR_STATUS'])
                                         if item[0].time_usec <= clock_us)
            age = self.clock.sample_age_s(odometry.time_usec, odom_receipt)
            est_age = self.clock.sample_age_s(estimator.time_usec, est_receipt)
        except (ValueError, StopIteration):
            return  # Wait for the next clock reply, keeping prior pose age.
        if self.monitor.update_px4(odometry, estimator, received_s=self.host_now_ns()/1e9,
                                  sample_age_s=age, estimator_age_s=est_age):
            if self.initial_reset is None:
                self.initial_reset = odometry.reset_counter

    def readiness(self, assessment):
        d, now = self.d, self.host_now_ns()
        if self.observer.fault or self.monitor.fault or self.clock.fault:
            raise ValueError(self.observer.fault or self.monitor.fault or self.clock.fault)
        if not self.observer.link_drained:
            raise ValueError('mavlink_queue_not_drained')
        if not self.fresh('HEARTBEAT', 3.) or not self.fresh('EXTENDED_SYS_STATE', 1.5):
            raise ValueError('flight_mode_or_landed_state_stale')
        if not self.fresh('ESTIMATOR_STATUS', .2) or self.origin is None:
            raise ValueError('estimator_status_or_origin_unavailable')
        heartbeat = self.messages['HEARTBEAT'][0]
        estimator = self.messages['ESTIMATOR_STATUS'][0]
        if heartbeat.autopilot != d.MAV_AUTOPILOT_PX4 or heartbeat.type != d.MAV_TYPE_QUADROTOR:
            raise ValueError('unexpected_sitl_vehicle')
        if heartbeat.system_status != d.MAV_STATE_ACTIVE:
            raise ValueError('px4_system_not_active')
        pose_age, uwb_age = assessment['pose_age_s'], assessment['uwb_age_s']
        if pose_age is None or uwb_age is None:
            raise ValueError('position_or_observation_missing')
        # Same simulation clock as the observation; host elapsed time alone
        # undercounts age when the simulator runs faster than real time.
        sim_uwb_age = (self.observer.clock.now_ns()//1000-self.last_uwb_sim_us)/1e6
        if sim_uwb_age < 0:
            raise ValueError('observation_simulation_time_reversed')
        px4_pose_age = (self.clock.now_us()-self.monitor.last_px4_sample_us)/1e6
        if px4_pose_age < 0:
            raise ValueError('px4_sample_newer_than_clock')
        flags = estimator.flags
        bad_flags = d.ESTIMATOR_CONST_POS_MODE | d.ESTIMATOR_ACCEL_ERROR
        state = NavigationReadiness(max(pose_age, px4_pose_age), max(uwb_age, sim_uwb_age),
            self.origin.reference_timestamp_us, self.monitor.reset_counter,
            armed=bool(heartbeat.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED),
            in_air=self.messages['EXTENDED_SYS_STATE'][0].landed_state == d.MAV_LANDED_STATE_IN_AIR,
            hold_mode=((heartbeat.custom_mode >> 16) & 255) == 4
                      and ((heartbeat.custom_mode >> 24) & 255) == 3,
            xy_valid=bool(flags & d.ESTIMATOR_POS_HORIZ_REL) and not flags & bad_flags,
            vxy_valid=bool(flags & d.ESTIMATOR_VELOCITY_HORIZ) and not flags & bad_flags,
            attitude_solution_valid=bool(flags & d.ESTIMATOR_ATTITUDE) and not flags & bad_flags,
            global_position_valid=bool(flags & d.ESTIMATOR_POS_HORIZ_ABS))
        if not all(name in self.params and 0 <= now-self.param_receipts[name] <= 5_000_000_000
                   for name in FAILSAFE_PARAMS):
            raise ValueError('navigation_failsafe_parameters_unavailable')
        if (self.params['NAV_DLL_ACT'] != 3 or self.params['COM_DL_LOSS_T'] != 5
                or int(self.params['COM_DLL_EXCEPT']) & 2 or self.params['COM_FAIL_ACT_T'] != 0
                or self.params['COM_POS_FS_ACT'] != 0):
            raise ValueError('navigation_failsafe_settings_mismatch')
        if self.other_gcs_seen:
            raise ValueError('other_gcs_can_mask_command_session_loss')
        reason = navigation_gate(self.settings, self.observer.settings, self.origin,
                                 state, self.initial_reset)
        if reason != 'ready':
            raise ValueError(reason)
        return state

    def warmed_up(self):
        now = self.host_now_ns()
        pose_us, uwb_us = self.monitor.last_px4_sample_us, self.last_uwb_sim_us
        counts = self.monitor.monitor.received_count
        if self.warmup is None:
            self.warmup = (now, pose_us, uwb_us, dict(counts))
            return False
        host, pose, uwb, initial = self.warmup
        config = self.monitor.monitor.config
        return (min((now-host)/1e9, (pose_us-pose)/1e6, (uwb_us-uwb)/1e6) >= config.recovery_s
                and all(counts[key]-initial[key] >= config.recovery_samples for key in counts))

    def start_leg(self, state):
        pose = self.monitor.monitor.pose
        if not self.targets:
            self.targets = [list(point) for point in self.plan.targets_map_xy_m]
            if self.plan.return_to_start:
                self.targets.append([pose.x, pose.y])
            self.emit('resolved_targets', targets_map_xy_m=self.targets,
                      start_reference_xy_m=[pose.x, pose.y])
        target = self.targets[self.leg_index]
        packet = reposition_fields(target, (pose.x, pose.y), self.settings,
            self.observer.settings, self.origin, state, expected_reset_counter=self.initial_reset)
        self.monitor.set_target(*target)
        self.monitor.monitor.reset_health()  # Even a repeated goal needs new dwell evidence.
        sent_px4_us = self.clock.now_us()
        self.emit('target_command_prepared', packet=packet)
        self.attempted_count += 1
        send_reposition_fields(self.mav, self.d, packet)
        self.sent_count += 1
        self.phase = 'MOVING'
        self.reason = 'target_sent'
        self.progress = RepositionProgress(packet, self.d, sent_host_s=self.host_now_ns()/1e9,
                                            sent_px4_us=sent_px4_us)
        self.leg_started_sim_us = self.observer.clock.now_ns()//1000
        self.emit('target_command_sent', packet=packet)

    def land(self, reason):
        nan = float('nan')
        self.land_sent_host_ns = self.host_now_ns()
        self.mav.command_long_send(1, 1, self.d.MAV_CMD_NAV_LAND, 0,
                                    0., 0., nan, nan, nan, nan, nan)
        self.emit('land_requested', reason=reason, landing_verified=False)

    def abort(self, reason):
        if self.phase in ('ABORTED', 'LANDED', 'COMPLETED'):
            return
        previous, self.phase, self.reason = self.phase, 'ABORTED', reason
        self.emit('navigation_aborted', reason=reason, previous_phase=previous)
        if self.mode != 'send' or not self.attempted_count or previous == 'LANDING':
            return
        try:
            # For a pure UWB outage with valid position, ask PX4 to compute its
            # braking point. Other failures use PX4's landing/fallback logic.
            recent_pose = self.monitor.evaluate(self.host_now_ns()/1e9)
            heartbeat = self.messages.get('HEARTBEAT', (None,))[0]
            fresh_hold = (self.fresh('HEARTBEAT', 3.)
                          and ((heartbeat.custom_mode >> 16) & 255) == 4
                          and ((heartbeat.custom_mode >> 24) & 255) == 3)
            xy_good = (not self.monitor.fault and self.last_good_state is not None
                       and fresh_hold and recent_pose['pose_age_s'] is not None
                       and 0 <= recent_pose['pose_age_s'] <= .2 and recent_pose['speed_m_s'] is not None)
            if xy_good and reason in ('stale_or_future_navigation_input', 'position_or_observation_missing',
                                       'user_stopped', 'duration_complete', 'leg_timeout'):
                nan = float('nan')
                self.mav.command_int_send(1, 1, self.d.MAV_FRAME_GLOBAL,
                    self.d.MAV_CMD_DO_REPOSITION, 0, 0, -1., 0., nan, nan,
                    2147483647, 2147483647, nan)
                self.emit('pause_requested', stop_verified=False)
            else:
                self.land(reason)
        except Exception as exc:
            self.emit('abort_command_error', reason=type(exc).__name__+':'+str(exc))

    def tick(self):
        now_ns = self.host_now_ns()
        if self.last_tick_ns is not None and now_ns-self.last_tick_ns < 20_000_000:
            return
        self.last_tick_ns = now_ns
        if not self.fresh('HEARTBEAT', 3.):
            if self.sent_count:
                self.abort('heartbeat_lost')
            return
        self.request_state()
        self.update_pose()
        assessment = self.monitor.evaluate(now_ns/1e9)
        self.emit('mission_assessment', assessment=assessment)
        if self.phase == 'LANDING':
            if (self.fresh('EXTENDED_SYS_STATE', 1.5)
                    and self.messages['EXTENDED_SYS_STATE'][0].landed_state == self.d.MAV_LANDED_STATE_ON_GROUND
                    and not self.messages['HEARTBEAT'][0].base_mode & self.d.MAV_MODE_FLAG_SAFETY_ARMED):
                self.phase = 'LANDED'
                self.reason = 'landed_and_disarmed'
                self.emit('landed_and_disarmed')
            return
        if self.phase in ('ABORTED', 'COMPLETED', 'LANDED') or self.mode != 'send':
            return
        try:
            state = self.readiness(assessment)
        except ValueError as exc:
            self.warmup = None
            self.reason = str(exc)
            self.emit('navigation_waiting', reason=self.reason)
            if self.sent_count:
                self.abort(self.reason)
            return
        self.last_good_state = state
        if not self.attempted_count and not self.warmed_up():
            self.reason = 'waiting_continuous_fresh_inputs'
            return
        if self.progress is None:
            try:
                self.start_leg(state)
            except Exception as exc:
                self.abort('target_command_error:'+str(exc))
            return
        target = self.messages.get('POSITION_TARGET_GLOBAL_INT')
        if target:
            try:
                age = self.clock.sample_age_s(target[0].time_boot_ms*1000, target[1])
            except ValueError:
                pass
            else:
                self.progress.handle(target[0], received_host_s=now_ns/1e9, sample_age_s=age)
        progress = self.progress.evaluate(now_ns/1e9, assessment)
        self.emit('target_progress', progress=progress)
        if progress['reason']:
            self.abort(progress['reason'])
        elif progress['arrival_valid']:
            self.completed_legs += 1
            self.emit('leg_arrived', target=self.targets[self.leg_index])
            self.leg_index += 1
            self.progress = None
            if self.leg_index == len(self.targets):
                self.phase = 'LANDING' if self.plan.land_on_completion else 'COMPLETED'
                if self.plan.land_on_completion:
                    self.land('route_completed')
                self.emit('route_completed', landing_verified=False)
        elif (self.observer.clock.now_ns()//1000-self.leg_started_sim_us)/1e6 > self.plan.max_leg_sim_s:
            self.abort('leg_timeout')

    def stop(self, reason):
        if self.phase not in ('LANDED', 'COMPLETED', 'ABORTED', 'WAITING'):
            self.abort(reason)

    def summary(self):
        return dict(phase=self.phase, reason=self.reason, commands_sent=self.sent_count,
                    command_attempts=self.attempted_count,
                    completed_legs=self.completed_legs, resolved_targets=self.targets,
                    route_completed=bool(self.targets) and self.completed_legs == len(self.targets),
                    landing_verified=self.phase == 'LANDED', flight_valid=False, fusion_verified=False)

    def close(self):
        pass


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__,
        epilog='Remaining arguments are passed to drone_uwb.integration.gazebo.gazebo_live_shadow.')
    parser.add_argument('--navigation-settings', type=Path, required=True)
    parser.add_argument('--navigation-plan', type=Path, required=True)
    parser.add_argument('--mission-config', type=Path, required=True)
    parser.add_argument('--navigation-mode', choices=('monitor', 'send'), default='monitor')
    opts, remaining = parser.parse_known_args(args)
    target_settings = SITLTargetSettings(**json.loads(opts.navigation_settings.read_text(encoding='utf-8')))
    plan = NavigationPlan(**json.loads(opts.navigation_plan.read_text(encoding='utf-8')))
    mission = MissionConfig(**json.loads(opts.mission_config.read_text(encoding='utf-8')))

    def factory(observer, output):
        for source, name in ((opts.navigation_settings, 'navigation_settings.json'),
                             (opts.navigation_plan, 'navigation_plan.json'),
                             (opts.mission_config, 'mission_config.json')):
            (output/name).write_text(source.read_text(encoding='utf-8'), encoding='utf-8')
        source_root = Path(__file__).resolve().parent
        manifest = dict(mode=opts.navigation_mode, sources_sha256={
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(source_root.glob('*.py'))}, flight_valid=False)
        (output/'navigation_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
        stream = (output/'navigation_events.jsonl').open('x', encoding='utf-8')
        def write(event):
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
            stream.flush()
        try:
            session = SITLNavigationSession(observer, target_settings, mission, plan,
                                            mode=opts.navigation_mode, write_event=write)
        except Exception:
            stream.close()
            raise
        def close():
            stream.close()
            (output/'navigation_summary.json').write_text(json.dumps(session.summary(),
                ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        session.close = close
        return session

    from drone_uwb.integration.gazebo.gazebo_live_shadow import main as record
    record(remaining, session_factory=factory)


if __name__ == '__main__':
    main()
