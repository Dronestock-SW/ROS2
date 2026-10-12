"""Instantaneous virtual scans and a scan-only motion estimator for SIH tests.

Truth is used ONLY by VirtualLidar to form rays, like a sensor simulator.
PlanarScanOdometry never receives position/velocity truth or UWB coordinates.
It receives ranges and the FC attitude, projects mild tilt into a horizontal
plane, then runs the same robust keyframe matcher used by RAW replay.
The covariance is an explicit SIMULATION model, not physical calibration.
This is local scan odometry, with no map optimization or loop closure.
"""
import math
from collections import deque

import numpy as np

from .lidar_scanmatch import Cloud,Config,KeyframeMatcher,ordered_cloud,rotation


ENU_FROM_NED=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
FRD_FROM_FLU=np.diag([1.,-1.,-1.])


def enu_flu_rotation(ned_frd_rotation):
    r=np.asarray(ned_frd_rotation,dtype=float)
    if (r.shape!=(3,3) or not np.isfinite(r).all()
            or not np.allclose(r.T@r,np.eye(3),atol=1e-4) or np.linalg.det(r)<.999):
        raise ValueError('valid_attitude_rotation_required')
    return ENU_FROM_NED@r@FRD_FROM_FLU


class VirtualLidar:
    """A ray-cast box; ideal instantaneous acquisition, with noisy ranges.

    Laser and IMU origins coincide in this explicit fixture. The real vehicle's
    lever arm and spinning-beam acquisition are not established by this test.
    """
    def __init__(self,seed,*,beams=430,range_noise_m=.01,scan_bias_noise_m=.003):
        self.rng=np.random.default_rng(seed)
        self.range_noise_m=range_noise_m;self.scan_bias_noise_m=scan_bias_noise_m
        self.angles=-math.pi+np.arange(beams)*2*math.pi/beams
        self.directions=np.column_stack((np.cos(self.angles),np.sin(self.angles),np.zeros(beams)))

    def observe(self,*,position_enu_m,rotation_enu_flu,wall_x_m=5.):
        pos=np.asarray(position_enu_m,dtype=float)
        directions=self.directions@np.asarray(rotation_enu_flu).T
        far=np.array([wall_x_m,5.,3.])
        distances=np.full_like(directions,np.inf)
        np.divide(far-pos,directions,out=distances,where=directions>1e-10)
        negative=np.full_like(directions,np.inf)
        np.divide(-pos,directions,out=negative,where=directions<-1e-10)
        ranges=np.minimum(distances,negative).min(axis=1)
        ranges+=self.rng.normal(0,self.range_noise_m,len(ranges))+self.rng.normal(0,self.scan_bias_noise_m)
        return ranges


class PlanarScanOdometry:
    """SITL-only range-to-motion adapter. No position or velocity truth input."""
    def __init__(self,*,simulation_covariance_model,velocity_stddev_m_s=.03,
                 heading_stddev_rad=.15,range_max_m=4.,config=Config(),
                 velocity_intervals=1,simulation_position_quantum_m=0.):
        if simulation_covariance_model is not True:
            raise ValueError('physical_covariance_not_calibrated')
        if (not np.isfinite([velocity_stddev_m_s,heading_stddev_rad,range_max_m]).all()
                or velocity_stddev_m_s<=0 or heading_stddev_rad<0 or range_max_m<=config.range_min_m):
            raise ValueError('finite_simulation_uncertainty_required')
        self.config=config;self.matcher=KeyframeMatcher(config,require_imu=False)
        self.velocity_stddev=velocity_stddev_m_s;self.heading_stddev=heading_stddev_rad
        self.range_max=range_max_m;self.previous_pose=None;self.heading=None
        self.segment=None
        if velocity_intervals not in (1,2) or not 0<=simulation_position_quantum_m<=.02:
            raise ValueError('bounded_simulation_velocity_interval_required')
        self.history=deque(maxlen=velocity_intervals+1)
        self.position_quantum=simulation_position_quantum_m

    def observe(self,*,ranges,stamp_s,rotation_enu_flu,height_above_floor_m):
        r=np.asarray(rotation_enu_flu,dtype=float)
        ranges=np.asarray(ranges,dtype=float)
        invalid=None
        if (r.shape!=(3,3) or not np.isfinite(r).all() or
                not np.allclose(r.T@r,np.eye(3),atol=1e-4) or np.linalg.det(r)<.999):
            raise ValueError('invalid_scan_attitude')
        yaw=math.atan2(r[1,0],r[0,0])
        if r[2,2]<math.cos(math.radians(10.)):invalid='scan_tilt_limit'
        if not .75<=height_above_floor_m<=2.25:invalid='floor_ceiling_clearance_unavailable'
        if invalid:
            self.matcher.observe(Cloud(np.empty((0,2)),np.empty((0,2))),stamp_s)
            self.previous_pose=None
            self.history.clear()
            return dict(accepted=False,reasons=[invalid],stamp_s=stamp_s,
                        simulation_only=True,reset_counter=max(0,self.matcher.segment)%256)
        angles=-math.pi+np.arange(len(ranges))*2*math.pi/len(ranges)
        valid=np.isfinite(ranges)&(ranges>=self.config.range_min_m)&(ranges<=self.range_max)
        distance=np.where(valid,ranges,0.)
        body=np.column_stack((distance*np.cos(angles),distance*np.sin(angles),np.zeros(len(ranges))))
        # De-yaw keeps the scan frame's horizontal heading. Relative yaw is
        # estimated from geometry, not fed back as an independent IMU yaw.
        level=body@r.T
        level[:,:2]=level[:,:2]@rotation(-yaw).T
        cloud=ordered_cloud(level[:,:2],valid,self.config)
        return self.observe_cloud(cloud, stamp_s, yaw)

    def observe_cloud(self, cloud, stamp_s, yaw):
        """Shared geometry-to-velocity path after declared scan preprocessing."""
        obs=self.matcher.observe(cloud,stamp_s)
        record=dict(accepted=obs.accepted,reasons=obs.reasons,stamp_s=stamp_s,
            segment=obs.segment,keyframe=obs.keyframe,reset_counter=max(0,obs.segment)%256,
            interval_s=obs.interval_s,sample_midpoint_s=obs.sample_midpoint_s,
            pose_odom_m_rad=obs.pose.tolist() if obs.pose is not None else None,
            simulation_only=True,physical_aiding_ready=False)
        if obs.match is not None:
            record['match']=dict(overlap=obs.match.overlap,residual_p90_m=obs.match.residual_p90_m,
                                condition=obs.match.condition,correspondences=obs.match.correspondences,
                                spatial_cells=obs.match.spatial_cells,
                                relative_range_bias_m=obs.match.range_bias_m,
                                translation_information_ratio=obs.match.translation_information_ratio)
            record['match']={k:(v if np.isfinite(v) else None) for k,v in record['match'].items()}
        if obs.segment!=self.segment:
            self.segment=obs.segment;self.heading=yaw;self.previous_pose=None;self.history.clear()
        if obs.pose is None:self.history.clear()
        else:self.history.append((stamp_s,obs.pose.copy()))
        if obs.accepted and len(self.history)==self.history.maxlen:
            start,previous_pose=self.history[0]
            dt=stamp_s-start
            velocity_enu=rotation(self.heading)@(obs.pose[:2]-previous_pose[:2])/dt
            yaw_jac=np.array([-velocity_enu[1],velocity_enu[0]])
            # HIL global coordinates quantize the ray-cast sensor origin.
            # Two bounded uniform endpoint errors contribute q^2/(6 dt^2).
            # This simulation transport term is separate from scan/heading noise.
            variance=self.velocity_stddev**2+self.position_quantum**2/(6*dt**2)
            c=np.eye(2)*variance+self.heading_stddev**2*np.outer(yaw_jac,yaw_jac)
            record.update(velocity_ne_m_s=velocity_enu[::-1].tolist(),
                          covariance_ne_m2_s2=c[::-1,::-1].tolist(),
                          displacement_odom_xy_m=(obs.pose[:2]-previous_pose[:2]).tolist(),
                          interval_s=dt,sample_midpoint_s=(start+stamp_s)/2,
                          source_lineage=['virtual_ranges','FC_attitude_for_tilt_and_initial_heading'])
        else:
            record['accepted']=False
            if not record['reasons']:record['reasons']=['interval_history_warmup']
        self.previous_pose=None if obs.pose is None else obs.pose.copy()
        return record
