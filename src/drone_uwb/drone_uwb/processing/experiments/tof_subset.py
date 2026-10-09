"""Experimental causal ToF/H80 integrity candidate. No simulator truth or FC state input."""
from collections import deque
from dataclasses import asdict
from types import SimpleNamespace
import numpy as np
from drone_uwb.processing.h80 import fit_h80
from drone_uwb.processing.experiments.h80_b import BSettings

class ToFSubsetCandidate:
    def __init__(self,config,*,rms_limit_m=.06,score_margin_m=.015):
        if any(type(v) not in (int,float) or not np.isfinite(v) or v <= 0 for v in (rms_limit_m,score_margin_m)):
            raise ValueError('invalid_subset_limits')
        self.anchors=np.asarray(config['anchors_xyz_m'],float)
        self.settings=SimpleNamespace(**asdict(BSettings(**config['B'])))
        self.settings.required_anchor_count=3
        self.limit=rms_limit_m;self.margin=score_margin_m;self.history=deque();self.last_stamp=None

    def process(self,result):
        stamp=result['time_us'];b=result['models']['B'];height=result.get('height_m')
        ranges=np.asarray(result['cal_slant_m'],float)
        if ranges.shape != (4,) or not np.isfinite(ranges).all() or np.any(ranges<=0) or np.any(ranges>80):
            return dict(ok=False,xy_m=None,source='rejected',reason='four_valid_ranges_required',hypotheses=[],truth_used=False)
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>round(self.settings.reset_gap_s*1e6)):self.history.clear()
        self.last_stamp=stamp
        sample_times = result.get('sample_time_us', [stamp]*4)
        if (len(sample_times) != 4 or any(type(t) is not int or t > stamp for t in sample_times)):
            return dict(ok=False,xy_m=None,source='rejected',reason='invalid_anchor_times',hypotheses=[],truth_used=False)
        self.history.append({'time_us':stamp,'ranges':list(result['cal_slant_m']),'height':height,
                             'sample_time_us':sample_times,
                             'sample_height_m':result.get('sample_height_m',[height]*4),
                             'asynchronous':'sample_time_us' in result})
        lower=stamp-round(self.settings.window_s*1e6)
        while self.history and self.history[0]['time_us']<lower:self.history.popleft()
        valid=b.get('ok') is True and b.get('fresh_observation_count',0)>0 and b.get('newest_sample_us')==stamp
        output={'ok':False,'xy_m':None,'source':'rejected','reason':'B_not_fresh_valid','hypotheses':[],'truth_used':False}
        if not valid:return output
        rms=(b.get('fit') or {}).get('rms_m')
        if type(rms) not in [int,float] or not np.isfinite(rms):return dict(output,reason='missing_fit_integrity')
        if 0<=rms<self.limit:return dict(output,ok=True,xy_m=list(b['xy_m']),source='unchanged_B4',reason='original_fit_consistent')
        if type(height) not in [int,float] or not np.isfinite(height) or result.get('height_source') not in ('gazebo_tof_imu','measured_tof_imu','ground_antenna_reference'):return dict(output,reason='measured_height_unavailable')
        hist=list(self.history);t=np.array([(row['time_us']-stamp)/1e6 for row in hist]);ranges=np.array([row['ranges'] for row in hist]);z=np.array([np.nan if row['height'] is None else row['height'] for row in hist]);valid_height=np.isfinite(z)
        idx=np.tile(np.arange(4),len(hist));times=np.repeat(t,4);all_ranges=ranges.reshape(-1)
        asynchronous=any(row['asynchronous'] for row in hist)
        if asynchronous:
            # Real tags measure anchors sequentially. Never label their ranges simultaneous.
            anchor_t=np.array([[(v-stamp)/1e6 for v in row['sample_time_us']] for row in hist])
            anchor_z=np.array([[np.nan if v is None else v for v in row['sample_height_m']] for row in hist],float)
            times=anchor_t.reshape(-1)
        for excluded in range(4):
            keep=idx!=excluded
            if asynchronous:
                keep &= (times >= -self.settings.window_s) & (times <= 0)
            fit=fit_h80(self.anchors,idx[keep],times[keep],all_ranges[keep],0.,self.settings)
            row={'excluded_anchor':excluded+1,'fit_ok':fit.ok,'reason':fit.reason}
            if asynchronous and fit.ok:
                included=np.arange(4)!=excluded
                finite_z=np.isfinite(anchor_z) & (anchor_t >= -self.settings.window_s) & (anchor_t <= 0)
                if min(finite_z[:,included].sum(axis=0))>=self.settings.min_samples_per_anchor:
                    u=anchor_t/self.settings.window_s
                    xyz=np.stack([fit.x0+fit.x1*u,fit.y0+fit.y1*u,anchor_z],axis=2)
                    residual=np.linalg.norm(xyz-self.anchors[None,:,:],axis=2)-ranges
                    score=float(np.sqrt(np.mean(residual[:,included][finite_z[:,included]]**2)))
                    excluded_values=residual[:,excluded][finite_z[:,excluded]]
                    row.update(xy_m=[float(fit.x0),float(fit.y0)],physical_range_rms_m=score,
                        excluded_mean_observed_minus_predicted_m=float(-excluded_values.mean()) if len(excluded_values) else None,
                        fit_rms_m=fit.rms_m,condition=fit.condition,height_samples=int(finite_z[:,included].sum()))
            elif fit.ok and int(valid_height.sum())>=self.settings.min_samples_per_anchor:
                u=t[valid_height]/self.settings.window_s;xy=np.column_stack([fit.x0+fit.x1*u,fit.y0+fit.y1*u]);xyz=np.column_stack([xy,z[valid_height]])
                residual=np.linalg.norm(xyz[:,None,:]-self.anchors[None,:,:],axis=2)-ranges[valid_height]
                included=np.arange(4)!=excluded;score=float(np.sqrt(np.mean(residual[:,included]**2)))
                row.update(xy_m=[float(fit.x0),float(fit.y0)],physical_range_rms_m=score,excluded_mean_observed_minus_predicted_m=float(-residual[:,excluded].mean()),fit_rms_m=fit.rms_m,condition=fit.condition,height_samples=int(valid_height.sum()))
            output['hypotheses'].append(row)
        eligible=sorted([row for row in output['hypotheses'] if 'physical_range_rms_m' in row],key=lambda row:row['physical_range_rms_m'])
        if len(eligible)<2:return dict(output,reason='insufficient_subset_hypotheses')
        best,second=eligible[:2];margin=second['physical_range_rms_m']-best['physical_range_rms_m'];output.update(best_excluded_anchor=best['excluded_anchor'],best_score_m=best['physical_range_rms_m'],score_margin_m=margin)
        if best['physical_range_rms_m']>=self.limit:return dict(output,reason='no_height_consistent_subset')
        if margin<self.margin:return dict(output,reason='ambiguous_subset')
        source = 'ground_reference_B3' if result.get('height_source') == 'ground_antenna_reference' else 'ToF_validated_B3'
        return dict(output,ok=True,xy_m=best['xy_m'],source=source,reason='unique_height_consistent_subset')


class ToFTrackedSubsetCandidate(ToFSubsetCandidate):
    def __init__(self,config,*,rms_limit_m=.06,score_margin_m=.015,max_speed_m_s=1.,continuity_margin_m=.1,max_prior_age_s=.1):
        if any(type(v) not in (int,float) or not np.isfinite(v) or v <= 0 for v in (max_speed_m_s,continuity_margin_m,max_prior_age_s)):
            raise ValueError('invalid_subset_continuity_limits')
        super().__init__(config,rms_limit_m=rms_limit_m,score_margin_m=score_margin_m)
        self.max_speed=max_speed_m_s;self.continuity_margin=continuity_margin_m;self.max_prior_age=max_prior_age_s
        self.selected_anchor=None;self.prior_xy=None;self.prior_stamp=None
        self.ground_initializer = GroundSubsetInitialization() if config.get('ground_subset_initialization') is True else None

    def process(self,result):
        stamp=result['time_us']
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>round(self.settings.reset_gap_s*1e6)):
            self.selected_anchor=None;self.prior_xy=None;self.prior_stamp=None
        decision=super().process(result)
        prior_age=None if self.prior_stamp is None else (stamp-self.prior_stamp)/1e6
        prior_fresh=prior_age is not None and 0<prior_age<=self.max_prior_age
        initial = False
        if self.ground_initializer is not None:
            near_prior = (self.prior_xy is None or decision.get('xy_m') is not None
                and np.linalg.norm(np.asarray(decision['xy_m'])-self.prior_xy)<=.1)
            initial, evidence = self.ground_initializer.check(stamp, decision,
                ground=result.get('height_source')=='ground_antenna_reference' and near_prior,
                already_initialized=prior_fresh)
            decision['ground_initialization'] = evidence
        decision['tracking_prior_age_s']=prior_age
        if self.selected_anchor is not None and decision['reason']=='ambiguous_subset':
            hypotheses=[h for h in decision['hypotheses'] if h.get('excluded_anchor')==self.selected_anchor and 'physical_range_rms_m' in h]
            if hypotheses and prior_fresh:
                h=hypotheses[0];step=float(np.linalg.norm(np.asarray(h['xy_m'])-self.prior_xy));limit=self.continuity_margin+self.max_speed*prior_age
                decision.update(tracked_excluded_anchor=self.selected_anchor,tracking_step_m=step,tracking_limit_m=limit)
                if h['physical_range_rms_m']<self.limit and step<=limit:
                    source = 'ground_reference_tracked_B3' if result.get('height_source') == 'ground_antenna_reference' else 'ToF_tracked_B3'
                    decision.update(ok=True,xy_m=h['xy_m'],source=source,reason='previously_unique_subset_still_consistent',best_excluded_anchor=self.selected_anchor)
        if decision['ok']:
            if decision['source'] in ['ToF_validated_B3','ToF_tracked_B3','ground_reference_B3','ground_reference_tracked_B3']:
                if not prior_fresh and not initial:
                    decision.update(ok=False,xy_m=None,source='rejected',reason='no_fresh_continuity_prior')
                elif prior_fresh:
                    step=float(np.linalg.norm(np.asarray(decision['xy_m'])-self.prior_xy));limit=self.continuity_margin+self.max_speed*prior_age
                    decision.update(tracking_step_m=step,tracking_limit_m=limit)
                    if step>limit:decision.update(ok=False,xy_m=None,source='rejected',reason='subset_position_discontinuous')
            if decision['ok']:
                self.selected_anchor=None if decision['source']=='unchanged_B4' else decision['best_excluded_anchor']
                self.prior_xy=np.asarray(decision['xy_m'],float);self.prior_stamp=stamp
        if self.prior_stamp is not None and (stamp-self.prior_stamp)/1e6>self.max_prior_age:self.selected_anchor=None
        return decision


class GroundSubsetInitialization:
    """Acquire a first measurement from a unique, stationary ground subset.

    This does not predict a pose or borrow FC XY. Ground eligibility is supplied
    by the height adapter. Every sample must pass the existing range integrity
    and subset uniqueness checks before entering the two-second acquisition.
    """
    def __init__(self):
        self.started = self.last = self.anchor = self.origin = None
        self.count = 0

    def clear(self):
        self.started = self.last = self.anchor = self.origin = None
        self.count = 0

    def check(self, stamp, decision, *, ground, already_initialized):
        eligible = (ground and not already_initialized and decision.get('ok') is True
            and decision.get('source') == 'ground_reference_B3'
            and decision.get('reason') == 'unique_height_consistent_subset')
        if not eligible:
            self.clear()
            return False, {'ready': False, 'reason': 'ground_unique_subset_required'}
        xy=np.asarray(decision['xy_m'],float);anchor=decision['best_excluded_anchor']
        if (xy.shape!=(2,) or not np.isfinite(xy).all() or type(stamp) is not int or stamp<=0):
            self.clear()
            return False, {'ready': False, 'reason': 'invalid_ground_candidate'}
        if (self.last is not None and (not 0<stamp-self.last<=100_000
                or anchor!=self.anchor or np.linalg.norm(xy-self.origin)>.05)):
            self.clear()
        if self.started is None:
            self.started=stamp;self.anchor=anchor;self.origin=xy.copy()
        self.last=stamp;self.count+=1
        elapsed=(stamp-self.started)/1e6
        ready=elapsed>=2. and self.count>=30
        return ready, {'ready':bool(ready),'elapsed_s':elapsed,'samples':self.count,
            'excluded_anchor':anchor,'reason':'stable_ground_subset' if ready else 'acquiring_ground_subset'}
