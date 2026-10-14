"""Exercise the compiled library through the actual RAW/clock/height adapter."""
import argparse
import importlib.util
from pathlib import Path
import sys
import unittest

p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--package-root',required=True)
args,remaining=p.parse_known_args();sys.argv=[sys.argv[0]]+remaining
root=Path(args.package_root).resolve();sys.path.insert(0,str(root))
from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.processing.native_btf import NativeBtf
spec=importlib.util.spec_from_file_location('baseline',root/'test/processing/test_measured_btf.py')
baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)

def native(config):
    return MeasuredBtf(dict(config,numerical_backend='cpp',native_btf_library=str(Path(args.library).resolve())))

class Adapter(unittest.TestCase):
    def test_existing_input_contracts_with_native_backend(self):
        original=baseline.MeasuredBtf
        baseline.MeasuredBtf=native
        try:
            for name in ('test_sequential_real_ranges_keep_sample_times_and_compute_moving_position',
                         'test_repeated_cycle_and_clock_restart_cannot_repeat_last_good_position',
                         'test_missing_anchor_does_not_return_a_position',
                         'test_ascent_b4_xy_never_invents_z_and_step_can_refill_geometry',
                         'test_explicit_ground_session_resets_geometry_not_clock_or_source_identity'):
                with self.subTest(name=name):getattr(baseline,name)()
        finally:baseline.MeasuredBtf=original

    def test_raw_pipeline_parity_with_rejection_and_reset(self):
        import numpy as np
        c=baseline.config();a=np.asarray(c['anchors_xyz_m']);py=MeasuredBtf(c);cpp=native(c)
        inputs=[baseline.event(baseline.status(),100000)]
        for k in range(1,160):
            end=100000+k*22000;msg=baseline.cycle(k,end,a)
            if 70<=k<78:msg['raw_slant_m'][2]+=.8
            if k==110:msg['valid_mask']=7
            inputs.append(baseline.event(msg,end))
        accepted=0;rejected=0
        for event in inputs:
            l,r=py.process(event),cpp.process(event)
            self.assertEqual((l['ok'],l['reason']),(r['ok'],r['reason']))
            if l['ok']:
                accepted+=1
                self.assertEqual(l['stamp_ns'],r['stamp_ns'])
                np.testing.assert_allclose(l['xy_m'],r['xy_m'],rtol=0,atol=1e-9)
            else:rejected+=1;self.assertFalse(r.get('xy_m'))
        self.assertGreater(accepted,50);self.assertGreater(rejected,10)

    def test_selection_fails_closed_and_instance_ownership(self):
        with self.assertRaises(ValueError):MeasuredBtf(dict(baseline.config(),numerical_backend='invalid'))
        with self.assertRaises(ValueError):MeasuredBtf(dict(baseline.config(),numerical_backend='cpp',native_btf_library='missing.so'))
        x=native(baseline.config());y=native(baseline.config())
        self.assertEqual(x.native.identity['backend'],'cpp')
        x.native.close();x.native.close()
        with self.assertRaises(ValueError):x.native.reset()
        y.native.reset();y.native.close()

    def test_reset_keeps_height_conflict_until_explicit_ground_reset(self):
        import numpy as np
        from drone_uwb.acquisition.validation import Cycle
        cfg=dict(baseline.config(),enforce_b4_height_integrity=True)
        proc=native(cfg).native
        anchors=np.asarray(cfg['anchors_xyz_m'])
        def step(k,height):
            end=100000+k*22000;msg=baseline.cycle(k,end,anchors)
            c=Cycle(k,msg['cycle_start_us'],max(msg['sample_time_us']),15,
                    np.array(msg['raw_slant_m']),msg['sample_time_us'],['ok']*4,list(range(4)))
            return proc.process(c,[height]*4,'measured_tof_imu' if height is not None else None)[1]
        for k in range(1,25):result=step(k,5.)
        self.assertTrue(result['b4_height_blocked'])
        proc.reset()
        for k in range(25,40):result=step(k,None)
        self.assertTrue(result['b4_height_blocked']);self.assertFalse(result['ok'])
        proc.reset(preserve_height_conflict=False)
        for k in range(40,55):result=step(k,None)
        self.assertFalse(result['b4_height_blocked']);self.assertTrue(result['ok'])

if __name__=='__main__':unittest.main()
