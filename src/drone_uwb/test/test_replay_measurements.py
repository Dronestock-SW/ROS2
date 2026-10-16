"""Adversarial identity and timing checks for the external replay observer."""
import importlib.util
import json
from pathlib import Path
import unittest

_path = Path(__file__).parents[1]/'tools'/'btf_replay_measurements.py'
_spec = importlib.util.spec_from_file_location('btf_replay_measurements', _path)
metrics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(metrics)


def raw(ordinal=1, seq=7, before=1_000_000):
    return dict(ordinal=ordinal, topic='/uwb/received', type='uwb_raw_cycle',
                seq=seq, cycle_end_us=99, target_ns=before-100,
                before_ns=before, after_ns=before+200, publish_returned=True)


def decision(index=0, seq=7, stamp=123, published=True):
    return dict(kind='decision', receive_ns=4_000_000+index,
        text=json.dumps(dict(seq=seq, source_cycle_end_us=99, stamp_ns=stamp,
            xy_m=[1.25, 2.5], reason='ok', processing_ms=1., published=published,
            tdma=dict(boot_id=8,session=2,sf_seq=100,seq=seq))))


def pose(receive=3_000_000, stamp=123):
    return dict(kind='pose', receive_ns=receive, stamp_ns=stamp, xy_m=[1.25,2.5])


def summarize(inputs, observed):
    return metrics.summarize(inputs, observed, start_ns=0, feed_end_ns=5_000_000,
                             observation_end_ns=7_000_000)


class ReplayMeasurementsTest(unittest.TestCase):
    def test_source_age_uses_ros_clock_and_is_separate_from_latency(self):
        row = dict(raw(),before_ros_ns=1_500_000_000,source_host_received_ros_ns=1_000_000_000,
                   ros_shift_ns=350_000_000)
        result, _ = summarize([row],[decision(),pose()])
        self.assertEqual(result['source_age_at_raw_publish_ms']['p50'],150.)
        self.assertEqual(result['raw_to_pose_ms']['p50'],2.)

    def test_pose_before_decision_arrival_and_exact_milliseconds(self):
        result, links = summarize([raw()], [pose(), decision()])
        self.assertEqual(result['pose_mapping'], {'matched':1})
        self.assertEqual(result['raw_to_pose_ms']['p50'], 2.)
        self.assertEqual(result['matched_raw_publish_call_ms']['max'], .0002)
        self.assertEqual(result['published_decisions_without_observed_pose'], 0)
        self.assertEqual(links[-1]['raw_ordinal'], 1)

    def test_epoch_endpoint_and_pair_wait_are_separate(self):
        epoch = dict(raw(ordinal=2, before=2_000_000), type='uwb_tdma_epoch',
                     boot_id=8,session=2,sf_seq=100)
        result, _ = summarize([raw(),epoch],[decision(),pose()])
        self.assertEqual(result['raw_to_pose_ms']['p50'],2.)
        self.assertEqual(result['epoch_to_pose_ms']['p50'],1.)
        self.assertEqual(result['raw_to_epoch_publish_ms']['p50'],1.)

    def test_duplicate_raw_is_not_resolved_by_arrival_order(self):
        result, _ = summarize([raw(),raw(ordinal=2,before=2_000_000)], [decision(),pose()])
        self.assertEqual(result['pose_mapping'], {'ambiguous_raw':1})
        self.assertEqual(result['raw_to_pose_ms']['count'],0)

    def test_duplicate_decision_is_ambiguous(self):
        result, _ = summarize([raw()], [decision(),decision(index=1),pose()])
        self.assertEqual(result['pose_mapping'], {'ambiguous_decision':1})

    def test_duplicate_pose_does_not_choose_fastest_copy(self):
        result, _ = summarize([raw()], [decision(),pose(),pose(receive=3_100_000)])
        self.assertEqual(result['pose_mapping'], {'multiple_poses_for_decision':2})
        self.assertEqual(result['raw_to_pose_ms']['count'],0)

    def test_position_disagreement_is_not_a_match(self):
        wrong = pose()
        wrong['xy_m'][0] += .1
        result, _ = summarize([raw()], [decision(),wrong])
        self.assertEqual(result['pose_mapping'], {'no_matching_decision':1})
        self.assertEqual(result['published_decisions_without_observed_pose'],1)

    def test_keyless_rejection_is_counted_without_inventing_identity(self):
        rejected = dict(kind='decision',receive_ns=2_000_000,
            text=json.dumps(dict(reason='receiver_queue_expired',published=False)))
        result, _ = summarize([raw()], [rejected])
        self.assertEqual(result['decision_raw_mapping'], {'missing':1})
        self.assertEqual(result['reasons'], {'receiver_queue_expired':1})
        self.assertEqual(result['raw_without_identified_decision'],1)
        self.assertEqual(result['raw_to_pose_ms']['count'],0)

    def test_missing_pose_is_retained(self):
        result, _ = summarize([raw()], [decision()])
        self.assertEqual(result['published_decisions_without_observed_pose'],1)
        self.assertEqual(result['gaps_during_feed']['max_including_edges_ms'],5.)

    def test_negative_latency_is_flagged_and_not_averaged(self):
        result, _ = summarize([raw(before=4_000_000)], [decision(),pose()])
        self.assertEqual(result['pose_mapping'], {'negative_latency':1})
        self.assertEqual(result['raw_to_pose_ms']['count'],0)

    def test_late_observation_is_preserved_but_outside_fixed_window(self):
        result, _ = summarize([raw()], [decision(),pose(receive=8_000_000)])
        self.assertEqual(result['pose_mapping'], {'outside_observation_window':1})
        self.assertEqual(result['raw_to_pose_ms']['count'],0)

    def test_edge_gaps_and_drain_are_not_hidden(self):
        result, _ = summarize([raw()], [decision(),pose()])
        self.assertEqual(result['gaps_during_feed']['first_output_after_start_ms'],3.)
        self.assertEqual(result['gaps_during_feed']['last_output_before_end_ms'],2.)
        self.assertEqual(result['gaps_with_drain']['last_output_before_end_ms'],4.)

    def test_failed_publish_does_not_become_source(self):
        failed = dict(raw(),publish_returned=False)
        result, _ = summarize([failed], [decision(),pose()])
        self.assertEqual(result['publish_returned_count'],0)
        self.assertEqual(result['pose_mapping'], {'no_matching_raw':1})

    def test_malformed_json_is_reported(self):
        result, _ = summarize([], [dict(kind='decision', receive_ns=1,text='{bad')])
        self.assertEqual(len(result['malformed_observed_records']),1)

    def test_reversed_window_is_rejected(self):
        with self.assertRaises(ValueError):
            metrics.gap_summary([],2,1)


if __name__ == '__main__':
    unittest.main()
