"""Known geometry and missing-data checks for offline diagnostics."""
import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest


def load(name):
    path=Path(__file__).parents[2]/'tools'/f'{name}.py'
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_motion_does_not_turn_missing_endpoints_into_zero_drift():
    tool=load('audit_hover_observations')
    assert tool.motion([[0,0,0],[1,1,1]],0,20)['endpoint_coverage'] is False
    result=tool.motion([[0,0,0],[1,float('nan'),1],[19,.3,.4]],0,20)
    assert result['endpoint_norm_m']==pytest.approx(.5)
    assert result['nonfinite_rows']==1


def test_registration_recovers_known_rigid_motion_with_unmatched_points():
    tool=load('compare_hover_scans')
    rng=np.random.default_rng(29)
    reference=np.column_stack([rng.uniform(-3,4,180),rng.uniform(-2,3,180)])
    theta=np.deg2rad(6);rot=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    shift=np.array([.18,-.12])
    current=(reference-shift)@rot+rng.normal(0,.001,reference.shape)
    current=np.vstack([current,rng.uniform(6,8,(12,2))])
    result=tool.register(current,reference)
    np.testing.assert_allclose(result['translation_m'],shift,atol=.005)
    assert result['yaw_deg']==pytest.approx(6,abs=.05)
    assert result['overlap_forward']<1


def test_registration_rejects_too_few_points():
    with pytest.raises(ValueError,match='Insufficient'):
        load('compare_hover_scans').register(np.zeros((4,2)),np.zeros((4,2)))


def test_audit_keeps_absent_rc_missing_and_checks_config_provenance(tmp_path):
    tool=load('audit_hover_observations')
    config=json.dumps({'anchors_xyz_m':[[0,0,0],[6.3,0,0],[0,4.6,0],[6.3,4.6,0]]})
    evidence=dict(name='field-btf.json',content_utf8=config,
                  sha256=hashlib.sha256(config.encode()).hexdigest())
    manifest=dict(boot_id='test',source_revision='test',config_evidence=[evidence])
    (tmp_path/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
    event=dict(received_monotonic_ns=1_000_000_000,topic='/mavros/rc/in',data={'channels':[]})
    (tmp_path/'events.jsonl').write_text(json.dumps(event)+'\n',encoding='utf-8')
    report=tool.audit(tmp_path,0,20,tmp_path/'result')
    assert report['incomplete_rc_samples']==1
    assert report['rc_first_four']==[]
    evidence['sha256']='0'*64
    (tmp_path/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
    with pytest.raises(ValueError,match='configuration hash'):
        tool.audit(tmp_path,0,20,tmp_path/'untrusted')
