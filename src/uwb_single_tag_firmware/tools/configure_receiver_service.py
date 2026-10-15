"""Create a versioned schema-2 launch and a reversible service drop-in."""
import hashlib
import json
from pathlib import Path
import subprocess
import argparse
import datetime


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--slot',default='candidate-v1')
    parser.add_argument('--replace',action='store_true')
    args=parser.parse_args()
    if not args.slot.replace('-','').isalnum():raise RuntimeError('Invalid slot name')
    home=Path.home()
    root=home/'esp32-single-tag-20261015'
    slot=root/args.slot
    baseline=home/'.config/dronestock/logging-off-20261015'
    source_launch=baseline/'manual_observe.launch.py'
    launch=source_launch.read_text(encoding='utf-8')
    old="if measured['tag_id'] != tag or measured.get('tdma_mode') != 'required':"
    assert launch.count(old)==1
    launch=launch.replace(old,"if measured['tag_id'] != tag or measured.get('protocol') != 'single_tag_schema_2':")
    old="Node(package='drone_uwb', executable='uwb_node', output='screen',"
    assert launch.count(old)==1
    launch=launch.replace(old,old+"\n             additional_env={'PYTHONPATH':value('btf_python_path')+os.pathsep+os.environ.get('PYTHONPATH','')},")
    old="{'port':value('uwb_port'), 'anchor_file':str(anchors),"
    assert launch.count(old)==1
    launch=launch.replace(old,"{'port':value('uwb_port'), 'anchor_file':str(anchors), 'single_tag_mode':True,")
    new_launch=slot/'single_tag_observe.launch.py'
    new_launch.write_text(launch,encoding='utf-8')
    run=(baseline/'run_observe.sh').read_text(encoding='utf-8')
    assert run.count(str(source_launch))==1
    run=run.replace(str(source_launch),str(new_launch))
    (slot/'run_observe.sh').write_text(run,encoding='utf-8')
    (slot/'run_observe.sh').chmod(0o700)
    config=json.loads((home/'btf-acache-release-20261015/field-btf-cpp.json').read_text(encoding='utf-8'))
    config.update(protocol='single_tag_schema_2',tdma_mode='disabled',single_tag_layout_confirmed=False)
    config['provenance']['timing']='RF poll/final midpoint; one four-timestamp affine mapping in serial owner; uncalibrated asymmetry reported'
    (slot/'field-btf-cpp.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
    dropin=home/'.config/systemd/user/dronestock-manual-observe.service.d/zzzzzz-single-tag-20261015.conf'
    if dropin.exists():
        if not args.replace:raise RuntimeError('Service overlay already exists; inspect before replacing')
        history=root/'service-history';history.mkdir(exist_ok=True)
        (history/(datetime.datetime.now().strftime('%Y%m%d-%H%M%S-')+args.slot+'.conf')).write_bytes(dropin.read_bytes())
    dropin.write_text('[Service]\nEnvironment="BTF_NATIVE_SLOT='+str(slot)+'"\nExecStart=\nExecStart='+str(slot/'run_observe.sh')+'\n',encoding='utf-8')
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    subprocess.run(['systemctl','--user','start','dronestock-manual-observe.service'],check=True)
    print('Activated receiver candidate '+str(slot),flush=True)


if __name__=='__main__':main()
