"""Prepare a minimal schema-2 overlay over an archived deployed package.

Preserves the deployed B_TF numerical code, A-cache and native library contract.
Does not overwrite an operating slot or change a service.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--backup',type=Path,required=True)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise SystemExit('Output must be a fresh directory')
    prefix='backup/drone_uwb-before/'
    package=args.output/'source/src/drone_uwb'
    with tarfile.open(args.backup) as archive:
        for member in archive.getmembers():
            if not member.name.startswith(prefix) or not member.isfile():
                continue
            relative=Path(member.name[len(prefix):])
            if relative.is_absolute() or '..' in relative.parts:
                raise RuntimeError('Unsafe archive member')
            target=package/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(archive.extractfile(member).read())
    source=args.repo/'src/drone_uwb/drone_uwb'
    target=package/'drone_uwb'
    changed={}
    for relative in ('acquisition/single_tag.py','acquisition/serial_io.py',
                     'contracts/protocol.py','integration/ros/node.py'):
        path=target/relative
        changed[relative]={'before':sha(path) if path.exists() else None}
        shutil.copyfile(source/relative,path)
        changed[relative]['after']=sha(path)
    path=target/'processing/measured_btf.py'
    old=path.read_text(encoding='utf-8')
    new=(source/'processing/measured_btf.py').read_text(encoding='utf-8')
    before=sha(path)
    expected='4f3814660a540b612e01751205e659fc0468a283d9ec6f09de5d1e36de17117f'
    if before!=expected:
        raise RuntimeError('Deployed B_TF baseline changed; inspect its numerical/native contract before overlaying')
    if 'self.single_input' in old or 'partial = len(cycle.indices)' in old:
        raise RuntimeError('Unexpected deployed B_TF baseline; review before patching')
    old=old.replace('from drone_uwb.acquisition.tdma import TdmaGate',
                    'from drone_uwb.acquisition.tdma import TdmaGate\nfrom drone_uwb.acquisition.single_tag import MappedSingleTagInput')
    old=old.replace('        self.clock = ClockMap(self.input_settings)',
                    '        self.clock = ClockMap(self.input_settings)\n        self.single_input = MappedSingleTagInput()')
    first=old.index('            dispatch = self.tdma.ingest(msg, mono, ros)')
    last=old.index('            out.update(seq=cycle.seq',first)
    branch=new[new.index("            single = msg.get('schema') == 2"):new.index('            else:\n                dispatch = self.tdma.ingest')]
    old=old[:first]+branch+'            else:\n'+''.join('    '+line if line.strip() else line for line in old[first:last].splitlines(True))+old[last:]
    first=old.index('            self.clock.update(cycle.end_us, mono)')
    last=old.index("            out['queue_age_s']",first)
    replacement=new[new.index('            if not single:\n'):new.index("            out['queue_age_s']")]
    old=old[:first]+replacement+old[last:]
    old=old.replace('stamp_ns = round(self.clock.host_s(stamp)*1e9)+offset','stamp_ns = host_ns(stamp)+offset')
    old=old.replace('self.height.at(round(self.clock.host_s(sample)*1e9)+offset)','self.height.at(host_ns(sample)+offset)')
    old=old.replace('clock_alpha=self.clock.alpha,',"clock_alpha=meta['clock']['alpha'] if single else self.clock.alpha,")
    old=old.replace('        except InvalidInput as exc:\n            if exc.reset:',
        "        except InvalidInput as exc:\n            if event.get('message', {}).get('schema') == 2 and 'clock' in exc.reason:\n                self.reset_models()\n            if exc.reset:")
    path.write_text(old,encoding='utf-8')
    changed['processing/measured_btf.py']={'before':before,'after':sha(path)}
    path=target/'integration/ros/btf_node.py'
    before=sha(path)
    text=path.read_text(encoding='utf-8')
    assert text.count("        if result['ok']:\n") == 1
    text=text.replace("        if result['ok']:\n", "        layout_ready = (event.get('message', {}).get('schema') != 2\n"
        "                        or self.config.get('single_tag_layout_confirmed') is True)\n"
        "        if result['ok'] and not layout_ready:\n"
        "            result['pose_blocked_reason'] = 'single_tag_layout_unconfirmed'\n"
        "        if result['ok'] and layout_ready:\n")
    marker="        self.decision_pub = self.create_publisher(String, '/uwb/btf_decision', qos_profile_sensor_data)"
    assert text.count(marker)==1
    text=text.replace(marker,marker+"\n        self.tag_feedback_pub = self.create_publisher(String, '/uwb/tag_pose_feedback', qos_profile_sensor_data)")
    marker="                result['published'] = True\n"
    assert text.count(marker)==1
    text=text.replace(marker,marker+"                if result.get('protocol') == 'single_tag_schema_2':\n"
        "                    keys = ('published','ok','source_identity','host_session','seq','time_us','xy_m')\n"
        "                    self.tag_feedback_pub.publish(String(data=json.dumps({k:result[k] for k in keys})))\n")
    path.write_text(text,encoding='utf-8')
    changed['integration/ros/btf_node.py']={'before':before,'after':sha(path)}
    manifest=dict(schema=1,source_backup_sha256=sha(args.backup),files=changed,
                  scope='single_tag_receiver_only',numerical_backend='preserved deployed A-cache/native B_TF')
    (args.output/'receiver-overlay.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(manifest))


if __name__=='__main__':
    main()
