"""Generate a separate PX4/Gazebo sensor model and surveyed-anchor world.

Reads the user's installed PX4-gazebo-models, preserving native sensor names
expected by GZBridge. Never edits upstream models or flight parameters.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET


MODEL = 'dronestock_x500'
WORLD = 'dronestock_uwb'
GZ_NAMESPACE = 'https://gazebosim.org'
ET.register_namespace('gz', GZ_NAMESPACE)


def read_xml(path):
    text = Path(path).read_text(encoding='utf-8-sig')
    # Upstream SDF permits gz:type without an XML namespace declaration.
    if 'gz:' in text and 'xmlns:gz=' not in text:
        text = text.replace('<sdf ', f'<sdf xmlns:gz="{GZ_NAMESPACE}" ', 1)
    return ET.fromstring(text)


def set_text(element, path, value):
    node = element
    for part in path.split('/'):
        child = node.find(part)
        if child is None:
            child = ET.SubElement(node, part)
        node = child
    node.text = str(value)
    return node


def pose_text(values, length=6):
    if len(values) != length or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise ValueError('finite_pose_required')
    return ' '.join(str(v) for v in values)


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    path.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True))


def box_visual(parent, name, size, xyz, color):
    visual = ET.SubElement(parent, 'visual', name=name)
    set_text(visual, 'pose', pose_text(list(xyz)+[0, 0, 0]))
    set_text(visual, 'geometry/box/size', pose_text(size, 3))
    set_text(visual, 'material/ambient', color)
    set_text(visual, 'material/diffuse', color)
    return visual


def build(px4_gz, anchors_path, equipment_path, trial_path, output):
    px4_gz, output = Path(px4_gz), Path(output)
    anchors = json.loads(Path(anchors_path).read_text(encoding='utf-8'))
    cfg = json.loads(Path(equipment_path).read_text(encoding='utf-8'))
    trial = json.loads(Path(trial_path).read_text(encoding='utf-8'))
    if (anchors['anchor_order'] != ['A1', 'A2', 'A3', 'A4'] or anchors['units'] != 'm'
            or cfg['frame'] != 'body_flu' or cfg['external_output_allowed'] is not False):
        raise ValueError('unsupported_rig_contract')
    if len(anchors['anchors_xyz_m']) != 4:
        raise ValueError('four_anchors_required')
    for xyz in anchors['anchors_xyz_m']:
        pose_text(xyz, 3)
    pose_text(cfg['tag_offset_body_flu_m'], 3)
    pose_text(cfg['fc_offset_body_flu_m'], 3)
    for key in ('tof', 'flow', 'lidar', 'camera'):
        pose_text(cfg[key+'_pose_body_flu'])
    pose_text(cfg['spawn_pose_world'])
    if cfg['real_total_mass_kg'] is not None:
        raise ValueError('measured_mass_requires_mass_and_inertia_model_review')
    for name in ('tof', 'lidar'):
        sensor = cfg[name]
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in sensor.values()) or not 0 < sensor['min_m'] < sensor['max_m']
                or sensor['rate_hz'] <= 0 or sensor['noise_sigma_m'] < 0):
            raise ValueError('invalid_sensor_config:'+name)
    for name in ('width', 'height'):
        if not isinstance(cfg['camera'][name], int) or isinstance(cfg['camera'][name], bool) or cfg['camera'][name] < 1:
            raise ValueError('positive_image_dimension_required')
    if not 0 < cfg['camera']['horizontal_fov_rad'] < math.pi or not 0 < cfg['camera']['rate_hz'] < 1000:
        raise ValueError('invalid_camera_config')
    if not isinstance(cfg['lidar']['samples'], int) or cfg['lidar']['samples'] < 2:
        raise ValueError('invalid_lidar_samples')
    sources = {}

    def native(relative):
        path = px4_gz/relative
        sources[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
        return read_xml(path)

    model_root = native('models/x500_base/model.sdf')
    model = model_root.find('model')
    model.set('name', MODEL)
    motors = native('models/x500/model.sdf').find('model')
    for child in motors:
        if child.tag != 'include':
            model.append(deepcopy(child))
    base = model.find("link[@name='base_link']")
    if base is None or base.find("sensor[@name='imu_sensor']") is None:
        raise ValueError('unsupported_native_x500_structure')
    set_text(base.find("sensor[@name='imu_sensor']"), 'pose', pose_text(cfg['fc_offset_body_flu_m']+[0, 0, 0]))
    box_visual(base, 'uwb_tag_marker', [.06, .04, .015], cfg['tag_offset_body_flu_m'], '0.1 0.8 1 1')

    def attach(relative, link_name, pose):
        link = deepcopy(native(relative).find(f"model/link[@name='{link_name}']"))
        if link is None:
            raise ValueError('missing_native_sensor_link:'+link_name)
        set_text(link, 'pose', pose_text(pose)).set('relative_to', 'base_link')
        model.append(link)
        joint = ET.SubElement(model, 'joint', name=link_name+'_mount', type='fixed')
        set_text(joint, 'parent', 'base_link')
        set_text(joint, 'child', link_name)
        return link

    down = attach('models/x500_lidar_down/model.sdf', 'lidar_sensor_link', cfg['tof_pose_body_flu'])
    tof = down.find("sensor[@name='lidar']")
    for key, path in [('min_m', 'ray/range/min'), ('max_m', 'ray/range/max'), ('rate_hz', 'update_rate')]:
        set_text(tof, path, cfg['tof'][key])
    set_text(tof, 'ray/noise/type', 'gaussian')
    set_text(tof, 'ray/noise/mean', 0)
    set_text(tof, 'ray/noise/stddev', cfg['tof']['noise_sigma_m'])
    box_visual(down, 'tof_proxy_housing', [.035, .0185, .021], [0, 0, 0], '0.2 0.2 0.2 1')
    attach('models/optical_flow/model.sdf', 'flow_link', cfg['flow_pose_body_flu'])
    lidar_link = attach('models/lidar_2d_v2/model.sdf', 'link', cfg['lidar_pose_body_flu'])
    # Keep the native bridge's names, with a small proxy below the scan plane.
    for element in list(lidar_link):
        if element.tag in ('collision', 'visual'):
            lidar_link.remove(element)
    box_visual(lidar_link, 'lidar_proxy_housing', [.06, .06, .04], [0, 0, -.04], '.15 .15 .15 1')
    lidar = lidar_link.find("sensor[@name='lidar_2d_v2']")
    set_text(lidar, 'pose', '0 0 0 0 0 0')
    for key, path in [('min_m', 'ray/range/min'), ('max_m', 'ray/range/max'), ('rate_hz', 'update_rate'), ('samples', 'ray/scan/horizontal/samples')]:
        set_text(lidar, path, cfg['lidar'][key])
    set_text(lidar, 'ray/scan/horizontal/min_angle', -math.pi)
    set_text(lidar, 'ray/scan/horizontal/max_angle', math.pi)
    set_text(lidar, 'ray/noise/stddev', cfg['lidar']['noise_sigma_m'])
    set_text(lidar, 'always_on', 'true')
    camera = attach('models/mono_cam/model.sdf', 'camera_link', cfg['camera_pose_body_flu']).find('sensor')
    for key, path in [('width', 'camera/image/width'), ('height', 'camera/image/height'), ('rate_hz', 'update_rate'), ('horizontal_fov_rad', 'camera/horizontal_fov')]:
        set_text(camera, path, cfg['camera'][key])

    native_mass = sum(float(link.findtext('inertial/mass', '0')) for link in model.findall('link'))
    target_mass = cfg['simulation_total_mass_kg']
    if not isinstance(target_mass, (int, float)) or isinstance(target_mass, bool) or not math.isfinite(target_mass) or target_mass <= 0:
        raise ValueError('positive_simulation_mass_required')
    mass_scale = target_mass/native_mass
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is not None:
            mass = inertial.find('mass')
            mass.text = str(float(mass.text)*mass_scale)
            for value in inertial.findall('inertia/*'):
                value.text = str(float(value.text)*mass_scale)

    world_root = native('worlds/default.sdf')
    world = world_root.find('world')
    world.set('name', WORLD)
    for plugin in native('server.config').findall('plugins/plugin'):
        if 'GstCamera' in plugin.get('name', ''):
            continue  # Raw camera images suffice; no video streaming server.
        plugin = deepcopy(plugin)
        plugin.attrib.pop('entity_name', None)
        plugin.attrib.pop('entity_type', None)
        world.append(plugin)
    ground = world.find("model[@name='ground_plane']/link")
    for x in range(-2, 16):
        for y in range(-3, 13):
            if (x+y) % 2 == 0:
                box_visual(ground, f'floor_texture_{x}_{y}', [.5, .5, .001], [x*.5, y*.5, .001], '0.15 0.15 0.15 1')
    for name, xyz in zip(anchors['anchor_order'], anchors['anchors_xyz_m']):
        marker = ET.SubElement(world, 'model', name='uwb_'+name)
        set_text(marker, 'static', 'true')
        set_text(marker, 'pose', pose_text(xyz+[0, 0, 0]))
        link = ET.SubElement(marker, 'link', name='anchor')
        box_visual(link, name, [.12, .12, .12], [0, 0, 0], '1 0.3 0.05 1')
        box_visual(link, 'support', [.025, .025, xyz[2]], [0, 0, -xyz[2]/2], '0.35 0.35 0.35 1')
    for obstacle in cfg['obstacles']:
        obj = ET.SubElement(world, 'model', name=obstacle['name'])
        set_text(obj, 'static', 'true')
        set_text(obj, 'pose', pose_text(obstacle['pose']))
        link = ET.SubElement(obj, 'link', name='obstacle')
        box_visual(link, 'wall', obstacle['size_m'], [0, 0, 0], '.6 .6 .7 1')
        collision = ET.SubElement(link, 'collision', name='wall_collision')
        set_text(collision, 'geometry/box/size', pose_text(obstacle['size_m'], 3))
    trial.update(anchors_xyz_m=anchors['anchors_xyz_m'], tag_offset_body_flu_m=cfg['tag_offset_body_flu_m'],
                 bias_m=[0., 0., 0., 0.], layout_source=anchors,
                 mount_status=cfg['mount_status'], fc_offset_body_flu_m=cfg['fc_offset_body_flu_m'],
                 note='Survey-coordinate prototype; user-reported horizontal mounts; sensor heights remain simulation assumptions. No flight output.')
    license_bytes = (px4_gz/'LICENSE').read_bytes()
    output.mkdir(parents=True, exist_ok=False)
    write_xml(output/'models'/MODEL/'model.sdf', model_root)
    config_root = ET.Element('model')
    set_text(config_root, 'name', MODEL)
    set_text(config_root, 'version', '1')
    set_text(config_root, 'sdf', 'model.sdf').set('version', '1.9')
    write_xml(output/'models'/MODEL/'model.config', config_root)
    write_xml(output/'worlds'/f'{WORLD}.sdf', world_root)
    (output/'LICENSE-PX4-gazebo-models').write_bytes(license_bytes)
    for name, value in [('equipment.json', cfg), ('anchors.json', anchors), ('trial.json', trial)]:
        (output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    manifest = dict(native_source_sha256=sources, mount_status=cfg['mount_status'],
                    native_total_mass_kg=native_mass, native_mass_inertia_scale=mass_scale,
                    simulated_total_mass_kg=sum(float(link.findtext('inertial/mass', '0')) for link in model.findall('link')),
                    real_total_mass_kg=None, flight_verified=False, gazebo_runtime_verified=False,
                    outputs_sha256={str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return manifest


def install_assets(rig, px4_gz):
    """Install only new names; existing upstream files are never overwritten."""
    rig, px4_gz = Path(rig), Path(px4_gz)
    targets = [px4_gz/'models'/MODEL, px4_gz/'worlds'/f'{WORLD}.sdf']
    if any(p.exists() for p in targets):
        raise FileExistsError('rig_name_already_installed')
    shutil.copytree(rig/'models'/MODEL, targets[0])
    with targets[1].open('xb') as stream:
        stream.write((rig/'worlds'/f'{WORLD}.sdf').read_bytes())
    shutil.copyfile(rig/'LICENSE-PX4-gazebo-models', targets[0]/'LICENSE')


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    config = Path(__file__).resolve().parents[2]/'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-gz', type=Path, required=True)
    parser.add_argument('--anchors', type=Path, default=config/'anchors_20260906.json')
    parser.add_argument('--equipment', type=Path, default=config/'gazebo_equipment.json')
    parser.add_argument('--trial', type=Path, default=config/'gazebo_shadow.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--install', action='store_true')
    opts = parser.parse_args(args)
    result = build(opts.px4_gz, opts.anchors, opts.equipment, opts.trial, opts.output)
    if opts.install:
        install_assets(opts.output, opts.px4_gz)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
