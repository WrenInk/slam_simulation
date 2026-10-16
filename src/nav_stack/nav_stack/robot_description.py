"""Load the same simple URDF links in MuJoCo and in robot_state_publisher."""
from pathlib import Path
import xml.etree.ElementTree as ET


def add_urdf(root, path, name, spawn, contact_clamp=False, driven_wheels=False):
    urdf = ET.parse(Path(path)).getroot()
    world = root.find('worldbody')
    actuator = root.find('actuator')
    if actuator is None:
        actuator = ET.SubElement(root, 'actuator')
    children = {}
    for joint in urdf.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    links = {link.get('name'): link for link in urdf.findall('link')}

    def add_link(link_name, parent, origin=None, joint=None):
        attrs = dict(name=f'{name}/{link_name}')
        if origin is not None:
            attrs.update(pos=origin.get('xyz', '0 0 0'), euler=origin.get('rpy', '0 0 0'))
        if link_name == 'base_link':
            attrs.update(mocap='true', pos=' '.join(map(str, spawn)))
        body = ET.SubElement(parent, 'body', **attrs)
        link = links[link_name]
        inertia = link.find('inertial')
        if inertia is not None:
            mass = inertia.find('mass').get('value')
            tensor = inertia.find('inertia')
            ET.SubElement(body, 'inertial', pos=inertia.find('origin').get('xyz', '0 0 0'), mass=mass,
                diaginertia=' '.join(tensor.get(k) for k in ['ixx', 'iyy', 'izz']))
        if joint is not None and joint.get('type') != 'fixed':
            jname = f'{name}/{joint.get("name")}'
            jattrs = dict(name=jname, type='slide' if joint.get('type') == 'prismatic' else 'hinge',
                          axis=joint.find('axis').get('xyz'), damping='10', armature='.02')
            if joint.get('type') != 'continuous':
                limit = joint.find('limit')
                jattrs.update(limited='true', range=f'{limit.get("lower")} {limit.get("upper")}')
            suspension=jname.endswith('_suspension_joint')
            if suspension:
                # Passive travel keeps four mecanum wheels loaded across a
                # slope transition. No motor or prescribed chassis motion.
                jattrs.update(damping='60',armature='.001',stiffness='4000',springref='-.006')
            ET.SubElement(body, 'joint', **jattrs)
            if suspension:
                pass
            elif 'wheel' in jname and driven_wheels:
                ET.SubElement(actuator, 'position', name=jname+'_servo', joint=jname,
                    kp='40', kv='2', forcelimited='true', forcerange='-20 20')
            elif 'wheel' not in jname:
                finger = 'finger_' in jname
                ET.SubElement(actuator, 'position', name=jname+'_servo', joint=jname,
                    kp='2500' if finger and contact_clamp else ('12000' if contact_clamp else '1500'),
                    kv='45' if finger and contact_clamp else ('200' if contact_clamp else '90'),
                    forcelimited='true', forcerange='-35 35' if finger and contact_clamp else '-400 400')
        for index, visual in enumerate(link.findall('visual')):
            origin = visual.find('origin')
            color = visual.find('material/color')
            attrs = dict(name=f'{name}/{link_name}_visual_{index}',
                rgba=color.get('rgba') if color is not None else '.6 .6 .6 1',
                pos=origin.get('xyz', '0 0 0') if origin is not None else '0 0 0',
                euler=origin.get('rpy', '0 0 0') if origin is not None else '0 0 0',
                contype='2' if link.find('collision') is not None else '0',
                conaffinity='1' if link.find('collision') is not None else '0',
                group='1', friction='.8 .01 .001')
            if index and 'wheel' in link_name:
                attrs.update(contype='0', conaffinity='0')
            # Preserve the original model's collision mask for comparison.
            # The new clamp model keeps arm/object contacts enabled even while
            # a contact-gated assisted grasp or cargo latch is active.
            if not contact_clamp and link_name not in ['base_link'] and 'wheel' not in link_name and not link_name.startswith('cargo_'):
                attrs.update(contype='0', conaffinity='0')
            if contact_clamp and any(part in link_name for part in ['pad_', 'plate_', 'fork_']):
                attrs.update(friction='1.0 .02 .002', condim='4', solref='.008 1', solimp='.95 .99 .001')
            geometry = visual.find('geometry')
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            mesh = geometry.find('mesh')
            if box is not None:
                attrs.update(type='box', size=' '.join(str(float(v)/2) for v in box.get('size').split()))
            elif cylinder is not None:
                attrs.update(type='cylinder', size=f'{cylinder.get("radius")} {float(cylinder.get("length"))/2}')
            elif mesh is not None:
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    package, relative = filename[len('package://'):].split('/', 1)
                    # Source-tree loading also works without sourcing ROS.
                    local_share = Path(path).resolve().parent.parent
                    if package == 'nav_stack' and (local_share / relative).is_file():
                        mesh_path = local_share / relative
                    else:
                        from ament_index_python.packages import get_package_share_directory
                        mesh_path = Path(get_package_share_directory(package)) / relative
                else:
                    mesh_path = Path(filename.removeprefix('file://'))
                    if not mesh_path.is_absolute():
                        mesh_path = Path(path).resolve().parent / mesh_path
                asset = root.find('asset')
                if asset is None:
                    asset = ET.SubElement(root, 'asset')
                mesh_name = f'{name}/{link_name}_mesh_{index}'
                ET.SubElement(asset, 'mesh', name=mesh_name, file=str(mesh_path.resolve()),
                              scale=mesh.get('scale', '1 1 1'))
                attrs.update(type='mesh', mesh=mesh_name)
            else:
                raise ValueError(f'Unsupported URDF visual geometry for {link_name}')
            ET.SubElement(body, 'geom', **attrs)
        for child in children.get(link_name, []):
            add_link(child.find('child').get('link'), body, child.find('origin'), child)
        return body

    base = add_link('base_link', world)
    ET.SubElement(base, 'site', name=name+'_label', pos='0 0 .75', size='.005', rgba='0 0 0 0')
