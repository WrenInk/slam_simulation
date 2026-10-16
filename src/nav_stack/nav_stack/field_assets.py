"""Runtime corrections to the pinned RC2027 assets; upstream files stay intact."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def load_field(path, competition=False, storage_pitch=.36):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    root.find('compiler').set('meshdir', str(path.parent/'meshes'))
    world = root.find('worldbody')
    if world.find("geom[@name='col_band_red_ramp']") is None:
        blue = world.find("geom[@name='col_band_blue_ramp']")
        ramp = ET.SubElement(world, 'geom', **blue.attrib)
        ramp.set('name', 'col_band_red_ramp')
        ramp.set('pos', '-3.5 0.946 0.3004')
    # Rule 4.1.3: after setup all selected blocks are INSIDE storage, <=2 high.
    # 5 columns x 2 rows x 2 layers; every 350 mm cube fits the 2 m x 1 m zone.
    for team, centre in [('red', -4.5), ('blue', 4.5)]:
        for i in range(20):
            layer, cell = i % 2, i // 2
            col, row = cell % 5, cell // 5
            body = world.find(f"body[@name='earthblock_{team}_{i+1:02d}']")
            x, y, z = centre+(col-2)*storage_pitch, 4.72+row*.4, .025+.175+layer*.351+.001
            body.set('pos', f'{x} {y} {z}')
    if competition:
        # The upstream ramp collider is a thin tilted box. Lateral lidar rays
        # can pass below it and hit the floor at the same XY as the ramp top.
        # Represent the actual solid ramp volume, preserving its top face and
        # width exactly. These are physical/raycast surfaces, not map masks.
        for team in ['red', 'blue']:
            ramp = world.find(f"geom[@name='col_band_{team}_ramp']")
            cx, cy, cz = map(float, ramp.get('pos').split())
            hx, hy, hz = map(float, ramp.get('size').split())
            qw, qx, _, _ = map(float, ramp.get('quat').split())
            angle = 2*math.atan2(qx, qw)
            top = [(cx+x, cy+y*math.cos(angle)-hz*math.sin(angle),
                    cz+y*math.sin(angle)+hz*math.cos(angle))
                   for x in [-hx, hx] for y in [-hy, hy]]
            vertices = top+[(x,y,0.) for x,y,z in top]
            name = f'solid_{team}_ramp'
            ET.SubElement(root.find('asset'), 'mesh', name=name,
                vertex=' '.join(str(v) for point in vertices for v in point))
            ramp.attrib.pop('size');ramp.attrib.pop('pos');ramp.attrib.pop('quat')
            ramp.set('type','mesh');ramp.set('mesh',name)
            # MuJoCo raycasts skip fully transparent geoms. Use the original
            # ramp color so the closed sides are also visible to the lidar.
            ramp.attrib.pop('rgba',None)
            ramp.set('material','rgb_200_150_140' if team=='red' else 'rgb_130_180_200')
            platform=world.find(f"geom[@name='col_band_{team}_plat']")
            platform.attrib.pop('rgba',None)
            platform.set('material','rgb_245_170_060' if team=='red' else 'rgb_060_170_245')
        ET.SubElement(root.find('asset'), 'texture', name='arena_sky', type='skybox',
            builtin='gradient', rgb1='.16 .20 .26', rgb2='.43 .49 .57', width='256', height='1536')
        # Domestic V1 (the user-supplied PDF), not the different ABU mass values.
        for body in world.findall('body'):
            name = body.get('name', '')
            if name.startswith('earthblock_'):
                mass, inertia = .275, .275*.35**2/6
                visual = body.find(f"geom[@name='{name}_vis']")
                visual.set('type', 'box')
                visual.attrib.pop('mesh', None)
                visual.set('size', '.175 .175 .175')
            elif name.startswith('skyblock_'):
                mass, inertia = .115, .115*.2**2/6
            elif name == 'mustika':
                mass, inertia = .420, .4*.420*.1**2
            else:
                continue
            body.find('inertial').set('mass', str(mass))
            body.find('inertial').set('diaginertia', ' '.join([str(inertia)]*3))
        # The upstream collision box closes the transfer entrance. Split the
        # side rail at the transfer boundary instead of driving through a wall.
        for side, x in [('w', -2.975), ('e', 2.975)]:
            rail = world.find(f"geom[@name='col_l1fence_{side}']")
            rail.set('size', '.025 .575 .025')
            rail.set('pos', f'{x} -2.375 .65')
            top = ET.SubElement(world, 'geom', **rail.attrib)
            top.set('name', f'col_l1fence_{side}_north')
            top.set('size', '.025 1.875 .025')
            top.set('pos', f'{x} 1.075 .65')
        # Central cup: 700 mm column plus a 100 mm rim, total height 800 mm.
        pillar = world.find("geom[@name='col_pillar_centre']")
        pillar.set('size', '.135 .35')
        pillar.set('pos', '0 0 1.275')
        for i in range(16):
            a = i*2*math.pi/16
            ET.SubElement(world, 'geom', name=f'central_cup_{i:02d}', type='box',
                size='.023 .023 .05', pos=f'{.1125*math.cos(a)} {.1125*math.sin(a)} 1.675',
                quat=f'{math.cos(a/2)} 0 0 {math.sin(a/2)}', rgba='.39 .24 0 1')
        for site in world.findall('site'):
            if site.get('name', '').startswith('site_建造点'):
                index = int(site.get('name').split('-')[-1])
                x, y, _ = map(float, site.get('pos').split())
                site.set('pos', f'{x} {y} {(.925 if index in [5,6,8,9] else .625)+.002}')
    return root
