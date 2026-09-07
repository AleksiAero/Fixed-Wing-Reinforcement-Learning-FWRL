"""Build a grassy landscape from the same world definition used by physics."""

import argparse

import json

import math

import random

import sys

from pathlib import Path

import bpy

from mathutils import Vector



ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(0, str(ROOT))

from fwrl.landscape import terrain_height, gate_normal

from fwrl.world import load_world



parser = argparse.ArgumentParser()

parser.add_argument('--world', default='worlds/mountain_gauntlet.json')
parser.add_argument('--output', default='worlds/mountain_gauntlet.blend')
parser.add_argument('--render', action='store_true')

parser.add_argument('--flight', help='Evaluation blender_flight.json to bake learned flight')

parser.add_argument('--policy-weights', help='Exported NumPy policy weights JSON')
parser.add_argument('--learning-steps', type=int, default=10000000)
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])

world = load_world(args.world)

aircraft = json.loads((ROOT/'configs/aircraft.json').read_text())

rng = random.Random(world.get('seed', 27))

bpy.ops.object.select_all(action='SELECT')

bpy.ops.object.delete(use_global=False)





def material(name, color, roughness=.8):

    mat = bpy.data.materials.new(name)

    mat.diffuse_color = (*color, 1)

    mat.use_nodes = True

    shader = mat.node_tree.nodes.get('Principled BSDF')

    shader.inputs['Base Color'].default_value = (*color, 1)

    shader.inputs['Roughness'].default_value = roughness

    return mat





grass = material('Meadow living grass', (.22, .36, .09))

nodes, links = grass.node_tree.nodes, grass.node_tree.links

position = nodes.new('ShaderNodeNewGeometry')

noise = nodes.new('ShaderNodeTexNoise')

noise.inputs['Scale'].default_value = .032

noise.inputs['Detail'].default_value = 5

links.new(position.outputs['Position'], noise.inputs['Vector'])

ramp = nodes.new('ShaderNodeValToRGB')

ramp.color_ramp.elements[0].position = .18

ramp.color_ramp.elements[0].color = (.055, .12, .021, 1)

ramp.color_ramp.elements[1].position = .84

ramp.color_ramp.elements[1].color = (.41, .49, .12, 1)

ramp.color_ramp.elements.new(.52).color = (.16, .29, .047, 1)

links.new(noise.outputs['Fac'], ramp.inputs[0])

links.new(ramp.outputs[0], nodes.get('Principled BSDF').inputs['Base Color'])

fine = nodes.new('ShaderNodeTexNoise')

fine.inputs['Scale'].default_value = 9

fine.inputs['Detail'].default_value = 2

links.new(position.outputs['Position'], fine.inputs['Vector'])

bump = nodes.new('ShaderNodeBump')

bump.inputs['Strength'].default_value = .32

bump.inputs['Distance'].default_value = .12

links.new(fine.outputs['Fac'], bump.inputs['Height'])

links.new(bump.outputs['Normal'], nodes.get('Principled BSDF').inputs['Normal'])

if world.get('control_mode') in ('direct','elevons'):

    height = nodes.new('ShaderNodeSeparateXYZ')

    links.new(position.outputs['Position'], height.inputs[0])

    height_map = nodes.new('ShaderNodeMapRange')

    height_map.inputs['From Min'].default_value = 0

    height_map.inputs['From Max'].default_value = 1100

    links.new(height.outputs['Z'], height_map.inputs['Value'])

    elevation = nodes.new('ShaderNodeValToRGB')

    elevation.color_ramp.elements[0].color = (.08,.19,.035,1)

    elevation.color_ramp.elements[1].color = (.91,.94,.97,1)

    elevation.color_ramp.elements.new(.38).color = (.21,.29,.10,1)

    elevation.color_ramp.elements.new(.62).color = (.22,.23,.21,1)

    elevation.color_ramp.elements.new(.82).color = (.62,.64,.63,1)

    links.new(height_map.outputs[0], elevation.inputs[0])

    links.new(elevation.outputs[0], nodes.get('Principled BSDF').inputs['Base Color'])

bark = material('Warm oak bark', (.16, .092, .047))

leaves = [material('Leaves fern', (.055, .16, .033)), material('Leaves olive', (.17, .25, .048)), material('Leaves spring', (.22, .35, .055))]

ivory = material('Gate porcelain', (.9, .92, .84), .3)

orange = material('Gate tangerine', (1., .19, .025), .35)

blue = material('Gate sky blue', (.035, .35, .75), .35)

label_mat = material('Gate labels graphite', (.027, .046, .032))





def mesh(name, vertices, faces, mat):

    data = bpy.data.meshes.new(name)

    data.from_pydata(vertices, [], faces)

    data.update()

    obj = bpy.data.objects.new(name, data)

    bpy.context.collection.objects.link(obj)

    obj.data.materials.append(mat)

    return obj





def box(name, center, size, mat):

    bpy.ops.mesh.primitive_cube_add(size=1, location=center)

    obj = bpy.context.object

    obj.name = name

    obj.dimensions = size

    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    obj.data.materials.append(mat)

    return obj





lo, hi = world['bounds']

grid = 420 if world.get('control_mode') in ('direct','elevons') else 220

vertices = []

for j in range(grid+1):

    y = lo[1] + (hi[1]-lo[1])*j/grid

    for i in range(grid+1):

        x = lo[0] + (hi[0]-lo[0])*i/grid

        vertices.append((x, y, terrain_height(x, y, world)))

faces = []

for j in range(grid):

    for i in range(grid):

        a = j*(grid+1)+i

        faces.append((a, a+1, a+grid+2, a+grid+1))

ground = mesh('Rolling meadow shared collision heightfield', vertices, faces, grass)

for p in ground.data.polygons:

    p.use_smooth = True

ground['physics_source'] = str(Path(args.world))

earth = material('Earth cutaway edge', (.12, .09, .048))

boundary = [i for i in range(grid+1)] + [j*(grid+1)+grid for j in range(1, grid+1)] + [grid*(grid+1)+i for i in reversed(range(grid))] + [j*(grid+1) for j in reversed(range(1, grid))]

edge_verts = [vertices[i] for i in boundary] + [(vertices[i][0], vertices[i][1], -12) for i in boundary]

n = len(boundary)

mesh('Meadow edge', edge_verts, [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)], earth)





def tree(obstacle):

    x, y, z = obstacle['center']

    sx, sy, h = obstacle['size']

    base = z-h/2

    radius = min(sx, sy)/2

    mat = leaves[obstacle.get('variant', 0)%len(leaves)]

    bpy.ops.mesh.primitive_cone_add(vertices=9, radius1=radius*.105, radius2=radius*.058, depth=h*.72, location=(x, y, base+h*.36))

    trunk = bpy.context.object

    trunk.name = obstacle['name']+' trunk'

    trunk.data.materials.append(bark)

    for p in trunk.data.polygons:

        p.use_smooth = True

    # All canopy clusters fit within the shared conservative tree collision box.

    for a, b, c, rw, rh in [(0, 0, .74, .76, .26), (-.38, .18, .66, .55, .22), (.35, -.2, .7, .57, .24), (.04, .34, .82, .51, .18)]:

        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=1, location=(x+a*radius, y+b*radius, base+c*h))

        obj = bpy.context.object

        obj.name = obstacle['name']+' foliage'

        for vertex in obj.data.vertices:

            vertex.co *= rng.uniform(.89, 1.0)

        obj.scale = (radius*rw, radius*rw, h*rh)

        obj.data.materials.append(mat)

        for p in obj.data.polygons:

            p.use_smooth = True





for obstacle in world['obstacles']:

    if obstacle.get('kind') == 'tree':

        tree(obstacle)

    else:

        box(obstacle['name'], obstacle['center'], obstacle['size'], bark)

for shrub in world.get('foliage', []):

    x, y, z = shrub['position']

    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1, radius=1, location=(x, y, z+shrub['height']*.45))

    obj = bpy.context.object

    obj.name = 'Wild foliage decorative'

    obj.scale = (shrub['radius'], shrub['radius']*.8, shrub['height']*.55)

    obj.data.materials.append(rng.choice(leaves))

    for p in obj.data.polygons:

        p.use_smooth = True



verts, faces = [], []

for _ in range(18000 if world.get('terrain') else 0):

    x, y = rng.uniform(lo[0]+3, hi[0]-3), rng.uniform(lo[1]+3, hi[1]-3)

    z = terrain_height(x, y, world)

    h, angle = rng.uniform(.25, .95), rng.uniform(0, math.tau)

    for offset in (0, 2.1, 4.2):

        dx, dy = .09*math.cos(angle+offset), .09*math.sin(angle+offset)

        a = len(verts)

        verts.extend([(x-dx, y-dy, z), (x+dx, y+dy, z), (x+dx*3, y+dy*3, z+h)])

        faces.append((a, a+1, a+2))

mesh('Meadow grass tufts decorative', verts, faces, grass)



overview_location = Vector((3200, -5400, 4100)) if 'launcher' in world else Vector((1130, -1480, 1000))

for i, p in enumerate(world['waypoints']):

    normal = Vector(gate_normal(world, i))

    bpy.ops.mesh.primitive_torus_add(major_segments=96, minor_segments=12, major_radius=world.get('gate_radii_m',[world['goal_radius_m']]*len(world['waypoints']))[i], minor_radius=world.get('hoops', {}).get('tube_radius_m', .85), location=p)

    obj = bpy.context.object

    obj.name = f'Hoop {i+1:02d} upright course marker'

    obj.rotation_mode = 'QUATERNION'

    obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(normal)

    obj.data.materials.append(orange if i%3 else blue)

    obj.data.materials.append(ivory)

    for poly in obj.data.polygons:

        angle = math.atan2(poly.center.y, poly.center.x)

        poly.material_index = int((angle % math.tau) / (math.tau/12)) % 2

        poly.use_smooth = True

    obj['collision'] = 'launcher' in world

    obj['normal_enu'] = list(normal)

    obj['waypoint_index'] = i

    bpy.ops.object.text_add(location=(p[0], p[1], p[2]+world.get('gate_radii_m',[world['goal_radius_m']]*len(world['waypoints']))[i]+7))

    label = bpy.context.object

    label.name = f'Hoop {i+1:02d} number'

    label.data.body = f'{i+1:02d}'

    label.data.size = 10

    label.data.align_x = 'CENTER'

    label.data.extrude = .06

    label.rotation_euler = (overview_location-label.location).to_track_quat('Z', 'Y').to_euler()

    label.data.materials.append(label_mat)



start = world['start']

body = box('Research aircraft 5kg', start, [1.05, .13, .15], ivory)

wing = box('Wing 1.651 m span', [start[0]-.08, start[1], start[2]], [.25, aircraft['wingspan_m_nominal'], .025], ivory)

wing.parent = body

wing.matrix_parent_inverse = body.matrix_world.inverted()

bpy.ops.object.camera_add(location=(start[0]+.55, start[1], start[2]))

fpv = bpy.context.object

fpv.name = 'Forward portrait camera'

fpv.rotation_euler = Vector((1, 0, 0)).to_track_quat('-Z', 'Y').to_euler()

fpv.data.lens = aircraft['camera_focal_length_mm']

fpv.data.sensor_fit = 'HORIZONTAL'

fpv.data.sensor_width = aircraft['camera_sensor_mm_portrait'][0]

fpv.data.clip_end = 4000

fpv.parent = body

fpv.matrix_parent_inverse = body.matrix_world.inverted()



bpy.ops.object.light_add(type='SUN', location=(0, 0, 900))

bpy.context.object.rotation_euler = (math.radians(24), math.radians(-28), math.radians(-28))

bpy.context.object.data.energy = 2.5

bpy.context.object.data.angle = .08

bpy.ops.object.camera_add(location=overview_location)

overview = bpy.context.object

overview.name = 'Meadow overview'

overview.rotation_euler = (Vector((3200, 0, 35) if 'launcher' in world else (0, 0, 35))-overview.location).to_track_quat('-Z', 'Y').to_euler()

overview.data.type = 'ORTHO'

overview.data.ortho_scale = 8700 if 'launcher' in world else 1870

overview.data.clip_end = 20000
if world.get('control_mode') in ('direct','elevons'):
    center = Vector(((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,250))
    overview.location = center+Vector((9000,-14000,15000))
    overview.rotation_euler = (center-overview.location).to_track_quat('-Z','Y').to_euler()
    overview.data.ortho_scale = 17000
    overview.data.clip_end = 50000
scene = bpy.context.scene
scene['learning_steps'] = args.learning_steps
scene.camera = overview

scene.render.engine = 'CYCLES'

scene.cycles.samples = 40

scene.cycles.use_denoising = False

scene.world.use_nodes = True

scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (.63, .73, .86, 1)

scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = .6

scene.view_settings.view_transform = 'AgX'

scene.render.resolution_x, scene.render.resolution_y = 1680, 1120

scene.render.resolution_percentage = 100

scene.render.fps = aircraft['camera_fps']

scene.unit_settings.system = 'METRIC'

scene['navigation_world_json'] = json.dumps(world)

scene['fpv_resolution'] = '1080 x 1920; select Forward portrait camera and set portrait output'

scene['hoop_note'] = 'Directed hoop-plane scoring; swept torus and terrain collisions.' if 'launcher' in world else 'Legacy proximity scoring.'

if 'launcher' in world:

    import numpy as np

    from fwrl.demo import physical_rollout

    from mathutils import Euler

    launcher = world['launcher']

    g, h = launcher['pitch_rad'], launcher['heading_rad']

    direction = Vector((math.cos(g)*math.cos(h), math.cos(g)*math.sin(h), math.sin(g)))

    side = Vector((-math.sin(h), math.cos(h), 0))

    steel = material('Launcher steel', (.08,.11,.14), .3)

    from fwrl.launcher import geometry as launcher_geometry
    hardware = launcher_geometry(world)
    for rail_data in hardware['rails']:
        rail = box('50 mph launch rail',rail_data['center'],rail_data['size'],steel)
        rail.rotation_euler = (0,-g,h)
    for support in hardware['supports']:
        obj = box('Ground anchored launcher support',support['center'],support['size'],steel)
        obj.rotation_euler.z = h

    if world.get('control_mode') == 'elevons':
        from fwrl.airframe import geometry as airframe_geometry
        body.data.clear_geometry()
        bpy.data.objects.remove(wing,do_unlink=True)
        for part in airframe_geometry():
            mat = material(part['name']+' carbon',tuple(part['color']),.55)
            obj = mesh(part['name'],part['vertices'],part['faces'],mat)
            obj.parent = bpy.data.objects[part['parent']] if part.get('parent') else body
            obj.location = part['pivot']
            if part.get('hinge_axis'): obj['hinge_axis']=part['hinge_axis']
            for polygon in obj.data.polygons:
                polygon.use_smooth = True

    if world.get('control_mode') == 'elevons':
        for sign,label in [(1,'Left'),(-1,'Right')]:
            start = Vector((.55*(-.22)-.35*.43+.06-.12,sign*.43*1.35,.065))
            end = Vector((.55*(-.39)-.35*.43+.06-.12,sign*.43*1.35,.065))
            bpy.ops.mesh.primitive_cylinder_add(vertices=12,radius=.003,depth=1)
            rod=bpy.context.object
            rod.name=label+' pushrod'
            rod.parent=body
            rod.location=(start+end)/2
            rod.rotation_mode='QUATERNION'
            rod.rotation_quaternion=(end-start).to_track_quat('Z','Y')
            rod.scale.z=(end-start).length
            rod.data.materials.append(material(label+' linkage silver',(.65,.7,.75),.25))

    bpy.ops.object.camera_add()

    chase = bpy.context.object

    chase.name = 'Aircraft chase camera'

    chase.parent = body

    chase.location = (-9,-5,3)

    chase.rotation_euler = Vector((12,5,-2)).to_track_quat('-Z','Y').to_euler()

    chase.data.lens = 24

    chase.data.clip_end = 15000

    scene.camera = chase

    if world.get('control_mode') in ('direct','elevons'):

        from fwrl.aerodynamics import initial_state

        states = np.array([initial_state(world)])

        dt = .05

        scene['flight_source'] = 'None: random initialization and live training only'

    elif args.flight:

        flight = json.loads(Path(args.flight).read_text())

        if flight['world'] != world:

            raise ValueError('Flight/world mismatch')

        states = np.array(flight['states'])

        dt = flight['dt']

        scene['flight_source'] = flight['source']

    else:

        states, report = physical_rollout(world)

        dt = .05

        scene['flight_source'] = 'Reference controller: '+json.dumps(report)

    scene.render.fps = round(1/dt)

    body.rotation_mode = 'ZYX'

    for frame, state in enumerate(states, 1):

        body.location = state[:3]

        body.rotation_euler = Euler((-state[5],-state[6],state[3]), 'ZYX')

        body.keyframe_insert(data_path='location', frame=frame)

        body.keyframe_insert(data_path='rotation_euler', frame=frame)

    scene.frame_end = len(states)+1

    body.location = start

    body.rotation_euler = Euler((0,-g,h), 'ZYX')

    body.keyframe_insert(data_path='location', frame=scene.frame_end)

    body.keyframe_insert(data_path='rotation_euler', frame=scene.frame_end)

    for curve in body.animation_data.action.fcurves:

        for key in curve.keyframe_points:

            key.interpolation = 'LINEAR'

    scene.frame_set(1)

    scene['project_root'] = str(ROOT)

    scene['playback_mode'] = 'Baked flight: Space to play. Run Live Simulation.py (Alt-P) for live physics and crash resets.'

    scene['target_speed_mph'] = 200

    scene['launch_speed_mph'] = 50

    scene['physics_model'] = 'Quaternion 6DOF with elevons and post-stall aerodynamics; assumed coefficients' if world.get('control_mode') == 'elevons' else 'Force-based 3DOF; assumed coefficients'

    if args.policy_weights and world.get('control_mode') not in ('direct','elevons'):

        scene['policy_weights_json'] = Path(args.policy_weights).read_text()

    text = bpy.data.texts.new('Live Simulation.py')

    text.write((ROOT/'blender/live_simulation.py').read_text())

    if world.get('control_mode') in ('direct','elevons'):

        body.animation_data_clear()

        scene.frame_end = 1

        scene['playback_mode'] = 'Live training only — use Start-Highspeed.ps1'

        text = bpy.data.texts.new('Watch Learning.py')

        text.write((ROOT/'blender/watch_learning.py').read_text())

    for screen in bpy.data.screens:

        for area in screen.areas:

            if area.type == 'VIEW_3D':

                area.spaces.active.region_3d.view_perspective = 'CAMERA'

out = Path(args.output).resolve()

out.parent.mkdir(parents=True, exist_ok=True)

bpy.ops.wm.save_as_mainfile(filepath=str(out))

if args.render:

    scene.render.filepath = str(out.with_suffix('.png'))

    bpy.ops.render.render(write_still=True)
