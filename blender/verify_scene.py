"""Headless integration check for the saved scene and its embedded live script."""
import bpy
import numpy as np

scene = bpy.context.scene
assert scene['target_speed_mph'] == 200
assert len([o for o in bpy.data.objects if 'launch rail' in o.name]) == 2
assert len([o for o in bpy.data.objects if o.name.startswith('Hoop ') and 'upright' in o.name]) == 9
namespace = {}
exec(compile(bpy.data.texts['Live Simulation.py'].as_string(), 'Live Simulation.py', 'exec'), namespace)
assert namespace['simulation']['state'][4] == 0
for frame in range(2, 42):
    scene.frame_set(frame)
assert abs(scene['airspeed_mph']-50) < 1e-6
namespace['simulation']['state'][2] = -10
namespace['simulation']['tick'] = 100
scene.frame_set(42)
assert scene['crash_reset_count'] == 1
assert scene['airspeed_mph'] == 0
assert scene['next_hoop'] == 1
assert np.allclose(namespace['simulation']['state'][:3], namespace['world']['start'])
print('PASS: saved scene, live launch to 50 mph, ground crash and launcher reset')
