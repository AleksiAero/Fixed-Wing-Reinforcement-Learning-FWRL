"""Integration test: real trainer -> telemetry -> Blender, with pause and stop."""
import bpy
import json
import time
import numpy as np

scene = bpy.context.scene
assert scene['flight_source'].startswith('None:')
assert 'policy_weights_json' not in scene
assert bpy.data.objects['Research aircraft 5kg'].animation_data is None
scene['learning_steps'] = 8192
namespace = {}
exec(compile(bpy.data.texts['Watch Learning.py'].as_string(),'Watch Learning.py','exec'),namespace)
bpy.ops.fwrl.start_learning()
try:
    deadline = time.monotonic()+25
    while time.monotonic()<deadline:
        namespace['poll']()
        packet = namespace['state']['packet']
        if packet and packet['live_tick']>5:
            break
        time.sleep(.1)
    assert packet and packet['live_tick']>5, 'No live training state received'
    np.testing.assert_allclose(bpy.data.objects['Research aircraft 5kg'].location[:],packet['state'][:3],atol=1e-4)
    bpy.ops.fwrl.pause_learning()
    time.sleep(.25)
    namespace['poll']()
    tick = namespace['state']['packet']['live_tick']
    time.sleep(.25)
    namespace['poll']()
    assert namespace['state']['packet']['live_tick']==tick
    assert scene['live_status'].startswith('paused')
    bpy.ops.fwrl.pause_learning()
    time.sleep(.25)
    namespace['poll']()
    assert namespace['state']['packet']['live_tick']>tick
    print('PASS: fresh CUDA training -> live Blender position; pause/resume holds simulation')
finally:
    bpy.ops.fwrl.stop_learning()
    namespace['state']['process'].wait(timeout=25)
    config = json.loads((namespace['state']['run']/'config.json').read_text())
    assert 'random initialization' in config['algorithm']
    assert (namespace['state']['run']/'policy.pt').exists()
    print('PASS: stopped and checkpoint saved; no guided or pretrained policy loaded')
