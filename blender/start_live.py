"""Enable the locally generated scene's simulation when launching Blender."""
import bpy

namespace = {}
exec(compile(bpy.data.texts['Live Simulation.py'].as_string(), 'Live Simulation.py', 'exec'), namespace)
print('Live flight ready. Press Space to launch; frame 1 resets to launcher.')
