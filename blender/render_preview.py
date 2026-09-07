import bpy
from pathlib import Path

scene = bpy.context.scene
scene.render.resolution_x = 1100
scene.render.resolution_y = 700
scene.cycles.samples = 16
scene.cycles.use_denoising = False
out = Path(bpy.data.filepath).parent
scene.frame_set(1)
scene.render.filepath = str(out/'highspeed_launch.png')
bpy.ops.render.render(write_still=True)
scene.frame_set(270)
scene.render.filepath = str(out/'highspeed_flight.png')
bpy.ops.render.render(write_still=True)
