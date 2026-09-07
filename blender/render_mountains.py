import bpy
from pathlib import Path

scene = bpy.context.scene
scene.render.resolution_x,scene.render.resolution_y = 1400,900
scene.cycles.samples = 24
scene.cycles.use_denoising = False
scene.camera = bpy.data.objects['Meadow overview']
scene.render.filepath = str(Path(bpy.data.filepath).with_name('mountain_course.png'))
bpy.ops.render.render(write_still=True)
