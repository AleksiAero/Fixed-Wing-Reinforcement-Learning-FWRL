import bpy

namespace = {}
exec(compile(bpy.data.texts['Watch Learning.py'].as_string(),'Watch Learning.py','exec'),namespace)
bpy.ops.fwrl.start_learning()
