from pathlib import Path
import cadquery as cq, trimesh, json, hashlib
src=Path('cfd/geometry/spider_v2_cfd.step')
shape=cq.importers.importStep(str(src)).val()
assert shape.isValid()
verts,faces=shape.tessellate(.08,.12)
mesh=trimesh.Trimesh(vertices=[[v.x/1000,v.y/1000,v.z/1000] for v in verts],faces=faces,process=True)
mesh.merge_vertices(digits_vertex=9)
mesh.fix_normals()
report={'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'units':'m','vertices':len(mesh.vertices),'triangles':len(mesh.faces),'watertight':mesh.is_watertight,'winding_consistent':mesh.is_winding_consistent,'connected_components':len(mesh.split()),'bounds_m':mesh.bounds.tolist(),'volume_m3':mesh.volume,'tessellation_tolerance_mm':.08}
Path('cfd/geometry/surface_audit.json').write_text(json.dumps(report,indent=2))
print(report,flush=True)
assert mesh.is_watertight and mesh.is_winding_consistent and len(mesh.split())==1
mesh.export('cfd/geometry/spider.stl')
