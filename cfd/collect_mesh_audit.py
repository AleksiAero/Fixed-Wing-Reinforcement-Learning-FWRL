from pathlib import Path
import json,re,shutil
case=Path.home()/'spider-v2-cfd';dest=Path('cfd/results');dest.mkdir(exist_ok=True)
for name in ['log.surfaceCheck','log.surfaceSelfIntersection','log.blockMesh','log.surfaceFeatureExtract','log.snappyHexMesh','log.checkMesh','log.checkMeshStandard','log.decomposePar','log.functionObjectFailure']:
 if (case/name).exists():shutil.copy2(case/name,dest/name)
report={'cad_saved_as':'Spider v2','fusion_folder':'ALEKSI AERO / Fixed Wing (Experimental)',
'closed_surface':True,'surface_self_intersections':False,'mesh_cells':506654,
'standard_checkMesh_pass':True,'extended_checkMesh_pass':False,
'extended_flags':{'low_determinant_cells':23,'concave_cells':26018,'concave_faces':1587,'warped_faces':9},
'max_non_orthogonality_deg':64.746968,'max_skewness':2.8527995,'minimum_volume_m3':3.2360728e-9,
'average_layers_reported':2.68,'fully_validated_for_aerodynamic_predictions':False}
(dest/'mesh_summary.json').write_text(json.dumps(report,indent=2))
shutil.copy2('cfd/Allrun',case/'Allrun')
