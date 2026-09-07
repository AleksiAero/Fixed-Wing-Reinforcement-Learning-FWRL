from pathlib import Path
import vtk,numpy as np,json
from vtk.util.numpy_support import vtk_to_numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
case=Path.home()/'spider-v2-cfd';out=Path('cfd/results');out.mkdir(exist_ok=True)
r=vtk.vtkOpenFOAMReader();r.SetFileName(str(case/'spider.foam'));r.UpdateInformation();r.EnableAllCellArrays();r.EnableAllPatchArrays()
times=r.GetTimeValues();last=times.GetValue(times.GetNumberOfTuples()-1);r.SetTimeValue(last);r.Update()
blocks=r.GetOutput();internal=blocks.GetBlock(0)
assert internal is not None
cp=vtk.vtkCellDataToPointData();cp.SetInputData(internal);cp.Update()
plane=vtk.vtkPlane();plane.SetOrigin(0,0,0);plane.SetNormal(0,1,0)
cutter=vtk.vtkCutter();cutter.SetInputConnection(cp.GetOutputPort());cutter.SetCutFunction(plane)
tri=vtk.vtkTriangleFilter();tri.SetInputConnection(cutter.GetOutputPort());tri.Update();s=tri.GetOutput()
v=vtk_to_numpy(s.GetPoints().GetData());cells=vtk_to_numpy(s.GetPolys().GetData()).reshape(-1,4)[:,1:]
p=vtk_to_numpy(s.GetPointData().GetArray('p'));u=vtk_to_numpy(s.GetPointData().GetArray('U'));speed=np.linalg.norm(u,axis=1)
report={'iteration':last,'slice_points':len(v),'slice_pressure_kinematic_range':p.min().item(),'max_speed_m_s':speed.max().item(),'all_finite':bool(np.isfinite(p).all() and np.isfinite(u).all())}
mesh=mtri.Triangulation(v[:,0],v[:,2],cells)
fig,axs=plt.subplots(2,1,figsize=(11,7),layout='constrained')
for ax,values,label,cmap,limits in [(axs[0],p/(.5*25**2),'Pressure coefficient Cp','coolwarm',(-1,1)),(axs[1],speed,'Speed (m/s)','viridis',(0,35))]:
 im=ax.tripcolor(mesh,values,shading='gouraud',cmap=cmap,vmin=limits[0],vmax=limits[1]);fig.colorbar(im,ax=ax,label=label)
 ax.set(xlim=(-2,1.6),ylim=(-.4,.65),xlabel='X (m) — nose to the right',ylabel='Z (m)');ax.set_aspect('equal');ax.set_facecolor('#bbbbbb')
fig.suptitle(f'Spider v2 | centre plane, iteration {last:g}\nCoarse unpowered CFD commissioning — flow right to left')
fig.savefig(out/'flow_slice.png',dpi=170);(out/'field_audit.json').write_text(json.dumps(report,indent=2));print(report)
