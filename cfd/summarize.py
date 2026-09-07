from pathlib import Path
import re,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
case=Path.home()/'spider-v2-cfd';out=Path('cfd/results');out.mkdir(exist_ok=True)
log=(case/'log.simpleFoam').read_text()
series={};t=0;continuity=[]
for line in log.splitlines():
 m=re.match(r'Time = ([\d.eE+-]+)',line)
 if m:t=float(m[1])
 m=re.search(r'Solving for (\w+), Initial residual = ([\d.eE+-]+)',line)
 if m:series.setdefault(m[1],[]).append([t,float(m[2])])
 m=re.search(r'sum local = ([\d.eE+-]+), global = ([\d.eE+-]+), cumulative = ([\d.eE+-]+)',line)
 if m:continuity.append([t,*map(float,m.groups())])
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for name,vals in series.items():
 v=np.array(vals);axs[0].semilogy(v[:,0],v[:,1],label=name,lw=1)
axs[0].set(xlabel='SIMPLE iteration',ylabel='Initial residual',title='Solver residual history');axs[0].legend(ncol=2);axs[0].grid(alpha=.2)
a=np.array(continuity)
axs[1].semilogy(a[:,0],np.abs(a[:,1]),label='Local continuity');axs[1].semilogy(a[:,0],np.maximum(np.abs(a[:,2]),1e-16),label='Global continuity');axs[1].set(xlabel='SIMPLE iteration',ylabel='Continuity error magnitude',title='Mass continuity');axs[1].legend();axs[1].grid(alpha=.2)
fig.suptitle('Spider v2 | 25 m/s, 0° | commissioning run — not performance validation')
fig.savefig(out/'convergence.png',dpi=160);plt.close(fig)
report={'completed_iteration':t,'solver_ended_normally':'\nEnd\n' in log,'simple_convergence_reported':'SIMPLE solution converged' in log,'final_initial_residuals':{n:v[-1][1] for n,v in series.items()},'last_continuity_row':continuity[-1],'force_and_yPlus_functionObjects':'disabled: Ubuntu v1912 sha1 IOstream runtime error','interpretation':'Coarse solver commissioning; advanced mesh quality flags remain; no mesh independence or aerodynamic validation.'}
(out/'solver_summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
