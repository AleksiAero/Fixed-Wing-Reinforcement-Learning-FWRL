"""Quaternion rigid-body dynamics for a flying wing, in ENU / body FLU.

State [position3,yaw,airspeed,left_bank,flight_path, quaternion_wxyz4,
       air-relative_velocity_world3, body_rates3, left/right_elevon_rad, throttle].
Coefficients are explicit assumptions; only the reference stall speed is specified
by the user. This is not an experimentally validated airframe model.
"""
import numpy as np


def initial(world):
    h,g = world['launcher']['heading_rad'],world['launcher']['pitch_rad']
    s = np.zeros(20)
    s[:7] = [*world['start'],h,0,0,g]
    # yaw about ENU Z followed by nose-up (negative body Y) pitch
    s[7:11] = [np.cos(h/2)*np.cos(g/2),np.sin(h/2)*np.sin(g/2),
               -np.cos(h/2)*np.sin(g/2),np.sin(h/2)*np.cos(g/2)]
    return s


def basis(q, ops=np):
    w,x,y,z = [q[:,i] for i in range(4)]
    stack = lambda v:ops.stack(v,-1)
    return (stack([1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)]),
            stack([2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)]),
            stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]))


def aero(state, world, ops=np):
    clip = ops.clip if ops is np else ops.clamp
    atan2 = np.arctan2 if ops is np else ops.atan2
    p = world['aerodynamics']
    forward,left,up = basis(state[:,7:11],ops)
    velocity = state[:,11:14]
    speed = clip((velocity*velocity).sum(-1)**.5,.01,None)
    u,v,w = [(velocity*axis).sum(-1) for axis in (forward,left,up)]
    alpha = atan2(-w,u)
    beta = atan2(v,(u*u+w*w)**.5+.001)
    collective = (state[:,17]+state[:,18])*.5
    critical = p['stall_angle_rad']
    attached = clip(p['cl_max']*(.12+.88*alpha/critical)-.18*collective,-p['cl_max'],p['cl_max'])
    fraction = clip((abs(alpha)-critical)/.08,0,1)
    blend = fraction*fraction*(3-2*fraction)
    cl = (1-blend)*attached + blend*p['cl_max']*.65*ops.sin(2*alpha)
    cd = p['cd0']+p['induced_drag']*cl*cl+1.4*ops.sin(alpha)**2+ .2*beta*beta
    rho = p['density_kg_m3']*ops.exp(-clip(state[:,2],0,None)/8500)
    qs = .5*rho*speed*speed*p['wing_area_m2']
    return alpha,beta,cl,cd,qs,speed,forward,left,up


def advance(state, action, world, time_s, dt=.05, wind=None, ops=np):
    clip = ops.clip if ops is np else ops.clamp
    atan2 = np.arctan2 if ops is np else ops.atan2
    stack = lambda values:ops.stack(values,-1)
    tensor = lambda values:np.asarray(values) if ops is np else ops.as_tensor(values,dtype=state.dtype,device=state.device)
    cross = lambda a,b:stack([a[:,1]*b[:,2]-a[:,2]*b[:,1],a[:,2]*b[:,0]-a[:,0]*b[:,2],a[:,0]*b[:,1]-a[:,1]*b[:,0]])
    s = state.copy() if ops is np else state.clone()
    a = clip(action,-1,1)
    p = world['aerodynamics']
    inertia = tensor(p['inertia_kg_m2'])
    # 100 Hz substeps resolve servo, angular-rate and post-stall dynamics.
    substeps = max(1,int(np.ceil(dt/.01)))
    subdt = dt/substeps
    for _ in range(substeps):
        s[:,17:19] += subdt*clip((a[:,:2]*p['elevon_limit_rad']-s[:,17:19])/p['servo_tau_s'],
                                -p['servo_rate_rad_s'],p['servo_rate_rad_s'])
        s[:,19] += subdt*((a[:,2]+1)*.5-s[:,19])/p['motor_tau_s']
        alpha,beta,cl,cd,qs,speed,forward,left,up = aero(s,world,ops)
        flow = s[:,11:14]/speed[:,None]
        lift_axis = cross(flow,left)
        lift_axis /= clip((lift_axis*lift_axis).sum(-1)**.5,.001,None)[:,None]
        thrust = s[:,19]*ops.minimum(speed*0+p['max_thrust_n'],p['propulsive_power_w']/clip(speed,1,None))
        force = (qs*cl)[:,None]*lift_axis-(qs*cd)[:,None]*flow+thrust[:,None]*forward
        force -= (qs*.6*beta)[:,None]*left
        acceleration = force/p['mass_kg'] + tensor([0,0,-9.81])
        s[:,11:14] += subdt*acceleration
        s[:,:3] += subdt*(s[:,11:14]+(0 if wind is None else wind))
        collective = (s[:,17]+s[:,18])*.5
        differential = (s[:,17]-s[:,18])*.5
        rates = s[:,14:17]
        normalized = rates/clip(2*speed,4,None)[:,None]
        span,chord = p['wingspan_m'],p['mean_chord_m']
        moment = stack([
            qs*span*(-.12*differential-.45*normalized[:,0]*span),
            qs*chord*(p.get('pitch_stability',.35)*alpha-.7*collective-p.get('pitch_damping',4.)*normalized[:,1]*chord),
            qs*span*(p.get('yaw_stability',.10)*beta-p.get('yaw_damping',.35)*normalized[:,2]*span)])
        rates += subdt*(moment-cross(rates,rates*inertia))/inertia
        q = s[:,7:11].copy() if ops is np else s[:,7:11].clone()
        qw,qv = q[:,:1],q[:,1:]
        s[:,7] += subdt*(-.5*(qv*rates).sum(-1))
        s[:,8:11] += subdt*.5*(qw*rates+cross(qv,rates))
        s[:,7:11] /= clip((s[:,7:11]**2).sum(-1)**.5,.001,None)[:,None]
    # Rail imposes position and orientation only during the two-second launch.
    launch = world['launcher']
    duration = 2*launch['length_m']/launch['exit_speed_mps']
    active = time_s < duration-1e-6
    t = clip(time_s+dt,0,duration)
    speed = launch['exit_speed_mps']*t/duration
    start = initial(world)
    h,g = launch['heading_rad'],launch['pitch_rad']
    direction = [np.cos(g)*np.cos(h),np.cos(g)*np.sin(h),np.sin(g)]
    for axis in range(3):
        s[:,axis] = ops.where(active,start[axis]+.5*launch['exit_speed_mps']/duration*t*t*direction[axis],s[:,axis])
        s[:,11+axis] = ops.where(active,speed*direction[axis],s[:,11+axis])
        s[:,14+axis] = ops.where(active,0.,s[:,14+axis])
    for axis in range(7,11):
        s[:,axis] = ops.where(active,start[axis],s[:,axis])
    forward,left,up = basis(s[:,7:11],ops)
    s[:,3] = atan2(forward[:,1],forward[:,0])
    s[:,4] = (s[:,11:14]**2).sum(-1)**.5
    s[:,5] = -atan2(left[:,2],up[:,2])
    s[:,6] = atan2(s[:,13],(s[:,11:13]**2).sum(-1)**.5)
    return s
