"""Shared reference-inspired flying-wing mesh, body X forward, Y left, Z up."""
import math


def geometry():
    parts=[]
    def add(name,vertices,faces,color=(.055,.065,.063),pivot=(0,0,0),parent=None):
        triangles=[]
        for face in faces:
            for i in range(1,len(face)-1):triangles.append([face[0],face[i],face[i+1]])
        parts.append(dict(name=name,vertices=vertices,faces=triangles,color=color,pivot=pivot,parent=parent))
    # Smooth tapered center body: long rounded nose, broad wing blend, rear motor.
    rings=[(1.15,.015,.018),(1.10,.037,.038),(.94,.059,.054),(.70,.085,.067),
           (.40,.11,.082),(.12,.13,.10),(-.15,.13,.10),(-.42,.108,.09),
           (-.65,.092,.082),(-.88,.080,.072),(-1.10,.065,.060),(-1.31,.052,.050)]
    verts=[];faces=[];n=32
    for x,ry,rz in rings:
        for i in range(n):
            a=i*math.tau/n;verts.append([x,1.16*ry*math.cos(a),.02+1.06*rz*math.sin(a)])
    for j in range(len(rings)-1):
        for i in range(n):faces.append([j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i])
    faces += [list(reversed(range(n))),list(range((len(rings)-1)*n,len(rings)*n))]
    add('Blended fuselage',verts,faces)
    # Continuous swept outline; hinge cutouts do not change the outer silhouette.
    def leading(y): return .20-.50*y-.07*y*y
    def trailing(y): return -.445-.01*y
    stations=[i*.78/64 for i in range(65)]
    stations=sorted(set(stations+[.2499,.25,.74,.7401]))
    def loft(name,sections,color):
        verts=[];faces=[];n=48
        # section: leading/trailing X, Y/Z center, thickness direction, thickness
        for le,te,y,z,ny,nz,th in sections:
            for i in range(n):
                a=i*math.tau/n;t=(1-math.cos(a))/2
                verts.append([le+(te-le)*t,y+ny*th*math.sin(a),z+nz*th*math.sin(a)])
        for j in range(len(sections)-1):
            for i in range(n):faces.append([j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i])
        faces.extend([list(reversed(range(n))),list(range((len(sections)-1)*n,len(sections)*n))])
        add(name,verts,faces,color)
    for sign,label in [(1,'Left'),(-1,'Right')]:
        sections=[]
        for y in stations:
            te=-.335 if .25<=y<=.74 else trailing(y)
            sections.append((leading(y),te,sign*y,.008,0,1,.045*(1-y/.90)))
        loft(label+' rounded swept wing',sections,(.055,.065,.063))
        add(label+' elevon',[[0,0,.008],[0,sign*.49,.008],
            [trailing(.74)+.335,sign*.49,.008],[trailing(.25)+.335,0,.008]],
            [[0,1,2,3]],(.16,.19,.20),(-.335,sign*.25,0))
        add(label+' control horn',[[-.06,sign*.18,.012],[-.025,sign*.18,.012],
            [-.055,sign*.18,.065]],[[0,1,2]],(.75,.79,.81),(0,0,0),label+' elevon')
        verts=[[x,sign*.43+y,z] for z in [.027,.057] for y in [-.018,.018] for x in [-.25,-.19]]
        add(label+' servo',verts,[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]],(.12,.14,.17))

    verts=[]
    for sign in [-1,1]:
        for y,z in [(0,0),(.08,.026),(.17,.015),(.185,0),(.10,-.013)]:
            verts.append([0,sign*y,sign*z])
    add('Rear propeller',verts,[[0,1,2,3,4],[5,6,7,8,9]],(.025,.029,.03),(-1.36,0,.02))
    # Small dark lens disc at the forward nose.
    verts=[[1.160,.014*math.cos(i*math.tau/24),.02+.014*math.sin(i*math.tau/24)] for i in range(24)]
    add('Nose camera',verts,[list(range(24))],(.005,.008,.011))
    # Conventional tail surfaces intersect the extended fuselage directly.
    tail=[]
    for j in range(33):
        y=-.34+j*.68/32
        sweep=.07*abs(y)/.34
        tail.append((-.99-sweep,-1.18-sweep,y,.08,0,1,.009*(1-.65*abs(y)/.34)))
    loft('Fixed horizontal stabilizer',tail,(.09,.12,.14))
    fin=[]
    for j in range(25):
        t=j/24;z=.04+.34*t
        fin.append((-1.00-.12*t,-1.23-.045*t,0,z,1,0,.008*(1-.85*t)))
    loft('Fixed vertical stabilizer',fin,(.10,.13,.15))
    # Sweep both edges aft while reducing chord: a tapered swept wing, not delta.
    origins={p['name']:p['pivot'] for p in parts}
    def swept(v,sign): return [.55*v[0]-.35*sign*v[1]+.06,v[1],v[2]]
    for part in parts:
        if not part['name'].startswith(('Left','Right')): continue
        sign=1 if part['name'].startswith('Left') else -1
        origin=origins[part['parent']] if part.get('parent') else [0,0,0]
        # Child control horns are expressed relative to their elevon's pivot.
        for v in part['vertices']:
            if part.get('parent'):
                v[0]=.55*v[0]-.35*sign*v[1]
            else:
                v[0]=.55*v[0]-.35*sign*v[1]
        part['pivot']=([.55*part['pivot'][0]-.35*sign*part['pivot'][1]+.06,part['pivot'][1],part['pivot'][2]]
                       if not part.get('parent') else part['pivot'])
        for v in part['vertices']: v[1]*=1.35
        part['pivot']=list(part['pivot'])
        part['pivot'][1]*=1.35
        if not part.get('parent'): part['pivot'][0]-=.12
        if part['name'].endswith('elevon'):
            norm=math.hypot(.35,1.35)
            part['hinge_axis']=[-.35*sign/norm,1.35/norm,0]
    return parts
