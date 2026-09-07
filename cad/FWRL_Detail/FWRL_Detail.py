"""Fusion-native packaging study. Source stations: fwrl/airframe.py, metres.

All helper coordinates below are mm; Fusion API units are cm. Re-run creates
a NEW document. Bays are concept envelopes, not confirmed hardware fits.
"""
import adsk
import adsk.core as C
import adsk.fusion as F
import json, math, traceback
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / 'output_R02'
RINGS = [(1150,15,18),(1100,37,38),(940,59,54),(700,85,67),
         (400,110,82),(120,130,100),(-150,130,100),(-420,108,90),
         (-650,92,82),(-880,80,72),(-1100,65,60),(-1310,52,50)]
SHELL = 3.0
WING_SHELL = 2.0
GAP = 0.5

def p(x,y,z): return C.Point3D.create(x/10,y/10,z/10)
def v(x,y,z): return C.Vector3D.create(x,y,z)
def log(s):
    with (OUT/'build.log').open('a',encoding='utf8') as f: f.write(str(s)+'\n')
    adsk.doEvents()

def box(x0,x1,y0,y1,z0,z1):
    return tmp.createBox(C.OrientedBoundingBox3D.create(
        p((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),v(1,0,0),v(0,1,0),
        (x1-x0)/10,(y1-y0)/10,(z1-z0)/10))

def cyl(a,b,r): return tmp.createCylinderOrCone(p(*a),r/10,p(*b),r/10)
def boolean(a,b,op):
    if not tmp.booleanOperation(a,b,op): raise RuntimeError('Boolean failed')
    return a
def cut(a,b): return boolean(a,b,F.BooleanTypes.DifferenceBooleanType)
def intersect(a,b): return boolean(a,b,F.BooleanTypes.IntersectionBooleanType)
def union(a,b): return boolean(a,b,F.BooleanTypes.UnionBooleanType)
def component(name):
    occ=root.occurrences.addNewComponent(C.Matrix3D.create())
    occ.component.name=name
    return occ.component

def add(comp,body,name,color='shell'):
    if not body or not body.isSolid or body.volume<=0: raise RuntimeError('Invalid solid: '+name)
    b=comp.bRepBodies.add(body); b.name=name
    if color in appearances: b.appearance=appearances[color]
    return b

def loft(sections,axis,name):
    # sections = world center, world major-radius vector, world minor-radius vector
    sketches=[]
    planes=[]
    base={'x':root.yZConstructionPlane,'y':root.xZConstructionPlane,'z':root.xYConstructionPlane}[axis]
    for center,major,minor in sections:
        ci=root.constructionPlanes.createInput()
        # xZ construction plane normal is -Y.
        offset=center['xyz'.index(axis)] * (-1 if axis=='y' else 1)
        ci.setByOffset(base,C.ValueInput.createByReal(offset/10))
        plane=root.constructionPlanes.add(ci); planes.append(plane)
        sk=root.sketches.add(plane); sk.name=name+' section'
        a=sk.modelToSketchSpace(p(*center))
        b=sk.modelToSketchSpace(p(*[center[i]+major[i] for i in range(3)]))
        c=sk.modelToSketchSpace(p(*[center[i]+minor[i] for i in range(3)]))
        sk.sketchCurves.sketchEllipses.add(a,b,c)
        if sk.profiles.count!=1: raise RuntimeError('Ellipse profile failure '+name)
        sketches.append(sk)
    li=root.features.loftFeatures.createInput(F.FeatureOperations.NewBodyFeatureOperation)
    li.isSolid=True
    for sk in sketches: li.loftSections.add(sk.profiles.item(0))
    feat=root.features.loftFeatures.add(li)
    body=tmp.copy(feat.bodies.item(0))
    feat.bodies.item(0).deleteMe()
    for sk in sketches: sk.deleteMe()
    for plane in planes: plane.deleteMe()
    return body

def fuselage(inset=0):
    return loft([((x,0,20),(0,ry*1.16-inset,0),(0,0,rz*1.06-inset))
                 for x,ry,rz in reversed(RINGS)],'x','Fuselage')

def naca(u,m=.02,pc=.4,t=.15):
    # Standard NACA four-digit equations, finite trailing edge (-0.1015).
    yt=5*t*(.2969*math.sqrt(u)-.126*u-.3516*u*u+.2843*u**3-.1015*u**4)
    if m==0: yc=dy=0
    elif u<pc: yc=m/pc**2*(2*pc*u-u*u); dy=2*m/pc**2*(pc-u)
    else: yc=m/(1-pc)**2*((1-2*pc)+2*pc*u-u*u); dy=2*m/(1-pc)**2*(pc-u)
    th=math.atan(dy)
    return (u-yt*math.sin(th),yc+yt*math.cos(th)),(u+yt*math.sin(th),yc-yt*math.cos(th))

def profile_loft(sections,axis,name,m=.02,t=.15,inset=0):
    # Each section: span coordinate, leading X, trailing X, chord-line Z/Y.
    sketches=[]; planes=[]
    for span,le,te,height in sections:
        ci=root.constructionPlanes.createInput()
        ci.setByOffset(root.xZConstructionPlane if axis=='y' else root.xYConstructionPlane,
                       C.ValueInput.createByReal(span/10))
        plane=root.constructionPlanes.add(ci); planes.append(plane)
        sk=root.sketches.add(plane); sk.name=name+' NACA section'
        sk.areProfilesShown=True
        log('Plane '+str(span)+' actual '+str(sk.sketchToModelSpace(C.Point3D.create(0,0,0)).asArray()))
        chord=le-te
        # Inner envelope is a reduced profile, not a constant normal wall offset.
        effective=max(.008,t-2*inset/chord)
        upper=[]; lower=[]
        for i in range(49):
            u=(1-math.cos(math.pi*i/48))/2
            up,lo=naca(u,m,.4,effective)
            for curve,pt in [(upper,up),(lower,lo)]:
                x=le-inset-(chord-2*inset)*pt[0]
                h=height+chord*pt[1]
                point=sk.modelToSketchSpace(p(x,span,h) if axis=='y' else p(x,h,span))
                point.z=0
                curve.append(point)
        pts=C.ObjectCollection.create()
        for pt in list(reversed(upper))+lower[1:]: pts.add(pt)
        spline=sk.sketchCurves.sketchFittedSplines.add(pts)
        sk.sketchCurves.sketchLines.addByTwoPoints(spline.endSketchPoint,spline.startSketchPoint)
        sk.isComputeDeferred=False
        adsk.doEvents()
        log('NACA section '+str(span)+' profiles '+str(sk.profiles.count))
        if sk.profiles.count!=1: raise RuntimeError('NACA profile failure '+name+' '+str(sk.profiles.count))
        sketches.append(sk)
    li=root.features.loftFeatures.createInput(F.FeatureOperations.NewBodyFeatureOperation)
    li.isSolid=True
    for sk in sketches: li.loftSections.add(sk.profiles.item(0))
    feat=root.features.loftFeatures.add(li)
    body=tmp.copy(feat.bodies.item(0)); feat.bodies.item(0).deleteMe()
    for sk in sketches: sk.deleteMe()
    for plane in planes: plane.deleteMe()
    return body

def wing(sign,inset=0):
    return profile_loft([(sign*q*1350,(.55*(.20-.50*q-.07*q*q)-.35*q-.06)*1000,
                          (.55*(-.445-.01*q)-.35*q-.06)*1000,8)
                         for q in [0,.13,.26,.39,.52,.65,.78]],'y','Wing NACA 2415',inset=inset)

def control_cutter(sign,gap=0):
    # Swept hinge plane matching the source elevon. A wide oriented box
    # keeps the control surface an exact portion of the parent airfoil.
    direction=v(-.35,sign*1.35,0); direction.normalize()
    normal=v(-1.35,-sign*.35,0); normal.normalize()
    q=.495
    center=p(-244.25-.35*q*1000,sign*q*1350,8)
    center.translateBy(C.Vector3D.create(normal.x*(100+gap)/10,normal.y*(100+gap)/10,0))
    body=tmp.createBox(C.OrientedBoundingBox3D.create(center,direction,normal,
                     math.sqrt(.35**2+1.35**2)*490/10,20,20))
    yr=sorted([sign*(337.5+gap),sign*(999-gap)])
    return intersect(body,box(-1000,300,*yr,-90,110))

def check_airfoil(body,sign):
    errors=[]
    for q in [.13,.195,.26,.325,.39,.455,.52,.585,.65,.715]:
        le=(.55*(.20-.50*q-.07*q*q)-.35*q-.06)*1000
        chord=.55*(.645-.49*q-.07*q*q)*1000
        for u in [.05,.1,.2,.3,.4,.6,.8,.95]:
            for side,(xx,zz) in enumerate(naca(u)):
                x=le-chord*xx; target=8+chord*zz
                a=target-1; b=target+1
                for _ in range(22):
                    h=(a+b)/2
                    inside=body.pointContainment(p(x,sign*q*1350,h))==F.PointContainment.PointInsidePointContainment
                    if inside==(side==0): a=h
                    else: b=h
                errors.append(abs((a+b)/2-target))
    return {'sample_count':len(errors),'max_vertical_error_mm':max(errors),
            'rms_vertical_error_mm':math.sqrt(sum(e*e for e in errors)/len(errors))}

def hatch(shell,comp,name,bounds,backing=None,framecomp=None,fasten=False):
    x0,x1,y0,y1,z0,z1=bounds
    lid=intersect(tmp.copy(shell),box(x0+GAP,x1-GAP,y0+GAP,y1-GAP,z0,z1))
    cut(shell,box(*bounds))
    if backing is not None:
        frame=intersect(tmp.copy(backing),box(x0-7,x1+7,y0-7,y1+7,z0,z1))
        cut(frame,box(x0+6,x1-6,y0+6,y1-6,z0-1,z1+1))
        if fasten:
            for xx in [x0+12,x1-12]:
                for yy in [y0+8,y1-8]:
                    zz=fus_top(xx,yy)
                    hole=cyl((xx,yy,zz-18),(xx,yy,zz+5),1.7)
                    cut(lid,hole); cut(frame,hole)
                    boss=cyl((xx,yy,zz-14),(xx,yy,zz-3),4)
                    cut(boss,cyl((xx,yy,zz-15),(xx,yy,zz),1.3))
                    add(framecomp,boss,name+' screw boss - pilot 2.6','tray')
                    head=cyl((xx,yy,zz+.1),(xx,yy,zz+2.1),2.8)
                    cut(head,box(xx-3,xx+3,yy-.55,yy+.55,zz+1.2,zz+3))
                    add(comp,head,name+' M3 screw head','metal')
        add(framecomp,frame,name+' internal seating flange','tray')
    add(comp,lid,name,'lid')

def fus_top(x,y):
    lo=20.; hi=170.
    for _ in range(32):
        mid=(lo+hi)/2
        if fus_outer.pointContainment(p(x,y,mid))==F.PointContainment.PointInsidePointContainment: lo=mid
        else: hi=mid
    return (lo+hi)/2


def tray(comp,name,bounds,holes=(),slots=()):
    t=box(*bounds)
    for x,y,r in holes: cut(t,cyl((x,y,bounds[4]-1),(x,y,bounds[5]+1),r))
    for bb in slots: cut(t,box(*bb))
    return add(comp,t,name,'tray')

def tube(comp,name,a,b,outer,inner,color='metal'):
    t=cyl(a,b,outer)
    cut(t,cyl(a,b,inner))
    return add(comp,t,name,color)

def run(context):
    global app,design,root,tmp,appearances,fus_outer
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'build.log').write_text('FWRL packaging concept build\n')
    try:
        app=C.Application.get()
        doc=app.documents.add(C.DocumentTypes.FusionDesignDocumentType)
        design=F.Design.cast(app.activeProduct)
        design.designType=F.DesignTypes.DirectDesignType
        root=design.rootComponent
        root.attributes.add('FWRL','title','SYNTHOSAR FWRL - packaging concept R02')
        tmp=F.TemporaryBRepManager.get()
        appearances={}
        base=None
        for lib in app.materialLibraries:
            if lib.appearances.count:
                base=next((a for a in lib.appearances if a.name=='Plastic - Matte (Black)'),None)
                if base: break
        if base:
            for name,rgb in {'shell':(140,153,159),'lid':(69,112,125),'tray':(221,159,53),
                             'metal':(164,172,177),'rubber':(26,28,30)}.items():
                ap=design.appearances.addByCopy(base,'FWRL '+name)
                prop=ap.appearanceProperties.itemById('opaque_albedo')
                if prop: prop.value=C.Color.create(*rgb,255)
                appearances[name]=ap
        shellcomp=component('01 Fuselage shell - hollow 3 mm nominal')
        covers=component('02 Removable access covers - hide to inspect cavities')
        fit=component('03 Equipment trays and mounting interfaces')
        log('Building original fuselage stations')
        outer=fuselage()
        fus_outer=tmp.copy(outer)
        inner_fuse=fuselage(SHELL)
        shell=cut(tmp.copy(outer),inner_fuse)
        backing=cut(tmp.copy(inner_fuse),fuselage(SHELL+4))
        # Nose splits at x=940, preserving external shape.
        nose=intersect(tmp.copy(shell),box(940+GAP,1200,-250,250,-200,250))
        cut(shell,box(940,1200,-250,250,-200,250))
        # Open lens bore and hollow rear motor access.
        cut(nose,cyl((1090,0,20),(1170,0,20),14))
        add(covers,nose,'Detachable nose - camera access','lid')
        hatch(shell,covers,'Battery cover', (330,820,-62,62,20,200),backing,fit,True)
        hatch(shell,covers,'FC and receiver cover',(-330,80,-82,82,25,200),backing,fit,True)
        hatch(shell,covers,'ESC service cover',(-830,-530,-49,49,20,200),backing,fit,True)
        # Wing connector and cable pass-throughs, separate power routing.
        for sign in [-1,1]:
            cut(shell,cyl((-230,sign*90,8),(-230,sign*175,8),7))
        cut(shell,cyl((-1320,0,20),(-1270,0,20),20))
        add(shellcomp,shell,'Open hollow fuselage')
        # Removable battery sled: two 6S envelopes, 12S electrically is only an assumption.
        tray(fit,'Battery sled - 420 x 110; strap slots',(360,780,-55,55,-24,-20),
             slots=[(x,x+22,y,y+7,-26,-18) for x in [410,650] for y in [-48,41]])
        for y in [-58,55]: tray(fit,'Battery lateral guide',(360,780,y,y+3,-24,-8))
        for x in [355,780]: tray(fit,'Battery stop',(x,x+5,-58,58,-24,-8))
        tray(fit,'FC isolated platform - 30.5 and 20 mm patterns',(-220,-100,-60,60,-18,-14),
             holes=[(-160+dx*s/2,dy*s/2,1.6) for s in [20,30.5] for dx in [-1,1] for dy in [-1,1]])
        for dx in [-1,1]:
            for dy in [-1,1]:
                tube(fit,'FC M3 standoff',(-160+dx*15.25,dy*15.25,-14),(-160+dx*15.25,dy*15.25,-4),3,1.6)
        tray(fit,'Receiver / GNSS auxiliary tray',(-70,35,-40,40,-18,-14))
        tray(fit,'ESC tray with ventilation',(-795,-565,-42,42,-12,-8),
             slots=[(x,x+8,-29,29,-14,-6) for x in range(-775,-585,24)])
        # Tray support beams span into inner fuselage walls; ribs with cable openings.
        for x,half,z in [(375,123,-24),(760,91,-24),(-210,146,-18),(-110,147,-18),(-780,97,-12),(-570,108,-12)]:
            beam=intersect(box(x,x+5,-half,half,z-7,z),inner_fuse)
            add(fit,beam,'Tray support crossbeam trimmed to fuselage','tray')
        # Camera cradle and lens ring, mounting axis along X.
        tray(fit,'Camera cradle floor',(970,1035,-24,24,-3,1))
        for y in [-27,24]:
            cheek=box(970,1020,y,y+3,-3,39)
            cut(cheek,cyl((995,y-1,22),(995,y+4,22),1.6))
            add(fit,cheek,'Camera tilt bracket M3','tray')
        collar=intersect(tmp.copy(backing),box(925,953,-100,100,-100,120))
        add(fit,collar,'Conformal nose locating collar','tray')
        # Motor firewall retained at original rear face.
        firewall=cyl((-1308,0,20),(-1302,0,20),48)
        cut(firewall,cyl((-1310,0,20),(-1300,0,20),20))
        for y,z in [(-15,-15),(-15,15),(15,-15),(15,15)]:
            cut(firewall,cyl((-1310,y,20+z),(-1300,y,20+z),1.7))
        add(fit,firewall,'Motor firewall - provisional 30 mm square M3.4 pattern','metal')
        prop=component('08 Pusher propeller envelope - visual only')
        tube(prop,'Motor envelope',(-1302,0,20),(-1350,0,20),25,8,'rubber')
        add(prop,cyl((-1370,0,20),(-1350,0,20),14),'Prop hub','metal')
        for sg in [-1,1]:
            blade=loft([((-1360,sg*q,20+sg*zz),(18*scale,0,0),(0,0,3*scale))
                        for q,zz,scale in [(12,0,.7),(80,26,1),(170,15,.55),(185,0,.15)]], 'y','Prop blade')
            add(prop,blade,'Propeller visual blade','rubber')
        log('Building swept hollow wings and service interfaces')
        foil_checks={}; internal_checks=[]
        for sign,label in [(1,'Left'),(-1,'Right')]:
            wc=component('04 '+label+' NACA 2415 wing')
            mc=component('05 '+label+' wing mechanisms')
            w=wing(sign)
            full_wing=tmp.copy(w)
            foil_checks[label]=check_airfoil(full_wing,sign)
            log('Wing bounds '+str(w.boundingBox.minPoint.asArray())+' '+str(w.boundingBox.maxPoint.asArray()))
            control=intersect(tmp.copy(w),control_cutter(sign,.5))
            cut(w,control_cutter(sign,-.5))
            inner=wing(sign,WING_SHELL)
            wing_backing=cut(tmp.copy(inner),wing(sign,WING_SHELL+2))
            # Keep root/tip end walls, remove inner volume.
            yr=sorted([sign*148,sign*1049])
            intersect(inner,box(-1000,300,*yr,-100,100))
            cut(w,inner)
            # Remove buried center portion from detachable outboard wing.
            yr=sorted([0,sign*1100])
            intersect(w,box(-1000,300,*yr,-100,100))
            cut(w,fus_outer)
            cy=sign*580.5
            hatch(w,covers,label+' servo lid',(-367,-317,cy-23,cy+23,8,100),wing_backing,mc)
            ay=sign*700
            hatch(w,covers,label+' underside accessory cover',(-401,-341,ay-25,ay+25,-100,8),wing_backing,mc)
            # Open wiring duct at root and into servo cavity.
            cut(w,cyl((-230,sign*140,8),(-230,sign*180,8),7))
            add(wc,w,label+' smooth NACA 2415 wing shell')
            tray(mc,label+' servo pocket floor - provisional low profile',(-365,-319,cy-19,cy+19,0,3))
            for xx in [-365,-322]: tray(mc,'Servo pocket wall',(xx,xx+3,cy-19,cy+19,3,14))
            # Spar follows sweep inside main wing, stops before thin tip.
            tube(mc,'Swept main spar tube',(-127,sign*155,12),(-486,sign*940,10),4,2.5)
            tube(mc,'Wing root pin sleeve',(-245,sign*145,9),(-245,sign*225,9),6,4.2)
            tube(mc,'Root locating pin',(-290,sign*138,7),(-290,sign*185,7),3,1.5)
            tube(mc,'Electrical harness conduit',(-230,sign*155,8),(-335,sign*550,8),4.5,3.5,'tray')
            # Control surface is cut directly from the parent NACA solid.
            add(mc,control,label+' airfoil-matched elevon','lid')
            for q in [.28,.48,.70]:
                x=(.55*(-.335)-.35*q-.06)*1000
                add(mc,cyl((x,sign*q*1350,8),(x-.35*12,sign*(q*1350+16.2),8),2),'Hinge pin','metal')
            add(mc,cyl((-343,cy,23),(-421,cy,31),1.5),'Elevon pushrod','metal')
            horn=box(-425,-421,cy-3,cy+3,8,33)
            cut(horn,cyl((-427,cy,29),(-419,cy,29),1.6))
            add(mc,horn,'Control horn','tray')
            # Small accessory well inboard enough to retain depth.
            tray(mc,'Auxiliary payload pad - 24 mm M2 pattern',(-393,-349,ay-18,ay+18,0,3),
                 holes=[(-371+dx*12,ay+dy*12,1.1) for dx in [-1,1] for dy in [-1,1]])
            for bd in mc.bRepBodies:
                if any(word in bd.name for word in ['spar tube','pocket floor','pocket wall','payload pad','harness conduit']):
                    remainder=cut(tmp.copy(bd),full_wing)
                    internal_checks.append({'side':label,'part':bd.name,'outside_wing_cm3':remainder.volume})
        tail=component('06 NACA 0010 conventional tail')
        for sign in [-1,1]:
            add(tail,profile_loft([(sign*q,-940-200*q/340,-1200-50*q/340,80)
                                   for q in [0,85,170,255,340]],'y','Horizontal tail',m=0,t=.10),
                'NACA 0010 horizontal stabilizer '+str(sign))
        add(tail,profile_loft([(40+340*t,-1000-120*t,-1230-45*t,0)
                               for t in [0,.25,.5,.75,1]],'z','Vertical tail',m=0,t=.10),
            'NACA 0010 vertical stabilizer')
        belly=component('07 Belly launch interface')
        tray(belly,'Launch hook load-spreader concept',(430,510,-25,25,-52,-47))
        hook=box(463,477,-4,4,-70,-45)
        cut(hook,cyl((470,-6,-62),(470,6,-62),4))
        add(belly,hook,'Launch lug - load sizing required','metal')
        # Completed model validation and independent local exports.
        log('Validating and exporting')
        report={'status':'packaging concept; hardware and structural fit unverified',
                'source':'fwrl/airframe.py','units':'mm','span_mm':2106,
                'fuselage_length_mm':2460,'nominal_shell_mm':SHELL,
                'wing_airfoil':'NACA 2415 finite trailing edge','tail_airfoil':'NACA 0010',
                'wing_inner_profile_reduction_mm':WING_SHELL,'hatch_clearance_per_side_mm':GAP,'bodies':[]}
        report['airfoil_surface_checks']=foil_checks
        report['internal_envelope_checks']=internal_checks
        for co in root.allOccurrences:
            co.component.isSketchFolderLightBulbOn=False
            co.component.isConstructionFolderLightBulbOn=False
            for b in co.component.bRepBodies:
                bb=b.boundingBox
                report['bodies'].append({'component':co.component.name,'name':b.name,
                                        'solid':b.isSolid,'volume_cm3':b.volume,
                                        'min_mm':[n*10 for n in bb.minPoint.asArray()],
                                        'max_mm':[n*10 for n in bb.maxPoint.asArray()]})
        if not all(b['solid'] and b['volume_cm3']>0 for b in report['bodies']):
            raise RuntimeError('Invalid output body')
        (OUT/'validation.json').write_text(json.dumps(report,indent=2))
        camera=app.activeViewport.camera
        camera.viewOrientation=C.ViewOrientations.IsoTopRightViewOrientation
        camera.isFitView=True
        app.activeViewport.camera=camera
        app.activeViewport.fit()
        app.activeViewport.visualStyle=C.VisualStyles.ShadedVisualStyle
        app.activeViewport.saveAsImageFile(str(OUT/'assembled.png'),1800,1400)
        exp=design.exportManager
        exp.execute(exp.createFusionArchiveExportOptions(str(OUT/'FWRL_packaging_R02.f3d')))
        exp.execute(exp.createSTEPExportOptions(str(OUT/'FWRL_packaging_R02.step')))
        for b in covers.bRepBodies: b.isLightBulbOn=False
        app.activeViewport.refresh()
        app.activeViewport.saveAsImageFile(str(OUT/'cavities.png'),1800,1400)
        exp.execute(exp.createFusionArchiveExportOptions(str(OUT/'FWRL_packaging_R02_open.f3d')))
        camera=app.activeViewport.camera
        camera.viewOrientation=C.ViewOrientations.BottomViewOrientation
        camera.isFitView=True
        app.activeViewport.camera=camera
        camera=app.activeViewport.camera
        camera.isFitView=False
        camera.viewExtents*=1.5
        app.activeViewport.camera=camera
        app.activeViewport.saveAsImageFile(str(OUT/'underside.png'),1800,1400)
        camera.viewOrientation=C.ViewOrientations.IsoTopRightViewOrientation
        camera.isFitView=True
        app.activeViewport.camera=camera
        log('SUCCESS - '+str(len(report['bodies']))+' solid bodies; F3D and STEP exported')
    except Exception:
        log(traceback.format_exc())
        if app: app.userInterface.messageBox('FWRL build stopped. See cad/output/build.log\n'+traceback.format_exc())
