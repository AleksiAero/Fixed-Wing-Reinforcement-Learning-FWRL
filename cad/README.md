# FWRL equipment packaging concept

This assembly reconstructs the existing drone from the station definitions and
coordinate transforms in `fwrl/airframe.py`, which also generate its Blender mesh.
The Mojito images informed service access and equipment organization only. The
FWRL long fuselage, swept wings, conventional tail and rear pusher are retained.

## R02 airfoil and proportions

The source planform is preserved: 2106 mm span, 354.75 mm centerline chord,
121.117 mm tip chord, 680 mm horizontal-tail span and 2460 mm fuselage length.
The fuselage is unusually long relative to span; preserving it follows the
request to keep the existing design. These dimensions are geometric references,
not evidence of aerodynamic optimization. No flight-dynamics coefficients were changed.
The preserved gross wing planform area is 0.5093 m2 (including the centerline
portion hidden by the fuselage), giving a gross aspect ratio of 8.71.
Zero geometric twist and the original wing chord-line height are retained.
Profile choice, incidence and washout require analysis at the intended flight
Reynolds numbers, loading and trim condition.

## Files

- `output_R02/FWRL_packaging_R02.f3d`: complete assembled Fusion solid model.
- `output_R02/FWRL_packaging_R02_open.f3d`: same assembly with access covers hidden.
- `output_R02/FWRL_packaging_R02.step`: neutral solid assembly, covers installed.
- `output_R02/assembled.png`, `cavities.png`, `underside.png`: Fusion viewport images.
- `output_R02/validation.json`: solid body inventory and measured positive volumes.
- `FWRL_Detail/FWRL_Detail.py`: editable, repeatable Fusion Python builder.

Open the F3D locally in Fusion. In its Browser, toggle component **02 Removable
access covers** to reveal the battery, controller, ESC, camera and wing bays.
The assembly has named components and editable BRep solids. It uses direct
modeling, not a dimension-driven feature timeline. Change helper dimensions in
the builder and rerun it to regenerate geometry in a new document; changing a
Fusion parameter does not automatically regenerate this model.

## Packaging dimensions (mm)

Body axes match the simulation: X forward, Y left, Z up. Dimensions refer to
the conceptual interfaces, not guaranteed usable hardware envelopes.

| Interface | Geometry provided |
|---|---|
| Main wing | 2106 overall span; original sweep and taper |
| Fuselage | Original 2460 length, approximately 301.6 maximum width |
| Shells | Fuselage radii reduced by 3; wing internal NACA envelope reduced by 2 (not a constant normal offset) |
| Battery | 420 x 110 sled; 490 x 124 top opening; two strap stations |
| Flight controller | 120 x 120 platform; 20 and 30.5 square mounting patterns; 3.2 holes |
| Controller access | 410 x 164 top opening |
| ESC | 230 x 84 slotted tray; 300 x 98 top opening |
| Camera | 48 wide cradle; two tilt cheeks with 3.2 mounting holes; 28 lens bore |
| Wing servos | 40 x 38 clear space between side walls; 50 x 46 access covers |
| Wing accessories | 44 x 36 mounting pads; 24 square M2 clearance pattern |
| Motor firewall | 96 diameter; 40 center clearance; 30 square, 3.4 hole pattern |
| Cover seams | 0.5 nominal clearance per side in X/Y |

## Details and limitations

The model includes actual hollow bodies, removable nose, separate access covers,
internal seating flanges, pilot-drilled screw bosses for fuselage covers, strap
slots and stops, controller standoffs, camera tilt bracket, ventilated ESC tray,
wing spar and pin sleeves, hollow harness conduits, separate elevons, hinge pins,
control horns, pushrods and a belly launch lug. Screw heads are visual simplified
solids. Threads, inserts and complete fastener stacks are not modeled.

Hardware has not been specified. The battery arrangement is a provisional sled
consistent with the project's existing 12S assumption; battery capacity, actual
pack dimensions and electrical wiring are not established. Mounting patterns
are placeholders. No proprietary Mojito parts are required.

This is a packaging concept, not a fabrication release. Section radius reduction
is not a true constant-normal shell offset. Nominal shell figures therefore do
not imply uniform wall thickness, especially around leading/trailing edges.
R02 replaces the visual elliptical sections with equation-generated NACA 2415
wing profiles and symmetric NACA 0010 tail profiles. The wing has 2% camber at
40% chord and 15% nominal thickness; tail thickness is 10% of chord.
The standard finite-trailing-edge coefficient -0.1015 is used. Each section
has 97 cosine-distributed points on a fitted spline and a trailing-edge closure.
Seven sections form each continuous wing loft. The elevons are separated from
the parent wing so their surfaces match. Fuselage sections now use a continuous
loft instead of separate straight spans. The reference propeller remains a visual envelope.
These are defined airfoil geometries, not a validated aerodynamic selection.

The next engineering pass needs actual hardware dimensions and masses, material
and manufacturing method, center-of-gravity calculation, continuous spar load
path and wing retention design, cover fastener/insert selection, tolerances,
servo travel and linkage clearance, thermal design, motor/propeller clearance
and launch-load analysis. The modeled root sleeves and pins locate components;
they are not a completed quick-release lock. Servo and accessory cover retention
still needs final hardware. The model is not print-ready or flight-validated.

Validation checks positive-volume closed solids, export completion and visual
layout. It does not certify assembly interference, mounting strength, equipment
fit, water resistance, electrical compatibility or aerodynamics.

## Regenerate

In Fusion, use Utilities > Scripts and Add-ins > + > Script or add-in from device.
Select the `FWRL_Detail` folder and run `FWRL_Detail`. Outputs go to `cad/output_R02`.
Each run creates a new document and replaces the generated exports. Existing
Blender worlds and simulation source files are not modified.

API references used: Autodesk's [loft feature reference](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/LoftFeatures_createInput.htm)
and the locally installed Autodesk Python API definitions.

Airfoil definition references: [NASA NACA designation](https://www.nasa.gov/wp-content/uploads/2023/04/sp-4409-vol2.pdf) and [NASA ordinate generation report](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760003945.pdf).


## R02 verification

Fusion exported 102 positive-volume closed solid bodies. The surface audit
compared 160 points on each full wing, at both loft stations and intermediate
stations, with the NACA equations. Maximum sampled vertical deviation was
0.000578 mm; this measures CAD representation accuracy, not manufacturing
accuracy or aerodynamic performance. Both wing bounding boxes reproduce the
2106 mm span. Left/right shell volumes differ by about 0.025% after booleans.

Twelve envelope checks found zero volume outside the full wing for the servo
floors/walls, spar tubes, wiring conduits and accessory pads. These checks do
not establish clearance from the inner skin, fasteners, each other, or actual
hardware. Full manufacturing and assembly interference review remains open.

The package includes normalized NACA2415.dat and NACA0010.dat coordinates,
proportions.json, and the Fusion validation report.

## Wing-root correction

Spider v2 was updated in Fusion to close the visible wing-root gaps. Both wing
shells now extend to, and are trimmed against, the exact exterior fuselage
surface. The saved master and local F3D/STEP exports include this correction.
`root_fix_validation.json` records the added solid volume. The original R02
`validation.json` predates this correction; its wing volume/bounds are superseded
by the root-fix report. Body count remains 102. The CFD derivative already used
the continuous full-wing/fuselage union, so its external envelope is unchanged.
