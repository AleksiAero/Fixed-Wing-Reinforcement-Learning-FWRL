# Spider v2 — OpenFOAM CFD preparation

Fusion design: **Spider v2**, saved under **ALEKSI AERO / Fixed Wing (Experimental)**.
The detailed packaging assembly remains the design master.

The CFD derivative uses the same R02 Fusion external loft definitions. It has
sealed access covers, sealed neutral control-surface gaps, no internal parts,
and omits screws, pushrods, the launch lug and the propeller. This is an unpowered
external-airframe case. The derivative is not a changed manufacturing master.
`geometry/cad_provenance.json` records its source and simplifications.

- Wing: NACA 2415, original swept planform, 2.106 m span.
- Tail: NACA 0010. Fuselage: 2.460 m length.
- STEP units: mm. CFD STL and domain units: m.
- Surface tessellation: 0.08 mm nominal linear tolerance, 0.12 rad angular tolerance.
- One connected, watertight surface with consistent outward normals.

## Initial case

The Ubuntu 24.04 WSL distribution runs OpenFOAM v1912 from Ubuntu's repository.
The case runs on the WSL filesystem at `/root/spider-v2-cfd` for better I/O.
The delivered copy includes the dictionaries, mesh, final fields and logs.

Baseline assumptions: 25 m/s, zero geometric angle of attack, sea-level density
1.225 kg/m3, kinematic viscosity 1.5e-5 m2/s, 1% inlet turbulence intensity and
0.02 m turbulence length scale. Flow travels in -X; +Z is lift. Reference area
is 0.5093091432 m2 (gross wing planform); coefficient length is area/span.
The pitching-moment reference is (0,0,0.02) m, not an established center of gravity.

Steady incompressible RANS uses k-omega SST, wall functions, a SIMPLE pressure-
velocity coupling and robust first-order convection for commissioning. The
initial box extends from (-8,-4,-4) to (5,4,4) m. Surface and feature refinement,
a near-body box, a wake box and three requested prism layers are configured.
Actual wall-layer coverage and y+ must be read from the outputs.

## Scope of the result

A successful run demonstrates geometry import, meshing, boundary-condition
setup and solver execution. It does not validate flight performance, stall,
stability, structural integrity, powered flow, or RL dynamics. One speed and
angle are a baseline, not an aerodynamic database. Do not replace flight-model
coefficients with these coarse-grid results.

Before performance use: inspect pressure and velocity fields, complete a mesh
and domain independence study, check layer coverage and y+, use higher-order
convection after convergence, sweep angle of attack and speed/Reynolds number,
assess transition and separation sensitivity, define mass/CG and control
settings, and add an appropriate propeller model when studying powered flight.

`Allrun` recreates the mesh and solves an initialized case. Run it in a fresh
copy to preserve prior results. `prepare_surface.py` tessellates the Fusion STEP;
`create_case.py` writes the initial dictionaries; `fusion_cfd_export.py` records
the export implementation (the registered packaging builder was restored).

References: [OpenFOAM snappyHexMesh](https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/)
and [surfaceCheck](https://www.openfoam.com/documentation/guides/latest/man/surfaceCheck.html).
The installed v1912 motorBike tutorial supplied compatible dictionary structure.

## Mesh quality and runtime caveats

The initial 506,654-cell mesh passes standard `checkMesh`: maximum non-
orthogonality 64.747 degrees, skewness 2.853, positive cell volumes. The extended
`checkMesh -allTopology -allGeometry` audit fails two checks: 23 cells with small
determinants and 26,018 concave cells. It also identifies 1,587 concave faces and
9 warped faces. These findings are preserved in logs and mesh sets. This mesh
is suitable for the recorded commissioning experiment; it has not passed a
strict performance-study mesh acceptance process. Further meshing work is
required before calling the airframe fully CFD-validated.

The Ubuntu packaged v1912 build reports a `sha1` IOstream error when initializing
optional function objects. The working solver command uses `-noFunctionObjects`.
Consequently this run does not supply validated force coefficients or wall y+.
The configured force/y+ dictionaries are retained for a corrected OpenFOAM build.
Parallel decomposition uses `hierarchical` because the packaged scotch method is
a dummy library. Neither workaround changes the physical model; both are
explicitly recorded for reproducibility.

## Recorded result

The eight-process run converged normally in 361 SIMPLE iterations. Final initial
residuals: Ux 4.73e-7, Uy 9.88e-6, Uz 1.69e-6, omega 1.01e-8 and k 3.90e-8.
The final pressure correction residual was 1.33e-7. Final global continuity
error was -6.09e-10. These are numerical convergence measures, not experimental
validation. Reconstructed fields are at time/iteration 361.

`results/convergence.png` and `results/flow_slice.png` show the recorded solution.
The centre-plane slice contains finite values. VTK warned that a few polyhedral
cells could not be contoured; the plot is for inspection and may omit those
cells. The native OpenFOAM fields are included without alteration.

`spider_v2_openfoam_case.tar.gz` contains the complete serial mesh, dictionaries,
initial fields, reconstructed converged fields and logs. Extract in WSL and
open `spider.foam` with ParaView. Processor subdirectories are omitted because
the solution has been reconstructed. `system/controlDict` starts at zero for
reproducible reruns; set startFrom to latestTime when deliberately continuing.

The corrected Fusion wing-root junction has the same sealed external envelope
already used by the CFD derivative; the cosmetic/structural root gap in the
packaging model was not present in this CFD mesh.
