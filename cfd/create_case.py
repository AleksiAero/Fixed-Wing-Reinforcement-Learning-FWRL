from pathlib import Path
import shutil,json
case=Path.home()/'spider-v2-cfd'
for d in ['0','constant/triSurface','system']: (case/d).mkdir(parents=True,exist_ok=True)
shutil.copy2('cfd/geometry/spider.stl',case/'constant/triSurface/spider.stl')
def put(name,body,cls='dictionary'):
 p=case/name;p.write_text('FoamFile { version 2.0; format ascii; class '+cls+'; object '+p.name+'; }\n'+body+'\n')
put('system/blockMeshDict','''convertToMeters 1;
vertices ((-8 -4 -4)(5 -4 -4)(5 4 -4)(-8 4 -4)(-8 -4 4)(5 -4 4)(5 4 4)(-8 4 4));
blocks (hex (0 1 2 3 4 5 6 7) (65 40 40) simpleGrading (1 1 1));
edges ();
boundary (
 inlet {type patch; faces ((1 2 6 5));}
 outlet {type patch; faces ((0 4 7 3));}
 farfield {type patch; faces ((0 1 5 4)(3 7 6 2)(0 3 2 1)(4 5 6 7));}
);
mergePatchPairs ();''')
put('system/surfaceFeatureExtractDict','''spider.stl { extractionMethod extractFromSurface; extractFromSurfaceCoeffs { includedAngle 150; } writeObj yes; }''')
put('system/snappyHexMeshDict','''castellatedMesh true; snap true; addLayers true;
geometry {
 spider.stl {type triSurfaceMesh; name airframe;}
 nearBody {type searchableBox; min (-1.6 -1.3 -0.3); max (1.5 1.3 0.6);}
 wake {type searchableBox; min (-5 -1.3 -0.4); max (-1.3 1.3 0.7);}
}
castellatedMeshControls {
 maxLocalCells 1500000; maxGlobalCells 3000000; minRefinementCells 10; maxLoadUnbalance 0.1; nCellsBetweenLevels 3;
 features ({file "spider.eMesh"; level 5;});
 refinementSurfaces {airframe {level (4 5); patchInfo {type wall;}}}
 resolveFeatureAngle 30;
 refinementRegions {nearBody {mode inside; levels ((1e15 2));} wake {mode inside; levels ((1e15 1));}}
 locationInMesh (4.123 2.123 2.123); allowFreeStandingZoneFaces true;
}
snapControls {nSmoothPatch 5; tolerance 2; nSolveIter 50; nRelaxIter 8; nFeatureSnapIter 15; implicitFeatureSnap false; explicitFeatureSnap true; multiRegionFeatureSnap false;}
addLayersControls {
 relativeSizes true; layers {"airframe.*" {nSurfaceLayers 3;}}
 expansionRatio 1.2; finalLayerThickness 0.4; minThickness 0.1; nGrow 0;
 featureAngle 60; slipFeatureAngle 30; nRelaxIter 5; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10;
 maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0; nLayerIter 50;
}
meshQualityControls {#includeEtc "caseDicts/meshQualityDict"
 nSmoothScale 4; errorReduction 0.75;}
writeFlags (scalarLevels layerSets layerFields); mergeTolerance 1e-6;''')
put('system/controlDict','''application simpleFoam;
startFrom startTime; startTime 0; stopAt endTime; endTime 800; deltaT 1;
writeControl timeStep; writeInterval 200; purgeWrite 2; writeFormat binary; writePrecision 8; writeCompression off; timeFormat general; timePrecision 6; runTimeModifiable true;
functions {
 forces {type forceCoeffs; libs ("libforces.so"); writeControl timeStep; writeInterval 1; patches ("airframe.*"); rho rhoInf; rhoInf 1.225; CofR (0 0 0.02); liftDir (0 0 1); dragDir (-1 0 0); pitchAxis (0 1 0); magUInf 25; lRef 0.241837; Aref 0.5093091432;}
 yPlus {type yPlus; libs ("libfieldFunctionObjects.so"); writeControl writeTime;}
}''')
put('system/fvSchemes','''ddtSchemes {default steadyState;}
gradSchemes {default Gauss linear; grad(U) cellLimited Gauss linear 1;}
divSchemes {default none; div(phi,U) bounded Gauss upwind; div(phi,k) bounded Gauss upwind; div(phi,omega) bounded Gauss upwind; div((nuEff*dev2(T(grad(U))))) Gauss linear;}
laplacianSchemes {default Gauss linear limited 0.5;}
interpolationSchemes {default linear;} snGradSchemes {default limited 0.5;}
wallDist {method meshWave;}''')
put('system/fvSolution','''solvers {
 p {solver GAMG; smoother GaussSeidel; tolerance 1e-7; relTol 0.05;}
 "(U|k|omega)" {solver smoothSolver; smoother symGaussSeidel; tolerance 1e-8; relTol 0.1;}
}
SIMPLE {nNonOrthogonalCorrectors 1; residualControl {p 1e-4; U 1e-5; k 1e-5; omega 1e-5;}}
relaxationFactors {fields {p 0.3;} equations {U 0.5; k 0.5; omega 0.5;}}''')
put('system/decomposeParDict','numberOfSubdomains 8; method hierarchical; hierarchicalCoeffs {n (2 2 2); delta 0.001; order xyz;}')
put('constant/transportProperties','transportModel Newtonian; nu [0 2 -1 0 0 0 0] 1.5e-5;')
put('constant/turbulenceProperties','simulationType RAS; RAS {RASModel kOmegaSST; turbulence on; printCoeffs on;}')
put('0/U','''dimensions [0 1 -1 0 0 0 0]; internalField uniform (-25 0 0);
boundaryField {
 inlet {type fixedValue; value uniform (-25 0 0);}
 outlet {type inletOutlet; inletValue uniform (-25 0 0); value uniform (-25 0 0);}
 farfield {type freestream; freestreamValue uniform (-25 0 0);}
 "airframe.*" {type noSlip;}
}''','volVectorField')
put('0/p','''dimensions [0 2 -2 0 0 0 0]; internalField uniform 0;
boundaryField {inlet {type zeroGradient;} outlet {type fixedValue; value uniform 0;} farfield {type freestreamPressure; freestreamValue uniform 0; value uniform 0;} "airframe.*" {type zeroGradient;}}''','volScalarField')
for name,dim,val,wall in [('k','0 2 -2 0 0 0 0',.09375,'kqRWallFunction'),('omega','0 0 -1 0 0 0 0',27.95,'omegaWallFunction'),('nut','0 2 -1 0 0 0 0',0,'nutkWallFunction')]:
 bc='calculated' if name=='nut' else 'fixedValue'
 outlet='calculated' if name=='nut' else 'zeroGradient'
 put('0/'+name,f'''dimensions [{dim}]; internalField uniform {val};
 boundaryField {{inlet {{type {bc}; value uniform {val};}} outlet {{type {outlet}; value uniform {val};}} farfield {{type {bc}; value uniform {val};}} "airframe.*" {{type {wall}; value uniform {val};}}}}''','volScalarField')
(case/'spider.foam').touch()
(case/'case_assumptions.json').write_text(json.dumps({'speed_m_s':25,'geometric_angle_of_attack_deg':0,'rho_kg_m3':1.225,'nu_m2_s':1.5e-5,'turbulence_intensity':.01,'turbulence_length_scale_m':.02,'model':'steady incompressible RANS kOmegaSST','purpose':'coarse CFD preparation and solver commissioning, not flight validation','propeller':'omitted unpowered'},indent=2))
print(case)
