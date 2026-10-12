"""Generate an OpenFOAM-11 steady RANS (simpleFoam, k-epsilon) case: neutral marine atmospheric
boundary layer over open sea with the obstacles from site.json.

The case is built in a flow-aligned frame (x' = direction the wind blows towards, z up); obstacles
and the sample points are rotated into it. Results are U/Uref and k/Uref^2 (Uref = 10 m/s at
10 m), which are independent of wind speed for this model, so one run per wind direction serves
every speed.

  python make_case.py --site site.json --dir 22 --out case_d022
"""
import argparse
import json
import math
import os

p = argparse.ArgumentParser()
p.add_argument("--site", default=os.path.join(os.path.dirname(__file__), "site.json"))
p.add_argument("--dir", type=float, required=True, help="meteorological direction the wind comes FROM, deg")
p.add_argument("--out", required=True)
p.add_argument("--cell", type=float, default=10.0, help="base cell size (m)")
p.add_argument("--level", type=int, default=3, help="snappyHexMesh refinement level at obstacle surfaces")
p.add_argument("--np", type=int, default=8, help="MPI ranks for the solver")
a = p.parse_args()

site = json.load(open(a.site))
dom, smp = site["domain"], site["sample"]
Z0, ZREF, UREF = site["z0"], site["zref"], 10.0

th = math.radians(a.dir)
ux, uy = -math.sin(th), -math.cos(th)  # world (east, north) unit vector the wind blows towards
vx, vy = -uy, ux  # flow-frame +y' (left of the flow)
ang = math.atan2(uy, ux)


def to_flow(x, y):
    return x * ux + y * uy, x * vx + y * vy


os.makedirs(os.path.join(a.out, "0"), exist_ok=True)
os.makedirs(os.path.join(a.out, "system"), exist_ok=True)
os.makedirs(os.path.join(a.out, "constant", "triSurface"), exist_ok=True)


def w(rel, body, cls="dictionary", obj=None):
    name = os.path.basename(rel)
    with open(os.path.join(a.out, rel), "w") as f:
        f.write(
            "FoamFile\n{\n    version 2.0;\n    format ascii;\n"
            f"    class {cls};\n    object {obj or name};\n}}\n\n{body}\n"
        )


# ---------------------------------------------------------------- geometry
obst = site["obstacles"]
tris = []
for o in obst:
    cx, cy = to_flow(*o["center"])
    lx, ly, lz = o["size"]
    yaw = math.radians(o.get("yaw", 0)) - ang
    c, s = math.cos(yaw), math.sin(yaw)
    z0, z1 = -2.0, lz  # base sunk below the ground patch so the cut is clean
    corner = lambda dx, dy, z: (cx + c * dx - s * dy, cy + s * dx + c * dy, z)
    P = [
        corner(-lx / 2, -ly / 2, z0), corner(lx / 2, -ly / 2, z0), corner(lx / 2, ly / 2, z0), corner(-lx / 2, ly / 2, z0),
        corner(-lx / 2, -ly / 2, z1), corner(lx / 2, -ly / 2, z1), corner(lx / 2, ly / 2, z1), corner(-lx / 2, ly / 2, z1),
    ]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]  # outward normals
    for q in quads:
        tris += [(P[q[0]], P[q[1]], P[q[2]]), (P[q[0]], P[q[2]], P[q[3]])]
if obst:
    with open(os.path.join(a.out, "constant", "triSurface", "obstacles.stl"), "w") as f:
        f.write("solid obstacles\n")
        for t in tris:
            f.write("facet normal 0 0 0\n outer loop\n")
            for v in t:
                f.write(f"  vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            f.write(" endloop\nendfacet\n")
        f.write("endsolid obstacles\n")

# ---------------------------------------------------------------- mesh
Lu, Ld, hw, Ht = dom["upstream"], dom["downstream"], dom["half_width"], dom["height"]
nx, ny = round((Lu + Ld) / a.cell), round(2 * hw / a.cell)
nz, grade = 24, 15
w("system/blockMeshDict", f"""scale 1;
vertices
(
    (-{Lu} -{hw} 0) ({Ld} -{hw} 0) ({Ld} {hw} 0) (-{Lu} {hw} 0)
    (-{Lu} -{hw} {Ht}) ({Ld} -{hw} {Ht}) ({Ld} {hw} {Ht}) (-{Lu} {hw} {Ht})
);
blocks ( hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 {grade}) );
edges ();
boundary
(
    inlet {{ type patch; faces ((0 4 7 3)); }}
    outlet {{ type patch; faces ((1 2 6 5)); }}
    ground {{ type wall; faces ((0 3 2 1)); }}
    top {{ type patch; faces ((4 5 6 7)); }}
    sides {{ type patch; faces ((0 1 5 4) (3 7 6 2)); }}
);
mergePatchPairs ();""")

if obst:
    xs = [v[0] for t in tris for v in t]
    ys = [v[1] for t in tris for v in t]
    hmax = max(o["size"][2] for o in obst)
    box = (min(xs) - 2 * hmax, min(ys) - 2 * hmax, 0, max(xs) + 12 * hmax, max(ys) + 2 * hmax, 3 * hmax)
    lvl = a.level
    w("system/snappyHexMeshDict", f"""castellatedMesh true;
snap true;
addLayers false;
geometry
{{
    obstacles {{ type triSurfaceMesh; file "obstacles.stl"; }}
    wake {{ type searchableBox; min ({box[0]:.2f} {box[1]:.2f} {box[2]:.2f}); max ({box[3]:.2f} {box[4]:.2f} {box[5]:.2f}); }}
}}
castellatedMeshControls
{{
    maxLocalCells 2000000;
    maxGlobalCells 4000000;
    minRefinementCells 0;
    nCellsBetweenLevels 3;
    maxLoadUnbalance 0.2;
    features ();
    refinementSurfaces {{ obstacles {{ level ({lvl} {lvl}); patchInfo {{ type wall; }} }} }}
    resolveFeatureAngle 30;
    refinementRegions {{ wake {{ mode inside; levels ((1E15 {max(1, lvl - 1)})); }} }}
    locationInMesh (-{Lu - 5} 1.0 {Ht / 2});
    allowFreeStandingZoneFaces true;
}}
snapControls {{ nSmoothPatch 3; tolerance 2.0; nSolveIter 30; nRelaxIter 5; nFeatureSnapIter 10; implicitFeatureSnap true; explicitFeatureSnap false; multiRegionFeatureSnap false; }}
addLayersControls {{ layers {{}} relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3; minThickness 0.1; nGrow 0; featureAngle 60; nRelaxIter 5; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10; maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0; nLayerIter 50; }}
meshQualityControls
{{
    maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13;
    minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.05;
    minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75;
}}
mergeTolerance 1e-6;""")

# ---------------------------------------------------------------- physics / numerics
w("constant/physicalProperties", "viscosityModel constant;\nnu 1.5e-05;")
w("constant/momentumTransport", """simulationType RAS;
RAS
{
    model kEpsilon;
    turbulence on;
    printCoeffs on;
    kEpsilonCoeffs { Cmu 0.09; C1 1.44; C2 1.92; sigmaEps 1.11; }  // Hargreaves & Wright 2007 for the ABL
}""")

abl = f"""flowDir (1 0 0);
        zDir (0 0 1);
        Uref {UREF};
        Zref {ZREF};
        z0 uniform {Z0};
        zGround uniform 0;"""

obst_u = "    obstacles { type noSlip; }\n" if obst else ""
w("0/U", f"""dimensions [0 1 -1 0 0 0 0];
internalField uniform ({UREF} 0 0);
boundaryField
{{
    inlet {{ type atmBoundaryLayerInletVelocity; {abl} }}
    outlet {{ type inletOutlet; inletValue uniform (0 0 0); value $internalField; }}
    ground {{ type noSlip; }}
    top {{ type slip; }}
    sides {{ type slip; }}
{obst_u}}}""", "volVectorField")

ustar = 0.41 * UREF / math.log((ZREF + Z0) / Z0)
k0 = ustar**2 / math.sqrt(0.09)
eps0 = ustar**3 / (0.41 * (ZREF + Z0))
obst_k = "    obstacles { type kqRWallFunction; value $internalField; }\n" if obst else ""
w("0/k", f"""dimensions [0 2 -2 0 0 0 0];
internalField uniform {k0:.5f};
boundaryField
{{
    inlet {{ type atmBoundaryLayerInletK; {abl} }}
    outlet {{ type inletOutlet; inletValue $internalField; value $internalField; }}
    ground {{ type kqRWallFunction; value $internalField; }}
    top {{ type zeroGradient; }}
    sides {{ type zeroGradient; }}
{obst_k}}}""", "volScalarField")

obst_e = "    obstacles { type epsilonWallFunction; value $internalField; }\n" if obst else ""
w("0/epsilon", f"""dimensions [0 2 -3 0 0 0 0];
internalField uniform {eps0:.6f};
boundaryField
{{
    inlet {{ type atmBoundaryLayerInletEpsilon; {abl} }}
    outlet {{ type inletOutlet; inletValue $internalField; value $internalField; }}
    ground {{ type epsilonWallFunction; value $internalField; }}
    top {{ type zeroGradient; }}
    sides {{ type zeroGradient; }}
{obst_e}}}""", "volScalarField")

obst_n = "    obstacles { type nutkWallFunction; value uniform 0; }\n" if obst else ""
w("0/nut", f"""dimensions [0 2 -1 0 0 0 0];
internalField uniform 0;
boundaryField
{{
    inlet {{ type calculated; value uniform 0; }}
    outlet {{ type calculated; value uniform 0; }}
    ground {{ type nutkAtmRoughWallFunction; z0 uniform {Z0}; value uniform 0; }}
    top {{ type calculated; value uniform 0; }}
    sides {{ type calculated; value uniform 0; }}
{obst_n}}}""", "volScalarField")

obst_p = "    obstacles { type zeroGradient; }\n" if obst else ""
w("0/p", f"""dimensions [0 2 -2 0 0 0 0];
internalField uniform 0;
boundaryField
{{
    inlet {{ type zeroGradient; }}
    outlet {{ type fixedValue; value uniform 0; }}
    ground {{ type zeroGradient; }}
    top {{ type zeroGradient; }}
    sides {{ type zeroGradient; }}
{obst_p}}}""", "volScalarField")

# sample grid in WORLD coordinates (east, north, up), rotated into the flow frame
he, dx, zmax, dz = smp["half_extent"], smp["dx"], smp["zmax"], smp["dz"]
gx = [-he + i * dx for i in range(int(2 * he / dx) + 1)]
gz = [dz / 2 + i * dz for i in range(int(zmax / dz))]
pts = []
for x in gx:
    for y in gx:
        fx, fy = to_flow(x, y)
        for z in gz:
            pts.append(f"({fx:.3f} {fy:.3f} {z:.3f})")
w("system/controlDict", f"""application foamRun;
solver incompressibleFluid;
libs ("libatmosphericModels.so");
startFrom startTime;
startTime 0;
stopAt endTime;
endTime 3000;
deltaT 1;
writeControl timeStep;
writeInterval 3000;
purgeWrite 1;
writeFormat ascii;
writePrecision 7;
timeFormat general;
timePrecision 6;
runTimeModifiable false;
functions
{{
    samples
    {{
        type sets;
        libs ("libsampling.so");
        executeControl writeTime;
        writeControl writeTime;
        interpolationScheme cellPoint;
        setFormat raw;
        fields (U k);
        sets ( cloud {{ type points; axis xyz; ordered no; points ({" ".join(pts)}); }} );
    }}
}}""")

w("system/fvSchemes", """ddtSchemes { default steadyState; }
gradSchemes { default Gauss linear; }
divSchemes
{
    default none;
    div(phi,U) bounded Gauss linearUpwind grad(U);
    div(phi,k) bounded Gauss upwind;
    div(phi,epsilon) bounded Gauss upwind;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear corrected; }
interpolationSchemes { default linear; }
snGradSchemes { default corrected; }""")

w("system/fvSolution", """solvers
{
    p { solver GAMG; smoother GaussSeidel; tolerance 1e-8; relTol 0.1; }
    "(U|k|epsilon)" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-8; relTol 0.1; }
}
SIMPLE
{
    nNonOrthogonalCorrectors 1;
    residualControl { p 1e-4; U 1e-4; "(k|epsilon)" 1e-4; }
}
relaxationFactors { fields { p 0.3; } equations { U 0.7; k 0.7; epsilon 0.7; } }""")
w("system/decomposeParDict", f"numberOfSubdomains {a.np};\nmethod scotch;")

json.dump({"dir_from": a.dir, "ux": ux, "uy": uy, "uref": UREF, "zref": ZREF, "z0": Z0,
           "ground_x": gx, "ground_z": gz},
          open(os.path.join(a.out, "frame.json"), "w"))
print(f"case {a.out}: dir_from={a.dir} flow_to=({ux:.3f},{uy:.3f}) cells~{nx*ny*nz} base, "
      f"obstacles={len(obst)}, sample pts={len(pts)}, u*={ustar:.3f}")
