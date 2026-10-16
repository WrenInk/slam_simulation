#!/usr/bin/env python3
"""Convert the unmodified Livox STEP download into a simulation mesh.

Optional conversion dependencies: cadquery-ocp==7.8.1.1.post1, vtk==9.3.1.
Runtime uses the checked-in mesh and does not require OpenCascade.
"""
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepTools import BRepTools_ReShape
from OCP.BRepBndLib import BRepBndLib

root = Path(__file__).resolve().parents[1]
reader = STEPControl_Reader()
assert reader.ReadFile(str(root/'third_party/mid360_official/mid-360-asm.stp')) == IFSelect_RetDone
reader.TransferRoots()
shape = reader.OneShape()
bounds = Bnd_Box()
BRepBndLib.Add_s(shape, bounds)
print('Official CAD bounds (mm):', bounds.Get())
# The first solid is the optical dome; retain all remaining shells and solids.
explorer = TopExp_Explorer(shape, TopAbs_SOLID)
dome = explorer.Current()
reshape = BRepTools_ReShape()
reshape.Remove(dome)
housing = reshape.Apply(shape)
for name, part in [('dome', dome), ('housing', housing)]:
    BRepMesh_IncrementalMesh(part, 0.15, False, 0.3, True).Perform()
    output = root/f'src/nav_stack/meshes/mid360_{name}.stl'
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    assert writer.Write(part, str(output))
    print('Saved', output)
