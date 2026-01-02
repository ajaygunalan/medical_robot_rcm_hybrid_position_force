#!/usr/bin/env python3
"""
Convert OBJ surface meshes to VTK tetrahedral meshes for Drake compliant hydroelastic.

Usage:
    python convert_obj_to_vtk.py                    # Convert all tissue meshes
    python convert_obj_to_vtk.py path/to/mesh.obj   # Convert specific mesh
"""
import sys
from pathlib import Path

import numpy as np
import pyvista as pv
import pytetwild


def convert_obj_to_vtk(obj_path: Path, output_path: Path = None) -> Path:
    """
    Convert an OBJ surface mesh to a VTK tetrahedral mesh.

    Args:
        obj_path: Path to input OBJ file
        output_path: Optional output path. Defaults to same directory with .vtk extension

    Returns:
        Path to the generated VTK file
    """
    if output_path is None:
        output_path = obj_path.with_suffix('.vtk')

    print(f"Loading surface mesh: {obj_path}")
    surface = pv.read(str(obj_path))

    # Extract vertices and faces
    points = surface.points

    # pyvista faces format: [n_verts, v0, v1, v2, n_verts, v0, v1, ...]
    # We need just the vertex indices for triangular faces
    faces_raw = surface.faces
    n_cells = surface.n_cells

    # Parse faces (assuming triangular mesh)
    faces = []
    i = 0
    while i < len(faces_raw):
        n_verts = faces_raw[i]
        if n_verts == 3:
            faces.append(faces_raw[i+1:i+4])
        else:
            print(f"Warning: Non-triangular face with {n_verts} vertices, skipping")
        i += n_verts + 1
    faces = np.array(faces)

    print(f"  Vertices: {len(points)}")
    print(f"  Faces: {len(faces)}")

    # Tetrahedralize using fTetWild
    # Use edge_length_fac=0.01 for finer resolution to preserve anatomical surface detail
    # Default 0.05 is too coarse and destroys complex geometry
    print("Tetrahedralizing with fTetWild (edge_length_fac=0.01)...")
    tet_points, tet_cells = pytetwild.tetrahedralize(points, faces, edge_length_fac=0.01)

    print(f"  Tetrahedral cells: {len(tet_cells)}")
    print(f"  Output vertices: {len(tet_points)}")

    # Fix tetrahedron orientation to ensure positive volumes
    # Drake requires positive volume tetrahedra
    print("Fixing tetrahedron orientations...")
    fixed_count = 0
    for i in range(len(tet_cells)):
        v0, v1, v2, v3 = tet_cells[i]
        p0, p1, p2, p3 = tet_points[v0], tet_points[v1], tet_points[v2], tet_points[v3]

        # Compute signed volume: (1/6) * det([p1-p0, p2-p0, p3-p0])
        d1 = p1 - p0
        d2 = p2 - p0
        d3 = p3 - p0
        signed_vol = np.dot(d1, np.cross(d2, d3))

        if signed_vol < 0:
            # Swap two vertices to flip orientation
            tet_cells[i] = [v0, v2, v1, v3]
            fixed_count += 1

    print(f"  Fixed {fixed_count} inverted tetrahedra")

    # Create pyvista UnstructuredGrid for VTK output
    # VTK cell format: [n_points, p0, p1, p2, p3, ...]
    # For tetrahedra, n_points = 4
    cells_vtk = np.hstack([
        np.full((len(tet_cells), 1), 4, dtype=np.int64),
        tet_cells
    ]).ravel()
    cell_types = np.full(len(tet_cells), pv.CellType.TETRA, dtype=np.uint8)

    tet_mesh = pv.UnstructuredGrid(cells_vtk, cell_types, tet_points)

    # Save as VTK (tetrahedral - for collision)
    print(f"Saving VTK (collision): {output_path}")
    tet_mesh.save(str(output_path))

    # Extract and save surface mesh (for visual - matches VTK boundary exactly)
    surface_path = output_path.with_name(output_path.stem + "_surface.obj")
    print(f"Extracting surface for visual...")
    surface = tet_mesh.extract_surface()
    print(f"  Surface faces: {surface.n_cells}")
    print(f"Saving surface OBJ (visual): {surface_path}")
    surface.save(str(surface_path))

    return output_path


def main():
    models_dir = Path(__file__).parent.parent / "models"

    if len(sys.argv) > 1:
        # Convert specific file
        obj_path = Path(sys.argv[1])
        if not obj_path.exists():
            print(f"Error: File not found: {obj_path}")
            sys.exit(1)
        convert_obj_to_vtk(obj_path)
    else:
        # Convert all tissue meshes
        tissue_files = [
            models_dir / "tissue" / "vaginal" / "vagina_tissue.obj",
            models_dir / "tissue" / "flat" / "flat_tissue.obj",
        ]

        for obj_path in tissue_files:
            if obj_path.exists():
                print(f"\n{'='*60}")
                convert_obj_to_vtk(obj_path)
            else:
                print(f"Skipping (not found): {obj_path}")

        print(f"\n{'='*60}")
        print("Done! VTK files created alongside OBJ files.")


if __name__ == "__main__":
    main()
