"""Inspect a GLB (from step_to_glb.py): node names, bbox (mm), principal axis, diameter profile.

    python tools/inspect_glb.py web/assets/objects/insulin_pen.glb
"""
import json
import struct
import sys

import numpy as np
import trimesh


def node_names(path):
    """Raw node tree from the GLB JSON chunk (trimesh renames some nodes)."""
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[12:16])[0]
    g = json.loads(data[20:20 + n])
    return [(i, nd.get("name"), "mesh" in nd, nd.get("children", [])) for i, nd in enumerate(g["nodes"])]


def main():
    path = sys.argv[1]
    for i, name, has_mesh, ch in node_names(path):
        print(f"node {i:2d} {name!r:40} mesh={has_mesh} children={ch}")
    scene = trimesh.load(path, force="scene")
    print("\nper-geometry bbox (mm, world):")
    for node in scene.graph.nodes_geometry:
        T, g = scene.graph[node]
        m = scene.geometry[g].copy(); m.apply_transform(T)
        print(f"  {node!r:40} {np.round(m.bounds * 1000, 2).tolist()}")
    mesh = scene.to_mesh() if hasattr(scene, "to_mesh") else scene.dump(concatenate=True)
    v = mesh.vertices * 1000
    lo, hi = v.min(0), v.max(0)
    print("\nbbox mm", lo.round(2), hi.round(2), "size", (hi - lo).round(2))
    ax = int(np.argmax(hi - lo))
    print("long axis", "XYZ"[ax])
    # cross-section diameters along the long axis
    other = [i for i in range(3) if i != ax]
    normal = np.zeros(3); normal[ax] = 1
    for t in np.arange(lo[ax] + 0.5, hi[ax], 4.0):
        o = np.zeros(3); o[ax] = t
        # raw plane/triangle intersection segments (mesh.section needs scipy)
        lines = trimesh.intersections.mesh_plane(mesh, normal, o / 1000)
        if len(lines) == 0:
            print(f"  {t:8.2f}  -"); continue
        p = lines.reshape(-1, 3) * 1000
        w = p[:, other].max(0) - p[:, other].min(0)
        c = (p[:, other].max(0) + p[:, other].min(0)) / 2
        print(f"  {t:8.2f}  width {w.round(2)}  centre {c.round(2)}")


if __name__ == "__main__":
    main()
