#!/usr/bin/env python3
"""Export the Fuel kelp meshes as double-sided OBJs for the sway shader.

gz-sim's ShaderParam replaces the kelp material, and that material cannot be
made double-sided, so the leaves would vanish when seen from behind. Each
triangle is therefore written twice: once as in the DAE and once with the
winding reversed and the normal flipped, so both faces are lit correctly.

  models/kelp_0N/meshes/Kelp0N.dae  ->  models/kelp_0N/meshes/Kelp0N_sway.obj

The OBJ is in metres (the DAE is in centimetres), Z-up, with the base at
z = 0, so the vertex shader can use vertex.z directly as height above the
base. UVs are copied unchanged.

Usage:
    python3 scripts/make_kelp_sway_meshes.py
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "src", "fishy_fish_navigation", "models")
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def floats(root, source_id):
    src = root.find(f".//c:source[@id='{source_id}']", NS)
    acc = src.find(".//c:accessor", NS)
    data = np.array(src.find("c:float_array", NS).text.split(), float)
    return data.reshape(-1, int(acc.get("stride")))


def load_dae(path):
    """Return (positions, normals, uvs) per triangle corner, in metres."""
    root = ET.parse(path).getroot()
    unit = float(root.find(".//c:asset/c:unit", NS).get("meter"))
    matrix = np.array(root.find(".//c:node/c:matrix", NS).text.split(),
                      float).reshape(4, 4)

    mesh = root.find(".//c:geometry/c:mesh", NS)
    vert_src = {v.get("id"): v.find("c:input", NS).get("source")[1:]
                for v in mesh.findall("c:vertices", NS)}
    pos, nrm, uv = [], [], []
    for tris in mesh.findall("c:triangles", NS):
        inputs = {i.get("semantic"): i for i in tris.findall("c:input", NS)}
        stride = max(int(i.get("offset")) for i in inputs.values()) + 1
        idx = np.array(tris.find("c:p", NS).text.split(), int).reshape(-1, stride)

        def column(semantic, source):
            return floats(root, source)[idx[:, int(inputs[semantic].get("offset"))]]

        pos.append(column("VERTEX", vert_src[inputs["VERTEX"].get("source")[1:]]))
        nrm.append(column("NORMAL", inputs["NORMAL"].get("source")[1:]))
        uv.append(column("TEXCOORD", inputs["TEXCOORD"].get("source")[1:])[:, :2])

    pos, nrm, uv = np.vstack(pos), np.vstack(nrm), np.vstack(uv)
    pos = (pos @ matrix[:3, :3].T + matrix[:3, 3]) * unit
    nrm = nrm @ matrix[:3, :3].T
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    pos[:, 2] -= pos[:, 2].min()
    return pos, nrm, uv


def write_obj(path, pos, nrm, uv):
    n = len(pos)
    with open(path, "w") as f:
        f.write("# Double-sided kelp for the sway shader "
                "(scripts/make_kelp_sway_meshes.py)\n")
        f.write("# Kelp model by Cole Biesemeyer, Gazebo Fuel, CC-BY 4.0\n")
        for p in pos:
            f.write(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
        for t in uv:
            f.write(f"vt {t[0]:.5f} {t[1]:.5f}\n")
        for v in np.vstack([nrm, -nrm]):
            f.write(f"vn {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        f.write("o kelp\n")
        for a in range(0, n, 3):
            i, j, k = a + 1, a + 2, a + 3     # OBJ indices are 1-based
            f.write(f"f {i}/{i}/{i} {j}/{j}/{j} {k}/{k}/{k}\n")
            # back face: reversed winding, flipped normal
            f.write(f"f {k}/{k}/{k + n} {j}/{j}/{j + n} {i}/{i}/{i + n}\n")


def main():
    for k in range(1, 5):
        mesh_dir = os.path.join(MODELS, f"kelp_0{k}", "meshes")
        pos, nrm, uv = load_dae(os.path.join(mesh_dir, f"Kelp0{k}.dae"))
        out = os.path.join(mesh_dir, f"Kelp0{k}_sway.obj")
        write_obj(out, pos, nrm, uv)
        size = pos.max(0) - pos.min(0)
        print(f"kelp_0{k}: {len(pos) // 3} triangles x2, "
              f"{size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m -> {out}")


if __name__ == "__main__":
    main()
