"""Convert the Fusion 360 STEP assembly to GLB for the three.js digital twin.

    python tools/step_to_glb.py "MAKECU FINAL CAD (TEND).step" web/assets/gripper.glb

Uses cascadio (OpenCASCADE). Keeps the assembly tree and part names ("Top Left:1", "finger tip:4",
...), so the viewer can find and move parts by name. Fusion's STL/OBJ export would flatten them.
Units: the GLB is in metres; axes are the CAD's (Z up), so the viewer rotates it to three.js's Y-up.
Re-run whenever the CAD changes (~5 s).
"""

import json
import struct
import sys
import time
from pathlib import Path

import cascadio

# Parts dropped from the twin (geometry removed, node kept so the tree stays intact).
EXCLUDE = {"=>[0:1:1:37]"}   # unnamed 1 mm sheet above the top layer


def drop_nodes(path: Path, names: set[str]) -> int:
    """Remove the mesh from nodes with these names by rewriting the GLB's JSON chunk."""
    data = path.read_bytes()
    json_len = struct.unpack("<I", data[12:16])[0]
    gltf = json.loads(data[20:20 + json_len])
    dropped = 0
    for node in gltf["nodes"]:
        if node.get("name") in names:
            node.pop("mesh", None)
            node.pop("children", None)
            dropped += 1
    chunk = json.dumps(gltf, separators=(",", ":")).encode()
    chunk += b" " * (-len(chunk) % 4)                   # JSON chunk is space-padded to 4 bytes
    rest = data[20 + json_len:]                         # BIN chunk, unchanged
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(chunk) + len(rest))
    out += struct.pack("<II", len(chunk), 0x4E4F534A) + chunk + rest
    path.write_bytes(out)
    return dropped


def main():
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2] if len(sys.argv) > 2 else "web/assets/gripper.glb")
    dst.parent.mkdir(parents=True, exist_ok=True)
    t = time.time()
    # tol_linear = max chord error in CAD units (mm): 0.05 mm is smooth enough for
    # gears/fingers and keeps the file ~2.5 MB
    cascadio.step_to_glb(str(src), str(dst), tol_linear=0.05, tol_angular=0.3)
    n = drop_nodes(dst, EXCLUDE)
    print(f"{dst} ({dst.stat().st_size / 1e6:.1f} MB) in {time.time() - t:.1f} s, dropped {n} part(s)")


if __name__ == "__main__":
    main()
