"""Webcam photo helper for the dataset (what the gripper's camera sees). No robot involved.

    python tools/photo.py --label pill_bottle            # live window: space = save, l = new label, q = quit
    python tools/photo.py --camera 1 --label eye_dropper
    python tools/photo.py --headless --count 5 --interval 2 --label bench   # no window: timed shots

Saves runs/photos/<label>/<n>.jpg (n continues after existing files) plus the full-resolution frame.
Uses src/camera.py (DirectShow, newest frame). Falls back to headless if OpenCV has no GUI.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2 as cv

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from camera import Camera  # noqa: E402

OUT = ROOT / "runs" / "photos"


def next_path(label: str) -> Path:
    d = OUT / label
    d.mkdir(parents=True, exist_ok=True)
    nums = [int(p.stem) for p in d.glob("*.jpg") if p.stem.isdigit()]
    return d / f"{max(nums, default=0) + 1:03d}.jpg"


def save(cam: Camera, label: str) -> Path | None:
    f = cam.frame()
    if f is None:
        print("no frame yet")
        return None
    p = next_path(label)
    cv.imwrite(str(p), f, [cv.IMWRITE_JPEG_QUALITY, 92])
    print(f"saved {p.relative_to(ROOT)}  ({f.shape[1]}x{f.shape[0]})")
    return p


def has_gui() -> bool:
    try:
        cv.namedWindow("photo")
        return True
    except cv.error:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--label", default="misc")
    ap.add_argument("--headless", action="store_true", help="no window: take --count shots every --interval s")
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--interval", type=float, default=2.0)
    args = ap.parse_args()

    cam = Camera(args.camera).start()
    label = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label) or "misc"
    try:
        if args.headless or not has_gui():
            print(f"headless: {args.count} shots every {args.interval} s -> runs/photos/{label}/")
            for i in range(args.count):
                if i:
                    time.sleep(args.interval)
                save(cam, label)
            return
        print("space = save   l = change label (type it in this console)   q / Esc = quit")
        while True:
            f = cam.frame()
            if f is not None:
                view = f.copy()
                cv.putText(view, f"label: {label}   [space] save  [l] label  [q] quit", (12, 32),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv.imshow("photo", view)
            k = cv.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                save(cam, label)
            elif k == ord("l"):
                new = input("new label: ").strip()
                label = "".join(c if c.isalnum() or c in "-_" else "_" for c in new) or label
                print(f"label = {label}")
            if cv.getWindowProperty("photo", cv.WND_PROP_VISIBLE) < 1:   # window closed with X
                break
    finally:
        cam.close()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()
