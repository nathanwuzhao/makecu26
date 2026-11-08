"""Run the vision planner (src/vision.py) over a folder of photos: what does Gemini see / plan?

    python tools/gemini_eval.py --dry                  # no calls: lists what would be sent
    python tools/gemini_eval.py --max 5                # 5 real calls (free tier: waits the min interval)
    python tools/gemini_eval.py --dir runs/photos --context "real webcam, gripper empty"

Photos: runs/photos/**/*.jpg|jpeg|png (subfolder names are a handy label, e.g. runs/photos/pill_bottle/).
Writes runs/gemini_eval/<ts>/results.json + results.csv (one row per photo, expected = folder name).
Same Budget as the app (runs/gemini_usage.json): daily cap, run cap, 429 cooldown.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "src"))
from dotenv import load_dotenv  # noqa: E402

load_dotenv(ROOT / ".env")

import cv2 as cv  # noqa: E402
import numpy as np  # noqa: E402

import config  # noqa: E402
import skill_registry  # noqa: E402
import vision  # noqa: E402


def load_jpeg(path: Path, width: int = 768) -> bytes | None:
    """Any image -> JPEG, downscaled to `width` (smaller request, same answer)."""
    img = cv.imdecode(np.fromfile(str(path), np.uint8), cv.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    if w > width:
        img = cv.resize(img, (width, int(h * width / w)), interpolation=cv.INTER_AREA)
    ok, buf = cv.imencode(".jpg", img, [cv.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes() if ok else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(ROOT / "runs" / "photos"))
    ap.add_argument("--max", type=int, default=10, help="max photos (= max Gemini calls)")
    ap.add_argument("--dry", action="store_true", help="no Gemini calls")
    ap.add_argument("--context", default="", help="extra context sent with every photo")
    args = ap.parse_args()

    src = Path(args.dir)
    photos = sorted(p for p in src.rglob("*") if p.suffix.lower() in (".jpg", ".jpeg", ".png"))[:args.max]
    if not photos:
        print(f"no photos under {src}")
        return
    config.GEMINI_RUN_CAP = max(config.GEMINI_RUN_CAP, args.max)   # --max is this run's cap; daily cap still applies
    catalog = skill_registry.catalog()
    out_dir = ROOT / "runs" / "gemini_eval" / time.strftime("%Y%m%d-%H%M%S")
    rows = []
    print(f"{len(photos)} photo(s), model {config.GEMINI_MODEL}, budget {vision.budget_info()}")
    for i, p in enumerate(photos, 1):
        expected = p.parent.name if p.parent != src else ""
        jpeg = load_jpeg(p)
        if jpeg is None:
            print(f"[{i}] {p}: unreadable, skipped")
            continue
        if not args.dry:   # wait out the min interval instead of being refused
            wait = vision.budget_info()["next_in_s"]
            if wait > 0:
                print(f"    waiting {wait:.0f} s (min interval / cooldown)")
                time.sleep(wait)
        a, msg = vision.analyze_scene(jpeg, catalog, args.context, dry=args.dry)
        row = {"photo": str(p.relative_to(src)), "expected": expected, "msg": msg,
               "object": a.object if a else "", "confidence": a.confidence if a else "",
               "cap_state": a.cap_state if a else "", "task": a.recommended_task if a else "",
               "plan": " > ".join(s.name for s in a.plan) if a else "",
               "match": (a.object == expected) if a and expected else "",
               "latency_s": a.latency_s if a else "", "analysis": a.model_dump() if a else None}
        rows.append(row)
        print(f"[{i}] {row['photo']}: {msg}" + (f" | {row['plan']}" if a else ""))
        if msg.startswith("budget: daily") or msg.startswith("429: daily"):
            print("stopping: quota")
            break
    if args.dry:
        print("dry run: nothing written")
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "results.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    with (out_dir / "results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k != "analysis"], extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    scored = [r for r in rows if r["match"] != ""]
    if scored:
        print(f"object accuracy: {sum(r['match'] for r in scored)}/{len(scored)}")
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
