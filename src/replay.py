"""Re-ask Gemini about ONE saved cycle, to iterate on the prompt without running the robot.

    python src/replay.py runs/20261107-180000/cycle03            # 1 call (budget-checked)
    python src/replay.py runs/20261107-180000/cycle03 --dry      # print what would be sent, 0 calls

Each real run costs one call against the daily cap in config.py.
"""

import argparse
import json
import sys
from pathlib import Path

from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).parent))
load_dotenv(Path(__file__).parent.parent / ".env")

import config  # noqa: E402
from agent import SYSTEM_PROMPT, Budget, GeminiSupervisor  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cycle_dir", type=Path)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--model", default=config.GEMINI_MODEL)
    args = ap.parse_args()

    summary = json.loads((args.cycle_dir / "summary.json").read_text())
    img_path = args.cycle_dir / "before_after.jpg"
    image = img_path.read_bytes() if img_path.exists() else None
    cycle = int(args.cycle_dir.name.removeprefix("cycle") or 0)

    print(f"model {args.model}, used today {Budget(args.model).used_today()}/{config.GEMINI_DAILY_CAP}")
    if args.dry:
        print(SYSTEM_PROMPT)
        print("telemetry:", json.dumps(summary))
        print("image:", f"{len(image)} bytes" if image else "none")
        return
    try:
        d = GeminiSupervisor(model=args.model).decide_once(cycle, image, summary)
    except Exception as e:  # 429s, network, budget: report, don't crash with a traceback
        print(f"no decision: {e}")
        sys.exit(1)
    print(json.dumps(d.model_dump(), indent=1))


if __name__ == "__main__":
    main()
