"""Runs the unscrew loop in a worker thread so it can be started / stopped / E-stopped from the
web UI (twin_server.py -> /api/*) or the CLI (main.py --autostart).

States: idle -> running -> (stopping) -> idle;  any -> estopped -> (reset) -> idle

  stop   graceful: the current move is aborted at its next tick, then the top fingers let go of
         the cap (bottle stays held). Use it to end a run.
  estop  immediate: motion is LOCKED on the PC first (robot.set raises MotionLocked, so the loop
         thread cannot send another S), then every servo is made limp. The gripper drops what it
         holds and the top half sinks. Needs reset before the next start.
"""

import threading
import time

import config
import servo_cal
from robot_serial import MotionLocked
from skills import Aborted, Skills


def run_unscrew(sk: Skills, agent, snap, log, cycles: int, should_stop) -> None:
    """home -> grip bottle -> twist cycles with periodic lift tests. Raises Aborted on stop."""
    from agent import side_by_side

    sk.home()
    sk.calibrate_tactile()
    g = sk.grip(config.BOTTOM_SQUEEZE)
    log("grip_bottle", **g.__dict__)
    if not g.contact:
        log("stop", reason="bottom fingers did not touch the bottle")
        return

    fraction = config.DEFAULT_TWIST_FRACTION
    for cycle in range(1, cycles + 1):
        if should_stop():
            raise Aborted()
        res = agent.poll()                       # newest decision, if one finished
        if isinstance(res, Exception):
            log("agent_error", error=repr(res)[:300])
        elif res is not None:
            log("decision", **res.model_dump())
            if res.action in ("stop_success", "stop_problem"):
                log("stop", reason=res.action, observation=res.observation)
                return
            if res.action == "regrip_bottom":
                sk.open(config.BOTTOM_SQUEEZE)
                log("grip_bottle", **sk.grip(config.BOTTOM_SQUEEZE).__dict__)
            fraction = res.twist_fraction

        before = snap()
        summary = sk.twist_cycle(fraction)
        after = snap()
        log("cycle", cycle=cycle, summary=summary)
        log.save_cycle(cycle, side_by_side(before, after), summary)

        agent.note(f"cycle {cycle}: ok={summary['ok']} grip_reason={summary['grip']['reason']} "
                   f"grip_angle={summary['grip']['angle']} fraction={fraction}")
        if agent.submit(cycle, before, after, summary):
            log("agent_asked", cycle=cycle)

        if cycle % config.LIFT_TEST_EVERY == 0 and lift_test(sk, agent, snap, log, cycle):
            log("stop", reason="stop_success", observation="cap lifted off")
            return
    log("stop", reason=f"reached max cycles ({cycles})")


def lift_test(sk, agent, snap, log, cycle) -> bool:
    """Grip cap, raise a few mm, ask the supervisor if the cap came off. True = off (and lifted)."""
    from agent import side_by_side

    before = snap()
    g = sk.lift_test_grip()
    if not g.contact:
        log("lift_test", cycle=cycle, result="missed the cap")
        sk.open(config.TOP_SQUEEZE)
        return False
    after = snap()
    verdict = agent.check_lift(before, after)
    log.save_cycle(cycle, side_by_side(before, after), {"lift_test": True, "grip": g.__dict__},
                   name=f"lift{cycle:02d}")
    if verdict is None:
        log("lift_test", cycle=cycle, result="no verdict (rules / budget / error), keep twisting")
        sk.lift_test_finish(cap_off=False)
        return False
    log("lift_test", cycle=cycle, **verdict.model_dump())
    off = verdict.visible and verdict.cap_off and verdict.confidence >= 0.6
    sk.lift_test_finish(cap_off=off)
    return off


class Controller:
    def __init__(self, robot, cam, log, reversed_flags: dict[int, int]):
        self.robot, self.cam, self.log = robot, cam, log
        self.reversed_flags = reversed_flags       # firmware R flags, for limp_all()
        self.sk = Skills(robot)
        self.snap = (lambda: cam.jpeg()) if cam else (lambda: None)
        self.state = "idle"
        self.run_info: dict = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    # ---------- public API (web + CLI) ----------

    def status(self) -> dict:
        from agent import Budget
        return {"state": self.state, **self.run_info,
                "gemini_used_today": Budget(config.GEMINI_MODEL).used_today(),
                "gemini_daily_cap": config.GEMINI_DAILY_CAP, "gemini_model": config.GEMINI_MODEL,
                "has_camera": self.cam is not None}

    def start(self, gemini: bool = False, cycles: int = config.MAX_CYCLES) -> tuple[bool, str]:
        with self._lock:
            if self.state == "estopped":
                return False, "E-stop is active: press Reset first"
            if self.state != "idle":
                return False, f"already {self.state}"
            if gemini and self.cam is None:
                return False, "Gemini needs the camera (main.py was started with --no-camera)"
            from agent import GeminiSupervisor, RuleSupervisor
            agent = GeminiSupervisor(log=self.log) if gemini else RuleSupervisor()
            self._stop.clear()
            self.sk.abort.clear()
            self.run_info = {"gemini": gemini, "cycles": cycles, "started": time.time()}
            self._set_state("running")
            self._thread = threading.Thread(target=self._worker, args=(agent, cycles), daemon=True)
            self._thread.start()
        self.log("run_start", gemini=gemini, model=config.GEMINI_MODEL if gemini else "rules", cycles=cycles)
        return True, "started"

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            if self.state != "running":
                return False, f"not running ({self.state})"
            self._set_state("stopping")
            self._stop.set()
            self.sk.abort.set()          # current move aborts at its next tick
        return True, "stopping"

    def estop(self) -> tuple[bool, str]:
        # no lock: must never wait behind anything
        self.robot.motion_locked = True  # 1) the loop thread can't send another S
        self._stop.set()
        self.sk.abort.set()
        try:
            servo_cal.limp_all(self.robot, self.reversed_flags)   # 2) every servo limp
            limp = "all servos limp"
        except Exception as e:           # serial gone: motion is still locked on the PC
            limp = f"LIMP COMMAND FAILED ({e!r}); motion is locked on the PC. Cut servo power."
        self._set_state("estopped")
        self.log("estop", result=limp)
        return True, limp

    def reset(self) -> tuple[bool, str]:
        with self._lock:
            if self.state != "estopped":
                return False, f"not E-stopped ({self.state})"
            if self._thread and self._thread.is_alive():
                return False, "run is still unwinding, try again in a second"
            self.robot.motion_locked = False
            self.sk._cmd.clear()         # positions unknown: everything is limp now
            self._set_state("idle")
        self.log("reset", note="servos stay limp until the next Start (first move jumps to the home pose)")
        return True, "reset"

    def wait(self):
        while self._thread and self._thread.is_alive():
            self._thread.join(0.2)

    # ---------- internals ----------

    def _set_state(self, s: str):
        self.state = s
        self.log("status", **self.status())

    def _worker(self, agent, cycles):
        try:
            run_unscrew(self.sk, agent, self.snap, self.log, cycles, self._stop.is_set)
        except (Aborted, MotionLocked):
            if self.state != "estopped":
                self.log("stop", reason="stopped by user")
        except Exception as e:
            self.log("stop", reason=f"error: {e!r}"[:300])
        finally:
            if self.state != "estopped":
                try:                     # graceful: let go of the cap, keep holding the bottle
                    self.sk.abort.clear()
                    self.sk.open(config.TOP_SQUEEZE)
                except Exception as e:
                    self.log("cleanup_error", error=repr(e)[:200])
                self._set_state("idle")
