"""Runs the unscrew loop in a worker thread so it can be started / stopped / E-stopped from the
web UI (twin_server.py -> /api/*) or the CLI (main.py --autostart).

States: idle -> running -> (stopping) -> idle;  any -> estopped -> (reset) -> idle

  stop   graceful: the current move is aborted at its next tick, then the top fingers let go of
         the cap (bottle stays held). Use it to end a run.
  skill  run_skill(name, params): one named skill from skill_registry.py (twin Skills panel).
         Same states, Stop and E-stop. A stopped/finished skill leaves the gripper where it is.
  plan   run_plan([{name, params}, ...]): several skills in one job (e.g. a Gemini vision plan),
         stops at the first blocked / failed step.
  estop  immediate: motion is LOCKED on the PC first (robot.set raises MotionLocked, so the loop
         thread cannot send another S), then every servo is made limp. The gripper drops what it
         holds and the top half sinks. Needs reset before the next start.
"""

import threading
import time

import config
import servo_cal
import skill_registry
from robot_serial import MotionLocked
from skills import Aborted, GripHolder, Skills


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
        sim = getattr(sk.robot, "sim_state", None)
        if callable(sim) and sim().get("cap_off"):     # simulator knows: grip the loose cap and lift it clear
            if sk.grip(config.TOP_SQUEEZE).contact:
                sk.move({config.HALVES.idx: config.HALVES.full}, config.LIFT_DEG_PER_TICK,
                        hook=GripHolder(sk, config.TOP_SQUEEZE, config.GRIP_TARGET_MA[config.TOP_SQUEEZE.idx]))
            log("stop", reason="stop_success", observation=f"cap off after {cycle} cycles, lifted clear")
            return
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
        self.frame_source = None        # set by main.py to Twin.request_frame (sim top cam from the page)
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
            sim = getattr(self.robot, "sim_state", None)
            if callable(sim) and sim().get("task") != "unscrew":
                return False, "Start runs the unscrew task: load the pill tube / bottle (or use Demo: eye drops)"
            from agent import GeminiSupervisor, RuleSupervisor
            agent = GeminiSupervisor(log=self.log) if gemini else RuleSupervisor()
            self._launch(lambda: run_unscrew(self.sk, agent, self.snap, self.log, cycles, self._stop.is_set),
                         {"gemini": gemini, "cycles": cycles}, cleanup=True)
        self.log("run_start", gemini=gemini, model=config.GEMINI_MODEL if gemini else "rules", cycles=cycles)
        return True, "started"

    def skills(self) -> list[dict]:
        """Skill catalog with a live 'blocked' reason for each (None = can run now)."""
        return skill_registry.catalog(self.sk)

    def run_skill(self, name: str, params: dict | None = None) -> tuple[bool, str]:
        skill = skill_registry.BY_NAME.get(name)
        if skill is None:
            return False, f"unknown skill {name!r}"
        with self._lock:
            if self.state == "estopped":
                return False, "E-stop is active: press Reset first"
            if self.state != "idle":
                return False, f"already {self.state}"
            why = skill_registry.blocked(skill, self.sk)
            if why:
                return False, f"{skill.label}: {why}"
            kw = {p.name: p.clamp((params or {}).get(p.name, p.default)) for p in skill.params}

            def job():
                t0 = time.time()
                res = skill.run(self.sk, **kw)
                self.log("skill", name=name, label=skill.label, params=kw,
                         duration_s=round(time.time() - t0, 2), result=res)

            self._launch(job, {"skill": name, "label": skill.label, "params": kw}, cleanup=False)
        self.log("skill_start", name=name, label=skill.label, params=kw)
        return True, f"running {skill.label}"

    def run_plan(self, steps: list[dict]) -> tuple[bool, str]:
        """Several skills in order, in ONE worker job (same lock / states / Stop / E-stop as run_skill).
        Each step is re-checked (blocked()) right before it runs; the plan stops at the first
        blocked / failed (result ok False) step or on Stop. Logs 'plan_step' per step + a final 'plan'."""
        plan = []
        for st in steps or []:
            name = str((st or {}).get("name", ""))
            skill = skill_registry.BY_NAME.get(name)
            if skill is None:
                return False, f"unknown skill {name!r}"
            params = st.get("params") if isinstance(st.get("params"), dict) else {}
            plan.append((skill, {p.name: p.clamp(params.get(p.name, p.default)) for p in skill.params}))
        if not plan:
            return False, "empty plan"
        if len(plan) > 20:
            return False, "plan too long (max 20 steps)"
        with self._lock:
            if self.state == "estopped":
                return False, "E-stop is active: press Reset first"
            if self.state != "idle":
                return False, f"already {self.state}"
            why = skill_registry.blocked(plan[0][0], self.sk)   # fail fast on the first step
            if why:
                return False, f"step 1 {plan[0][0].label}: {why}"
            names = [s.name for s, _ in plan]

            def job():
                t0, done, outcome = time.time(), 0, "done"
                try:
                    for i, (skill, kw) in enumerate(plan, 1):
                        if self._stop.is_set():
                            raise Aborted()
                        why = skill_registry.blocked(skill, self.sk)
                        if why:
                            outcome = f"blocked at step {i} ({skill.name}): {why}"
                            self.log("plan_step", i=i, n=len(plan), name=skill.name, params=kw, blocked=why)
                            break
                        self.run_info.update(step=i, skill=skill.name, label=skill.label)
                        ts = time.time()
                        res = skill.run(self.sk, **kw)
                        self.log("plan_step", i=i, n=len(plan), name=skill.name, label=skill.label, params=kw,
                                 duration_s=round(time.time() - ts, 2), result=res)
                        if isinstance(res, dict) and res.get("ok") is False:
                            outcome = f"failed at step {i} ({skill.name})"
                            break
                        done = i
                except (Aborted, MotionLocked):
                    outcome = f"stopped at step {done + 1}"
                    raise
                finally:
                    self.log("plan", ok=done == len(plan), steps=names, completed=done, outcome=outcome,
                             duration_s=round(time.time() - t0, 2))

            self._launch(job, {"plan": names, "steps": len(plan)}, cleanup=False)
        self.log("plan_start", steps=names)
        return True, f"running {len(plan)}-step plan"

    def run_agent(self, goal: str, max_steps: int = 8, source: str = "auto") -> tuple[bool, str]:
        """Gemini closed loop (gemini_agent.py): each turn Gemini sees image + sensor text and calls
        ONE skill as a function; runs in one worker job with the same Stop / E-stop / lock.
        source: "camera" (webcam), "sim" (the twin page's simulated top cam), "auto" (camera if any)."""
        if not goal:
            return False, "empty goal"
        with self._lock:
            if self.state == "estopped":
                return False, "E-stop is active: press Reset first"
            if self.state != "idle":
                return False, f"already {self.state}"
            sim = hasattr(self.robot, "sim_info")          # simulator: the page's top cam shows the scene
            use_cam = self.cam is not None and (source == "camera" or (source == "auto" and not sim))
            if source == "camera" and self.cam is None:
                return False, "no webcam (main.py started with --no-camera)"
            if not use_cam and self.frame_source is None:
                return False, "no image source: open the twin page (sim top cam) or start with a camera"
            get_frame = self.cam.jpeg if use_cam else self.frame_source
            try:
                from gemini_agent import GeminiAgent
                ag = GeminiAgent(self.sk, get_frame, self.log)
            except Exception as e:
                return False, f"Gemini unavailable: {e!r}"[:200]

            def job():
                res = ag.run(goal, self._stop.is_set, max_steps)
                self.log("agent_done", goal=goal, **res)

            self._launch(job, {"agent": goal, "label": "Gemini agent", "max_steps": max_steps,
                               "source": "camera" if use_cam else "sim"}, cleanup=False)
        self.log("agent_start", goal=goal, max_steps=max_steps, source="camera" if use_cam else "sim top cam")
        return True, f"Gemini agent running (up to {max_steps} calls)"

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
            import skill_registry
            if skill_registry.MIRROR is not None:               # --real-fingers: real servos limp too
                skill_registry.MIRROR.estop()
        except Exception as e:
            print(f"[estop] real fingers: {e!r}")
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
            self.sk.holding.clear()
            self._set_state("idle")
        self.log("reset", note="servos stay limp until the next Start (first move jumps to the home pose)")
        return True, "reset"

    def wait(self):
        while self._thread and self._thread.is_alive():
            self._thread.join(0.2)

    # ---------- internals ----------

    def _launch(self, fn, info: dict, cleanup: bool):
        """Caller holds self._lock and has checked state == idle."""
        self._stop.clear()
        self.sk.abort.clear()
        self.run_info = {**info, "started": time.time()}
        self._set_state("running")
        self._thread = threading.Thread(target=self._worker, args=(fn, cleanup), daemon=True)
        self._thread.start()

    def _set_state(self, s: str):
        self.state = s
        self.log("status", **self.status())

    def _worker(self, fn, cleanup: bool):
        try:
            fn()
        except (Aborted, MotionLocked):
            if self.state != "estopped":
                self.log("stop", reason="stopped by user")
        except Exception as e:
            self.log("stop", reason=f"error: {e!r}"[:300])
        finally:
            sim = getattr(self.robot, "sim_state", None)
            if callable(sim) and sim().get("cap_off"):
                cleanup = False          # success: keep holding the lifted cap up for the camera
            if self.state != "estopped" and cleanup:
                try:                     # graceful: let go of the cap, keep holding the bottle
                    self.sk.abort.clear()
                    self.sk.open(config.TOP_SQUEEZE)
                except Exception as e:
                    self.log("cleanup_error", error=repr(e)[:200])
            if self.state != "estopped":
                self.run_info = {}
                self._set_state("idle")
