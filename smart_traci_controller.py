#!/usr/bin/env python3
import os, sys, argparse, time, math, json
from collections import defaultdict, deque

# ---------- TraCI bootstrap ----------
def _setup_sumo_tools():
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        raise SystemExit("ERROR: Please set SUMO_HOME to your SUMO install (e.g., C:\\Program Files\\DLR\\SUMO or /usr/share/sumo).")
    tools = os.path.join(sumo_home, "tools")
    if tools not in sys.path:
        sys.path.append(tools)
_setup_sumo_tools()
import traci  # noqa: E402


# ---------- Utility ----------
def ema(prev, new, alpha):
    if prev is None:
        return new
    return alpha * new + (1.0 - alpha) * prev


def is_green_phase(phase_state: str) -> bool:
    return any(c in ("G", "g") for c in phase_state)


def has_emergency_on_lanes(lanes):
    # Detect emergency vClass presence on any given lanes
    for ln in lanes:
        try:
            for vid in traci.lane.getLastStepVehicleIDs(ln):
                if traci.vehicle.getVehicleClass(vid).lower() == "emergency":
                    return True
        except traci.TraCIException:
            pass
    return False


# ---------- Core Controller ----------
class SmartTLSController:
    def __init__(self, tl_id, min_green=10, max_green=45, extend_step=1,
                 ema_alpha=0.35, spillback_occ=0.85, fairness_bonus=0.2,
                 emergency_boost=10.0, starvation_limit=90, ctrl_period=1.0,
                 use_downstream=True, debug=False):
        self.tl = tl_id
        self.min_green = min_green
        self.max_green = max_green
        self.extend_step = extend_step
        self.alpha = ema_alpha
        self.spillback_occ = spillback_occ
        self.fairness_bonus = fairness_bonus     # added to pressure by seconds since served * fairness_bonus
        self.emergency_boost = emergency_boost   # additive pressure when emergency present
        self.starvation_limit = starvation_limit # hard switch if a phase unserved too long
        self.ctrl_period = ctrl_period
        self.use_downstream = use_downstream
        self.debug = debug

        # Build topology from SUMO:
        self._init_topology()

        # State
        self.green_elapsed = 0.0
        self.last_served_time = {i: 0.0 for i in self.green_phases}   # sim time when phase last started green
        self.last_seen = {i: None for i in self.green_phases}         # EMA queues per phase
        self.last_choice = None

    def _init_topology(self):
        # Controlled links & program
        self.ctrl_links = traci.trafficlight.getControlledLinks(self.tl)  # [ [(in,out,via), ...] , ...] per link index
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(self.tl)[0]
        self.phases = logic.getPhases()  # phase.state, phase.duration etc.
        # Map each phase -> set of incoming lanes it serves
        self.phase_in_lanes = []
        self.phase_out_lanes = []
        for p in self.phases:
            in_set, out_set = set(), set()
            for idx, link_group in enumerate(self.ctrl_links):
                if not link_group: 
                    continue
                if idx >= len(p.state): 
                    continue
                sig = p.state[idx]
                if sig in ("G", "g"):  # served in this phase
                    for (ln_in, ln_out, _via) in link_group:
                        if ln_in: in_set.add(ln_in)
                        if ln_out: out_set.add(ln_out)
            self.phase_in_lanes.append(in_set)
            self.phase_out_lanes.append(out_set)

        # Identify green phases (skip pure yellow/all-red)
        self.green_phases = [i for i, ph in enumerate(self.phases) if is_green_phase(ph.state)]
        if not self.green_phases:
            raise RuntimeError(f"No green-capable phases in TLS '{self.tl}'. Check your signal program.")

        if self.debug:
            print(f"[TLS {self.tl}] Phases: {len(self.phases)}, green phases: {self.green_phases}")

    # ---- Measurements ----
    def _upstream_queue(self, lanes):
        q = 0.0
        for ln in lanes:
            try:
                q += traci.lane.getLastStepHaltingNumber(ln)  # vehicles stopped on lane
            except traci.TraCIException:
                pass
        return q

    def _downstream_pressure(self, lanes):
        # If downstream is jammed (high occupancy), penalize that phase
        if not self.use_downstream:
            return 0.0
        occ_sum, cnt = 0.0, 0
        for ln in lanes:
            try:
                occ = traci.lane.getLastStepOccupancy(ln)  # 0..1
                occ_sum += occ
                cnt += 1
            except traci.TraCIException:
                pass
        if cnt == 0:
            return 0.0
        return occ_sum / cnt  # mean occupancy

    def _phase_pressure(self, phase_idx, sim_time):
        in_lanes = self.phase_in_lanes[phase_idx]
        out_lanes = self.phase_out_lanes[phase_idx]

        # Base demand = upstream queue
        upstream = self._upstream_queue(in_lanes)

        # Downstream gating
        down_occ = self._downstream_pressure(out_lanes)  # 0..1
        spill_gate = 0.0
        if down_occ >= self.spillback_occ:
            spill_gate = -1000.0  # hard discourage selecting this phase

        # Emergency priority
        emerg = has_emergency_on_lanes(in_lanes)
        emerg_bonus = self.emergency_boost if emerg else 0.0

        # Fairness: time since last served (seconds)
        last_start = self.last_served_time.get(phase_idx, 0.0)
        since_served = max(0.0, sim_time - last_start)
        fair = since_served * self.fairness_bonus

        # Smooth the upstream (EMA) to avoid jitter
        smoothed = ema(self.last_seen.get(phase_idx), upstream, self.alpha)
        self.last_seen[phase_idx] = smoothed

        pressure = (smoothed or 0.0) + emerg_bonus + fair + spill_gate
        return pressure, dict(upstream=upstream, smoothed=smoothed, down_occ=down_occ,
                              emerg=emerg, fair=fair, spill=spill_gate)

    # ---- Decision ----
    def step(self):
        sim_time = traci.simulation.getTime()
        cur = traci.trafficlight.getPhase(self.tl)
        rem = traci.trafficlight.getPhaseDuration(self.tl)

        # If we are in yellow/all-red, let SUMO progress naturally
        if cur not in self.green_phases:
            self.green_elapsed = 0.0
            return

        # Compute pressures for all green phases
        pressures = {}
        details = {}
        for i in self.green_phases:
            p, meta = self._phase_pressure(i, sim_time)
            pressures[i] = p
            details[i] = meta

        # Best candidate phase
        best = max(pressures, key=pressures.get)
        cur_p = pressures[cur]
        best_p = pressures[best]

        # Hard starvation guard: if any phase not served for too long, force it
        starving = None
        for i in self.green_phases:
            since_served = max(0.0, sim_time - self.last_served_time.get(i, 0.0))
            if since_served >= self.starvation_limit:
                starving = i
                break

        # 1) Enforce minGreen
        if self.green_elapsed < self.min_green:
            traci.trafficlight.setPhaseDuration(self.tl, rem + self.extend_step)
            self.green_elapsed += self.ctrl_period
            self.last_choice = cur
            if self.debug:
                print(f"[{sim_time:.1f}] Hold MIN {cur}; Pcur={cur_p:.2f} best={best}({best_p:.2f})")
            return

        # 2) Starvation override
        if starving is not None and starving != cur:
            # end this green now; SUMO will go to yellow -> next
            traci.trafficlight.setPhaseDuration(self.tl, 0)
            self.green_elapsed = 0.0
            self.last_choice = starving
            if self.debug:
                print(f"[{sim_time:.1f}] Starvation switch to {starving}")
            return

        # 3) If current still competitive and under maxGreen, extend a bit
        if (cur == best or cur_p >= best_p) and self.green_elapsed < self.max_green:
            traci.trafficlight.setPhaseDuration(self.tl, rem + self.extend_step)
            self.green_elapsed += self.ctrl_period
            self.last_choice = cur
            if self.debug:
                meta = details[cur]
                print(f"[{sim_time:.1f}] Extend {cur}; P={cur_p:.2f} (up={meta['upstream']:.1f},down={meta['down_occ']:.2f}) "
                      f"elapsed={self.green_elapsed:.0f}")
            return

        # 4) Otherwise: switch (trigger yellow/all-red by zeroing remaining)
        traci.trafficlight.setPhaseDuration(self.tl, 0)
        self.green_elapsed = 0.0
        self.last_choice = best
        self.last_served_time[best] = sim_time  # record when we *decided* to serve best next (approx)
        if self.debug:
            print(f"[{sim_time:.1f}] Switch {cur} -> {best}; Pcur={cur_p:.2f} Pbest={best_p:.2f}")


def main():
    ap = argparse.ArgumentParser(description="Strong-logic adaptive TLS controller (pressure, spillback, fairness, priority).")
    ap.add_argument("--config", required=True, help="SUMO .sumocfg (e.g., sim_fixed.sumocfg or demo2.sumocfg)")
    ap.add_argument("--sumo-binary", default="sumo-gui", help="sumo or sumo-gui (default: sumo-gui)")
    ap.add_argument("--min-green", type=int, default=10)
    ap.add_argument("--max-green", type=int, default=45)
    ap.add_argument("--extend-step", type=int, default=1)
    ap.add_argument("--ema-alpha", type=float, default=0.35)
    ap.add_argument("--spillback-occ", type=float, default=0.85, help="0..1 occupancy threshold to gate greens")
    ap.add_argument("--fairness-bonus", type=float, default=0.2, help="added pressure per sec since last served")
    ap.add_argument("--emergency-boost", type=float, default=10.0)
    ap.add_argument("--starvation-limit", type=int, default=90, help="seconds until phase must be served")
    ap.add_argument("--ctrl-period", type=float, default=1.0)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    traci.start([args.sumo_binary, "-c", args.config])
    try:
        tl_ids = traci.trafficlight.getIDList()
        if not tl_ids:
            raise SystemExit("No traffic lights in this network. Convert node to traffic_light or enable --tls.guess during netconvert.")
        # If multiple, control all (you can extend to per-TLS instances)
        controllers = []
        for tl in tl_ids:
            ctl = SmartTLSController(
                tl_id=tl,
                min_green=args.min_green,
                max_green=args.max_green,
                extend_step=args.extend_step,
                ema_alpha=args.ema_alpha,
                spillback_occ=args.spillback_occ,
                fairness_bonus=args.fairness_bonus,
                emergency_boost=args.emergency_boost,
                starvation_limit=args.starvation_limit,
                ctrl_period=args.ctrl_period,
                use_downstream=True,
                debug=args.debug
            )
            controllers.append(ctl)

        # Align with step-length in cfg if you want:
        # step_len = traci.simulation.getDeltaT()/1000.0

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            for ctl in controllers:
                ctl.step()
        if args.debug:
            print("[INFO] Simulation finished.")
    finally:
        traci.close()


if __name__ == "__main__":
    main()
