import os, sys, argparse, json
from datetime import timedelta

# ---- TraCI bootstrap ----
def _setup_sumo_tools():
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        raise SystemExit("ERROR: Please set SUMO_HOME (location of your SUMO install).")
    tools = os.path.join(sumo_home, "tools")
    if tools not in sys.path:
        sys.path.append(tools)
_setup_sumo_tools()
import traci  # noqa: E402

# ---- Helpers ----
def fmt_span(start_s: float, end_s: float) -> str:
    """Return 'mm:ss-mm:ss'."""
    def mmss(x): return str(timedelta(seconds=int(x)))[2:7]
    return f"{mmss(start_s)}-{mmss(end_s)}"

def vclass_bucket(vclass: str) -> str:
    """Map SUMO vClass to our 4 buckets."""
    v = (vclass or "").lower()
    if v in {"bus"}: return "bus"
    if v in {"truck", "trailer"}: return "truck"
    if v in {"emergency"}: return "ambulance"
    # treat all other motorized/light classes as 'car'
    return "car"

def parse_dir_map(arg: str):
    """
    Parse mapping like:
      'N_in:N_S,S_in:S_N,E_in:E_W,W_in:W_E'
    or for two-lane line:
      'E0_:E_W,-E0_:W_E'
    It matches by *prefix* against lane IDs (e.g., 'N_in' matches lanes 'N_in_0', 'N_in_1', ...).
    Returns list of (prefix, label) in the given order.
    """
    pairs = []
    for part in arg.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"Bad mapping entry '{part}'. Expected 'prefix:LABEL'.")
        pref, lab = part.split(":", 1)
        pairs.append((pref.strip(), lab.strip()))
    return pairs

def label_for_lane(lane_id: str, pref_map):
    """Return the first label whose prefix matches lane_id, else None."""
    for pref, lab in pref_map:
        if lane_id.startswith(pref):
            return lab
    return None

def new_bucket(labels):
    return {lab: {"car": set(), "bus": set(), "truck": set(), "ambulance": set()} for lab in labels}

# ---- Main ----
def main():
    ap = argparse.ArgumentParser(description="Emit real-time lane-direction vehicle counts as JSON.")
    ap.add_argument("--config", required=True, help="SUMO .sumocfg to run (e.g., sim_fixed.sumocfg)")
    ap.add_argument("--sumo-binary", default="sumo-gui", help="sumo or sumo-gui (default: sumo-gui)")
    ap.add_argument("--bin", type=int, default=5, help="bin size (seconds), default 5")
    ap.add_argument("--intersection-id", default="id1", help="value for 'intersection_id' in output")
    ap.add_argument(
        "--dir-map",
        default="N_in:N_S,S_in:S_N,E_in:E_W,W_in:W_E",
        help=("Comma list of 'lanePrefix:Label'. "
              "Example 4-leg: 'N_in:N_S,S_in:S_N,E_in:E_W,W_in:W_E' "
              "Example 2-lane line: 'E0_:E_W,-E0_:W_E' "
              "(prefix matches start of lane IDs like 'E0__0')"))
    args = ap.parse_args()

    pref_map = parse_dir_map(args.dir_map)
    out_labels = list({lab for _, lab in pref_map})

    traci.start([args.sumo_binary, "-c", args.config])
    try:
        # optional: use SUMO step length if you want, otherwise 1s is fine
        # step_len = traci.simulation.getDeltaT() / 1000.0

        bin_start = 0.0
        bin_end = args.bin
        seen = new_bucket(out_labels)

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()  # seconds (float)

            # Collect this step
            for vid in traci.vehicle.getIDList():
                try:
                    lane_id = traci.vehicle.getLaneID(vid)
                    lab = label_for_lane(lane_id, pref_map)
                    if lab is None:
                        continue
                    bucket = vclass_bucket(traci.vehicle.getVehicleClass(vid))
                    seen[lab][bucket].add(vid)
                except traci.TraCIException:
                    continue

            # End of bin → emit JSON
            if t >= bin_end:
                payload = {
                    "intersection_id": args.intersection_id,
                    "timestamp": fmt_span(bin_start, bin_end),
                    "lanes": {lab: {k: len(vs) for k, vs in seen[lab].items()} for lab in out_labels}
                }
                print(json.dumps(payload, ensure_ascii=False))
                # prepare next bin
                bin_start = bin_end
                bin_end = bin_start + args.bin
                seen = new_bucket(out_labels)

        # Flush final partial bin if anything was seen
        if any(len(vs) for lab in out_labels for vs in seen[lab].values()):
            payload = {
                "intersection_id": args.intersection_id,
                "timestamp": fmt_span(bin_start, bin_end),
                "lanes": {lab: {k: len(vs) for k, vs in seen[lab].items()} for lab in out_labels}
            }
            print(json.dumps(payload, ensure_ascii=False))

    finally:
        traci.close()

if __name__ == "__main__":
    main()

