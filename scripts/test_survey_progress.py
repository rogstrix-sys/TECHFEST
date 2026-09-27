import sys, os, time
sys.path.insert(0, os.path.abspath("."))
from vis.server import create_default_simulation

sim = create_default_simulation()
print(f"[*] Fleet: {len(sim.drones)} Drones, {len(sim.pois)} POIs, {len(sim.mission_manager.survivors)} Survivors", flush=True)

# Run up to 3000 steps (dt = 0.05 -> 150 seconds)
for step in range(3000):
    sim.step(0.05)
    while len(sim.history) > 50:
        sim.history.popleft()
    if (step + 1) % 150 == 0:
        pois = sim.pois
        survs = sim.mission_manager.survivors
        discovered = [s for s in survs.values() if s.discovered]
        completed = [p for p in pois.values() if p.get("is_completed", False)]
        print(f"[t={(step+1)*0.05:5.1f}s] POIs Completed: {len(completed)}/{len(pois)}, Survivors Located: {len(discovered)}/{len(survs)}", flush=True)
        if len(completed) == len(pois) and len(discovered) == len(survs):
            print(f"[SUCCESS] All {len(pois)} POIs completed and all {len(survs)} survivors located at t={(step+1)*0.05:.1f}s!", flush=True)
            break

pois = sim.pois
survs = sim.mission_manager.survivors
discovered = [s for s in survs.values() if s.discovered]
completed = [p for p in pois.values() if p.get("is_completed", False)]
print(f"\n--- Final Status at t={(step+1)*0.05:.1f}s ---", flush=True)
print(f"POIs Completed: {len(completed)}/{len(pois)}", flush=True)
print(f"Survivors Located: {len(discovered)}/{len(survs)}", flush=True)

for p_id in sorted(pois.keys()):
    p = pois[p_id]
    print(f"  {p_id:15s}: completed={p.get('is_completed')}, dwell={p.get('current_dwell_time', 0.0):.1f}/{p.get('required_dwell_time', 0.0)}s, assigned={p.get('assigned_drone_id')}", flush=True)
