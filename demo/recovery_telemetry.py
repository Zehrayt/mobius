"""Yerden kalkma telemetrisi (Adim 34-39): motor torklari, dis kuvvet cifti, yuk paylasimi.

Usage:
python3 demo/recovery_telemetry.py            # +-150 px, tam zincir (yurumeye donus dahil)

Birimler (README "Kalkma telemetrisi"):
- kutle birimi = 70 kg / 5.18, 1 px = 0.9 m / 184, 1 kare = 1/30 s
- motor torku (kutle*px^2/kare^2) x 0.2909 = Nm  (480 birim = 140 Nm)
- kuvvet (kutle*px/kare^2) x 59.47 = N           (agirlik 5.18*g = 687 N)

Motorlar (physics/ground_recovery.py `_motor`) eklemde iki parca arasinda degil, tek
bir parcaya (a, b noktalari) uygulanan, dunyaya referansli kuvvet ciftleridir. Gercek
bir vucutta ic kas torklarinin toplami sifirdir; tum govdeyi yalnizca zemin
dondurebilir. Bu yuzden her karedeki motor ciftlerinin TOPLAMI ("net dis cift")
fiziksel olmayan acisal momentum girdisinin dogrudan olcusudur.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14  # noqa: E402

MASS_KG = 70.0 / 5.18
PX_M = 0.9 / 184.0
FPS = 30.0
NM = MASS_KG * PX_M ** 2 * FPS ** 2          # 0.2909
NEWTON = MASS_KG * PX_M * FPS ** 2           # 59.47
HIP_LIMIT_NM = 140.0                         # Adim 23 kalca torku tavani
CHAIN = dict(ground_recovery=True, recovery_reposition=True, recovery_transfer=True,
             recovery_rise=True, recovery_stand=True, recovery_walk=True)


def pair_name(sim, a, b):
    names = {sim.idx["hip"]: "kalca", sim.idx["shoulder"]: "omuz", sim.idx["head"]: "bas"}
    if "waist" in sim.idx:
        names[sim.idx["waist"]] = "bel"
    for side, (k, f) in sim.fallen_legs.items():
        names[k], names[f] = "diz_" + side, "ayak_" + side
    for side, (e, h) in sim.arms.idx.items():
        names[e], names[h] = "dirsek_" + side, "el_" + side
    return f"{names.get(a, a)}->{names.get(b, b)}"


def run(push: float, frames: int = 3700, **overrides) -> dict:
    sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=7.0, **{**CHAIN, **overrides})
    per_frame = []
    prev_com_v = None
    prev_contact_x = {}
    for _ in range(frames):
        sim.step()
        if not sim.collapsed or sim.recovery is None:
            continue
        body = sim.body
        p, q, m = body.points, body.prev_points, body.masses
        free = [i for i in range(len(p)) if i not in body.pinned]
        com_v = np.average((p - q)[free], axis=0, weights=m[free])
        com_a = None if prev_com_v is None else float(np.linalg.norm(com_v - prev_com_v))
        prev_com_v = com_v
        # bu karede zemin cozucusunun temas saydigi noktalar
        contact = {i for f, i, _v in sim.ground_contacts[-len(p) * 2:] if f == sim.frame}
        slips = [abs(float(p[i, 0] - prev_contact_x[i])) for i in contact if i in prev_contact_x]
        prev_contact_x = {i: float(p[i, 0]) for i in contact}
        residual = max(abs(float(np.linalg.norm(p[i] - p[j])) - L)
                       for i, j, L, c in body.sticks if c < 0.5 and i not in body.pinned and j not in body.pinned)
        per_frame.append(dict(frame=sim.frame, state=sim.recovery.state, com_accel=com_a,
                              slip=max(slips) if slips else 0.0, residual=residual))
    rec = sim.recovery
    by_frame = {}
    motors = {}
    for f, state, a, b, tau, demand, cap, *rest in rec.motor_log:
        by_frame.setdefault(f, {"state": state, "sum": 0.0, "abs": 0.0})
        by_frame[f]["sum"] += tau
        by_frame[f]["abs"] += abs(tau)
        key = (state, pair_name(sim, a, b))
        d = motors.setdefault(key, dict(n=0, sat=0, peak=0.0, peak_demand=0.0, cap=cap))
        d["n"] += 1
        d["sat"] += abs(demand) > cap + 1e-9
        d["peak"] = max(d["peak"], abs(tau))
        d["peak_demand"] = max(d["peak_demand"], abs(demand))
        d["cap"] = max(d["cap"], cap)      # baslangic rampasinda (gain < 1) tavan kucuk kaydedilir
    for row in rec.force_log:
        f, state = row[0], row[1]
        if len(row) > 7:      # yurume: 2B kalca-ayak kuvvet cifti (eksen disi bilesen = cift)
            force, delta = row[7], row[8]
            couple = float(delta[0] * force[1] - delta[1] * force[0])
            by_frame.setdefault(f, {"state": state, "sum": 0.0, "abs": 0.0})
            by_frame[f]["sum"] += couple
            by_frame[f]["abs"] += abs(couple)
    for f, state, c, d, tau in rec.reaction_log:      # ic tork tepkileri (INTERNAL_TORQUES)
        by_frame.setdefault(f, {"state": state, "sum": 0.0, "abs": 0.0})
        by_frame[f]["sum"] += tau
    states = []
    for r in per_frame:
        if r["state"] not in states:
            states.append(r["state"])
    summary = {}
    for st in states:
        rows = [r for r in per_frame if r["state"] == st]
        fr = [v for f, v in by_frame.items() if v["state"] == st]
        net = [abs(v["sum"]) * NM for v in fr]
        cancel = [1.0 - abs(v["sum"]) / v["abs"] for v in fr if v["abs"] > 1e-9]
        ms = {name: dict(peak_nm=round(d["peak"] * NM, 1), peak_demand_nm=round(d["peak_demand"] * NM, 1),
                         cap_nm=round(d["cap"] * NM, 1), saturated_pct=round(100.0 * d["sat"] / d["n"], 1))
              for (s_, name), d in motors.items() if s_ == st}
        summary[st] = dict(
            frames=len(rows),
            net_external_couple_nm=dict(mean=round(float(np.mean(net)), 1) if net else 0.0,
                                        max=round(float(np.max(net)), 1) if net else 0.0),
            motor_cancellation_pct=round(100.0 * float(np.mean(cancel)), 1) if cancel else None,
            com_accel_peak_ms2=round(max((r["com_accel"] or 0.0) for r in rows) * PX_M * FPS ** 2, 2),
            contact_slip_peak_mm_per_frame=round(max(r["slip"] for r in rows) * PX_M * 1000, 2),
            stick_residual_peak_px=round(max(r["residual"] for r in rows), 3),
            motors=ms)
    hip_trunk = [d["peak"] * NM for (s_, name), d in motors.items() if name in ("kalca->omuz",)]
    thigh = [d["peak"] * NM for (s_, name), d in motors.items() if name.startswith("diz_") and name.endswith("->kalca")]
    hip_sat = [(s_, round(100.0 * d["sat"] / d["n"], 1)) for (s_, name), d in motors.items()
               if (name == "kalca->omuz" or (name.startswith("diz_") and name.endswith("->kalca"))) and d["sat"]]
    leg_forces = [row[4] * NEWTON for row in rec.force_log if row[1] in ("standing_rising", "standing")
                  and len(row) <= 7 and row[2] in [f for _, f in sim.fallen_legs.values()]]
    return dict(push_px=push, final_state=rec.state, failure_reason=rec.failure_reason,
                hip_trunk_peak_nm=round(max(hip_trunk), 1) if hip_trunk else None,
                thigh_peak_nm=round(max(thigh), 1) if thigh else None,
                hip_limit_nm=HIP_LIMIT_NM,
                hip_motor_saturation_by_state=hip_sat,
                stand_leg_force_peak_n=round(max(leg_forces), 1) if leg_forces else None,
                body_weight_n=round(float(sum(sim.body.masses[i] for i in range(len(sim.body.masses))
                                              if i not in sim.body.pinned)) * float(sim.body.gravity[1]) * NEWTON, 1),
                states=summary)


def main():
    report = dict(units=dict(torque_nm_per_unit=NM, newton_per_unit=NEWTON), runs=[run(150.0), run(-150.0)])
    out = ROOT / "docs" / "validation" / "recovery_telemetry_report.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    for r in report["runs"]:
        print(f"itki {r['push_px']:+.0f} px -> {r['final_state']}  kalca(govde) tepe {r['hip_trunk_peak_nm']} Nm, "
              f"uyluk tepe {r['thigh_peak_nm']} Nm, sinir {r['hip_limit_nm']} Nm; ayakta bacak kuvveti tepe "
              f"{r['stand_leg_force_peak_n']} N (agirlik {r['body_weight_n']} N)")
        for st, d in r["states"].items():
            print(f"  {st:18s} {d['frames']:5d} kare | net dis cift ort {d['net_external_couple_nm']['mean']:7.1f} "
                  f"max {d['net_external_couple_nm']['max']:7.1f} Nm | iptal %{d['motor_cancellation_pct']} | "
                  f"KM ivme tepe {d['com_accel_peak_ms2']} m/s2 | kayma {d['contact_slip_peak_mm_per_frame']} mm/kare | "
                  f"cubuk hatasi {d['stick_residual_peak_px']} px")
    print(out)


if __name__ == "__main__":
    main()
