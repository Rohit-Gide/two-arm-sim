"""What a real cell has and an ideal simulation does not: the errors between what the controllers BELIEVE and what is.

A `Reality` is one draw of those errors (from a seed). The physics is built from the true values; the controllers,
the planner and your code keep working from the nominal ones, exactly as they would on site.

    Reality.ideal()        no errors: the controller's model is the world
    Reality.sample(seed)   one plausible cell

The sizes (one standard deviation unless said) are typical figures I chose, NOT measurements of your cell:
change SIGMA below, or pass your own to Reality.sample(seed, **overrides).

  base_mm, base_yaw_deg   where each arm really stands against where the layout says (cell calibration)
  zero_deg                each joint's encoder zero (mastering): this is the arm's absolute-accuracy error
  part_mm, part_yaw_deg   where a part really lies against where the program expects it (no camera here)
  mass_frac               each link's true mass against the controller's model
  friction_frac           true gearbox friction against the estimate
  pad_mu                  spread of the jaw pads' friction around 0.9
  clock_ms                how far apart the two controllers' clocks are when they start a move "together"
                          (uniform between 0 and this)
  flex                    gearboxes are springs: each joint twists 0.6 degrees at its motor's torque limit.
                          The encoder is on the motor side, so the controller does not see the twist.
"""
from __future__ import annotations

import copy
import dataclasses
import math

import numpy as np

from . import robot as R
from . import scene as S

SIGMA = dict(base_mm=1.0, base_yaw_deg=0.03, zero_deg=0.02, part_mm=3.0, part_yaw_deg=1.5, mass_frac=0.05,
             friction_frac=0.20, pad_mu=0.10, clock_ms=8.0, flex=1.0)
FLEX_AT_LIMIT_RAD = math.radians(0.6)
FACTORS = ("base", "zero", "part", "model", "pad", "clock", "flex")
#: which SIGMA entries each factor switches on
FACTOR_KEYS = dict(base=("base_mm", "base_yaw_deg"), zero=("zero_deg",), part=("part_mm", "part_yaw_deg"),
                   model=("mass_frac", "friction_frac"), pad=("pad_mu",), clock=("clock_ms",), flex=("flex",))


@dataclasses.dataclass
class Reality:
    seed: int | None
    sigma: dict
    bases: dict            # arm -> (x, y, z, yaw) true
    zero: dict             # arm -> 6 rad: true joint angle = encoder + zero
    parts: dict            # part -> ((x, y, z), yaw) true start
    link_scale: dict       # arm -> {link: true mass / model mass}
    friction_scale: dict   # arm -> 6: true gearbox friction / estimate
    pad_mu: float
    clock_cycles: int      # arm `late` starts a two-arm move this many 4 ms cycles after the other
    late: str
    flex: bool

    @staticmethod
    def ideal() -> "Reality":
        return Reality.sample(None, **{k: 0.0 for k in SIGMA})

    @staticmethod
    def sample(seed, only=None, **overrides) -> "Reality":
        """only: a factor name (or a list of them) to switch on alone, everything else exact."""
        sg = dict(SIGMA)
        if only is not None:
            keep = {k for f in ([only] if isinstance(only, str) else only) for k in FACTOR_KEYS[f]}
            sg = {k: (v if k in keep else 0.0) for k, v in sg.items()}
        sg.update(overrides)
        rng = np.random.default_rng(seed)
        n = rng.normal
        bases, zero, link_scale, fr = {}, {}, {}, {}
        for p, (x, y, yaw) in S.ARMS.items():               # the same draws whatever is switched on, so runs compare
            d = n(size=4)
            bases[p] = (x + d[0] * sg["base_mm"] / 1e3, y + d[1] * sg["base_mm"] / 1e3, d[2] * sg["base_mm"] / 2e3,
                        yaw + math.radians(d[3] * sg["base_yaw_deg"]))
            zero[p] = np.radians(n(size=6) * sg["zero_deg"])
            link_scale[p] = {l: float(1 + s * sg["mass_frac"]) for l, s in zip(R.LINKS, n(size=7))}
            fr[p] = np.clip(1 + n(size=6) * sg["friction_frac"], 0.3, 2.0)
        parts = {}
        for k, v in S.PARTS.items():
            d = n(size=3)
            parts[k] = ((v["pos"][0] + d[0] * sg["part_mm"] / 1e3, v["pos"][1] + d[1] * sg["part_mm"] / 1e3, v["pos"][2]),
                        v["yaw"] + math.radians(d[2] * sg["part_yaw_deg"]))
        pad = float(np.clip(0.9 + n() * sg["pad_mu"], 0.3, 1.5))
        u, late = rng.uniform(), ("a", "b")[int(rng.integers(2))]
        return Reality(seed, sg, bases, zero, parts, link_scale, fr, pad, int(round(u * sg["clock_ms"] / 4.0)), late, sg["flex"] > 0)

    def world(self, dyn: dict) -> tuple[dict, dict]:
        """(the true dynamic parameters, the true world layout) for scene.mjcf."""
        true = copy.deepcopy(dyn)
        true["friction"]["finger_pad"] = self.pad_mu
        flex = None
        if self.flex:
            flex = {jn: (j["torque_limit"] / FLEX_AT_LIMIT_RAD, 0.1 * math.sqrt(j["torque_limit"] / FLEX_AT_LIMIT_RAD * j["armature"]))
                    for jn, j in dyn["joints"].items()}
        return true, dict(bases=self.bases, parts=self.parts, flex=flex, link_scale=self.link_scale,
                          friction_scale={p: v.tolist() for p, v in self.friction_scale.items()})

    def summary(self) -> dict:
        return {"seed": self.seed, "sigma": self.sigma,
                "base_error_mm": {p: [round((self.bases[p][i] - (list(S.ARMS[p][:2]) + [0.0])[i]) * 1e3, 2) for i in range(3)] for p in S.ARMS},
                "base_yaw_error_deg": {p: round(math.degrees(self.bases[p][3] - S.ARMS[p][2]), 4) for p in S.ARMS},
                "joint_zero_error_deg": {p: np.degrees(self.zero[p]).round(4).tolist() for p in S.ARMS},
                "part_error_mm_deg": {k: [round((self.parts[k][0][0] - S.PARTS[k]["pos"][0]) * 1e3, 2),
                                          round((self.parts[k][0][1] - S.PARTS[k]["pos"][1]) * 1e3, 2),
                                          round(math.degrees(self.parts[k][1] - S.PARTS[k]["yaw"]), 3)] for k in S.PARTS},
                "pad_friction": round(self.pad_mu, 3), "clock_offset_ms": self.clock_cycles * 4, "late_arm": self.late,
                "gearbox_flex": self.flex}
