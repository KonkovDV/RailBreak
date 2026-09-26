"""Reference along-track odometer (Python twin of the ROS core).

Numerical state and filter decisions match track_odometer.hpp. lockstep.py
compares s and v. Diagnostic counters are separate: a bad bogie sample and a
stamp behind the filter are dropped here without an n_rejected counter.

State x = [s, v, k, ba]
  s   coordinate on the track ring (m)
  v   speed (m/s)
  k   common wheel scale, true speed = k * corrected bogie speed
  ba  bias of the notch acceleration model (m/s^2), learned while wheels are trusted

Three time scales keep calibration, noise and slip apart:
  * rho, the rear/front ratio, is a slow robust average (minutes) taken only on
    trusted cruising samples. Both bogies are corrected half-way to their
    geometric mean; the absolute level is k, observable through station anchors.
  * the noise level r is estimated from the corrected front-rear difference.
    It grows only while that difference is sign-balanced (noise); a one-signed
    run (slip or slide on one bogie) freezes it.
  * slip is a fast test: normalised innovation against the prediction, and a
    two-bogie consensus gate. A flagged bogie is not dropped: its update uses
    r_bad, and the speed weight is about Pvv/(Pvv+r_bad). With both flagged
    the notch model carries the prediction; the measurements still update.
Predict:  s += v dt,  v += (a_tab(n, v) - g i(s) + ba) dt.
Anchor:   once per dwell, unique station within the gate, s = s_stop + e.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.80665
N = 4
IS, IV, IK, IBA = range(N)


@dataclass
class Params:
    unit: float = 1.0 / 3.6
    k0: float = 1.0027
    sigma_k0: float = 0.004
    q_k: float = 1e-9
    q_v: float = 0.05
    q_s: float = 1e-4
    q_ba: float = 1e-5
    sigma_ba0: float = 0.05
    r0: float = 0.05 ** 2
    r_min: float = 0.02 ** 2
    r_max: float = 1.0
    r_alpha: float = 0.01          # per paired sample, inside the gate
    r_alpha_grow: float = 0.0005   # per paired sample, outside the gate, sign-balanced
    bias_alpha: float = 0.03       # short signed mean of the difference (~3 s at 10 Hz)
    rho_step: float = 2e-5         # per paired sample, sign-step median tracker
    rho_notch_max: int = 6         # learn the ratio only off heavy traction/braking
    rho_max: float = 0.08
    rho_v_min: float = 3.0
    r_bad: float = 25.0
    nis_gate: float = 16.0
    fr_sigma_gate: float = 4.0
    fr_floor: float = 0.3
    zupt_v: float = 0.05
    zupt_hold_s: float = 1.0
    zupt_noise_mult: float = 3.0
    stop_sd_max: float = 3.0
    stop_sigma_floor: float = 1.0
    stop_gate: float = 3.0
    wheel_stale_s: float = 0.35
    dt_max: float = 0.2
    max_gap_s: float = 30.0
    v_max: float = 30.0
    recover_s: float = 3.0         # bogies agree but the model does not: trust bogies after this
    load_factor: float = 1.0       # scales a_tab only
    davis_a: float = 0.0           # extra resistance; 0 is already inside a_tab
    davis_b: float = 0.0
    davis_c: float = 0.0


def _require_config(p: Params) -> None:
    """Reject a configuration before the first step. The numerical guard is not a substitute."""
    if not math.isfinite(p.unit) or not p.unit > 0.0:
        raise ValueError("wheel_unit_scale must be > 0")
    for name in ("q_s", "q_v", "q_k", "q_ba"):
        value = getattr(p, name)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be >= 0")
    for name in ("r0", "r_min", "r_max", "r_bad"):
        value = getattr(p, name)
        if not math.isfinite(value) or not value > 0.0:
            raise ValueError(f"{name} must be > 0")
    if not math.isfinite(p.sigma_k0) or p.sigma_k0 == 0.0:
        raise ValueError("sigma_k0 variance must be > 0")
    if not math.isfinite(p.sigma_ba0) or p.sigma_ba0 == 0.0:
        raise ValueError("sigma_ba0 variance must be > 0")
    if not math.isfinite(p.k0) or not (0.5 < p.k0 < 1.5):
        raise ValueError("initial k must be in (0.5, 1.5)")


@dataclass
class Odometer:
    branch_s: np.ndarray
    grade: np.ndarray
    table: np.ndarray
    notches: np.ndarray
    v_edges: np.ndarray
    stops: list[dict]
    p: Params = field(default_factory=Params)
    ring_len: float = 0.0

    def __post_init__(self) -> None:
        self.x = np.array([0.0, 0.0, self.p.k0, 0.0])
        self.P = np.diag([1.0, 0.01, self.p.sigma_k0 ** 2, self.p.sigma_ba0 ** 2])
        self.t = None
        self.notch = 0
        self.last = {"front": (None, math.nan), "rear": (None, math.nan)}
        self.r = self.p.r0
        self.log_rho = 0.0
        self.d_bias = 0.0
        self.still_since = None
        self.anchored_this_dwell = False
        self.slip = False
        self.slip_side = {"front": False, "rear": False}
        self.slip_run = {"front": 0, "rear": 0}
        self.slip_nis = {"front": 0.0, "rear": 0.0}
        self.slip_since = None
        self.slip_channel_since = {"front": None, "rear": None}
        # Pre-update copies for the adhesion proxy. predict and update do not read them.
        self.model_resid = {"front": None, "rear": None}
        self.wheel_consensus = None
        self.mode = "WHEELS"
        self.n_anchor = 0
        self.n_guard = 0
        self.anchor_log: list[tuple[float, float, float, float]] = []
        self.have_v = False
        self.step_frozen = False
        self.disagree_since = None
        _require_config(self.p)

    # Numerical stops, not a physical scale or bias. 1/k stays in [2/3, 2].
    SCALE_MIN = 0.5
    SCALE_MAX = 1.5
    BIAS_ABS_MAX = 5.0

    def _state_numerical(self) -> bool:
        if not (self.SCALE_MIN <= self.x[IK] <= self.SCALE_MAX):
            return False
        if abs(self.x[IBA]) > self.BIAS_ABS_MAX:
            return False
        if not np.isfinite(self.x).all() or not np.isfinite(self.P).all():
            return False
        if not np.all(np.diag(self.P) >= 0.0):
            return False
        try:
            np.linalg.cholesky(0.5 * (self.P + self.P.T))
        except np.linalg.LinAlgError:
            return False
        return True

    def _note_slip(self, which: str, flagged: bool, t: float, nis: float) -> None:
        if not math.isfinite(nis) or nis < 0.0:
            nis = 0.0
        self.slip_side[which] = flagged
        self.slip_nis[which] = nis
        self.slip_run[which] = self.slip_run[which] + 1 if flagged else 0
        if flagged:
            if self.slip_channel_since[which] is None:
                self.slip_channel_since[which] = t
        else:
            self.slip_channel_since[which] = None
        if self.slip_side["front"] or self.slip_side["rear"]:
            if self.slip_since is None:
                self.slip_since = t
        else:
            self.slip_since = None

    # --- helpers -------------------------------------------------------------
    def init(self, s0: float, sigma_s0: float) -> None:
        self.x[IS] = s0
        self.P[IS, IS] = sigma_s0 ** 2

    def wrap(self, s: float) -> float:
        return s % self.ring_len if self.ring_len > 0 else s

    def ring_delta(self, a: float, b: float) -> float:
        d = a - b
        if self.ring_len > 0:
            d = (d + 0.5 * self.ring_len) % self.ring_len - 0.5 * self.ring_len
        return d

    def corrected(self, which: str, u: float) -> float:
        half = 0.5 * self.log_rho
        return u * math.exp(half) if which == "front" else u * math.exp(-half)

    def a_model(self, v: float, s: float) -> float:
        i = int(np.clip(self.notch, -15, 15)) + 15
        q = max(v, 0.0)
        a = float(np.interp(q, self.v_edges + 0.5, self.table[i]))
        gr = float(np.interp(self.wrap(s), self.branch_s, self.grade))
        return self.p.load_factor * a - G * gr - (self.p.davis_a + self.p.davis_b * q + self.p.davis_c * q * q)

    # --- filter --------------------------------------------------------------
    def predict(self, t: float) -> None:
        if self.t is None:
            self.t = t
            return
        dt_all = t - self.t
        if not (dt_all > 0):
            return
        if dt_all > self.p.max_gap_s:
            self.t = t
            return
        x_keep = self.x.copy()
        p_keep = self.P.copy()
        while dt_all > 1e-9:
            dt = min(dt_all, self.p.dt_max)
            s, v = self.x[IS], self.x[IV]
            zupt = self.mode == "ZUPT"
            a = 0.0 if zupt else self.a_model(v, s) + self.x[IBA]
            v_new = 0.0 if zupt else min(self.p.v_max, max(0.0, v + a * dt))
            self.x[IS] = s + 0.5 * (v + v_new) * dt
            self.x[IV] = v_new
            F = np.eye(N)
            F[IS, IV] = dt
            F[IV, IBA] = 0.0 if zupt else dt
            Q = np.diag([self.p.q_s * dt, self.p.q_v * dt, self.p.q_k * dt, self.p.q_ba * dt])
            self.P = F @ self.P @ F.T + Q
            dt_all -= dt
        if not self._state_numerical():
            # Freeze. The clock still moves, so this interval is not integrated later.
            self.x = x_keep
            self.P = p_keep
            self._note_freeze()
        self.t = t

    def _update_scalar(self, h: np.ndarray, innov: float, r: float, consider_k: bool = False) -> None:
        S = float(h @ self.P @ h + r)
        if not (S > 0) or not math.isfinite(innov):
            return
        K = (self.P @ h) / S
        if consider_k:
            # Schmidt-Kalman: k is a consider state for bogie updates. Wheels
            # and model cannot tell v from k apart; only anchors move k.
            K[IK] = 0.0
        x_keep = self.x.copy()
        p_keep = self.P.copy()
        self.x = self.x + K * innov
        A = np.eye(N) - np.outer(K, h)
        self.P = A @ self.P @ A.T + r * np.outer(K, K)
        if not self._state_numerical():
            self.x = x_keep
            self.P = p_keep
            self._note_freeze()

    def _note_freeze(self) -> None:
        self.n_guard += 1
        self.step_frozen = True
        self.mode = "FREEZE"

    def _stamp_ok(self, t: float) -> bool:
        return math.isfinite(t) and (self.t is None or t >= self.t)

    def on_cmd(self, t: float, position: int) -> None:
        if not self._stamp_ok(t):
            return
        self.step_frozen = False
        self.predict(t)
        if self.step_frozen:
            self.mode = "FREEZE"
        elif self.mode == "FREEZE":
            self.mode = "WHEELS"
        self.notch = int(position)

    def _learn_pair(self, uf: float, ur: float) -> None:
        """Noise level and rear/front ratio from one fresh pair."""
        d = self.corrected("front", uf) - self.corrected("rear", ur)
        sd = math.sqrt(2.0 * self.r)
        e = d - self.d_bias
        self.d_bias += self.p.bias_alpha * (d - self.d_bias)
        biased = abs(self.d_bias) > 2.0 * sd + 0.05
        if abs(e) <= self.p.fr_sigma_gate * sd:
            self.r += self.p.r_alpha * (0.5 * e * e - self.r)
        elif not biased:
            ec = self.p.fr_sigma_gate * sd
            self.r += self.p.r_alpha_grow * (0.5 * ec * ec - self.r)
        self.r = min(max(self.r, self.p.r_min), self.p.r_max)
        # Sign-step on each fresh pair, not on a second. At about 10 Hz per bogie
        # both callbacks learn, so a persistent offset takes on the order of two
        # minutes and a 30 s slip moves the ratio by about one percent. At another
        # rate the same rho_step is a different time constant.
        if uf > self.p.rho_v_min and ur > self.p.rho_v_min and abs(self.notch) <= self.p.rho_notch_max:
            lr = math.log(ur / uf)
            if abs(lr) < self.p.rho_max + 0.02:
                self.log_rho += self.p.rho_step if lr > self.log_rho else -self.p.rho_step
                self.log_rho = max(-self.p.rho_max, min(self.p.rho_max, self.log_rho))

    def on_bogie(self, t: float, which: str, raw: float) -> None:
        if not self._stamp_ok(t):
            return
        self.step_frozen = False
        self.predict(t)
        if not math.isfinite(raw):
            return
        u_raw = raw * self.p.unit
        if abs(u_raw) > self.p.v_max:
            return
        other_name = "rear" if which == "front" else "front"
        other = self.last[other_name]
        self.last[which] = (t, u_raw)
        fresh_other = other[0] is not None and t >= other[0] and t - other[0] < self.p.wheel_stale_s and math.isfinite(other[1])
        u = self.corrected(which, u_raw)
        if not self.have_v:
            # The first reading sets the speed; there is nothing to gate it against.
            self.x[IV] = max(0.0, u * self.x[IK])
            self.have_v = True
        v, k = self.x[IV], self.x[IK]
        pred = v / k
        r = self.r
        h = np.array([0.0, 1.0 / k, -v / (k * k), 0.0])
        S = float(h @ self.P @ h + r)
        innov = u - pred
        self.model_resid[which] = innov if math.isfinite(innov) else None
        slip = innov * innov / S > self.p.nis_gate
        agree = False
        if fresh_other:
            uo = self.corrected(other_name, other[1])
            gate = max(self.p.fr_floor, self.p.fr_sigma_gate * math.sqrt(2.0 * r))
            agree = abs(u - uo) <= gate
            self.wheel_consensus = abs(u - uo)
            if not agree and abs(u - pred) > abs(uo - pred):
                slip = True
            uf, ur = (u_raw, other[1]) if which == "front" else (other[1], u_raw)
            self._learn_pair(uf, ur)
        # Both bogies agree with each other but not with the model: a short run is
        # a slide or spin of both; a long one means the model state is wrong
        # (after a dropout). Re-acquire from the bogies.
        if slip and agree:
            if self.disagree_since is None:
                self.disagree_since = t
            elif t - self.disagree_since >= self.p.recover_s:
                x_keep = self.x.copy()
                p_keep = self.P.copy()
                self.x[IV] = max(0.0, u * k)
                self.P[IV, :] = 0.0
                self.P[:, IV] = 0.0
                self.P[IV, IV] = r * k * k
                if not self._state_numerical():
                    self.x = x_keep
                    self.P = p_keep
                    self._note_freeze()
                    self.disagree_since = None
                    return
                self.disagree_since = None
                slip = False
                self._note_slip(which, False, t, innov * innov / S)
                self.slip = False
                self._zupt(t)
                return
        else:
            self.disagree_since = None
        self._note_slip(which, slip, t, innov * innov / S)
        self.slip = slip
        self._update_scalar(h, innov, self.p.r_bad if slip else r, consider_k=True)
        self.x[IV] = max(0.0, self.x[IV])
        self._zupt(t)
        if self.step_frozen:
            self.mode = "FREEZE"
        elif self.mode == "FREEZE":
            self.mode = "WHEELS"

    def _zupt(self, t: float) -> None:
        f, r = self.last["front"], self.last["rear"]
        thr = max(self.p.zupt_v, self.p.zupt_noise_mult * math.sqrt(self.r))
        both_still = (
            f[0] is not None and r[0] is not None
            and abs(f[1]) < thr and abs(r[1]) < thr
            and abs(f[0] - r[0]) < self.p.wheel_stale_s
        )
        if not both_still:
            self.still_since = None
            self.anchored_this_dwell = False
            if self.mode == "ZUPT":
                self.mode = "WHEELS"
            return
        if self.still_since is None:
            self.still_since = t
        if t - self.still_since >= self.p.zupt_hold_s:
            self.mode = "ZUPT"
            self.x[IV] = 0.0
            self.P[IV, :] = 0.0
            self.P[:, IV] = 0.0
            self.P[IV, IV] = 1e-6
            if not self.anchored_this_dwell:
                self._anchor()
                self.anchored_this_dwell = True

    def _anchor(self) -> None:
        s = self.x[IS]
        sig = math.sqrt(max(self.P[IS, IS], 0.0))
        cands = []
        for st in self.stops:
            if st["sd"] > self.p.stop_sd_max:
                continue
            r_sd = max(st["sd"], self.p.stop_sigma_floor)
            gate = self.p.stop_gate * math.sqrt(sig ** 2 + r_sd ** 2)
            d = self.ring_delta(st["s"], s)
            if abs(d) <= gate:
                cands.append((d, st, r_sd))
        if len(cands) != 1:
            return
        d, st, r_sd = cands[0]
        h = np.zeros(N)
        h[IS] = 1.0
        before = float(self.wrap(self.x[IS]))
        self._update_scalar(h, d, r_sd ** 2)
        self.n_anchor += 1
        self.anchor_log.append((self.t, before, float(st["s"]), float(self.wrap(self.x[IS]))))

    @property
    def k(self) -> float:
        return float(self.x[IK])

    def state(self) -> tuple[float, float, float, float]:
        return float(self.wrap(self.x[IS])), float(self.x[IV]), self.k, float(math.sqrt(max(self.P[IS, IS], 0.0)))
