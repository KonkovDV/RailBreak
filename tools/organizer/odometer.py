"""Reference along-track odometer (Python twin of the ROS core).

Numerical state and filter decisions match track_odometer.hpp. lockstep.py
compares s and v. Diagnostic counters are separate: a bad bogie sample and a
stamp behind the filter are dropped here without an n_rejected counter.

Probabilistic filter state x = [s, v, k, ba]
  s   coordinate on the track ring (m)
  v   speed (m/s)
  k   common wheel scale, true speed = k * corrected bogie speed
  ba  bias of the notch acceleration model (m/s^2), learned while wheels are trusted

The same array is 8 long. The last four entries are diagnostic parameters, not
Kalman states: [k_front, k_rear, b_front, b_rear]. The wheel Jacobian is nonzero
only in v and k, K_k is 0 on wheel updates, and those four variances do not
enter S. k_front and k_rear hold the rear/front ratio split at the geometric
mean; b_front and b_rear stay 0. There is no joint identifiability analysis
with k and ba.

Three time scales keep calibration, noise and slip apart:
  * rho, the rear/front ratio, is a slow sign-step median over fresh pairs off
    heavy traction and braking. Both bogies are corrected half-way to their
    geometric mean. The ratio never moves toward the model prediction: that
    would pull the wheels onto the notch table and hide the common scale from
    the anchors. The absolute level is k, observable through station anchors.
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
N = 8
IS, IV, IK, IBA, IKF, IKR, IBF, IBR = range(N)


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
    zupt_v_max: float = 0.5          # cap: 3*sqrt(r_max) would be 3 m/s
    stop_sd_max: float = 3.0
    stop_sigma_floor: float = 1.0
    stop_gate: float = 3.0
    wheel_stale_s: float = 0.35
    dt_max: float = 0.2
    max_gap_s: float = 30.0
    v_max: float = 30.0
    recover_s: float = 3.0         # after this, agreeing bogies are not treated as trustworthy
    load_factor: float = 1.0       # scales a_tab only
    davis_a: float = 0.0           # extra resistance; 0 is already inside a_tab
    davis_b: float = 0.0
    davis_c: float = 0.0
    excite_v_start: float = 1.0
    excite_dv: float = 0.3
    excite_uniform_s: float = 5.0
    excite_notch_s: float = 2.0
    excite_grade: float = 0.005


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
    if not math.isfinite(p.zupt_v) or p.zupt_v < 0.0:
        raise ValueError("zupt_v must be >= 0")
    if not math.isfinite(p.zupt_v_max) or not p.zupt_v_max >= p.zupt_v:
        raise ValueError("zupt_v_max must be >= zupt_v")
    if not math.isfinite(p.excite_uniform_s) or not p.excite_uniform_s > 0.0:
        raise ValueError("excite_uniform_s must be > 0")
    if not math.isfinite(p.excite_dv) or not p.excite_dv > 0.0:
        raise ValueError("excite_dv must be > 0")


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
        self.x = np.array([0.0, 0.0, self.p.k0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([1.0, 0.01, self.p.sigma_k0 ** 2, self.p.sigma_ba0 ** 2,
                          0.02 ** 2, 0.02 ** 2, 0.05 ** 2, 0.05 ** 2])
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
        self.n_gnss_anchor = 0
        self.n_guard = 0
        self.anchor_log = []
        self.s_anchor_ref = 0.0
        self.path_since_anchor = 0.0
        self.have_v = False
        self.common_unobservable = False
        self.step_frozen = False
        self.disagree_since = None
        self.uniform_since = None
        self.v_uniform = 0.0
        self.notch_mark = 0
        self.notch_changed_t = None
        self.last_cmd_t = None
        self.params_frozen = False
        self.drive_segment = "start"
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
        self.s_anchor_ref = s0
        self.path_since_anchor = 0.0

    def wrap(self, s: float) -> float:
        return s % self.ring_len if self.ring_len > 0 else s

    def ring_delta(self, a: float, b: float) -> float:
        d = a - b
        if self.ring_len > 0:
            d = (d + 0.5 * self.ring_len) % self.ring_len - 0.5 * self.ring_len
        return d

    def corrected(self, which: str, u: float) -> float:
        ki = float(self.x[IKF if which == "front" else IKR])
        bi = float(self.x[IBF if which == "front" else IBR])
        denom = 1.0 + ki
        if not (abs(denom) > 0.5):
            return u
        return (u - bi) / denom

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
            motion = 10.0 if self.common_unobservable else 1.0
            Q = np.diag([self.p.q_s * dt * motion, self.p.q_v * dt * motion,
                         self.p.q_k * dt, self.p.q_ba * dt, 0.0, 0.0, 0.0, 0.0])
            self.P = F @ self.P @ F.T + Q
            dt_all -= dt
        if not self._state_numerical():
            # Freeze. The clock still moves, so this interval is not integrated later.
            self.x = x_keep
            self.P = p_keep
            self._note_freeze()
        else:
            self.path_since_anchor += abs(float(self.x[IS]) - float(x_keep[IS]))
        self.t = t

    def _update_scalar(self, h: np.ndarray, innov: float, r: float, consider_k: bool = False,
                       gate_bias: bool = False) -> None:
        self._note_drive(self.t)
        S = float(h @ self.P @ h + r)
        if not (S > 0) or not math.isfinite(innov):
            return
        K = (self.P @ h) / S
        if consider_k:
            # Schmidt-Kalman: k is a consider state for bogie updates. Wheels
            # and model cannot tell v from k apart; only anchors move k.
            K[IK] = 0.0
        if gate_bias and self.params_frozen:
            K[IBA] = 0.0
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
        if not math.isfinite(t):
            return
        if self.t is not None and t < self.t:
            newer = self.last_cmd_t is None or t >= self.last_cmd_t
            if newer and (self.t - t) <= 0.5:
                self.notch = int(position)
                self._note_drive(self.t)
                self.last_cmd_t = t
            return
        if not self._stamp_ok(t):
            return
        self.step_frozen = False
        self.predict(t)
        if self.step_frozen:
            self.mode = "FREEZE"
        elif self.mode == "FREEZE":
            self.mode = "WHEELS"
        self.notch = int(position)
        self._note_drive(t)
        self.last_cmd_t = t

    def _note_drive(self, t: float) -> None:
        if self.notch != self.notch_mark:
            self.notch_changed_t = t
            self.notch_mark = self.notch
        grade = 0.0 if len(self.grade) == 0 else float(
            np.interp(self.wrap(self.x[IS]), self.branch_s, self.grade))
        notch_age = 1.0e9 if self.notch_changed_t is None else max(0.0, t - self.notch_changed_t)
        start = self.mode == "ZUPT" or self.x[IV] < self.p.excite_v_start
        notch_edge = self.notch_changed_t is not None and notch_age <= self.p.excite_notch_s
        brake = self.notch < 0
        coast = self.notch == 0
        grade_on = abs(grade) >= self.p.excite_grade
        accel = self.uniform_since is not None and abs(self.x[IV] - self.v_uniform) >= self.p.excite_dv
        if start or brake or coast or notch_edge or grade_on or accel:
            self.uniform_since = None
            self.v_uniform = float(self.x[IV])
            self.params_frozen = False
            if start:
                self.drive_segment = "start"
            elif brake:
                self.drive_segment = "brake"
            elif coast:
                self.drive_segment = "coast"
            elif notch_edge:
                self.drive_segment = "notch"
            elif grade_on:
                self.drive_segment = "grade"
            else:
                self.drive_segment = "accel"
            return
        if self.uniform_since is None:
            self.uniform_since = t
            self.v_uniform = float(self.x[IV])
        self.params_frozen = (t - self.uniform_since) >= self.p.excite_uniform_s
        self.drive_segment = "uniform" if self.params_frozen else "hold"

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
        # Sign-step on each fresh pair. At about 10 Hz per bogie a persistent
        # offset takes on the order of two minutes. At another rate the same
        # rho_step is a different time constant.
        if uf > self.p.rho_v_min and ur > self.p.rho_v_min and abs(self.notch) <= self.p.rho_notch_max:
            lr = math.log(ur / uf)
            if abs(lr) < self.p.rho_max + 0.02:
                self.log_rho += self.p.rho_step if lr > self.log_rho else -self.p.rho_step
                self.log_rho = max(-self.p.rho_max, min(self.p.rho_max, self.log_rho))
                self.x[IKF] = math.exp(-0.5 * self.log_rho) - 1.0
                self.x[IKR] = math.exp(0.5 * self.log_rho) - 1.0

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
        h = np.zeros(N)
        h[IV] = 1.0 / k
        h[IK] = -v / (k * k)
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
            if not self.common_unobservable:
                self._learn_pair(uf, ur)
        # Agreement with each other is not an independent measurement of speed.
        if slip and agree and not self.common_unobservable:
            if self.disagree_since is None:
                self.disagree_since = t
            if t - self.disagree_since >= self.p.recover_s:
                self.common_unobservable = True
                self.P[IS, IS] += 1.0
                self.P[IV, IV] += 1.0
        elif not self.common_unobservable:
            self.disagree_since = None
        if self.common_unobservable:
            nis = innov * innov / S if S > 0 else 0.0
            self._note_slip("front", True, t, nis)
            self._note_slip("rear", True, t, nis)
            self.slip = True
            self._zupt(t)
            if self.common_unobservable and self.mode not in ("ZUPT", "FREEZE"):
                self.mode = "COMMON_MODE_UNOBSERVABLE"
            return
        self._note_slip(which, slip, t, innov * innov / S)
        self.slip = slip
        self._update_scalar(h, innov, self.p.r_bad if slip else r, consider_k=True, gate_bias=True)
        self.x[IV] = max(0.0, self.x[IV])
        self._zupt(t)
        if self.step_frozen:
            self.mode = "FREEZE"
        elif self.mode != "ZUPT":
            self.mode = "MODEL" if slip else "WHEELS"

    def _zupt(self, t: float) -> None:
        f, r = self.last["front"], self.last["rear"]
        noise = self.p.zupt_noise_mult * math.sqrt(max(self.r, 0.0))
        thr = min(self.p.zupt_v_max, max(self.p.zupt_v, noise))
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
        s = self.wrap(self.x[IS])
        sig = math.sqrt(max(self.P[IS, IS], 0.0))
        n_cand = 0
        n_seen = 0
        best = None
        near = None
        for st in self.stops:
            if st["sd"] > self.p.stop_sd_max:
                continue
            n_seen += 1
            r_sd = max(st["sd"], self.p.stop_sigma_floor)
            gate = self.p.stop_gate * math.sqrt(sig ** 2 + r_sd ** 2)
            d = self.ring_delta(st["s"], s)
            if near is None or abs(d) < abs(near[0]):
                near = (d, st, gate)
            if abs(d) <= gate:
                n_cand += 1
                if best is None or abs(d) < abs(best[0]):
                    best = (d, st, r_sd, gate)
        row = {
            "distance_since_anchor": self.distance_since_anchor(),
            "predicted_s": s,
            "candidate_s": float("nan"),
            "innovation": 0.0,
            "gate": 0.0,
            "accepted": False,
            "reason": "no_station",
        }
        if n_cand == 1:
            d, st, r_sd, gate = best
            row.update(candidate_s=float(st["s"]), innovation=d, gate=gate,
                       accepted=True, reason="accepted")
        elif n_seen == 0:
            pass
        elif n_cand == 0:
            d, st, gate = near
            row.update(candidate_s=float(st["s"]), innovation=d, gate=gate, reason="outside_gate")
        else:
            d, st, _r_sd, gate = best
            row.update(candidate_s=float(st["s"]), innovation=d, gate=gate, reason="ambiguous")
        self.anchor_log.append(row)
        if n_cand != 1:
            return
        d, st, r_sd, _gate = best
        h = np.zeros(N)
        h[IS] = 1.0
        # k moves through its covariance with s. A second update of k from the
        # same missed distance would count this station twice.
        self._update_scalar(h, d, r_sd ** 2)
        self.s_anchor_ref = float(self.wrap(self.x[IS]))
        self.path_since_anchor = 0.0
        self.n_anchor += 1
        self.common_unobservable = False
        self.disagree_since = None

    def gnss_anchor(self, s_meas: float, sigma_m: float) -> bool:
        """Absolute arc from a fix already accepted as on-axis. Same s update as a station."""
        if self.t is None or not math.isfinite(s_meas) or not (sigma_m > 0.0):
            return False
        d = self.ring_delta(s_meas, self.wrap(self.x[IS]))
        h = np.zeros(N)
        h[IS] = 1.0
        self._update_scalar(h, d, sigma_m ** 2)
        self.s_anchor_ref = float(self.wrap(self.x[IS]))
        self.path_since_anchor = 0.0
        self.n_anchor += 1
        self.n_gnss_anchor += 1
        self.common_unobservable = False
        self.disagree_since = None
        return True

    def gnss_snap(self, s_meas: float, sigma_m: float) -> bool:
        """Put s on a measured arc. Does not move k. Wheel updates keep P_ss
        too small for gnss_anchor to correct a drifted path."""
        if self.t is None or not math.isfinite(s_meas) or not (sigma_m > 0.0):
            return False
        d = self.ring_delta(s_meas, self.wrap(self.x[IS]))
        self.x[IS] = self.wrap(float(self.x[IS]) + d)
        self.P[IS, :] = 0.0
        self.P[:, IS] = 0.0
        self.P[IS, IS] = sigma_m ** 2
        self.s_anchor_ref = float(self.wrap(self.x[IS]))
        self.path_since_anchor = 0.0
        self.n_anchor += 1
        self.n_gnss_anchor += 1
        self.common_unobservable = False
        self.disagree_since = None
        return True

    def distance_since_anchor(self) -> float:
        return self.path_since_anchor

    @property
    def k(self) -> float:
        return float(self.x[IK])

    def state(self) -> tuple[float, float, float, float]:
        return float(self.wrap(self.x[IS])), float(self.x[IV]), self.k, float(math.sqrt(max(self.P[IS, IS], 0.0)))
