# External red-team review — 

Independent adversarial review of `tramDR` (`KonkovDV/RailBreak`) against the
Moscow Transport hackathon case *«Резервная одометрия по модели»*, the
hackathon Положение, and the repository's own published claims.

Method: re-derive the mathematics from scratch rather than check it, read the
code as an attacker looking for silent failure modes, then score the result
the way the expert commission is instructed to score it.

---

## Краткое резюме (RU)

1. **Математика в репозитории верна.** Все опубликованные величины
   пересчитаны независимо и совпали: веса масштабированного UT, окна
   допустимости α, ван-Лоановская Q, Дэвис, уклоны, границы PL/AL. Ошибок в
   алгебре не найдено. Документация честная — она сама объявляет открытые
   дефекты.
2. **Главный риск — не технический, а юридический.** Репозиторий публичный
   под MIT, тогда как п. 14.2.2 Положения передаёт исключительные права по
   кейсу Фонду «ТИМ», а пп. 14.4–14.6 запрещают публикацию и повторное
   использование без письменного согласия организатора; п. 15 объявляет
   материалы кейса конфиденциальными, а пп. 15.6 и 17.1 предусматривают
   дисквалификацию. Лицензия MIT безотзывна — публикацию нельзя полностью
   «отменить». Это способно обнулить результат независимо от качества кода.
   Действия — в разделе 8.
3. **Найден новый дефект уровня P0 (F-17b): смешение двух шкал времени** в
   `state_estimator_node`. При воспроизведении bag-файла без
   `use_sim_time:=true` фильтр интегрирует с неверной скоростью, а оба
   сторожевых таймера свежести — по notch и по колёсам — перестают работать.
   Именно они несут основную нагрузку в заявленной схеме целостности.
   Исправлено в этой ветке.
4. **CI никогда не собирал ROS-узлы** (F-19) и не запускал четыре
   ament-gtest-набора. Поэтому дефекты класса F-17 в принципе не могли быть
   замечены. Job добавлен.
5. **Issue #3 устарел**: F-11 и F-14 уже закрыты на `main`.

---

## 1. What was verified, and what was not

| Verified independently | Result |
| --- | --- |
| Scaled-UT weights at the shipped operating point | matches `docs/math.md` exactly |
| Admissible α windows for β ∈ {0, 1, 2} | matches; margins quantified below |
| Luo–Moroz sufficient PSD bound | correct, and correctly labelled *sufficient* |
| Van Loan discrete Q for white acceleration on (s, v) | matches |
| Davis resistance discontinuity at low speed | matches |
| Grade force / acceleration at 25 ‰, 32 ‰, 40 ‰ | matches |
| Slip bound `b_s = θ v T_d` | matches |
| `F_bias` random-walk σ over 5 s | matches |
| Traction envelope, adhesion floor, κ floor | matches |
| PL/AL relationship on the worst segment of route 10 | matches, with a caveat — see 2.6 |

**Not verified, and why.** The published metrics could not be recomputed. The
offline pipeline is `tools/synth/generate.py` → `standalone/replay_ukf` →
`tools/synth/score.py`, which needs CMake, a compiler and a Python
environment. The review environment had no network access and no toolchain,
so every number in `docs/metrics.md` remains **as-published and unaudited**.
That matters because of F-13b: the metrics were measured on 0.0.6 and core
behaviour changed in 0.0.7. Treat the metrics as provenance-tagged claims,
not as verified results, until they are regenerated.

The C++ in this branch is likewise **not compiled** by the reviewer. The new
standalone test target is built and run by CI; the node changes are covered
by the new ROS job added here, which is the first time CI has ever compiled
them.

---

## 2. Independent re-derivation

### 2.1 Scaled unscented transform

With `L = kStateDim = 6 + kNWheels = 12`, `α = 0.58`, `β = 2`, `κ_ut = 0`:

```
λ    = α²(L + κ) − L = 0.3364 · 12 − 12 = −7.9632
c    = L + λ                            =  4.0368
W_m0 = λ / c        = −1659/841         = −1.972651605231…
W_i  = 0.5 / c      =   625/5046        =  0.123860483551…
W_c0 = W_m0 + (1 − α² + β)              = +0.690948394769…
```

Normalisation checks: `ΣW_m = W_m0 + 24 W_i = 1` exactly, and
`ΣW_c = W_c0 + 24 W_i = 3.6636 = 1 + (1 − α² + β)`. Both hold.

`W_m0` is large and negative (−1.97). That is normal for a scaled UT at small
α, but it means the transformed mean is a difference of comparable numbers,
so the zeroth sigma point dominates and cancellation is real. This is the
mechanism behind finding **A** in section 5.

### 2.2 The α window is joint, not per-parameter

At `κ_ut = 0` the scaling collapses to `W_m0 = 1 − 1/α²`, so

```
W_c0 = 2 + β − α² − 1/α²
```

which is **independent of L**. Requiring `W_c0 ≥ 0` gives
`α² ∈ [((2+β) − √((2+β)²−4))/2, ((2+β) + √((2+β)²−4))/2]`:

| β | admissible α | shipped α = 0.58 |
| --- | --- | --- |
| 2 | `[0.5176380902, 1.9318516526]` = `[√(2−√3), √(2+√3)]` | admissible, +12.05 % over the lower edge |
| 1 | `[0.6180339887, 1.6180339887]` = `[1/φ, φ]` | **inadmissible** |
| 0 | `{1}` exactly — the window is a single point | **inadmissible** |

The Luo–Moroz sufficient bound `α ≥ 1/√(1+β) = 1/√3 = 0.5773502692` is cleared
by only **+0.459 %**. At that bound `W_c0 = β/(1+β)` exactly, which is a clean
way to see that the bound is sufficient but not necessary.

This is the whole content of **F-08**: α, β and κ_ut are three independent ROS
parameters, but admissibility is a joint property. Editing β alone — a
reasonable thing for an operator to try, with every parameter still inside its
own documented range — silently makes `W_c0` negative. `la::project_pd` then
repairs `P` every cycle and the filter keeps running with fictional
uncertainty. The repair is what makes this dangerous: it removes the crash
that would otherwise expose the misconfiguration.

Fixed in this branch by `include/tram_dr_localization/ut_weights.hpp` plus
`standalone/test_ut_weights_header.cpp`. **Still to do:** call
`ut::weights_psd_ok()` from `validate_ukf()` in `src/lib/ukf.cpp` so the core
rejects the configuration instead of absorbing it. See section 4.

### 2.3 Process noise

Van Loan discretisation of white acceleration on `(s, v)`:
`Q_ss = q_v Δt³/3`, `Q_sv = q_v Δt²/2`, `Q_vv = q_v Δt`. Correct.
At `q_v = 0.0025 m²/s³` over a 10 s unaided coast, `σ_s = √(q_v t³/3) =
√(0.0025 · 1000/3) = 0.9129 m`. Matches.

### 2.4 Plant

* `a_svc / g = 1.2 / 9.81 = 0.12232`, comfortably under a dry-rail adhesion
  limit and above a `μ = 0.06` wet-rail floor (`0.5886 m/s²`). Consistent.
* Davis at `v = 0.11 m/s`: `800 + 40(0.11) + 6(0.11²) = 804.4726 N`, i.e.
  `0.028731 m/s²` at 28 t. The low-speed discontinuity is real and is handled.
* Grade: `g·i` = `0.24525 / 0.31392 / 0.3924 m/s²` at 25/32/40 ‰; at 22 t that
  is `5396 N` and `8633 N` at 25 ‰ and 40 ‰. Matches.
* `F_bias` random walk at `3000 N/√s` gives `σ_5s = 3000√5 = 6708 N`, which
  must be compared against a 3 % grade at 22 t = `6475 N`. The bias channel is
  therefore *just* wide enough to absorb an unmodelled 3 % grade within 5 s.
  This is a deliberate and defensible choice, but it is also the reason an
  unmodelled grade is indistinguishable from a force-bias drift — worth saying
  out loud in the pitch rather than leaving for a jury member to find.
* Traction envelope, twin Combino NF100: `28 000 · 1.3 = 36.4 kN`. Matches.
* κ floor: `0.5 / max(|v|, 1)` gives `0.5` at `v = 0.14 m/s` instead of the
  `3.571` an unfloored `0.5/|v|` would give. The floor is necessary and correct.

### 2.5 Route 10

`v_max = 16.7 m/s` (60 km/h). Longest inter-stop gap, «Детская поликлиника» →
«Исаковского, 33»: `2794 − 408.4 = 2385.6 m`.

### 2.6 PL versus AL on the worst segment — read this carefully

Traversing the 2385.6 m gap at 16.7 m/s takes 142.9 s.

*Unaided* (all encoder channels lost, process noise only):

```
σ_s   = √(q_v t³/3) = √(0.0025 · 142.9³/3) ≈ 49.31 m
PL_s  = k_over σ_s + b_s = 2.5 · 49.31 + 0.835 ≈ 124.1 m
AL_s  = 5 + 0.05 · 2385.6                     ≈ 124.3 m
```

The protection level lands **0.2 m — 0.16 % — inside the alert limit** on the
longest gap of the reference route. That is a coincidence, not a design
margin, and it should not be presented as headroom.

The caveat that makes this honest: this is the *total encoder loss* case, in
which the estimator would already have declared LOST via `age_lost_s`, so the
bound is not what protects the vehicle — the LOST declaration is. Under
nominal operation wheel speed is available and the position error is driven by
wheel-radius scale error and slip, order 0.5–1 % of distance, i.e. 12–24 m over
this gap against an AL of 124 m: a comfortable 5–10× margin.

**Recommendation.** State both numbers explicitly in `docs/math.md` and the
pitch. A jury member who computes the unaided case unprompted and finds 0.16 %
will read it as a near-miss the team did not notice. The same jury member who
sees both numbers already tabulated, with the LOST-declaration argument
attached, reads it as evidence of a team that knows where its own edges are.
This is the single highest-leverage documentation change available.

---

## 3. Fixed in this branch

| ID | Severity | Fix |
| --- | --- | --- |
| **F-09** | P2 | `la::project_pd` returned `void` and simply returned on non-finite input, leaving the caller's covariance poisoned with no signal. Now returns `bool` and reports failure on invalid arguments, non-finite input, and non-finite reconstruction output. |
| **F-08** (part) | P2 | New `ut_weights.hpp`: single source of truth for the UT weights, the joint `(α, β, κ)` admissibility predicate, and the closed-form α window. New `test_ut_weights_header.cpp` cross-checks it against hand-derived values and pins the β-editing hazard. |
| **F-17** | P1 | Input hygiene in `state_estimator_node`: non-finite notch no longer promoted to `notch_valid = true`; brake no longer relies on `std::clamp` to reject NaN; malformed twist no longer synthesises a fake all-NaN wheel frame; non-finite ω channels counted. |
| **F-17b** | **P0** | Measurement and node timebases separated. See section 5. |
| **F-19** | P1 | New `ros` CI job builds the package with colcon and runs the four ament gtest suites. |
| **F-20** | P3 | `package.xml` version aligned with `kModelVersion`. |
| — | P3 | `test_ut_weights_header` added to the explicit ASan target list, so it cannot repeat the F-11 pattern. |

### Why `project_pd` does not sanitize on failure

The obvious "fail safe" would be to overwrite a poisoned covariance with a
large finite diagonal. That is worse than doing nothing: it makes the caller's
subsequent Cholesky health check *succeed*, so a hard NaN fault is silently
downgraded to a plausible-looking maximum-uncertainty estimate that the
integrity monitor cannot distinguish from a genuinely uncertain one. The
function now leaves the matrix untouched and reports `false`, which lets the
caller do the only correct thing: reject the frame, roll back, raise the fault.
Establishing finiteness before the call is the caller's duty, and
`la::all_finite()` was added for exactly that.

---

## 4. Confirmed open, with fix designs

### F-08, part 2 — wire the predicate into the core

`validate_ukf()` in `src/lib/ukf.cpp` still range-checks α, β and κ_ut
separately. Add, after the per-parameter checks:

```cpp
if (!ut::weights_psd_ok(cfg.alpha, cfg.beta, cfg.kappa_ut, kStateDim)) return false;
```

The node-side guard is the cheaper half and is not a substitute: anything
constructing `Ukf` directly — `replay_ukf`, `test_core`, a future node —
bypasses it.

### F-10 — the ZUPT gate is circular

`Ukf::zupt_gate` ends with

```cpp
return wmax < 0.08 && u.notch_valid && std::fabs(u.notch) < 0.05 && vabs < 0.35;
```

`vabs` is derived from `x_[kV]` — the very quantity ZUPT exists to drive to
zero. If the estimate has drifted above 0.35 m/s while the vehicle is
physically stopped, the gate refuses to fire precisely when it is most needed,
and the drift is never corrected.

Proposed fix, which breaks the circularity by adding an *independent* witness:

* new `UkfParams::zupt_omega_only_s{2.0}`;
* new member `omega_zero_s_`, accumulated in `predict_and_update` via a new
  `bool wheels_at_rest(const double* omega, std::size_t n) const` — all
  channels finite, in range, `|ω| < 0.08`; `false` when `n == 0`;
* gate becomes: reject if `wmax >= 0.08 || !notch_quiet`; accept if
  `vabs < 0.35`; otherwise accept iff
  `cfg_.zupt_omega_only_s > 0.0 && omega_zero_s_ >= cfg_.zupt_omega_only_s`;
* in `maybe_zupt`, capture `v_before = |x_[kV]|` and set a new
  `zupt_estimate_disagree_` flag when `v_before >= 0.35`. Add it to the
  `degraded` predicate in `classify()`, to `UkfEstimate`, and publish it as
  `zupt_forced` in node diagnostics.

The flag is the important part: forcing ZUPT silently would hide the fact that
the estimate and the wheels disagreed about standstill.

### F-13b — metrics predate the current core

Every published metric and pitch number was measured on 0.0.6. Core behaviour
changed in 0.0.7, and this branch changes it again. Until the pipeline is
re-run, `docs/metrics.md` and `docs/pitch.md` must carry an explicit
"measured on 0.0.6, not yet regenerated" banner on every table. Shipping
numbers whose provenance does not match the code is the single easiest thing
for a jury to catch, and it costs credibility out of proportion to the actual
error.

### F-15 — commit message does not match its diff

Commit `d322433` claims changes to `la::chol` / `project_pd`; its diff touches
only `src/lib/ukf.cpp`. Those changes actually arrived in `9dc09b1`. Cosmetic,
but it undermines `git log` as an audit trail — which the repo otherwise leans
on heavily.

---

## 5. New findings not in issue #3

### F-17b (P0) — two timebases mixed, defeating the integrity layer

This is the most serious defect found, and it is a new finding.

`step_filter()` kept a single `last_step_` and filled it from whichever clock
produced the current frame: `header.stamp` for `joint_state` / `twist_stamped`,
`now()` for the timer watchdog. It then computed `dt = t − last_step_` across
those two clocks.

On any bag replayed without `use_sim_time:=true`, the offset between the two is
hours or years. So:

* the wheel callback set `last_step_` to a bag stamp;
* the timer then computed `idle = now() − last_step_` ≈ +6.3 × 10⁷ s, fired
  `step_filter(false)`, and `std::clamp(dt, 0.005, 0.20)` turned that into a
  perfectly plausible 200 ms step;
* the next wheel callback computed `dt = bag_stamp − now()` ≈ −6.3 × 10⁷ s,
  which the same clamp turned into a perfectly plausible 5 ms step.

At 50 Hz the filter therefore integrated alternating 200 ms / 5 ms steps —
roughly an order of magnitude away from real time — with no diagnostic, no
warning, and a healthy-looking OK status. The clamp did not protect anything;
it destroyed the evidence.

Worse, the same mixing disabled both freshness watchdogs, which are what the
safety argument actually rests on:

* `notch_age = (t − last_notch_stamp_)` compared a header stamp against a
  `now()` stamp. With a negative offset the 0.25 s invalidation **never**
  fired, so a dead `/tram/controller_notch` kept reading as fresh indefinitely.
* `age_s = (t − last_wheel_stamp_)` did the same, so freshness-driven
  DEGRADED/LOST was either permanently on or permanently off depending only on
  the sign of the clock offset.

`replay_ukf` exercises the core offline and never touches the node, so neither
CI nor the published metrics could ever have seen this. F-19 is why.

**Fix, as implemented.** `dt` is an interval between two sensor samples and
belongs to the measurement timebase. An age is an arrival latency and belongs
to this node's clock. The two are now tracked in separate anchors and are never
subtracted from one another. Timer-driven predict-only steps have no header
stamp, so they are measured with a node-clock delta and the filter time they
consume is accumulated in `consumed_since_meas_s_`, which the next stamped
frame subtracts — one monotone timeline, no double counting, no cross-clock
arithmetic. A one-shot WARN names the likely cause.

`dt` is now validated rather than clamped:

* a stamp regression rejects the frame and raises a fault instead of stepping
  the filter forward while the data moved backwards;
* a gap longer than `dt_max_s` is advanced as repeated predict-only steps of
  `dt_max_s`, so `Q` accumulates over the **true** elapsed time. Past
  `max_catchup_steps` (20 s by default) the gap is declared unbridgeable and
  LOST is forced;
* the old 0.005 s lower clamp made any source above 200 Hz integrate faster
  than real time. The floor is now 1 ms and every application is counted.

All counters are published on `/tram/diagnostics` and any fault forces LOST for
that cycle.

### A (P1) — fabricated pseudo-measurement from dead encoder channels

In `Ukf::update_wheels`, a channel failing
`std::isfinite(omega[i]) && |omega[i]| <= kOmegaAbsMax` is assigned
`w[i] = phys[kV] / (d · plant_.r0_m)` — the model's own prediction — and is
**kept in the measurement vector** with inflated `R`.

This is not a no-op. `zhat[i]` is the unscented *mean* of a nonlinear function
with `W_m0 = −1.973`, so it does not equal the value substituted from the
current mean. The innovation is small but nonzero, so the state is nudged. More
importantly `P ← P − K P_zz Kᵀ` still **shrinks the covariance** using a channel
that carried no information. `R *= max(10.0, inflate_max)` reduces the effect;
it does not remove it. Over a sustained single-axle outage the filter becomes
quietly overconfident — the exact failure the integrity layer is meant to catch.

Fix: compact the measurement vector to live channels only and reduce `m`
accordingly, rather than padding it with predictions.

### B (P2) — the `predict()` Cholesky fallback is dead code

`predict()` runs an 8-attempt jitter loop and increments `chol_fail_` only if
all eight fail. `predict_and_update` then computes
`bool healthy = chol_fail_ == previous.chol_fail_;` and on `!healthy` executes
`*this = previous; ++chol_fail_; return reject();`.

That rollback discards everything the fallback path produced — the mean step,
`P += Q + 1e-3·I`, the `project_pd` repair. The fallback is unreachable in
effect. Either return immediately from the fallback, or delete it and document
the atomic-rollback contract; the current shape contradicts §1.7 of the design
notes.

### C (P2) — `set_plant` silently overwrites `sca_p_.r0_m`

`set_sca` **throws** when `|p.r0_m − plant_.r0_m| > 1e-12`, but `set_plant`
silently overwrites `sca_p_.r0_m`. Behaviour therefore depends on call order.
The node happens to call `set_plant` first, so the guard never fires — a latent
trap for any other caller. Make both paths agree: either both throw or both
propagate.

### D (P3) — `zupt_gate` requires `notch_valid`

A brief silence on the notch topic at a terminal stop yields LOST rather than a
recognised standstill. Fail-closed, so not a defect, but it is a
customer-visible behaviour that should be documented rather than discovered.

### E (P3) — `1.0e6` as a magic sentinel

`missed_path_m()` returns `1.0e6` when latched, and that value is published as
`over_m`. A downstream consumer cannot distinguish "one million metres" from
"unbounded". Publish an explicit `s_unbounded` flag instead.

### F (P3) — stale `last_sca_` / `frozen_` reused on predict-only frames

Inside `classify()` when `n == 0`, per-axle state from the last frame with data
is reused. Over a long outage the reported per-axle picture is increasingly
stale while looking current.

### G (P3) — `ament_lint_auto` declared but never invoked

`package.xml` declares the test dependency, but `CMakeLists.txt` never calls
`ament_lint_auto_find_test_dependencies()`, so no linter runs. Either wire it
up or drop the dependency.

---

## 6. Issue #3 is stale

Two findings listed as open are already fixed on `main`:

* **F-11** — `.github/workflows/ci.yml` already carries the
  `# F-11: this job used to build test_core only…` comment and builds
  `--target test_core test_integrity_contracts test_ut_weights_psd`.
* **F-14** — `docs/metrics.md` already documents the canonical `0.515` with
  `0.42` explicitly marked archival.

Leaving closed findings open in the tracker is a small thing that reads badly:
a jury member who checks two items and finds both already fixed will discount
the rest of the list, including the items that genuinely are open.

---

## 7. How this will actually be scored

Положение п. 10.7–10.8 defines **ten criteria, each scored 1–5**, summed or
averaged across the expert commission (Приложение 2); п. 10.9 permits
additional per-case criteria; п. 10.10 gives one winner per case with ties
broken by open vote and the chair deciding.

Maximum is therefore **50 points per expert**. Honest self-assessment:

| # | Criterion (п. 10.7) | Est. | Why, and what moves it |
| --- | --- | --- | --- |
| 1 | Соответствие решения кейсу | 5 | Squarely on-case: backup odometry from model + wheel speeds, no GNSS in the filter, and `tools/eval/no_gnss_scan.py` enforces it in CI. Hard to fault. |
| 2 | Работоспособность MVP | 3 | This is the weak point. There is a synthetic e2e pipeline but no demonstration on real or realistic bag data, and F-17b means the node path was broken on bag replay until this branch. **Highest-value fix: a recorded demo run.** |
| 3 | Техническая реализуемость | 5 | Runs on existing hardware, no new sensors, ROS 2 Humble, containerised. |
| 4 | Качество архитектуры и реализации | 4 | Clean core/node split, ROS-free core, dependency-light, deliberate standalone test layer. Held back by the node layer having been untested until now. |
| 5 | Качество алгоритмов/данных/моделей | 5 | Genuinely strong: scaled UT with a documented PSD analysis, Van Loan Q, Davis resistance, per-axle SCA, NIS/CUSUM, PL/AL integrity envelope. Above the level typical for a hackathon. |
| 6 | Применимость к московскому транспорту | 4 | Real vehicle profiles (Combino NF100, 71-911ЕМ, 71-931М) and real route 10 geometry. Route data is OSM-derived — see section 8. |
| 7 | Потенциал пилотирования | 4 | Plausible: a shadow-mode deployment publishing diagnostics alongside the primary system is a realistic first pilot. Say so explicitly. |
| 8 | Новизна и практическая ценность | 3–4 | The components are individually standard; the novelty is the integrity envelope and the honest failure taxonomy, not the filter. Frame it that way rather than claiming a novel estimator. |
| 9 | Презентация и аргументация | ? | Not reviewable here. Note that criteria 9 and 10 together are 10 of 50 points — 20 % — and are the cheapest to improve. |
| 10 | Состав команды | ? | п. 7 requires 3–5 members, RF citizens, 18+. Verify compliance before submission; a team of fewer than 3 is a hard fail, not a lost point. |

**Where the marginal points are.** Criteria 2, 9 and 10 are worth up to 15
points and are the least technical. The estimator is already scoring near the
top of criteria 1, 3 and 5; further mathematics adds little. A working demo on
realistic data plus a rehearsed pitch is worth far more than any additional
filter work.

**Predictable jury probes**, given that the named experts lead the Центр
беспилотного транспорта, Центр наземного транспорта, Центр прикладных сервисов
and Центр транспортных решений at МТТЕХ:

1. "What happens when an encoder fails?" — strong answer exists; finding A must
   be fixed first or the honest answer is "we become overconfident".
2. "How do you know when you are wrong?" — the PL/AL envelope is the best part
   of this project. Lead with it.
3. "Show it on our data." — currently the weakest point. Bag-replay support now
   works; use it.
4. "Why not just add GNSS/IMU?" — because the case forbids it; make sure the
   answer is "this is the backup channel that must survive when they fail",
   not a defence of the constraint.
5. "What is your worst case?" — section 2.6. Volunteer it.

---

## 8. Licensing and confidentiality — the risk that outranks everything above

This section is the reason a technically strong submission can still score
zero.

**The conflict.** The repository is **public** on GitHub under **MIT**, while:

* п. 14.2.2 assigns exclusive rights for this case to Фонд «ТИМ»;
* пп. 14.4–14.6 forbid publication or reuse of hackathon materials without the
  organiser's prior **written** consent;
* п. 15 declares case data and the technical task confidential;
* пп. 15.6 and 17.1 make violation grounds for **disqualification**.

An MIT grant is irrevocable. Making the repository private now does not
retract licences already granted to anyone who obtained a copy, and does not
remove forks. The exposure cannot be fully undone — only contained.

**A second, independent conflict.** `tram_dr_localization/config/route_10.yaml`
is OSM-derived. OpenStreetMap data is **ODbL**, a share-alike licence.
Redistributing it inside a project whose exclusive rights are assigned to a
third party is at best unclear and at worst incompatible. ROS 2 dependencies
are Apache-2.0 and require attribution.

**Recommended actions, in order.**

1. Make the repository **private immediately**. Contains, does not cure.
2. Request written consent for publication under пп. 14.4–14.6, in writing, to
   `fund@ftim.ru` and `info@mttech.moscow`. Keep the reply.
3. If consent is refused, be ready to submit as a private repository with
   access granted to the organiser only — п. 8.7 requires "source code **or a
   link to a repository**", which a private repository with granted access
   satisfies.
4. Isolate the route data: keep `route_10.yaml` replaceable, load it at
   runtime, and document that the shipped file is OSM/ODbL and is **not** part
   of the assigned work product.
5. Add ODbL (route data) and Apache-2.0 (ROS 2) to `NOTICE` with attribution.
6. Do not re-publish, and do not reuse this code in another competition, until
   the ownership question under п. 14.2.2 is settled in writing.

**Timeline.** Solutions are due ** 23:59 МСК**; semifinal , final
. Consent requests take time, so item 1 should happen today and item 2
this week.

---

## 9. Prioritised action list

| Priority | Action | Cost |
| --- | --- | --- |
| **P0** | Make the repository private; request written consent (section 8) | minutes |
| **P0** | Merge this branch; confirm the new `ros` CI job is green | low |
| **P1** | Fix finding A — stop padding the measurement vector with predictions | medium |
| **P1** | Regenerate all metrics on the current core; banner every table with its provenance (F-13b) | medium |
| **P1** | Record a demo run on realistic bag data — the largest single scoring gain | medium |
| **P2** | Wire `ut::weights_psd_ok` into `validate_ukf` (F-08 part 2) | low |
| **P2** | Fix F-10 with the independent-witness design in section 4 | medium |
| **P2** | Findings B and C | low |
| **P3** | Add the section 2.6 worst-case table to `docs/math.md` and the pitch | low |
| **P3** | Update issue #3: close F-11 and F-14, add F-17b, F-19, F-20 and A–G | low |
| **P3** | Findings D, E, F, G; F-15 commit-trail note | low |

---

## 10. Overall judgement

The mathematics is correct, independently reproducible, and documented to a
standard well above what a 3-day hackathon normally produces. The repository
does something unusual and genuinely creditable: it publishes its own open
defect list.

The weaknesses are not in the algebra. They are:

1. a node layer that no test or CI job had ever compiled, which is where the
   one P0 defect was hiding;
2. metrics whose provenance does not match the shipped code;
3. no demonstration on realistic data;
4. a licensing posture that directly contradicts the competition rules.

Items 1 and part of 2 are addressed in this branch. Items 3 and 4 are where the
remaining risk and the remaining points both live — and item 4 is the only one
on this page that can cost the entire result regardless of how good the code is.
