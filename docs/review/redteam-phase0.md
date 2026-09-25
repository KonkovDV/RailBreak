# Red Team — фаза 0.11 и флаги M5/M11 — 

Предмет: незакоммиченное дерево после `eb2f6e4` (T0 pipeline, `r_adapt`,
`stop_update`, Docker Humble). Идентификаторы `RT24-*`. Формулировка
«ошибок нет» не используется. Триаж: [triage-phase0.md](triage-phase0.md).

База ядра по-прежнему `tramDR-0.0.11`. Умолчания M5/M11 выключены, seed 42
этим проходом не переснимался.

---

## Краткое резюме

1. `ros2 bag play` на writer без ключа `files:` и со строковым QoS
   (`keep_last`, `nsec: 4294967295`) не читает bag. Закрыто в writer:
   блок `files:` и целые `rmw` enum. Повторный play — finished cleanly.
2. Граф из четырёх нод недостаточен как приёмка. Один `echo` после play
   показал, что колёса дошли: \(s=0.709\,\mathrm{m}\), \(v=0.959\,\mathrm{m/s}\),
   `n_omega_used=4`, `chol_fail=0`. Статус **DEGRADED**, не OK. Клип — 75 кадров
   twin-CSV на вагоне `lvenok_moscow`; в числа хакатона не переносится.
3. После конца bag `/clock` замирает. Ночью `wheels_fresh` оставался 1.
   Утро : возраст публикации включает `clock_stall_s` (steady clock,
   не \(dt\)). Echo хвоста: stall 6.46 s, `wheels_fresh=0`, LOST. Внутри
   клипа статус по-прежнему не заявка качества (ночной кадр был DEGRADED).
4. `{}` в `declare_parameter<vector<double>>` Humble читает как
   `ParameterDescriptor`. Повторный `colcon` — без этого stderr.

---

## 1. Граница проверки

| Что | Уровень | Результат |
| :--- | :-: | :--- |
| `write_bag` metadata v5 + QoS | **E** | `BagRoundtripTests`; `ros2 bag info` и `play` в `railbreak-tram_dr` |
| `live.launch.py` / `replay.launch.py` | **E** | четыре ноды; play `--clock` finished cleanly |
| Один `/tram/state_estimate` + diagnostics | **E** | echo 22:15 UTC, [ros-rehearsal.md](../ros-rehearsal.md) |
| `create_wall_timer` при стоящем `/clock` | **E** | утро : `clock_stall_s=6.46`, `wheels_fresh=0`, LOST на хвосте; stamp 1.462 s |
| `Ukf::update_wheels` ветка `r_adapt` | **R** | NIS на номинальном \(R\); откат шага копией `Ukf` возвращает \(\alpha,\beta\) |
| `maybe_stop_update` | **R** | default-off; `project_pd==false` увеличивает `chol_fail_` и откатывает весь шаг |
| `zupt_at_stop` в diagnostics | **R** | это `mass_door_allowed`, не признак стоянки |
| seed 42 e2e после M5/M11 | **N** | умолчания не меняют шаг; таблицы не переснимались |
| `bench.py` на записанном Odometry-bag | **N** | запись не `filter.csv` |
| bag организатора, CMA-ES, пп. 14.4–14.6 | **N** | не этот патч |

---

## 2. Находки

### RT24-01 — P1 — writer, который Humble не играет

`ros2 bag info`: `invalid node; first invalid key: "files"`.
После добавления `files:` play падал на `yaml-cpp: bad conversion`:
именованные политики и `uint32` nsec `4294967295` не конвертируются в
`rmw_time_t`. Нативный `ros2 bag record` пишет `history: 1`,
`reliability: 2`, duration `9223372036 / 854775807`.

Закрыто в `tools/eval/rosbag2_io.py`. Контракт: `BagRoundtripTests`
(`files:`, `history: 1`). Исполняемо: play finished cleanly.

### RT24-02 — P3 — ложный `ParameterDescriptor`

`map_projector_node`: `declare_parameter<std::vector<double>>("route_s_m", {})`.
`{}` — не пустой вектор. Заменено на `std::vector<double>{}`.
Повторный `colcon` в том же образе: `Finished`, stderr пустой.

### RT24-03 — P2 — свежесть после стопа `/clock`

Открыто. `timer_ = create_wall_timer(...)` не следует `/clock`.
Пока bag играется 1×, wall и sim близки. Когда player выходит, stamp
публикации замирает (наблюдено 1.463 s), а таймер шлёт тот же кадр.
`dt < dt_min` на измерении не выдумывает шаг — состояние не ползёт.
Врёт диагностика: `wheels_fresh=1` при отсутствии новых колёс.

Не чинить смешением `steady` и stamp в качестве \(dt\).

Утро : закрыто. `note_clock_stall` копит steady-время, пока наносекунда
`now()` та же, и прибавляет его только к опубликованному `age_s`.
Шаг `predict_and_update` это число не видит. Humble echo после play:
`clock_stall_s=6.460743`, `wheels_fresh=0`, confidence LOST, stamp 1.462 s.
`colcon` без stderr.

### RT24-04 — наблюдение, не дефект приёмки графа

Счётчик `ros2 bag record` (676 или 389) — частота watchdog ~50 Hz по wall,
не 75 шагов фильтра. Приёмка фильтра — echo: четыре оси, конечные \(s,v\),
не UNINITIALIZED. DEGRADED на коротком клипе чужого вагона остаётся в отчёте.

`zupt_at_stop=1` при \(v\approx0.96\) не означает ZUPT: ключ исторический,
`zupt_forced=0`.

---

## 3. Что не открывали

Подписку фильтра на NavSatFix / IMU / PointCloud2, bump `kModelVersion`,
включение `r_adapt` или `stop_update` по умолчанию, пересъём seed 42,
оценку записанного bag стендом `bench.py`.
