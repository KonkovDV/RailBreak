# Архитектура

Продукт: **tramDR** (`tramDR-0.0.10`). Пакет ROS 2 Humble: `tram_dr_localization`.
Ядро `libtram_dr` — C++17 без `rclcpp` (сборка и тесты через `standalone/`).
История изменений и открытые пункты — [`../CHANGELOG.md`](../CHANGELOG.md);
граница проверенного — [`verification.md`](verification.md).

```
topic_adapter_node     чужие имена/типы → /tram/* (по умолчанию выкл.)
state_estimator_node   предикт по колёсам; публикация 50 Гц
map_projector_node     s → NavSatFix (STATUS_NO_FIX, не приёмник GNSS)
fault_monitor_node     watchdog на /tram/diagnostics
libtram_dr             контракт входа → plant (Дэвис+Кулон) → SCA → scaled UKF
входы фильтра          notch, brake, wheel_odom  (нет GNSS / IMU / лидара)
GT                     /gt/* только оффлайн
```

## Профили вагона

Launch: `vehicle:=` накладывается на `estimator.yaml`. Оверлей Львёнка **не**
заменяет `mass_kg` / `wheel_radius_m`: их нет в YAML вагона. До `identify_*`
по bag в фильтре остаются числа twin (28 т, 0.35 м) плюс клип [15, 40] т,
`sca_pair_lr=false`, `mass_door_kg=12000`, `r0_uncalibrated=true` (статус
пути DEGRADED, пока `identify_coast` не запишет `wheel_radius_m`).

| YAML | Зачем |
| --- | --- |
| `vehicle_lvenok_moscow.yaml` | **default launch**: 71-911ЕМ, Bo-Bo, все 4 оси моторные, `sca_pair_lr: false`; тара/Ø TBD (разброс источников, не РЭ); клип [15, 40] т; `mass_door_kg=12000`. Не путать с автономным ходом маршрута 90 |
| `vehicle_combino_nf100.yaml` | синтетический twin: 28 т, 4 оси, $r_0=0.35$, `sca_pair_lr: true` |
| `vehicle_vityaz_m.yaml` | 37 т тары, 6 осей, `sca_pair_lr: false`; не вагон маршрута 10 |

$a_{\max}$, $P_{\max}$, Ø Львёнка и Витязя **не** паспорт.
`replay_ukf filter.csv out.jsonl --vehicle path.yaml`. Без флага — Combino
(e2e twin). `run_e2e.py` для `six_axle` передаёт YAML Витязя.
Лидар / NavSatFix / одометрия ЦБТ в bag — GT оффлайн (`identify_*`, чекер),
никогда подписка оценщика.

## Контракты ядра (0.0.10)

Это не «проверки на всякий случай», а граница ответственности между узлом и
фильтром. Каждая строка закреплена тестом, а не комментарием.

| Контракт | Поведение ядра | Где закреплено |
| --- | --- | --- |
| $\Delta t\in[5,200]\,\mathrm{ms}$ | шаг отклонён, состояние не тронуто | `test_integrity_contracts` |
| конечность `notch`, `brake`, $\omega_i$ | шаг отклонён; NaN не проникает в $P$ | `test_integrity_contracts` |
| атомарность шага | при отказе Холецкого — полный откат (`*this = previous`), `chol_fail`++ | `test_integrity_contracts` |
| стоянка только по свидетельству | нулевые $\omega$ при отпущенном тормозе сразу ZUPT не включают; $\omega\equiv 0$ дольше 2 с при тихой ручке — `zupt_forced` + `DEGRADED` | `test_core`, `test_integrity_contracts` |
| live-only измерение колёс | мёртвый канал не входит в $z$; $P$ по нему не сжимается | `test_ukf` |
| `s_unbounded` | latch не публикует `over_m=1e6`; JSON `null` | `test_integrity_contracts` |
| режим не подтверждает себя | `classify` не читает свой предыдущий вывод как вход | `test_integrity_contracts` |
| свежесть энкодеров во времени | `freeze_s` в секундах, не в тактах | `test_integrity_contracts` |
| алгебра весов UT | $W_c^{(0)}\ge 0$ и достаточное условие PSD | `test_ut_weights_psd` (ядро не линкуется) |

Сборка и прогон:

```bash
cmake -S standalone -B standalone/build -DCMAKE_BUILD_TYPE=Release
cmake --build standalone/build --parallel
ctest --test-dir standalone/build --output-on-failure
```

Цели `standalone/`: `test_core`, `test_integrity_contracts`,
`test_ut_weights_psd`, `test_ut_weights_header`, `test_prior_and_nis`,
`replay_ukf`. Все пять тестовых целей зарегистрированы в `ctest` и все пять
собираются под ASan/UBSan в CI. `-ffast-math` запрещён.

## Исполнение

Предикт — в колбеке колёс. $\Delta t$ из `header.stamp` у `JointState` /
`TwistStamped`; иначе время приёма. Канон `Float64MultiArray` штампа не несёт.
`wheel_latency_s` только в диагностике, из шага plant **не** вычитается.
Таймер 50 Гц — watchdog `age_s` и публикация последней оценки: топик не
замолкает при потере колёс. UKF стартует по колёсам; одна ручка фильтр не
поднимает. Входной QoS: keep-last 1, best-effort. Выход odom: keep-last 1,
reliable. Нефинитные $\omega$ → frozen, не медиана.

Важно для рецензента: узел **валидирует** $\Delta t$, а не зажимает его
в контракт. Регрессия штампа отклоняет кадр; разрыв больше `dt_max_s`
проходит серией предсказаний, чтобы $Q$ копился за истинное время (`F-17b`).

Live и offline — один алгоритм, не bit-identical (штампы ROS vs CSV dt).
`replay.launch.py`: `use_sim_time:=true`, `ros2 bag play --clock`.
`live.launch.py` включает тот же граф на wall-clock.

```bash
ros2 launch tram_dr_localization replay.launch.py bag:=/data/bags/run01
ros2 launch tram_dr_localization replay.launch.py \
  vehicle:=/path/to/vehicle_vityaz_m.yaml bag:=/data/bags/run01
```

После bag:

```bash
python tools/eval/inspect_bag.py data/bags/run01 \
  --write-yaml tram_dr_localization/config/customer_topics.yaml
```

`customer_topics.yaml` грузится и в адаптер, и в оценщик: типы DDS на уже
канонических `/tram/*` правятся без петли адаптера (`enable: false`, если
имена уже `/tram/*`).

## Топики

Канон оценщика: **Float32** notch и **Float64MultiArray** колёс на `/tram/*`.
Иначе — `notch_type` / `wheels_type` или адаптер с **других** имен.
`TwistStamped.linear.x` — м/с → $\omega=v/r_0$, пока `twist_is_omega` не true.

| | Топик | Тип (канон) |
| --- | --- | --- |
| вход | `/tram/controller_notch` | Float32 |
| вход | `/tram/brake_cmd` | Float32 |
| вход | `/tram/wheel_odom` | Float64MultiArray |
| выход | `/tram/state_estimate` | `nav_msgs/Odometry` (`pose.x=s`, `twist.x=v`, $P_{ss}$, $P_{vv}$) |
| выход | `/tram/diagnostics` | OK / WARN / ERROR + ключи |
| выход | `/tram/fix` | `NavSatFix`, `STATUS_NO_FIX`, `frame_id=dead_reckoning` |
| GT | `/gt/*` | только eval |

Ключи diagnostics (фрагмент): `confidence`, `confidence_v`, `confidence_s`,
`mode`, `model_version`, `vehicle_profile`, `n_wheels`, `relative_wheel_slide`,
`path_disagree_m`, `path_disagree_latched`, `b_s_m`, `pl_s_m`, `al_s_m`, `age_s`, `wheels_fresh`, `n_frozen`, `n_slip_axles`,
`nis`, `chol_fail`, `slip_latched`, `zupt_at_stop`, `zupt_forced`,
`s_unbounded`, `sca_current`.
`over_m` $=PL_s=k_{\mathrm{over}}\sqrt{P_{ss}}+b_s$.
Фильтр: $AL_s=5+0.05\max(\hat s,0)$. Чекер HMI: $5+0.05\lvert s_{\mathrm{gt}}\rvert$.
`chol_fail` — не косметика: рост счётчика означает отклонённые шаги, то есть
предикт без обновления; при ненулевом значении запись надо разбирать, а не
сдавать как чистый прогон.

`route_10.yaml` — вершины остановок OSM (9 точек; ginfo: 9 туда / 8 обратно
без Бурназяна; списки «8» не удаляют стоп без GTFS), не ось пути и не $i(s)$.
Тот же `route_s_m` грузится в оценщик: `mass_door_kg` только у вершины
(`stop_gate_m=40`). Строгинский мост — рамно-подвесной (Носарев/Скрябина 2004),
не «арочный 120 м». Уклон неизвестен: оценка 25–40 ‰ (центр 32 ‰) только в
генераторе `*route10*`. С июня 2026 временная линия на русловой части
(mos.ru / АГН / Метро 17.06); режим июня 2026, не майский заголовок.
Bag 25.09: вероятна однопутка, не 60 км/ч. DEM на мосту не доверять (DSM).
$h(s)$ — `profile_from_bag.py` оффлайн. Длину 5.5 км не цитировать.
`/tram/brake_cmd` — семантика только из payload (магниторельс не включать
из OSINT).

## Деградация

| Условие | Статус |
| --- | --- |
| все оси живы, малый $\lvert\kappa\rvert$ | `OK` |
| одна ось freeze | inflate этой оси, $v$ `OK` |
| мотор буксует, trailer живы | медиана по trailer, если `axle_role` задан |
| все оси юзят при тормозе | `DEGRADED`, latch по $s$ |
| WSP держит $\lvert\kappa\rvert<\kappa_{\mathrm{cut}}$, $D\ge AL_s$ | `DEGRADED` по $s$ (интеграл двух принципов) |
| юз + уклон, кузов ещё тормозит | `DEGRADED` (синтетика 1:56) |
| юз + уклон, $a_{\mathrm{kin}}>0.05$ | `LOST` |
| $n\le 2$, любое рассогласование | `DEGRADED` |
| все оси inflated ($n\ge 4$) / все NaN | `LOST` |
| нет notch дольше $2$ с (после stale $0.25$ с) | `LOST` |
| `r0_uncalibrated` (Львёнок) | `DEGRADED` по $s$ до `identify_coast` |
| нет колёс дольше $1$ с | `DEGRADED`→`LOST`; публикация идёт |
| нет колёс на старте | `UNINITIALIZED`; топик публикуется |
| нефинитный вход или $\Delta t$ вне контракта | шаг отклонён, полный откат, `chol_fail`++; статус по возрасту данных |
| нулевые $\omega$ при отпущенном тормозе | **не** мгновенный ZUPT (`F-01`); после 2 с тихой ручки — `zupt_forced` |

По умолчанию `axle_role` все motor. У Львёнка это факт Bo-Bo, не гипотеза:
ветки trailer нет. При потере связи с берегом борт считает дальше; на берегу
растёт `age_s`.

## Узел после PR #4

`F-17` и `F-17b` закрыты в `state_estimator_node.cpp`: шкалы штампов датчика и
часов узла разведены; NaN notch/brake/twist отклоняются; регрессия штампа не
зажимается в шаг вперёд. Сборка узла в CI — job `ros` (`F-19`: `continue-on-error`
снят, `source setup.bash` без `set -u`). Ядро по-прежнему отклоняет нефинитный вход и
$\Delta t$ вне контракта — защита в глубину остаётся.
