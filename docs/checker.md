# Чекер конверта

`tools/eval/check_envelope.py` не импортирует UKF. Точка не принимается,
если нет интервала, нет статуса доверия или GT пробивает конверт
$\lvert \hat s - s_{\mathrm{gt}}\rvert > 5 + 0.05\lvert s_{\mathrm{gt}}\rvert$ при статусе OK.

Класс `ENVELOPE_GT` — детектор hazardously-misleading (HMI): система сказала
OK, пока ошибка уже выше alert limit. Главная метрика — **HMI-rate**
(доля OK-записей с пробитием), не RMSE. Чекер печатает `HMI-rate=…` и
`missed_path_until_degraded` (ошибка пути в момент первого DEGRADED).

Если в записи есть GT, конверт проверяется **всегда**. `--require-gt` —
«ошибка, если GT нет», а не «не смотреть GT». Без GT: `ENVELOPE_GT skipped: no GT`,
конверт не считается пройденным.

| Класс | Когда |
| --- | --- |
| `NO_ESTIMATE` | нет `/tram/state_estimate` и нет jsonl UKF |
| `NO_COVARIANCE` | ковариация нулевая или NaN |
| `NO_CONFIDENCE` | нет OK / DEGRADED / LOST / UNINITIALIZED |
| `GNSS_IN_FILTER` | нода фильтра подписана на Fix / IMU / cloud (`no_gnss_scan.py`) |
| `UNINITIALIZED` | нет колёс на старте или стартовая $P_{ss}$ |
| `ENVELOPE_GT` | HMI: конверт пробит при OK (только если GT есть) |

Коды выхода: `0` чисто, `2` грязно, `1` ошибка вызова.
`mismatch_r0` даёт HMI намеренно (`run_e2e.py` не валит сборку).
Каноническое число этого сценария — **0.515** (архивное 0.42 соответствует старому
клипу генератора), см. [`metrics.md`](metrics.md).

RMSE vs baseline — строка отчёта, не класс отказа.
Конверт 5 м + 5% — калибровочная линия vs GT, не сертификат.

`nav_msgs/Odometry` сам по себе не несёт статус. `bag_to_jsonl.py` берёт
OK / DEGRADED / LOST из `/tram/diagnostics` (окно 50 мс).
$P_{ss}\ge 5\cdot10^5$ остаётся `UNINITIALIZED`, даже если diag говорит OK.

rosbag2 без ROS: sqlite3 + CDR; mcap — опциональный `pip install rosbags`.

```
tools/eval/check_envelope.py
tools/eval/inspect_bag.py
tools/eval/bag_to_jsonl.py
tools/eval/run_bag.py
tools/eval/run_e2e.py
tools/eval/baselines.py
tools/eval/no_gnss_scan.py
tools/eval/identify_coast.py
tools/eval/identify_notch.py
tools/eval/identify_jerk.py
```

## Второй слой: контракт-тесты ядра

Чекер выше отвечает на вопрос «врёт ли оценка относительно GT». Он **не**
отвечает на вопрос «соблюдает ли ядро свои инварианты безопасности»: сценария,
где колёса согласно лгут, в конверте могут выглядеть безукоризненно.
Поэтому второй, независимый слой — пять целей `ctest`:

| Цель | Что доказывает |
| --- | --- |
| `test_core` | сквозное поведение фильтра, режимы, ZUPT, стоянка после реального торможения |
| `test_integrity_contracts` | контракты целостности: свидетельство стоянки, $\omega$-only ZUPT, live-only $z$, `s_unbounded`, свежесть во времени, контракт входа, атомарный откат, отказ `chol`/`inv_spd` на NaN |
| `test_ut_weights_psd` | 28 проверок алгебры весов scaled UT; **ядро не линкуется** — тест нельзя «починить» правкой фильтра |
| `test_ut_weights_header` | сверка поставляемого `ut_weights.hpp` с той же алгеброй |
| `test_prior_and_nis` | неподвижная точка дефектного mass prior и насыщение Huber-NIS; header-only |

```bash
cmake -S standalone -B standalone/build -DCMAKE_BUILD_TYPE=Release
cmake --build standalone/build --parallel
ctest --test-dir standalone/build --output-on-failure
```

Коды выхода тестовых бинарей: `0` — все проверки пройдены, ненулевой —
печатается имя провалившегося контракта. В CI все пять целей собираются ещё и
под ASan/UBSan (`detect_leaks=1`, `halt_on_error=1`); до этого под санитайзерами шёл
только `test_core` — это была находка `F-11`.

Граница доверия: фикстура `zupt off stop` после правки `F-12` прогоняется
целью `test_core` (`ctest`). Подробно — [`verification.md`](verification.md).
