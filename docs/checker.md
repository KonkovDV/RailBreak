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
