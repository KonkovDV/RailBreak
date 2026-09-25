# Каркас чисел хакатона

Слоты заполняются после T0. Пока стоит `TBD`, число в доклад не переносится.

| Слот | Значение | Источник |
| --- | --- | --- |
| RMSE_v UKF на записи организатора | TBD | `tools/eval/bench.py` test set, один прогон |
| RMSE_s UKF на записи организатора | TBD | тот же прогон |
| HMI-rate (OK exposure) | TBD | `check_envelope.py` |
| availability OK | TBD | знаменатель кадров |
| p50 шага `replay_ukf` | TBD | stderr бинаря |
| p99 шага `replay_ukf` | TBD | stderr бинаря |
| p99 цепочки ROS | TBD | stamp входа → публикации |
| \(\kappa_{ob}\) overbounding | TBD | M9 на val |
| открытый вопрос организаторам | TBD | `docs/organizer-questions.md` |

Не подставлять seed-42 из [`metrics.md`](metrics.md) в эти слоты.
