# Tools

Python 3.11+, стандартная библиотека. `rosbags` опционален (mcap).

Профили вагона: `vehicle_lvenok_moscow.yaml` (маршрут 10, Bo-Bo, default
launch; тара TBD), `vehicle_combino_nf100.yaml` (twin e2e), `vehicle_vityaz_m.yaml`
(6 осей, 37 т тары, $r_0$ с bag).

| Скрипт | |
| --- | --- |
| `tools/eval/no_gnss_scan.py` | нет GNSS/IMU/lidar в фильтре |
| `tools/eval/check_envelope.py` | конверт vs GT; HMI-rate |
| `tools/eval/run_e2e.py` | синтетика → `replay_ukf` → чекер |
| `tools/eval/run_bag.py` | rosbag2 организатора |
| `tools/eval/inspect_bag.py` | топики, Hz, notch; квантование $\omega$; `--write-yaml`; sqlite3 и mcap (`pip install rosbags`) |
| `tools/eval/identify_coast.py` | выбег → $A_d$; при `gt_v` ещё $r_0$ |
| `tools/eval/identify_notch.py` | низкая $v$: OLS $a$ vs $n$ → `notch_as_accel`; выше $v_b$ — колено |
| `tools/eval/identify_jerk.py` | первый разгон → `j_max_mps3`, `tau_drv_s` |
| `tools/eval/stop_associate.py` | JSONL оценки + `route_10.yaml`; не измерение UKF |
| `tools/eval/profile_from_bag.py` | $z$ лидарной позы → $h(s)$ оффлайн; кап 250k; не $i(s)$ в ноде |
| `tools/synth/generate.py` | сценарии, seed 42; `*route10*` — оценка 25–40 ‰ (центр 32 ‰), не $i(s)$ в ноде |
| `tools/synth/score.py` | таблица vs baseline |
