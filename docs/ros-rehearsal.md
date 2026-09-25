# ROS dress rehearsal

Чеклист фазы 0.11. Прогон на этом Windows-хосте: Docker Desktop **29.8.0**,
образ `railbreak-tram_dr` (`sha256:48bc4309c98e3f656f8392225cc27bd0942a22be16b0d5779999f71e99890dc4`),
`FROM osrf/ros:humble-desktop`, `ROS_DISTRO=humble`. Дата контейнера:
**–22:16 UTC** ( ~01:01–01:16 MSK). Повтор 22:15 UTC:
`colcon` без stderr (предупреждение `ParameterDescriptor` снято).

Офлайн-писатель rosbag2 (без `ros2`) плюс `ros2 bag info` / `play` в Humble.

Повтор:

```bash
docker compose build tram_dr
docker compose run --rm -T -e BAG=/data/bags/synth_rehearsal tram_dr \
  bash /opt/tram_dr/tools/hackathon/ros_rehearsal_inner.sh
```

На Git Bash: `scripts/ros_rehearsal.sh`. `AMENT_TRACE_SETUP_FILES` ломает
`set -u` до `source /opt/ros/humble/setup.bash` — inner-скрипт и compose
ставят `set +u` перед source. Синтетический bag: 75 кадров из
`reports/t0-synth/run/filter.csv` → `csv_to_bag` (`data/bags/` в git не входит).

| Шаг | Статус |
| --- | --- |
| `python tools/synth/generate.py --out synth/runs` | команда есть; полный пакет seed 42 в `reports/baseline-0.0.11/` |
| синтетический rosbag2 writer | **да**: `csv_to_bag` → sqlite3 v5; блок `files:`; QoS — целые `rmw` enum (KEEP_LAST=1, BEST_EFFORT=2, VOLATILE=2, AUTOMATIC=1), depth 10 |
| `ros2 launch … live.launch.py` | **да**: `/topic_adapter` `/state_estimator` `/fault_monitor` `/map_projector`; `use_sim_time=false`; model=`tramDR-0.0.11` vehicle=`lvenok_moscow` n=4 |
| `ros2 launch … replay.launch.py bag:=…` | **да**: `ros2 bag play … --clock` finished cleanly; `/clock`; канон `/tram/controller_notch` Float32, `/tram/brake_cmd` Float32, `/tram/wheel_odom` Float64MultiArray |
| `ros2 bag record /tram/state_estimate /tram/diagnostics` | **да**, но счётчик — wall-timer, не число шагов фильтра. Прогон 22:15 UTC: 389+389 за 7.76 s wall (`data/bags/rehearsal_record/`, gitignored) |
| `python tools/eval/bench.py` на записанном bag | **да, не seed 42**: `--odom-bag` вызывает тот же `score_method`. Запись `data/bags/rehearsal_record`: 60 уникальных header stamp, \(t\le 1.463\,\mathrm{s}\), RMSE\(_s=0.027\,\mathrm{m}\), RMSE\(_v=0.056\,\mathrm{m/s}\), `env_s=env_v=1`. Горизонты 2–30 с — NaN, клип короче. Локальный json не в git |
| имена топиков совпадают с `customer_topics.yaml` | вход канон `/tram/*`; adapter `enable=false` |
| QoS depth входа | writer depth 10 best_effort; нода: `KeepLast(10).best_effort()` на входе. Выход оценки — reliable depth 1 |
| `use_sim_time` | `replay.launch` default `true` + `--clock`; `live.launch` — `false` |

Имена/типы с графа replay (после play, подписчики ещё держат топики):

```
/tram/brake_cmd           std_msgs/msg/Float32
/tram/controller_notch    std_msgs/msg/Float32
/tram/wheel_odom          std_msgs/msg/Float64MultiArray
/tram/state_estimate      nav_msgs/msg/Odometry
/tram/diagnostics         diagnostic_msgs/msg/DiagnosticArray
/tram/diagnostics_watchdog diagnostic_msgs/msg/DiagnosticArray
/tram/fix                 sensor_msgs/msg/NavSatFix
/clock                    rosgraph_msgs/msg/Clock
```

`/tram/fix` — выход `map_projector` (`STATUS_NO_FIX`), не измерение UKF.

## Один кадр после play (22:15 UTC)

`ros2 topic echo --once`. Это не метрика и не seed 42. Клип — 75 кадров
twin-CSV, launch — `lvenok_moscow`. Конец клипа в CSV: \(s_{gt}=0.649\,\mathrm{m}\),
\(v_{gt}=0.860\,\mathrm{m/s}\).

| Поле | Наблюдение |
| --- | --- |
| `pose.position.x` | 0.709 m |
| `twist.linear.x` | 0.959 m/s |
| `header.stamp` | 1.463 s (время `/clock`, не wall 2026) |
| `n_omega_used` | 4 |
| `model_version` | `tramDR-0.0.11` |
| `confidence` | DEGRADED (`level` 1 = WARN), `chol_fail=0`, `bad_wheels=0` |
| `zupt_at_stop` | 1 — это `mass_door_allowed`, не «стоим» (`zupt_forced=0`) |

После конца bag `/clock` стоит. Утро : к возрасту прибавляется
`clock_stall_s` (steady clock, не \(dt\)). Echo через ~6 с после play:
`clock_stall_s=6.46`, `wheels_fresh=0`, confidence **LOST**, stamp по-прежнему
1.462 s. LOST здесь — хвост без новых колёс, не статус внутри клипа.
Ночной кадр сразу после play был DEGRADED при `wheels_fresh=1`.

## Что сломалось и как починили

1. `set -u` + Humble `setup.bash` → `AMENT_TRACE_SETUP_FILES: unbound variable`.
2. Metadata без ключа `files:` → `ros2 bag info`: `invalid node; first invalid key: "files"`.
3. `offered_qos_profiles` со строками `keep_last` / `nsec: 4294967295` →
   `ros2 bag play`: `yaml-cpp: bad conversion`. Humble ждёт целые enum и
   Duration `sec: 9223372036` / `nsec: 854775807` (как у нативного `ros2 bag record`).

Предупреждение сборки: `map_projector_node` `declare_parameter<vector<double>>({})`
(explicit `ParameterDescriptor`). Не блокирует launch.

Не закрыто: bag организатора; `bench.py` на *их* записи; p99 цепочки
stamp входа → публикации на полевой частоте; CMA-ES .
