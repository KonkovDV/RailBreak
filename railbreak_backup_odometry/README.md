<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# railbreak_backup_odometry

Пакет ROS 2 Humble на C++17. Считает продольную скорость и положение трамвая по двум тележкам и ручке контроллера. GNSS читается только 3 с на старте, чтобы поставить вагон на карту маршрута, затем подписка удаляется.

| Что | Где |
|---|---|
| Нода | `src/backup_odometry_node.cpp` |
| Фильтр и слои вокруг него | `include/railbreak_backup_odometry/` (фильтр — `track_odometer.hpp`) |
| Запуск | `launch/backup_odometry.launch.py` |
| Параметры | `config/params.yaml` |
| Карта, остановки, таблица ручки | `assets/ring.csv`, `assets/stops.csv`, `assets/notch.csv`, `assets/meta.yaml` |
| Тесты | `test/test_gnss_stress.cpp` (ROS), `tools/test_core.cpp` (ядро без ROS) |

Модель — [model.md](../docs/solution/model.md), допущения и параметры — [assumptions.md](../docs/solution/assumptions.md), результаты — [results.md](../docs/solution/results.md).

## Входы и выходы

| Топик | Тип | Что делает нода |
|---|---|---|
| `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | поле `velocity` в записях — км/ч; нода делит на 3.6 (`wheel_unit_scale`) |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | позиция ручки −15…+15: больше нуля тяга, меньше нуля торможение, ноль выбег |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | только окно старта 3 с, затем подписки удаляются |
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | продольная скорость, м/с |
| `/result/position` | `nav_msgs/Odometry` | точка `base_link` в MGRS, `frame_id` `map`, `child_frame_id` `base_link`, скорость в `twist.twist.linear.x` |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, целостность, доверие скорости и положения, GNSS, порядок входов |

Без типа `DriverControllerCommand` нода не стартует: ручка не подменяется нулём. В `tram_vehicle_msgs` из архива `check-code-with-bag.zip` этого сообщения нет, поэтому собирать нужно с полным пакетом сообщений организатора. Отдельного топика тормоза нет и не нужно: торможение — отрицательная ручка.

`base_link` — ось вращения первой тележки в точке касания колеса и рельса. Кадр по умолчанию — UTM 37N минус угол квадрата 300000 м на восток и 6100000 м на север: x — восток, y — север, на маршруте x около 99–103 км. z — эллипсоидальная высота WGS84 точки касания. От антенны master точка сдвинута на +9.873 м вдоль пути и на −3 м по высоте (tf организаторов: master (−9.873, 0, 3), rover (2.563, 0, 3)).

`header.stamp` каждого выхода — штамп входа, который его породил. Часы ноды и `/clock` не используются, отдельного таймера нет.

## Сборка без интернета

Нужны ROS 2 Humble и пакет сообщений организатора `tram_vehicle_msgs`. Карта маршрута уже лежит в `assets/`. Из корня клона:

```bash
mkdir -p ~/ws/src
cp -a railbreak_backup_odometry ~/ws/src/
cp -a <датасет>/tram_vehicle_msgs ~/ws/src/
source scripts/ensure_maintainer.sh
ensure_maintainer ~/ws/src/tram_vehicle_msgs/package.xml
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`ensure_maintainer` нужен потому, что в выданном `package.xml` нет тега `<maintainer>`, и Humble такой пакет не собирает. Сценарии `scripts/jury.sh` и `scripts/jury.ps1` делают это сами.

Зависимости — только пакеты Humble: `rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`.

Тесты: `colcon test --packages-select railbreak_backup_odometry` запускает `test_gnss_stress` (окно GNSS через исполнитель ROS) и `test_core` (фильтр, целостность, очередь).

## Запуск

С Docker, из корня клона, на Windows, Linux и macOS:

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

Без Docker, Ubuntu 22.04 с Humble. Нода поднимается до проигрывателя, иначе окно GNSS будет пропущено:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py      # терминал 1
ros2 bag play <каталог rosbag2> --rate 1                            # терминал 2
```

Остальные сценарии, приёмка и запуск официального checker — в [корневом README](../README.md).

## Что должно появиться

```bash
ros2 topic echo /result/velocity
ros2 topic hz /result/velocity
ros2 topic echo /result/position --field pose.pose.position
ros2 topic echo /result/diagnostics --field status[0].values
```

- `/result/velocity` — с первого сообщения тележки, на каждый вход (около 29 Гц на записи checker).
- `/result/position` — через 3 с после первого валидного фикса любой антенны.
- В диагностике `gnss` = `closed`: GNSS больше не читается.
- `integrity_status` = `NOMINAL` на исправных данных.

Поля диагностики, которые стоит знать:

| Поле | Что значит |
|---|---|
| `mode` | `WHEELS`, `MODEL` (этот шаг ведёт модель), `ZUPT` (стоянка), `COMMON_MODE_UNOBSERVABLE`, `FREEZE` (численный откат шага) |
| `integrity_status` | `NOMINAL`, одна из деградаций, `POSITION_UNTRUSTED` или `LOST` |
| `velocity_confidence`, `position_confidence` | `HIGH`, `LOW` или `NONE`, раздельно для скорости и положения |
| `slip`, `slip_front`, `slip_rear` | невязка колеса к модели выше порога. Это не доказанный юз: то же дают ошибка таблицы, уклон или сбой штампа |
| `time_to_lost`, `distance_since_last_trusted_anchor`, `common_mode_exit` | остаток слепого бюджета в общей моде; `common_mode_exit` = `anchor` только после принятой станции |
| `n_anchor`, `sigma_s` | число принятых якорей станций и шкала неопределённости пути |
| `order_reason`, `n_behind_out` | порядок входов и число штампов, пришедших позади уже опубликованного выхода |
| `callback_max_us` | самое долгое время обработки одного входа. Это не задержка от приёма входа до публикации |

## Как нода ведёт себя при сбоях

| Ситуация | Что происходит |
|---|---|
| Одна тележка врёт или молчит | Её вес падает (шум R = 25 (м/с)²), скорость держат вторая тележка и модель тяги |
| Обе тележки молчат | Скорость ведёт модель по ручке; положение публикуется, пока целостность это позволяет |
| Обе тележки согласны друг с другом, но расходятся с моделью дольше 3 с | `COMMON_MODE_UNOBSERVABLE`: колёса не берутся до станции. Через 5 с или 100 м без якоря — `LOST`, `/result/position` не публикуется, скорость остаётся с доверием `LOW` |
| Разрыв штампов больше 30 с | Интервал не интегрируется |
| Штамп шагнул назад, обе тележки старше 30 с, нет абсолютного старта | `POSITION_UNTRUSTED`, положение на этом шаге не публикуется |
| Нет фикса GNSS 10 с | Относительный путь: x — пройденный путь, y = z = 0 |
| Нет каталога карты | Нода не падает: скорость и относительный путь, y = z = 0 |

Порядок входов. Тележки и ручка стоят в одной очереди. Вход выпускается, когда все живые потоки дошли до его штампа, и применяется по порядку штампов. Поток, который отстал больше чем на 1 с, помечается `ORDER_NOT_RESTORED`. Пока он приходит, его ждут: в начале записи `30618_88aea4d9` задняя тележка отстаёт на 1.3 с и догоняет. Замолчавший поток перестают ждать после 50 чужих входов. Удержания сверх этого нет (`stamp_reorder_s` = 0).

## Параметры

Все — в `config/params.yaml`, полный перечень с пояснениями — в [assumptions.md](../docs/solution/assumptions.md). Без пересборки через `ros2 launch`:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py assets_dir:=/путь/к/assets
ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=mkrs_start
```

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `output_frame` | `mgrs` | оси выхода: `mgrs`, `mkrs_start`, `mkrs`, `grid_start`, `enu` |
| `gnss_init_window_s`, `gnss_wait_s` | 3 с, 10 с | окно старта и ожидание первого фикса |
| `wheel_unit_scale` | 1/3.6 | единица скорости тележек |
| `initial_s_m`, `initial_lat_deg`, `initial_lon_deg` | не заданы | ручной старт без GNSS |
| `load_factor`, `wheel_radius_m`, `davis_*` | 1, 0, 0 | масса, радиус и сопротивление: по умолчанию не подставляются, всё уже в таблице тяги |

## Сравнение с GNSS

Официальный checker из `check-code-with-bag.zip` и его запуск — в [корневом README](../README.md), числа — в [results.md](../docs/solution/results.md). На учебных записях эталона `kinematic_state` нет, поэтому для них есть свой скрипт:

```bash
ros2 bag record -o out /result/velocity /result/position /result/diagnostics
python3 tools/organizer/score_ros.py out <исходный bag> --frame mgrs
```

Он сравнивает выход с точкой `base_link` на отрезке антенн master→rover (доля 9.873/12.436, высота минус 3 м). Фикс rover берётся, только если база 12.436 ± 2 м и разность высот антенн не больше 1 м. Это прокси, не эталон судьи.

## Состав и права

Снимок пакета — [`MANIFEST.json`](MANIFEST.json): хеши карты и таблицы и параметры по умолчанию. Карта собрана по GNSS рейсов train, не из OSM, и заменяется параметром `assets_dir`. Отдельной публичной лицензии нет: код передаётся организаторам по Положению хакатона; сторонние компоненты — [NOTICE](../NOTICE). Исследовательское ядро `tramDR-0.0.11` в этой папке нет, в сдачу оно не входит.
