<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# railbreak_backup_odometry

Резервная одометрия трамвая по двум тележкам и ручке контроллера, ROS 2 Humble, C++17.

| Что открыть | Где |
|---|---|
| Нода | `src/backup_odometry_node.cpp`, заголовки в `include/railbreak_backup_odometry/` |
| Launch | `launch/backup_odometry.launch.py` |
| Параметры | `config/params.yaml` |
| Карта, остановки, таблица ручки | `assets/ring.csv`, `assets/stops.csv`, `assets/notch.csv`, `assets/meta.yaml` |
Все сценарии запуска — в [корневом README](../README.md): на Windows, Linux и macOS
это `scripts/jury.ps1` / `scripts/jury.sh`, на Ubuntu без Docker — команды ниже.
Питч и разбор практики — [`../docs/solution/pitch.md`](../docs/solution/pitch.md).
Отдельной публичной лицензии нет: код передаётся организаторам по Положению.
Карта сдачи — заменяемый каталог `assets_dir`, она с train, не из OSM.
`route_10.yaml` — полилиния OSM (ODbL), в эту сдачу не входит.
Сторонние компоненты — [`../NOTICE`](../NOTICE): ROS 2 Humble Apache 2.0, пакет их не копирует. Исследовательское ядро `tramDR-0.0.11`
в эту сдачу не входит.
GNSS читается только в окне старта: 3 с от первого валидного фикса любой
антенны, затем подписка удаляется. Поздний master это начало не переносит. Модель — [`../docs/solution/model.md`](../docs/solution/model.md),
допущения — [`../docs/solution/assumptions.md`](../docs/solution/assumptions.md),
точность — [`../docs/solution/results.md`](../docs/solution/results.md).
Снимок пакета — [`MANIFEST.json`](MANIFEST.json): хеши карты и таблицы, параметры
по умолчанию и опубликованные 1.467 м / 5.828 м. Эти числа являются результатами старого проверочного дерева `6af0037`. Они не являются автоматически результатами дерева, которое клонирует жюри. Фильтр после них менялся, начиная с `2ce42d7`, и в этой сдаче тоже. `published_numbers_recomputed_after_filter_change` равен false: числа заново не считались, повтор `pending`. Хешей записей там нет.

Входы: `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`
(`tram_vehicle_msgs/VelocitySensor`), `/vehicle/driver_position_cmd`
(`DriverControllerCommand`). Если этого файла в пакете сообщений нет, нода собирается без подписки на ручку, и положение ручки остаётся 0. Такая сборка прогоном не является. Прогон rate 1 2026-09-27 шёл с пакетом, где файл есть: `n_pub_cmd` 24019. В записях поле `velocity` тележки ведёт себя как
км/ч; в ноде оно переводится параметром `wheel_unit_scale` (по умолчанию 1/3.6).
README датасета называет это поле м/с; на проверочной записи деление на 3.6
совпадает со скоростью `/localization/kinematic_state`. Штампы тележек и ручки стоят в одной очереди. Водяной знак —
минимум последних штампов живых потоков минус `stamp_reorder_s` (0.10 с).
До этого знака сообщения применяются по порядку штампа. Поток, который отстаёт
больше чем на `order_stall_s` (1 с), из минимума выходит: в диагностике
`order_reason=ORDER_NOT_RESTORED`. Пока отставание не больше 5 с, знак не
уходит дальше чем на 1 с от его последнего штампа. Большее отставание это
ограничение снимает. Штамп позади уже опубликованного выхода считается в
`n_behind_out`. Таймера нет, `/clock` не нужен.
Выход `/result/velocity` — м/с. Отдельного топика тормоза нет и не ожидается.
Торможение — знак `driver_position_cmd.position`: больше нуля — тяга, меньше
нуля — торможение по строке таблицы, ноль — выбег. Пропуск такого топика не
является неисправностью. ZUPT смотрит на скорости тележек, не на тормоз.

## Сборка (без интернета)

Нужны: ROS 2 Humble, пакет сообщений организатора `tram_vehicle_msgs`, этот пакет
и каталог `assets/` внутри него (карта пути, остановки, таблица ручки).
Каталог `assets/` собран офлайн из GNSS train и лежит в пакете: клон репозитория
собирает карту вместе с нодой. Без каталога нода не падает и публикует скорость
и пройденный путь (`y = z = 0`); с ним — точку маршрута 10.

```bash
mkdir -p ~/ws/src && cd ~/ws/src
cp -r <сдача>/railbreak_backup_odometry .
cp -r <датасет>/tram_vehicle_msgs .
# Выданный package.xml без <maintainer> не проходит проверку catkin_pkg в Humble.
grep -q "<maintainer" tram_vehicle_msgs/package.xml || \
  sed -i 's#<license>#<maintainer email="organiser@example.invalid">organiser</maintainer>\n  <license>#' \
  tram_vehicle_msgs/package.xml
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Внешних зависимостей, кроме пакетов Humble (`rclcpp`, `nav_msgs`, `sensor_msgs`,
`diagnostic_msgs`, `ament_index_cpp`, `launch_ros`), нет.

`colcon test --packages-select railbreak_backup_odometry` запускает два теста пакета: `test_gnss_stress` (окно GNSS) и `test_core` (фильтр, FaultScore, целостность). Тот же `test_core` без ROS регистрирует `tools/CMakeLists.txt`.

## Запуск

Из корня клона, если есть Docker Desktop или Docker Engine. Запись и `tram_vehicle_msgs` в репозиторий не входят.

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

Windows: `.\scripts\jury.ps1 play -Bag <каталог> -Msgs <tram_vehicle_msgs>`.
Остальные сценарии и лимиты ТЗ — в [корневом README](../README.md).

Без Docker, Ubuntu 22.04 с Humble. Терминал 1:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py
```

Терминал 2:

```bash
ros2 bag play data/<bag_id>
```

Нода не зависит от `/clock` и работает с `--clock` и без него: время берётся из
`header.stamp` входных сообщений. На прежнем водяном знаке от самого нового
штампа `jury.ps1 record` записи `30618_e9a34502` при `--rate 1` дал 3D RMSE
15.293 м и `n_behind_out` 9969, при `--rate 10` — 2.684 м. Офлайн-двойник того
же рейса — 3D 0.767 м. Этот прогон с водяным знаком по самому медленному потоку
на клоне `0a2253d` переигран: rate 1 дал 3D 14.282 м, `n_behind_out` 68 и код `acceptance` 1; rate 10 дал 3D 14.320 м и `n_behind_out` 0. Выданный checker на этой очереди не запускался.

## Что ожидать

| Топик | Тип | Содержимое |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | `velocity`, продольная скорость, м/с |
| `/result/position` | `nav_msgs/Odometry` | MGRS, точка `base_link` (ось первой тележки, касание колеса и рельса): UTM 37N минус 300000 / 6100000, x восток, y север. Окно — 3 с от первого валидного фикса любой антенны. Дуга — медиана master внутри этого окна, затем +9.873 м вдоль пути и −3 м по высоте. tf: master (−9.873, 0, 3), rover (2.563, 0, 3). Если master пуст, дуга rover отступает на 12.436 м. На шаге нет, если `integrity_use_position` ложен. `mkrs_start` — городская сетка от старта. `twist.twist.linear.x` — скорость, м/с |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, `integrity_status`, `fault_score`, доверие скорости и положения, масштаб колеса, путь s, σ_s, число якорей, состояние GNSS, `order_reason`, `n_behind_out`, максимальное время входа |

`/result/velocity` публикуется как `tram_vehicle_msgs/msg/VelocitySensor`:
`std_msgs/Header header` и `float64 velocity`. Это не `TwistStamped` и не
`Vector3Stamped`. Чекер из `check-code-with-bag.zip` подписывается на этот тип,
читает поле `velocity` и сравнивает его с `twist.twist.linear.x` эталона, м/с.
`header.frame_id` — `base_link`. `header.stamp` — штамп входа, который породил
публикацию, не часы узла и не `/clock`.

Выпущенный вход публикует `/result/velocity` и, когда положение разрешено,
`/result/position`, со штампом этого входа. Штамп позади последнего выхода эти
два топика не публикует. Диагностика пишется каждый 20-й опубликованный выход
и на каждом таком отставшем штампе. Измеренные 36–38 Гц лежат в
[`../docs/solution/results.md`](../docs/solution/results.md) и сняты до текущего
водяного знака. `/result/position` появляется после окна GNSS (3 с от первого
валидного фикса любой антенны); `/result/velocity` — с первого выпущенного
сообщения тележки.

Пропуск обеих тележек ведёт модель по ручке. `/result/position` на этих шагах
выходит, пока `integrity_use_position` истинен. После 3 с согласия обеих тележек
в юзе режим `COMMON_MODE_UNOBSERVABLE`: скорость с колёс в состояние не
копируется. Когда слепой бюджет выбран (5 с или 100 м от последнего якоря),
`integrity_status=LOST` и положение на этом шаге не публикуется. Скорость при
этой потере остаётся, `velocity_confidence=LOW`. `POSITION_UNTRUSTED` тоже
снимает положение: шаг штампа назад, обе тележки старше 30 с или нет абсолютного
старта. Флаг `slip` — последний колбэк; статус `DEGRADED` включает `fault_score`,
не один NIS. Отдельного таймера нет.
Вход — best effort, очередь 500. При `ros2 bag play --rate 10` глубина 10
давала разрывы выхода до 2 с; с очередью 500 на тех же записях разрыв
0.097–0.190 с. В `/result/diagnostics` есть `front_age_s` и `rear_age_s`:
возраст последнего штампа тележки против времени фильтра. У каждого входа
свой `front_kind`, `rear_kind` и `cmd_kind`: `missing`, `stale`, `outlier`,
`impossible`, `disagree` или `ok`. Это не один статус `DEGRADED`.
`quality_score` равен 1 только при `ok` и не является вероятностью. Неверная
единица, которая после масштаба всё ещё меньше 30 м/с, отдельно не распознаётся.
Надёжный подписчик к
best-effort издателю не подключается.

## Логи, метрики, задержка

```bash
ros2 topic type /result/velocity
ros2 interface show tram_vehicle_msgs/msg/VelocitySensor
ros2 topic echo /result/velocity
ros2 topic hz /result/velocity
ros2 topic echo /result/diagnostics --field status[0].values
```

- `callback_max_us` — максимальное время обработки одного входа, мкс.
- `gnss` = `closed` и `gnss_note` — подтверждение, что GNSS больше не читается.
- `slip` = `true` — порог невязки колеса к модели, не подтверждённый юз. Режим `MODEL` — шаг недостоверен,
  положение ещё может выходить. `COMMON_MODE_UNOBSERVABLE` дольше слепого
  бюджета даёт `integrity_status=LOST` и пустой `/result/position` на этом шаге.
- `integrity_use_position`, `velocity_confidence`, `position_confidence`,
  `fault_score`, `time_to_lost`, `order_reason`, `n_behind_out` — в той же диагностике.

Сравнение с GNSS по записи выхода (из корня репозитория):

```bash
ros2 bag record -o out /result/velocity /result/position /result/diagnostics
python3 tools/organizer/score_ros.py out data/<bag_id> --frame mgrs   # исходный bag с master и rover
```

`--frame` должен совпадать с `output_frame` ноды. Эталон скрипта — `base_link`:
точка на доле 9.873/12.436 отрезка master→rover, высота минус 3 м. Сравнение
с самой антенной master даёт около 10 м на ровном месте. Фикс rover берётся,
только если планарная база лежит в пределах 12.436 ± 2 м и разность высот
антенн не больше 1 м.

## Параметры

`config/params.yaml`; полный перечень — `docs/solution/assumptions.md`.
Пересборка не нужна: `ros2 launch ... assets_dir:=/путь/к/assets`.
Оси выхода меняются там же или аргументом запуска
(`ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=grid_start`):
`output_frame` (`mgrs`, `mkrs_start`, `mkrs`, `grid_start`, `enu`), угол
квадрата MGRS `mgrs_square_easting` / `mgrs_square_northing`, сдвиг точки выхода
от master-антенны `output_offset_along_m` / `output_offset_up_m`.

Если каталога нет или `ring.csv` не читается, нода не падает: в лог пишется
`assets not loaded`, GNSS-старт на пустой карте не принимается, и
`/result/position.x` становится пройденным путём от первой входной метки
(y = z = 0). Проверено на Humble: `scripts/check_no_assets.sh`, 25 с
воспроизведения `--rate 10`, процесс жив, x растёт (около 752 м за окно).
