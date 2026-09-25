# railbreak_backup_odometry

Резервная одометрия трамвая по двум тележкам и ручке контроллера, ROS 2 Humble.
Питч и разбор практики — [`../docs/solution/pitch.md`](../docs/solution/pitch.md).
Сторонние компоненты и лицензии — [`../NOTICE`](../NOTICE): код MIT, ROS 2 Humble
Apache 2.0, пакет их не копирует. Исследовательское ядро `tramDR-0.0.11`
в эту сдачу не входит.
GNSS читается только в окне старта (3 с) для начальной выставки, затем подписка
удаляется. Модель — [`../docs/solution/model.md`](../docs/solution/model.md),
допущения — [`../docs/solution/assumptions.md`](../docs/solution/assumptions.md),
точность — [`../docs/solution/results.md`](../docs/solution/results.md).

Входы: `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`
(`tram_vehicle_msgs/VelocitySensor`), `/vehicle/driver_position_cmd`
(`DriverControllerCommand`). В записях поле `velocity` тележки ведёт себя как
км/ч; в ноде оно переводится параметром `wheel_unit_scale` (по умолчанию 1/3.6).
Выход `/result/velocity` — м/с.

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

## Запуск

Терминал 1:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py
```

Терминал 2:

```bash
ros2 bag play data/<bag_id>
```

Нода не зависит от `/clock` и работает с `--clock` и без него, на любой скорости
проигрывания: время берётся из `header.stamp` входных сообщений.

## Что ожидать

| Топик | Тип | Содержимое |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | `velocity`, м/с |
| `/result/position` | `nav_msgs/Odometry` | MGRS, точка `base_link`: UTM 37N минус 400000 / 6100000, x восток, y север. Дуга — медиана master за 3 с, затем +9.873 м вдоль пути и −3 м по высоте. Если master пуст, дуга rover отступает на 12.436 м. `mkrs_start` — городская сетка от старта. `twist.twist.linear.x` — скорость, м/с |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, флаг проскальзывания, масштаб колеса, путь s, σ_s, число якорей, состояние GNSS, максимальное время колбэка |

Каждое сообщение `/result/*` публикуется в колбэке входа и несёт его
`header.stamp`. Частота — на каждый вход: тележки по ~10 Гц и ручка 20 Гц, итого
около 40 Гц. `/result/position` появляется после окна GNSS (3 с от первого
фикса); `/result/velocity` — с первого сообщения тележки.

Положение не замирает при сбое датчика. Пропуск обеих тележек ведёт модель по
ручке, и `/result/position` продолжает выходить на сообщениях ручки. Флаг
проскальзывания и режим — в `/result/diagnostics`, не вместо координат.
Отдельного таймера нет: штамп без входного сообщения судья не с чем сравнить.
Вход — best effort, очередь 500: при `ros2 bag play --rate 10` глубина 10
теряла окно старта и давала разрывы выхода до 2 с. Надёжный подписчик к
best-effort издателю не подключается.

## Логи, метрики, задержка

```bash
ros2 topic hz /result/velocity
ros2 topic echo /result/diagnostics --field status[0].values
```

- `callback_max_us` — максимальное время обработки одного входа, мкс.
- `gnss` = `closed` и `gnss_note` — подтверждение, что GNSS больше не читается.
- `slip` = `true`, режим `MODEL` — тележка отмечена недостоверной.

Сравнение с GNSS по записи выхода (из корня репозитория):

```bash
ros2 bag record -o out /result/velocity /result/position /result/diagnostics
python3 tools/organizer/score_ros.py out data/<bag_id> --frame mgrs   # исходный bag с master и rover
```

`--frame` должен совпадать с `output_frame` ноды. Эталон скрипта — `base_link`:
точка на доле 9.873/12.436 отрезка master→rover, высота минус 3 м. Сравнение
с самой антенной master даёт около 10 м на ровном месте.

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
