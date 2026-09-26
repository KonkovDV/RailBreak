# RailBreak — резервная трамвайная одометрия

Пакет для проверки — [`railbreak_backup_odometry`](railbreak_backup_odometry/).
ROS 2 Humble считает скорость и положение трамвая по двум тележкам и ручке
контроллера. GNSS читается только первые 3 с и нужен для выставки на карту,
дальше подписка снимается.

Запустить можно на Windows, Linux и macOS. Нода живёт в одном контейнере
Humble: Docker Desktop или Docker Engine, сеть хоста не нужна. Запись и пакет
`tram_vehicle_msgs` в репозиторий не входят, карта маршрута уже в пакете.
Без Docker и без ROS собирается только ядро, сценарий `core`.

```text
scripts/jury.sh  <сценарий> --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
scripts/jury.ps1 <сценарий> -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

| Сценарий | Что делает |
|---|---|
| `smoke` | проигрывание обрывается через 25 с. Положение выходит, когда метка сообщения уходит дальше 3 с от первого фикса |
| `play` | вся запись, кадр MGRS, без `/clock` |
| `clock` | `use_sim_time` и `ros2 bag play --clock` |
| `fast` | `--rate 10` |
| `frame` | `output_frame:=mkrs_start` |
| `no-gnss` | в проигрыватель не попадают топики GNSS, выход относительный |
| `no-assets` | каталог карты подменён, нода не падает |
| `arc` | без GNSS, дуга старта `-InitialS` / `--initial-s` (по умолчанию 0) |
| `record` | пишет `/result/*` в `jury_out/result` и вызывает `score_ros.py` |
| `core` | CMake и тест ядра, без Docker, без записи и без сообщений |

Пример на Windows:

```text
.\scripts\jury.ps1 smoke -Bag D:\bags\30618_01f73500 -Msgs D:\dataset\tram_vehicle_msgs
```

Тот же вызов в bash. Первый запуск скачивает образ и собирает пакеты, это несколько минут.
Дальше сборка инкрементальная. `docker compose` из корня репозитория,
файл [`docker-compose.jury.yml`](docker-compose.jury.yml).

Ниже те же сценарии командами Humble на Ubuntu, если Docker нет.

| Что читать | Файл |
|---|---|
| Модель | [`docs/solution/model.md`](docs/solution/model.md) |
| Допущения и параметры | [`docs/solution/assumptions.md`](docs/solution/assumptions.md) |
| Точность и быстродействие | [`docs/solution/results.md`](docs/solution/results.md) |
| Текст полей формы | [`docs/solution/form.md`](docs/solution/form.md) |
| Питч | [`docs/solution/pitch.md`](docs/solution/pitch.md) |

## Контракт

Входы:

| Топик | Тип |
|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix`, только окно старта |

Поле `velocity` тележки в записях — км/ч. Нода переводит его в м/с
(`wheel_unit_scale = 1/3.6`). Выход скорости — м/с.

Выходы, каждый в колбэке входа, со штампом этого входа:

| Топик | Тип | Смысл |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | скорость, м/с; с первого сообщения тележки |
| `/result/position` | `nav_msgs/Odometry` | точка `base_link` в MGRS; после 3 с от первого фикса master |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, флаг проскальзывания, `k`, `s`, `σ_s`, GNSS, `callback_max_us` |

`base_link` — ось вращения первой тележки в точке касания колеса и рельса.
Кадр по умолчанию: UTM 37N минус угол квадрата 300000 м на восток и
6100000 м на север, x — восток, y — север. От антенны master точка сдвинута
на +9.873 м вдоль пути и на −3 м по высоте. tf: master `(−9.873, 0, 3)`,
rover `(2.563, 0, 3)`. Если в окне есть только rover, дуга старта отступает
на 12.436 м к master, и тот же сдвиг снова попадает в `base_link`.

Ориентиры ТЗ, с которыми сверялись прогоны в [`docs/solution/results.md`](docs/solution/results.md):
задержка 100 мс (пик 250 мс), частота не ниже 10 Гц, не больше 2 ядер и 0.5 ГБ.
Отдельного таймера нет: без входного сообщения выход не публикуется.
На номинальном потоке выход идёт на каждый вход тележек (~10 Гц) и ручки (20 Гц).

Вход — best effort, очередь 500. Выход — reliable, очередь 10.
`ros2 topic echo` к выходу подключается. Надёжный подписчик к best-effort
издателю входа не подключается.

## Что должно быть установлено

Универсальный путь: Docker. Образ ставит Humble сам. На Windows и macOS это Docker Desktop,
на Linux — Docker Engine и плагин Compose. Каталог записи содержит `metadata.yaml`.

Путь без Docker — только Ubuntu 22.04 с уже установленным ROS 2 Humble
(`rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`, `ros2 bag`).
`rosdep` для пакета сдачи не нужен.

Сценарий `core` на любой ОС: CMake 3.16+, компилятор C++17. ROS не нужен.

## 1. Сборка на Ubuntu без Docker

Из корня клона. `<датасет>` — каталог выдачи, где лежит `tram_vehicle_msgs`.
Контейнерный путь этот шаг делает сам.

```bash
mkdir -p ~/ws/src
cp -a railbreak_backup_odometry ~/ws/src/
cp -a <датасет>/tram_vehicle_msgs ~/ws/src/
# В выданном package.xml нет <maintainer>, и catkin_pkg на Humble его отвергает.
grep -q "<maintainer" ~/ws/src/tram_vehicle_msgs/package.xml || \
  sed -i 's#<license>#<maintainer email="organiser@example.invalid">organiser</maintainer>\n  <license>#' \
  ~/ws/src/tram_vehicle_msgs/package.xml
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Проверка, что пакет виден:

```bash
ros2 pkg executables railbreak_backup_odometry
```

Ожидаются `backup_odometry_node` и `replay_events`.
Карта ставится в `install/railbreak_backup_odometry/share/railbreak_backup_odometry/assets`
(`ring.csv`, `stops.csv`, `notch.csv`).

Каждый новый терминал начинает с `source ~/ws/install/setup.bash`.

## 2. Обычный прогон записи

Ноду поднять до проигрывателя. Если запись уже идёт, окно GNSS в 3 с будет пропущено,
и положение останется относительным.

Терминал 1:

```bash
source ~/ws/install/setup.bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py
```

В логе строка вида `assets ...: ring ... m, ... stops`. Это карта загружена.

Терминал 2, после этой строки:

```bash
source ~/ws/install/setup.bash
ros2 bag play /путь/к/каталогу_bag --rate 1
```

`/clock` не нужен: интегратор читает `header.stamp` сообщения, не часы ROS.
Для сверки с лимитами ТЗ оставляйте `--rate 1`.

Терминал 3:

```bash
source ~/ws/install/setup.bash
ros2 topic hz /result/velocity
ros2 topic echo /result/position --field pose.pose.position
ros2 topic echo /result/diagnostics --field status[0].values
```

Что считать нормальным:

- `/result/velocity` появляется с первым сообщением тележки;
- `/result/position` — после 3 с от первого фикса master, x порядка 99–103 км (не около нуля);
- в диагностике `gnss` становится `closed`: после окна старта GNSS больше не читается;
- `callback_max_us` — самое долгое время одного колбэка, микросекунды;
- `slip` = `true` и режим `MODEL` — тележка на этом шаге недостоверна, координата всё равно публикуется.

Пропуск обеих тележек не останавливает положение: пока идёт ручка, модель ведёт выход.
Флаг проскальзывания лежит в диагностике, а не вместо координат.

## 3. Прогон с `/clock`

По умолчанию `use_sim_time` выключен, и так и надо, если проигрыватель без `--clock`.

Если нужен симуляционный час:

Терминал 1:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py use_sim_time:=true
```

Терминал 2:

```bash
ros2 bag play /путь/к/каталогу_bag --clock --rate 1
```

Штамп выхода по-прежнему равен штампу входа. Не включайте `use_sim_time` без `--clock`:
узлы, которые ждут `/clock`, не получат время.

## 4. Ускоренный прогон

```bash
ros2 bag play /путь/к/каталогу_bag --rate 10
```

Очередь входа — 500. При `--rate 10` максимальный разрыв выхода на четырёх
записях — 0.097–0.190 с. Прежние 2 с были при глубине 10. Для чисел задержки и частоты
берите сценарий 2.

## 5. Запись выхода и сравнение с антеннами

Терминал 1 — нода, как в сценарии 2. Терминал 2 — запись, её поднять до play:

```bash
source ~/ws/install/setup.bash
ros2 bag record -o /tmp/result_bag /result/velocity /result/position /result/diagnostics
```

Терминал 3:

```bash
ros2 bag play /путь/к/исходному_bag --rate 1
```

Остановить запись `Ctrl+C` после конца play. Сравнение из корня этого репозитория
(нужен Python 3 и `numpy`; читатель bag — стандартный `sqlite3`, ROS для скрипта не нужен):

```bash
python3 -c "import numpy"
python3 tools/organizer/score_ros.py /tmp/result_bag /путь/к/исходному_bag --frame mgrs
```

`--frame` должен совпадать с кадром ноды. Скрипт сравнивает выход с точкой
`base_link` на отрезке master→rover, не с антенной master. Сравнение с самой
антенной даёт около 10 м на ровном месте. Фикс rover входит в эталон, только
если планарная база master–rover лежит в пределах 12.436 ± 2 м и разность
высот антенн не больше 1 м: фикс в десятках метров от вагона — не антенна. В исходном bag нужны оба топика
`/sensing/gnss/master/fix` и `/sensing/gnss/rover/fix`; без rover скрипт завершается с кодом 2.

Таблица по всем выданным записям в [`docs/solution/results.md`](docs/solution/results.md)
посчитана офлайн тем же ядром. Команда `tools/organizer/eval_odometer.py` ждёт
локальное дерево записей и на чистом клоне не запускается.

## 6. Другой кадр выхода

Аргументы `ros2 launch`: `use_sim_time`, `assets_dir`, `output_frame`.
Остальные параметры — в `config/params.yaml` и в значениях по умолчанию ноды.
После правки yaml в исходниках нужна пересборка: launch читает файл из `install`.

Городская сетка от старта, первый отсчёт около нуля:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=mkrs_start
```

Допустимые значения `output_frame`: `mgrs` (по умолчанию), `mkrs_start`, `mkrs`,
`grid_start`, `enu`. Для `enu` и сеток от старта начало — точка карты в момент
выставки. Сдача — `mgrs`.

Свой каталог карты без пересборки:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py assets_dir:=/путь/к/assets
```

В каталоге те же имена: `ring.csv`, `stops.csv`, `notch.csv`, `meta.yaml`.

Параметр, которого нет в launch, передаётся так:

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p output_frame:=mgrs \
  -p initial_s_m:=0.0
```

`ros2 run` берёт значения по умолчанию ноды. Они совпадают с `params.yaml`
для кадра, сдвигов и масштаба колеса. Пустой `assets_dir` означает карту из
share пакета.

## 7. Нет GNSS в начале записи

Если за первые 10 с нет ни одного фикса, выход положения — пройденный путь:
x растёт от первой входной метки, y = z = 0. Скорость публикуется как обычно.

Явная выставка без GNSS в записи:

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p initial_lat_deg:=55.810417 \
  -p initial_lon_deg:=37.462308
```

Либо дуга вдоль карты, в метрах: `-p initial_s_m:=1234.0`.
Пока эти параметры не заданы (NaN), нода их не подставляет сама.

Если master в окне пуст, а rover есть, старт берётся с rover и дуга отступает
на `rover_baseline_m` (12.436). Отдельного флага для этого нет.

## 8. Нет карты

Нода не падает. В логе `assets not loaded`. Скорость есть.
`/result/position.x` — пройденный путь, y = z = 0. GNSS-старт на пустую карту
не принимается.

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p assets_dir:=/nonexistent
```

## 9. Ядро без ROS

Так проверяется то же ядро, без Humble и без записи. Это сценарий `core`:

```text
scripts/jury.sh core
.\scripts\jury.ps1 core
```

Вручную из корня репозитория:

Linux:

```bash
cmake -S railbreak_backup_odometry/tools -B build/rbo -DCMAKE_BUILD_TYPE=Release
cmake --build build/rbo --parallel
ctest --test-dir build/rbo --output-on-failure
```

Windows (Visual Studio, конфигурация задаётся на сборке):

```bash
cmake -S railbreak_backup_odometry/tools -B build/rbo
cmake --build build/rbo --config Release
ctest --test-dir build/rbo -C Release --output-on-failure
```

Офлайн-повтор готового CSV тем же ядром, не замена сценария 2:

```text
replay_events <каталог assets> <events.csv> <out.csv>
```

Колонки `events.csv`: `t,kind,value,s0,sigma_s0`. `kind`: 0 — передняя тележка,
1 — задняя, 2 — ручка. Первая строка несёт начальную дугу `s0`.
Бинарь после сборки пакета: `ros2 run railbreak_backup_odometry replay_events`.

## Что не запускать как сдачу

`tram_dr_localization` и `docker-compose.yml` — прежний синтетический UKF,
версия `tramDR-0.0.11`. В фильтр сдачи он не входит, GNSS, IMU и lidar там
не используются, таблицы seed 42 не являются результатом маршрута 10.
Проверки того ядра — [`docs/verification.md`](docs/verification.md).

## Передача кода

Отдельной публичной лицензии нет: код передаётся организаторам.
Сторонние компоненты — [NOTICE](NOTICE).
