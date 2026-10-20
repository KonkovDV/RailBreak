<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# railbreak_backup_odometry

Автономный пакет ROS 2 Humble на C++17 для резервной одометрии беспилотного трамвая маршрута № 10 в Москве. Вычисляет продольную скорость (`/result/velocity`), 3D-положение точки `base_link` в квадрате MGRS 37U CB (`/result/position`) и карточку достоверности (`/result/diagnostics`) по двум скоростям тележек, позиции ручки контроллера водителя и встроенной геодезической карте маршрута (`assets/`).

Сборка и прогон одной командой из корня репозитория (Docker):

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
scripts/jury.sh acceptance --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

На проверочной записи `30618_88aea4d9` официальный `hackathon_solution_checker` (прогон 3 октября 2026, `--rate 1`) показал RMSE положения **1.157702 м** и RMSE скорости **0.024691 м/с** (на момент сдачи `5cd35fa` — `2.151 м` и `0.051 м/с`). Полный журнал прогонов приведён в [`docs/solution/results.md`](../docs/solution/results.md).

## Как работает нода

- **Старт и редкая путевая привязка по GNSS.** Первые 3 секунды (`gnss_init_window_s`) ставят вагон на кольцо маршрута (`assets/ring.csv`, геодезическая длина WGS84 `10 945.6099 м`, `k0 = 1.001378`, `sigma_k0 = 0.0015`): при наличии двух антенн берётся среднее их дуг (если расхождение меньше 5 м) и направление master→rover. При `gnss_correction: true` (по умолчанию) каждое последующее окно master RTK (`status = 2`, разрыв больше 2 с) один раз корректирует дугу `s` без изменения масштаба `k`, если фикс не дальше `1.5 м` поперёк оси и продольная невязка не больше `max(5 м, 4σ_s)`. `gnss_correction: false` отписывает оба топика GNSS сразу после старта.
- **Фильтр Шмидта — Калмана `[s, v, k, b_a]` и защита знака ручки (`notch_fault`).** Между отсчётами тележек скорость ведёт таблица тяги и торможения `a_tab(n, v)` за вычетом уклона `g · i(s)` и со смещением `b_a`. На колёсном шаге коэффициент Калмана по масштабу равен нулю (`K_k = 0`), чтобы погрешности таблицы тяги не перетекали в радиус колеса; масштаб `k` уточняется только на однозначных остановках (`assets/stops.csv`). Если обе тележки согласованы, движутся быстрее `1 м/с` и дольше `0.5 с` ускоряются (`a > 0.25 м/с²`) при тормозной ручке (`notch <= -4`) или замедляются сильнее `1.0 м/с²` при ходовой (`notch >= +4`), выставляется `notch_fault = true` и модель переходит на выбег (`notch = 0`). Ускорение берётся по парам с шагом `0.25–0.60 с`. Уже защёлкнутый `COMMON_MODE_UNOBSERVABLE` это правило не снимает.
- **Выход скорости и положения.** Пока обе тележки свежие и согласны, в `/result/velocity` публикуется их среднее на сетке `0.04 с` со сдвигом штампа `+0.105 с`; при расхождении или обрыве выход плавно (за `0.5 с`) переходит на скорость фильтра. Положение `/result/position` публикуется со штампом тележки (с упреждением дуги `-v · 0.05 с`) и на промежуточной сетке `0.04 с` по сохранённой дуге строго между штампами тележек (разрывы длиннее 1 с и скачки дуги больше 8 м не заполняются). Сообщения ручки контроллера собственную позу не публикуют.

| Файл или компонент | Фактическое имя файла в репозитории |
|---|---|
| Исполняемый узел ROS 2 | `src/backup_odometry_node.cpp` |
| Фильтр Шмидта — Калмана и защита знака ручки (`notch_fault`) | `include/railbreak_backup_odometry/track_odometer.hpp` |
| Сетка `0.04 с` для скорости и сохранённой дуги положения | `include/railbreak_backup_odometry/velocity_output.hpp` |
| Водяной знак очереди штампов и контроль входов | `include/railbreak_backup_odometry/input_reorder.hpp`, `source_quality.hpp`, `extrap_stamp.hpp` |
| Стартовое окно GNSS и поправки по окнам master RTK | `include/railbreak_backup_odometry/gnss_window.hpp`, `start_epoch.hpp`, `gnss_correction.hpp` |
| Монитор целостности, граница и карточка `advisory_class` | `include/railbreak_backup_odometry/integrity_monitor.hpp`, `integrity_bound.hpp`, `fault_score.hpp`, `interval.hpp` |
| Диагностический банк моделей и прокси сцепления | `include/railbreak_backup_odometry/model_bank.hpp`, `adhesion_proxy.hpp`, `slip_hypothesis.hpp`, `map_match.hpp` |
| Launch-файл и параметры | `launch/backup_odometry.launch.py`, `config/params.yaml` |
| Карта маршрута, остановки, таблица тяги и граница | `assets/ring.csv`, `assets/stops.csv`, `assets/notch.csv`, `assets/meta.yaml`, `assets/integrity_bound.json` |
| Автотесты | `test/test_gnss_stress.cpp` (ROS), `tools/test_core.cpp` (ядро C++17 без ROS) |

Подробное описание модели — в [`docs/solution/model.md`](../docs/solution/model.md), допущения и параметры — в [`docs/solution/assumptions.md`](../docs/solution/assumptions.md), метрики — в [`docs/solution/results.md`](../docs/solution/results.md).

## Входы и выходы

| Топик | Тип | Назначение |
|---|---|---|
| `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | Скорости тележек (в записях датасета — км/ч; нода умножает на `wheel_unit_scale = 1/3.6`) |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | Позиция ручки контроллера `-15..+15`: `> 0` — тяга, `< 0` — торможение, `0` — выбег |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | Окно старта `3 с`; далее редкие пачки master RTK на оси пути (`rover` после старта путь не правит) |
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | Продольная скорость вагона, м/с |
| `/result/position` | `nav_msgs/Odometry` | 3D-поза `base_link` в MGRS 37U CB (`frame_id = map`, `child_frame_id = base_link`, `twist.twist.linear.x = v`) |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Режим, целостность, доверие скорости/положения, `advisory_class`, `notch_fault`, GNSS, очередь штампов |

Если в окружении собран урезанный `tram_vehicle_msgs` без `DriverControllerCommand.msg` (так устроен архив `check-code-with-bag.zip`), `CMakeLists.txt` автоматически собирает ноду с `RAILBREAK_HAS_DRIVER_CMD = 0`: узел работает по двум тележкам при нулевой ручке, пишет предупреждение в лог, выставляет `notch_input = type_absent` и между тележками повторяет последнюю скорость с частотой `extrapolate_hz` (по умолчанию 20 Гц). С полным пакетом сообщений из датасета этот таймер не создаётся.

Точка `base_link` — проекция оси вращения первой тележки на уровень головки рельса. Кадр по умолчанию (`mgrs`) — UTM 37N за вычетом юго-западного угла квадрата (`300000 м` на восток, `6100000 м` на север): `x` — восток (на маршруте `99–103 км`), `y` — север, `z` — эллипсоидальная высота WGS84. Смещение от антенны master составляет `+9.873 м` вдоль пути и `-3.0 м` по высоте (согласно штатному `tf`: master `(-9.873, 0, 3)`, rover `(2.563, 0, 3)`). В кадрах `mgrs` и `mkrs` при загруженной карте топик `/result/position` начинает публиковаться только после получения абсолютного старта.

## Сборка без интернета

Требуются ROS 2 Humble и пакет `tram_vehicle_msgs`. Карта маршрута уже включена в `assets/`. Из корня репозитория:

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

Функция `ensure_maintainer` добавляет тег `<maintainer>` в `tram_vehicle_msgs/package.xml`, без которого `colcon` в Humble отклоняет пакет (сценарии `scripts/jury.sh` и `scripts/jury.ps1` делают это автоматически).

Зависимости — только стандартные пакеты ROS 2 Humble: `rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`.

Запуск автотестов: `colcon test --packages-select railbreak_backup_odometry` выполняет `test_gnss_stress` (стресс-тест окна GNSS через исполнитель ROS) и `test_core` (модульные тесты фильтра, очереди, сетки и целостности).

## Запуск

В Docker из корня репозитория (Windows, Linux, macOS):

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

Нативно на Ubuntu 22.04 + ROS 2 Humble (ноду запускать **до** `ros2 bag play`, чтобы не пропустить стартовое окно GNSS):

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py      # терминал 1
ros2 bag play <каталог rosbag2> --rate 1                            # терминал 2
```

Остальные сценарии приёмки и запуск официального checker приведены в [корневом README](../README.md).

## Контроль выходных топиков и диагностики

```bash
ros2 topic echo /result/velocity
ros2 topic hz /result/velocity
ros2 topic echo /result/position --field pose.pose.position
ros2 topic echo /result/diagnostics --field status[0].values
```

Ключевые поля `/result/diagnostics`:

| Поле | Назначение |
|---|---|
| `mode` | `WHEELS`, `MODEL` (шаг ведёт модель), `ZUPT` (стоянка), `COMMON_MODE_UNOBSERVABLE`, `FREEZE` (численный откат шага) |
| `integrity_status` | `NOMINAL`, вид деградации, `POSITION_UNTRUSTED` или `LOST` |
| `velocity_confidence`, `position_confidence` | Раздельная оценка доверия для скорости и положения: `HIGH`, `LOW` или `NONE` |
| `slip`, `slip_front`, `slip_rear` | Невязка тележки к модели выше порога `4σ` (прокси расхождения `wheel_model_residual`, не прямое измерение коэффициента сцепления) |
| `advisory_class` | Объяснимая карточка состояния: `input_quality`, `common_mode`, `one_bogie_residual`, `both_residual`, `nominal` |
| `notch_fault` | `true`, если обе тележки согласованы, быстрее `1 м/с` и более `0.5 с` ускоряются (`a > 0.25 м/с²`) при тормозной ручке (`<= -4`) или замедляются сильнее `1.0 м/с²` при ходовой (`>= +4`); модель при этом переводится на выбег (`notch = 0`) |
| `time_to_lost`, `distance_since_last_trusted_anchor`, `common_mode_exit` | Остаток слепого бюджета в общей моде; `common_mode_exit = anchor` выставляется только после подтверждённой станции |
| `n_anchor`, `sigma_s` | Число принятых станционных якорей и текущая оценка СКО продольной дуги |
| `order_reason`, `n_behind_out` | Состояние водяного знака очереди входов и число штампов, пришедших позади уже опубликованного выхода |
| `notch_input` | `ok`, `no_messages` (сообщения ручки не приходили) или `type_absent` (сборка без `DriverControllerCommand`) |
| `callback_max_us` | Максимальное время исполнения одного колбэка в микросекундах (не равно полной задержке вход→выход) |

## Поведение при сбоях и аномалиях

| Ситуация | Реакция ноды |
|---|---|
| Одна тележка проскальзывает или замолкает | Её дисперсия измерения раздувается до `R = 25 (м/с)²`; скорость и дугу удерживают вторая тележка и модель тяги |
| Обе тележки замолкают | Прогноз скорости до 30 с ведёт модель по ручке и уклону; через 30 с `velocity_confidence = NONE` и `/result/velocity` прекращает публикацию до возврата тележек |
| Ручка противоречит разгону/торможению согласных тележек | Через `0.5 с` выставляется `notch_fault = true`, прогноз переходит на выбег (`notch = 0`), не сбрасывая `COMMON_MODE_UNOBSERVABLE` |
| Обе тележки согласны между собой, но расходятся с моделью дольше 3 с | Переход в `COMMON_MODE_UNOBSERVABLE`: подстройка `k` и `b_a` замораживается до станции или RTK-пачки. Через 5 с или 100 м без якоря — статус `LOST` (`/result/position` продолжает выходить с `σ = 1 км` и `integrity_use_position = false`) |
| Разрыв штампов больше 30 с или шаг штампа назад | Интервал не интегрируется; при шаге назад или устаревании обеих тележек выставляется `POSITION_UNTRUSTED` (`σ = 1 км`) |
| Отсутствует тип или поток `DriverControllerCommand` | Работа по двум тележкам при `notch = 0`, `notch_input = type_absent` или `no_messages` |
| Нет фикса GNSS за 10 с или отсутствует папка `assets/` | Нода не падает и переходит в относительную одометрию (`x` — пройденный путь, `y = z = 0`) |

Порядок обработки потоков: сообщения обеих тележек и ручки поступают в общую очередь с водяным знаком по минимальному последнему штампу живых потоков (в начале записи `30618_88aea4d9` задняя тележка отстаёт на `1.3 с` и аккуратно догоняет очередь). Поток, отставший более чем на `1 с`, отмечается `ORDER_NOT_RESTORED`; замолчавший поток перестаёт удерживать очередь после 50 чужих сообщений.

## Основные параметры (`config/params.yaml`)

Полное описание всех параметров приведено в [`docs/solution/assumptions.md`](../docs/solution/assumptions.md). Любой параметр можно переопределить при запуске:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py assets_dir:=/путь/к/assets
ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=mkrs_start
```

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `output_frame` | `mgrs` | Кадр координат `/result/position`: `mgrs`, `mkrs_start`, `mkrs`, `grid_start`, `enu` |
| `gnss_init_window_s`, `gnss_wait_s` | `3 с`, `10 с` | Длительность стартового окна GNSS и таймаут перехода на относительный путь |
| `gnss_correction` | `true` | Разрешить редкие поправки дуги `s` по пачкам master RTK на оси пути (`false` отписывает GNSS после старта) |
| `gnss_correction_min_s`, `gnss_correction_min_m` | `30 с`, `0 м` | Минимальный интервал между поправками после предыдущего абсолютного обновления |
| `gnss_correction_cross_m`, `gnss_correction_gate_m` | `1.5 м`, `5 м` | Поперечный допуск к оси и минимальный продольный порог `max(5 м, 4σ_s)` |
| `output_stamp_offset_s` | `0.105` | Сдвиг `header.stamp` для `/result/velocity` (штамп положения не сдвигается) |
| `output_pos_lag_s` | `0.05` | Поправка публикуемой дуги `-v · τ` под фазу эталона |
| `output_velocity_source`, `output_velocity_fade_s` | `wheels_mean`, `0.5` | Публикация среднего согласных тележек с плавным переходом на фильтр (`filter` — всегда фильтр) |
| `output_velocity_resample_s` | `0.04` | Шаг сетки интерполяции скорости между парами тележек (`0` — одно сообщение на пару) |
| `output_position_resample_s` | `0.04` | Шаг сетки выдачи сохранённой дуги между штампами тележек (`0` — одна поза на тележку) |
| `gnss_init_dual_antenna` | `true` | Усреднение стартовой дуги master и rover (с учётом базы `12.436 м`), если они ближе `5 м` |
| `wheel_unit_scale` | `1 / 3.6` | Перевод скорости тележек из км/ч в м/с |
| `initial_s_m`, `initial_lat_deg`, `initial_lon_deg` | не заданы | Ручной старт без GNSS |
| `load_factor`, `wheel_radius_m`, `davis_*` | `1`, `0`, `0` | Множитель загрузки, паспортный радиус и коэффициенты Дэвиса (по умолчанию учтены в таблице `notch.csv`) |
| `extrapolate_hz` | `20` | Частота повтора скорости между тележками при сборке без `DriverControllerCommand` (`0` выключает) |

## Оценка на учебных записях без эталона судьи

На учебных бэгах нет топика `/localization/kinematic_state`, поэтому для них используется скрипт `tools/organizer/score_ros.py`, восстанавливающий точку `base_link` по отрезку антенн master→rover (доля `9.873 / 12.436` при контроле базы `12.436 ± 2 м` и разности высот `≤ 1 м`):

```bash
ros2 bag record -o out /result/velocity /result/position /result/diagnostics
python3 tools/organizer/score_ros.py out <исходный bag> --frame mgrs
```

## Состав пакета и права

Контрольные суммы встроенной карты и таблицы тяги зафиксированы в [`MANIFEST.json`](MANIFEST.json). Карта построена по GNSS обучающих проездов и полилиниям `pathgraph` организаторов (не из OpenStreetMap) и может быть заменена через параметр `assets_dir`. Код передаётся организаторам согласно Положению хакатона; уведомления о сторонних компонентах — в [`NOTICE`](../NOTICE). Исследовательский пакет `tramDR-0.0.11` в `railbreak_backup_odometry` не входит.
