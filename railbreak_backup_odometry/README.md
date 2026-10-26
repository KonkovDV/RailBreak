<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# railbreak_backup_odometry

Пакет ROS 2 Humble на C++17 — резервная одометрия беспилотного трамвая маршрута № 10 в Москве. По скоростям двух тележек, позиции ручки контроллера и встроенной геодезической карте маршрута (`assets/`) нода публикует продольную скорость (`/result/velocity`), 3D-положение точки `base_link` в квадрате MGRS 37U CB (`/result/position`) и карточку достоверности (`/result/diagnostics`).

Официальный `hackathon_solution_checker`, проверочная запись `30618_88aea4d9`, прогон 2026-10-03, `--rate 1`: RMSE положения **1.157702 м**, RMSE скорости **0.024691 м/с**, медианная задержка вход→публикация **0.20 мс**, **24.1 МБ** RSS. Журнал прогонов — [`docs/solution/results.md`](../docs/solution/results.md).

Сборка и прогон одной командой из корня репозитория (Docker):

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
scripts/jury.sh acceptance --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

## Как работает нода

- **Старт и путевая привязка по GNSS.** Первые 3 секунды (`gnss_init_window_s`) ставят вагон на кольцо маршрута (`assets/ring.csv`, геодезическая длина WGS84 `10 945.6099 м`, `k0 = 1.001378`, `sigma_k0 = 0.0015`): при двух антеннах берётся среднее их дуг (если они ближе 5 м) и направление master→rover. При `gnss_correction: true` (по умолчанию) каждое следующее окно master RTK (`status = 2`, разрыв больше 2 с) один раз корректирует дугу `s` без изменения масштаба `k`, если фикс не дальше `1.5 м` поперёк оси и продольная невязка не больше `max(5 м, 4σ_s)`. `gnss_correction: false` отписывает оба топика GNSS сразу после старта.
- **Фильтр Шмидта — Калмана `[s, v, k, b_a]`.** Между отсчётами тележек скорость ведёт таблица тяги и торможения `a_tab(n, v)` за вычетом уклона `g · i(s)` со смещением `b_a`. На колёсном шаге усиление по масштабу равно нулю (`K_k = 0`), поэтому погрешность таблицы тяги не перетекает в радиус колеса; масштаб `k` уточняется на однозначных стоянках у станций (`assets/stops.csv`).
- **Защита знака ручки (`notch_fault`).** Если обе тележки согласованы, движутся быстрее `1 м/с` и дольше `0.5 с` ускоряются (`a > 0.25 м/с²`) при тормозной ручке (`notch ≤ −4`) или замедляются сильнее `1.0 м/с²` при ходовой (`notch ≥ +4`), модель переходит на выбег (`notch = 0`). Ускорение берётся по согласным парам с шагом `0.25–0.60 с`.
- **Выход скорости и положения.** Пока обе тележки свежие и согласны, `/result/velocity` публикует их среднее на сетке `0.04 с` со сдвигом штампа `+0.105 с`; при расхождении или обрыве выход за `0.5 с` переходит на скорость фильтра. `/result/position` публикуется со штампом тележки (с поправкой дуги `−v · 0.05 с`) и строго между штампами — по сохранённой дуге на сетке `0.04 с` (разрывы длиннее 1 с и скачки дуги больше 8 м не заполняются).

| Компонент | Файл |
|---|---|
| Исполняемый узел ROS 2 | `src/backup_odometry_node.cpp` |
| Фильтр Шмидта — Калмана, `notch_fault`, якоря станций | `include/railbreak_backup_odometry/track_odometer.hpp` |
| Сетка `0.04 с` для скорости и сохранённой дуги | `include/railbreak_backup_odometry/velocity_output.hpp` |
| Водяной знак очереди штампов и контроль входов | `include/railbreak_backup_odometry/input_reorder.hpp`, `source_quality.hpp`, `extrap_stamp.hpp` |
| Стартовое окно GNSS и поправки по окнам master RTK | `include/railbreak_backup_odometry/gnss_window.hpp`, `start_epoch.hpp`, `gnss_correction.hpp` |
| Монитор целостности, граница и карточка `advisory_class` | `include/railbreak_backup_odometry/integrity_monitor.hpp`, `integrity_bound.hpp`, `fault_score.hpp`, `interval.hpp` |
| Теневой банк моделей и прокси сцепления | `include/railbreak_backup_odometry/model_bank.hpp`, `adhesion_proxy.hpp`, `slip_hypothesis.hpp`, `map_match.hpp` |
| Launch и параметры | `launch/backup_odometry.launch.py`, `config/params.yaml` |
| Карта, остановки, таблица тяги, граница | `assets/ring.csv`, `assets/stops.csv`, `assets/notch.csv`, `assets/meta.yaml`, `assets/integrity_bound.json` |
| Тесты | `test/test_gnss_stress.cpp` (исполнитель ROS), `tools/test_core.cpp` (ядро без ROS) |

Модель — [`docs/solution/model.md`](../docs/solution/model.md), параметры — [`docs/solution/assumptions.md`](../docs/solution/assumptions.md), метрики — [`docs/solution/results.md`](../docs/solution/results.md).

## Входы и выходы

| Топик | Тип | Назначение |
|---|---|---|
| `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | Скорости тележек (в записях — км/ч; нода умножает на `wheel_unit_scale = 1/3.6`) |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | Позиция ручки `−15..+15`: `> 0` — тяга, `< 0` — торможение, `0` — выбег |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | Окно старта `3 с`; далее редкие пачки master RTK на оси пути (rover после старта путь не правит) |
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | Продольная скорость вагона, м/с |
| `/result/position` | `nav_msgs/Odometry` | 3D-поза `base_link` в MGRS 37U CB (`frame_id = map`, `child_frame_id = base_link`, `twist.twist.linear.x = v`) |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Режим, целостность, доверие скорости и положения, `advisory_class`, `notch_fault`, GNSS, очередь штампов |

Пакет собирается с любым вариантом `tram_vehicle_msgs`. Наличие `DriverControllerCommand` определяется в `CMakeLists.txt` по реальному таргету typesupport (`RAILBREAK_HAS_DRIVER_CMD`). Без этого типа (так устроен архив `check-code-with-bag.zip`) узел работает по двум тележкам при нулевой ручке, выставляет `notch_input = type_absent` и между тележками повторяет последнюю скорость с частотой `extrapolate_hz` (20 Гц). С полным пакетом сообщений этот таймер не создаётся.

`base_link` — проекция оси вращения первой тележки на уровень головки рельса. Кадр по умолчанию (`mgrs`) — UTM 37N за вычетом юго-западного угла квадрата (`300000 м` E, `6100000 м` N): `x` — восток (на маршруте `99–103 км`), `y` — север, `z` — эллипсоидальная высота WGS84. Смещение от антенны master: `+9.873 м` вдоль пути и `−3.0 м` по высоте (штатный `tf`: master `(−9.873, 0, 3)`, rover `(2.563, 0, 3)`). В кадрах `mgrs` и `mkrs` с загруженной картой `/result/position` публикуется после абсолютного старта.

## Сборка без интернета

Нужны ROS 2 Humble и пакет `tram_vehicle_msgs`. Карта маршрута уже в `assets/`.

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

`ensure_maintainer` добавляет тег `<maintainer>` в `tram_vehicle_msgs/package.xml`, без которого `colcon` в Humble отклоняет пакет (`scripts/jury.sh` и `scripts/jury.ps1` делают это сами). Зависимости — только стандартные пакеты Humble: `rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`.

Тесты: `colcon test --packages-select railbreak_backup_odometry` запускает `test_gnss_stress` (окно GNSS через исполнитель ROS) и `test_core` (фильтр, очередь, сетка, карта, целостность).

## Запуск

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>     # Docker
ros2 launch railbreak_backup_odometry backup_odometry.launch.py              # нативно, терминал 1
ros2 bag play <каталог rosbag2> --rate 1                                    # нативно, терминал 2
```

Ноду запускайте до `ros2 bag play`. Если стартовое окно GNSS всё же пропущено, нода идёт в относительном режиме и захватывает абсолютную дугу на первой пачке master RTK на оси. Остальные сценарии и официальный checker — в [корневом README](../README.md).

## Диагностика

```bash
ros2 topic echo /result/diagnostics --field status[0].values
```

| Поле | Назначение |
|---|---|
| `mode` | `WHEELS`, `MODEL` (шаг ведёт модель), `ZUPT` (стоянка), `COMMON_MODE_UNOBSERVABLE`, `FREEZE` (численный откат шага) |
| `integrity_status` | `NOMINAL`, вид деградации, `POSITION_UNTRUSTED` или `LOST` |
| `velocity_confidence`, `position_confidence` | Раздельное доверие к скорости и положению: `HIGH`, `LOW`, `NONE` |
| `slip`, `slip_front`, `slip_rear` | Невязка тележки к модели выше `4σ` (`wheel_model_residual`) |
| `advisory_class` | Карточка состояния: `input_quality`, `common_mode`, `one_bogie_residual`, `both_residual`, `nominal` |
| `notch_fault` | Ручка противоречит согласованному разгону или торможению тележек; модель на выбеге |
| `time_to_lost`, `distance_since_last_trusted_anchor`, `common_mode_exit` | Остаток слепого бюджета в общей моде; `common_mode_exit = anchor` — режим снят подтверждённой станцией |
| `n_anchor`, `sigma_s` | Число принятых якорей станций и текущая СКО дуги |
| `order_reason`, `n_behind_out` | Состояние водяного знака очереди и число штампов, пришедших позади опубликованного выхода |
| `notch_input` | `ok`, `no_messages` или `type_absent` |
| `callback_max_us` | Максимальное время одного колбэка, мкс |

## Поведение при сбоях

| Ситуация | Реакция ноды |
|---|---|
| Одна тележка проскальзывает или замолкает | Её дисперсия раздувается до `R = 25 (м/с)²`; скорость и дугу ведут вторая тележка и модель тяги |
| Обе тележки замолкают | До 30 с скорость ведёт модель по ручке и уклону; дальше `velocity_confidence = NONE`, публикация скорости приостанавливается до возврата тележек |
| Ручка противоречит согласованным тележкам | Через `0.5 с` `notch_fault = true`, прогноз переходит на выбег |
| Обе тележки согласны, но расходятся с моделью дольше 3 с | `COMMON_MODE_UNOBSERVABLE`: `k` и `b_a` заморожены до станции или пачки RTK; через 5 с или 100 м без якоря — `LOST`, положение выходит с `σ = 1 км` и `integrity_use_position = false` |
| Разрыв штампов больше 30 с или шаг штампа назад | Интервал не интегрируется; при шаге назад или устаревании обеих тележек — `POSITION_UNTRUSTED` (`σ = 1 км`) |
| Нет типа или потока `DriverControllerCommand` | Работа по двум тележкам при `notch = 0`, `notch_input = type_absent` или `no_messages` |
| Нет фикса GNSS за 10 с или нет каталога `assets/` | Относительная одометрия (`x` — пройденный путь, `y = z = 0`) |

Сообщения обеих тележек проходят через общую очередь с водяным знаком по минимальному последнему штампу живых потоков (в начале `30618_88aea4d9` задняя тележка отстаёт на 1.3 с и догоняет очередь). Поток, отставший больше чем на `1 с`, отмечается `ORDER_NOT_RESTORED`; замолчавший поток перестаёт удерживать очередь после 50 чужих сообщений.

## Основные параметры (`config/params.yaml`)

Полная таблица — [`docs/solution/assumptions.md`](../docs/solution/assumptions.md). Любой параметр переопределяется при запуске:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py assets_dir:=/путь/к/assets
ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=mkrs_start
```

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `output_frame` | `mgrs` | Кадр `/result/position`: `mgrs`, `mkrs_start`, `mkrs`, `grid_start`, `enu` |
| `gnss_init_window_s`, `gnss_wait_s` | `3 с`, `10 с` | Стартовое окно GNSS и таймаут перехода на относительный путь |
| `gnss_correction` | `true` | Редкие поправки дуги по пачкам master RTK на оси (`false` отписывает GNSS после старта) |
| `gnss_correction_min_s`, `gnss_correction_min_m` | `30 с`, `0 м` | Минимальный интервал между поправками |
| `gnss_correction_cross_m`, `gnss_correction_gate_m` | `1.5 м`, `5 м` | Поперечный допуск к оси и продольный порог `max(5 м, 4σ_s)` |
| `output_stamp_offset_s` | `0.105` | Сдвиг `header.stamp` для `/result/velocity` |
| `output_pos_lag_s` | `0.05` | Поправка публикуемой дуги `−v · τ` |
| `output_velocity_source`, `output_velocity_fade_s` | `wheels_mean`, `0.5` | Среднее согласных тележек с плавным переходом на фильтр (`filter` — всегда фильтр) |
| `output_velocity_resample_s` | `0.04` | Шаг сетки скорости между парами тележек (`0` — одно сообщение на пару) |
| `output_position_resample_s` | `0.04` | Шаг сетки сохранённой дуги между штампами тележек (`0` — одна поза на тележку) |
| `gnss_init_dual_antenna` | `true` | Усреднение стартовой дуги master и rover (база `12.436 м`), если они ближе `5 м` |
| `wheel_unit_scale` | `1 / 3.6` | Перевод скорости тележек из км/ч в м/с |
| `initial_s_m`, `initial_lat_deg`, `initial_lon_deg` | не заданы | Ручной старт без GNSS |
| `load_factor`, `wheel_radius_m`, `davis_*` | `1`, `0`, `0` | Множитель загрузки, паспортный радиус и коэффициенты Дэвиса (сопротивление уже в `notch.csv`) |
| `extrapolate_hz` | `20` | Частота повтора скорости при сборке без `DriverControllerCommand` (`0` выключает) |

## Оценка на учебных записях

В учебных записях нет `/localization/kinematic_state`, поэтому `tools/organizer/score_ros.py` восстанавливает `base_link` по отрезку антенн master→rover (доля `9.873 / 12.436`, база `12.436 ± 2 м`, разность высот `≤ 1 м`):

```bash
ros2 bag record -o out /result/velocity /result/position /result/diagnostics
python3 tools/organizer/score_ros.py out <исходный bag> --frame mgrs
```

## Состав пакета и права

Контрольные суммы карты и таблицы тяги — в [`MANIFEST.json`](MANIFEST.json). Карта построена по GNSS обучающих проездов и полилиниям `pathgraph` организаторов и заменяется параметром `assets_dir` без пересборки. Код передаётся организаторам согласно Положению хакатона; сторонние компоненты — в [`NOTICE`](../NOTICE).
