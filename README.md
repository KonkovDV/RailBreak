<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

<p align="center">
  <a href="docs/RailBreak_МТТЕХ_demo.pdf"><img src="https://img.shields.io/badge/%D0%9F%D1%80%D0%B5%D0%B7%D0%B5%D0%BD%D1%82%D0%B0%D1%86%D0%B8%D1%8F-PDF-D32F2F?style=for-the-badge&logo=adobeacrobatreader&logoColor=white" alt="Презентация PDF"></a>
  &nbsp;
  <a href="docs/RailBreak_МТТЕХ_demo.pptx"><img src="https://img.shields.io/badge/%D0%9F%D1%80%D0%B5%D0%B7%D0%B5%D0%BD%D1%82%D0%B0%D1%86%D0%B8%D1%8F-PowerPoint-B7472A?style=for-the-badge&logo=microsoftpowerpoint&logoColor=white" alt="Презентация PowerPoint"></a>
</p>

<p align="center"><a href="https://github.com/KonkovDV/RailBreak/actions/workflows/ci.yml"><img src="https://github.com/KonkovDV/RailBreak/actions/workflows/ci.yml/badge.svg" alt="Тесты CI"></a></p>

# RailBreak: резервная одометрия для автономного трамвая Москвы

Резервный канал скорости и пути для маршрута 10. Пакет [`railbreak_backup_odometry`](railbreak_backup_odometry/) на ROS 2 Humble считает продольную скорость и положение по двум тележкам и ручке контроллера. Первые 3 с GNSS ставят вагон на карту. Дальше подписка master остаётся: каждое окно RTK (разрыв больше 2 с) один раз ставит путь на ось. `gnss_correction: false` возвращает режим без подписок. IMU, лидар и камеры не используются. Это не замена основного стека и не сертифицированный контур остановки.

## Запуск для жюри

Нужен Docker (Docker Desktop на Windows и macOS, Docker Engine с Compose на Linux). Из корня клона:

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

Контейнер собирает пакет вместе с пакетом сообщений организатора и проигрывает запись. Сеть хоста не нужна, первый запуск скачивает образ Humble. Запись и `tram_vehicle_msgs` в репозиторий не входят; карта маршрута уже лежит в пакете.

**Собирать с полным `tram_vehicle_msgs` из датасета.** В архиве `check-code-with-bag.zip` нет `DriverControllerCommand`. Без этого типа нода едет по тележкам с ручкой 0 и между сообщениями тележек повторяет последнюю скорость (`extrapolate_hz`, по умолчанию 20 Гц). Фильтр эти повторы не видит. На всей записи checker без ручки, с экстраполяцией 20 Гц: положение 5.454 м (максимум 22.191 м), скорость 0.044 м/с, 19.4 Гц и 19.5 Гц, штамп назад 0 раз. До экстраполяции на том же контейнере было 5.216 м, 0.055 м/с и 9.3 Гц. С полным пакетом таймера нет: положение 2.15 м, скорость 0.051 м/с, 29.3 Гц. На текущем дереве полный пакет и `--rate 1` дают 1.615 м и 0.035 м/с. Строка 2.151 м остаётся измерением `5cd35fa`. На текущем дереве тот же архив и `metrics.py` при `--rate 1` дают 1.325 м и 0.027 м/с. Строка 5.454 м этой цифрой не заменяется.

Что должно появиться:

- `/result/velocity` — с первого сообщения тележки, м/с;
- `/result/position` — через 3 с после первого фикса GNSS: `frame_id` `map`, `child_frame_id` `base_link`, x около 99–103 км;
- в `ros2 topic echo /result/diagnostics --field status[0].values` поле `gnss` = `correcting`, пока подписка master жива, или `closed`, если `gnss_correction: false`. Значение `open` значит, что стартовое окно ещё не закрыто.

Автоматическая приёмка с кодом возврата:

```text
scripts/jury.sh acceptance --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

Она пишет выход, считает метрики и завершается ненулевым кодом, если нода упала, нет скорости или положения, штамп пошёл назад, частота ниже 10 Гц, разрыв больше 0.25 с, кадры не `map` / `base_link` или GNSS не закрылся.

## Итог проверки

Коммит `5cd35fa`, 2026-09-27, ROS 2 Humble в Docker, `ros2 bag play --rate 1`, полный пакет сообщений организатора. В `1178c67` нода научилась работать без ручки и выдавать положение всегда; фильтр тот же. Одиночный повтор checker на этой ноде: положение 2.111 м (максимум 5.503 м), скорость 0.0505 м/с, штамп назад 0 раз.

| Проверка | Итог |
|---|---|
| Официальный checker, запись `30618_88aea4d9`, положение 3D RMSE | 2.151 м (x 1.530, y 1.474, z 0.336), максимум 5.649 м |
| Официальный checker, скорость RMSE | 0.051 м/с |
| Частота `/result/velocity` и `/result/position` | 29.3 Гц; разрыв штампов 0.051 с, у скорости в первую секунду 0.196 с; штамп назад 0 раз |
| Задержка вход→выход | медиана 53 мс, p95 154 мс, максимум 305 мс |
| Память и CPU | 24.1 МБ RSS, 0.89 % одного ядра |
| `scripts/jury.sh acceptance`, учебная запись `30618_e9a34502` | код 0 |
| Контейнер организатора без `DriverControllerCommand`, с экстраполяцией 20 Гц | положение 5.454 м, скорость 0.044 м/с, 19.4 Гц и 19.5 Гц, штамп назад 0. До экстраполяции: 5.216 м, 0.055 м/с, 9.3 Гц |
| Python-двойник фильтра, val, 21 рейс | медиана ошибки вдоль пути 1.469 м, p95 5.828 м |

Медиана задержки укладывается в 100 мс из ТЗ, хвост выше пика 250 мс у 0.09 % выходов. Методика, сбои и разбор чисел — в [results.md](docs/solution/results.md).

## Документы

| Страница | Что внутри |
|---|---|
| [Пакет](railbreak_backup_odometry/README.md) | входы и выходы, сборка, параметры, диагностика |
| [Модель](docs/solution/model.md) | путь s и скорость v, таблица тяги, масштаб колёс, якоря станций, режимы целостности |
| [Допущения](docs/solution/assumptions.md) | единицы, параметры фильтра, GNSS-старт, карта, что не наблюдается |
| [Результаты](docs/solution/results.md) | официальный checker, точность офлайн, сбои, реальное время, воспроизводимость |
| [Разбор ТЗ](docs/solution/tz-audit.md) | пункт за пунктом, риски и план |
| [После отбора](POSTMORTEM.md) | что измерено на текущем дереве и что в фильтр не вошло |

## О решении

**Проблема.** Нужны скорость и пройденный путь, когда GNSS потерян или ему нельзя верить, а основной стек молчит: без этого не спланировать безопасную остановку. Во входах только две тележки и ручка контроллера. Простое интегрирование колёс не работает: юз, износ и общий масштаб колёс копятся в пути.

**Идея.** Рельс сводит задачу к одной координате — пути s вдоль известной оси маршрута. Фильтр оценивает [s, v, k, bₐ]: путь, скорость, масштаб колёс относительно карты и смещение модели тяги. Координаты x, y, z даёт карта в точке s. Ускорение берётся из таблицы среднего ускорения по ручке и скорости, снятой с поездок train, минус уклон с карты. Две тележки сверяются друг с другом и с моделью. Стоянка у единственной станции в воротах привязывает путь. Режимы целостности говорят, когда оценке уже нельзя верить.

**Ключевое ограничение.** Две тележки не независимы: одна шина, одно сцепление, одна карта. Если обе врут одинаково дольше 3 с, фильтр не принимает их скорость, режим `COMMON_MODE_UNOBSERVABLE`. Доверие возвращает стоянка у станции или снимок master RTK: оба увеличивают счётчик якоря. Без нового якоря через 5 с или 100 м статус `LOST`: положение продолжает выходить, но с ковариацией σ = 1 км и `integrity_use_position=false` — пользоваться им нельзя. Коэффициент сцепления по этим входам не наблюдаем и не оценивается.

**Что не покрыто.** Одно кольцо маршрута 10 без стрелок, депо и заднего хода. Другой вагон и сезон требуют заново снять таблицу и масштаб. Между станциями ошибка пути растёт с масштабом колеса. Граница вдоль пути — эмпирическая, не protection level; SIL не заявляется.

**Опора.** Путь вдоль известной геометрии с позой из карты — Hasberg, Hensel, Stiller (IEEE T-ITS 2012) и von Einem и др. (ITSC 2023); раздувание шума несогласного датчика — Palmer, Nourani-Vatani (IROS 2018). Их оцениватели в ноду не перенесены. Полный список — [references.md](docs/solution/references.md).

**После хакатона.** Сверка с выданным pathgraph, банк гипотез пути для стрелок и депо, таблица и масштаб для другого вагона, теневой прогон рядом с эталонной локализацией вагона. План по пунктам — в [разборе ТЗ](docs/solution/tz-audit.md).

## Сценарии

```text
scripts/jury.sh  <сценарий> --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
scripts/jury.ps1 <сценарий> -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

| Сценарий | Что делает |
|---|---|
| `play` | вся запись, кадр MGRS, без `/clock` |
| `smoke` | первые 25 с записи |
| `acceptance` | запись выхода, метрики, fail-closed проверки (см. выше); код 0 — всё прошло |
| `record` | то же, что `acceptance`, но интерактивно: итог решает код скорера, остальные проверки только предупреждают |
| `clock` | `use_sim_time` и `ros2 bag play --clock` |
| `fast` | `--rate 10`; задержку для ТЗ так не меряют |
| `frame` | `output_frame:=mkrs_start`, сетка от старта |
| `no-gnss` | GNSS не проигрывается, выход — относительный путь |
| `no-assets` | карта подменена пустой: положение — пройденный путь, y = z = 0, нода жива |
| `arc` | без GNSS, старт с дуги `--initial-s` / `-InitialS` |
| `core` | CMake и тест ядра без Docker, записи и сообщений |

Первый запуск собирает пакеты несколько минут, дальше сборка инкрементальная. Контейнер описан в [`docker-compose.jury.yml`](docker-compose.jury.yml).

## Официальный checker и замер задержки

`hackathon_solution_checker` из `check-code-with-bag.zip` собирается в том же рабочем каталоге, что и пакет. `tram_vehicle_msgs` для полного прогона берётся из датасета. В архиве `check-code-with-bag.zip` нет `DriverControllerCommand`; с этим пакетом сообщений нода работает без ручки.

```text
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry hackathon_solution_checker
ros2 run railbreak_backup_odometry backup_odometry_node       # терминал 1
ros2 run hackathon_solution_checker metrics                   # терминал 2, итог после Ctrl+C
ros2 bag play <каталог rosbag2> --rate 1                      # терминал 3
```

Задержка вход→выход: `ros2 bag record` трёх входов и `/result/velocity` при `--rate 1`, затем `python3 tools/organizer/latency_probe.py --bag <запись>`. Скрипт берёт время приёма выхода минус время приёма входа с тем же `header.stamp`. `callback_max_us` в диагностике — время одного колбэка, не эта задержка.

## Контракт

| Вход | Тип |
|---|---|
| `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor`, в записях км/ч; нода делит на 3.6 |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand`, ручка −15…+15 |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix`, окно старта и редкая поправка на оси |

| Выход | Тип | Смысл |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | продольная скорость, м/с; с первого сообщения тележки |
| `/result/position` | `nav_msgs/Odometry` | точка `base_link` в MGRS, скорость в `twist.twist.linear.x`; после окна GNSS — всегда; если целостность запрещает им пользоваться, ковариация σ = 1 км |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, целостность, доверие, GNSS, порядок входов |

`base_link` — ось вращения первой тележки в точке касания колеса и рельса. Кадр по умолчанию — UTM 37N минус угол квадрата 300000 м на восток и 6100000 м на север: x — восток, y — север. От антенны master точка сдвинута на +9.873 м вдоль пути и на −3 м по высоте; tf: master `(−9.873, 0, 3)`, rover `(2.563, 0, 3)`. Если в окне старта есть только rover, дуга отступает на 12.436 м к master.

Торможение — отрицательная ручка, отдельного топика тормоза нет. `header.stamp` выхода — штамп входа, который его породил: часы ноды и `/clock` не используются. При полном пакете сообщений таймера нет. Если типа ручки нет, между тележками выход дополняется экстраполяцией 20 Гц; такой штамп не двигает отметку последнего входа. Три входных потока стоят в одной очереди: вход выпускается, когда все живые потоки дошли до его штампа. Отставший, но живой поток ждут; замолчавший перестают ждать после 50 чужих входов. Вход — best effort, очередь 500; выход — reliable, очередь 10.

Ориентиры ТЗ: задержка 100 мс (пик 250 мс), не ниже 10 Гц, не больше 2 ядер и 0.5 ГБ. Замер — в таблице выше.

## Без Docker: Ubuntu 22.04 и Humble

Разделы 1–9 — те же прогоны командами Humble. Каталог записи содержит `metadata.yaml`.

Нужна Ubuntu 22.04 с уже установленным ROS 2 Humble
(`rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`, `ros2 bag`).
`rosdep` для пакета сдачи не нужен.

Сценарий `core` на любой ОС: CMake 3.16+, компилятор C++17. ROS не нужен.

## 1. Сборка на Ubuntu без Docker

Из корня клона. `<датасет>` — каталог выдачи, где лежит `tram_vehicle_msgs`.
Контейнерный путь этот шаг делает сам: `scripts/jury.sh` перед сборкой вставляет
`<maintainer>`, если в выданном `package.xml` его нет.

```bash
mkdir -p ~/ws/src
cp -a railbreak_backup_odometry ~/ws/src/
cp -a <датасет>/tram_vehicle_msgs ~/ws/src/
# В выданном package.xml нет <maintainer>, и catkin_pkg на Humble его отвергает.
source scripts/ensure_maintainer.sh
ensure_maintainer ~/ws/src/tram_vehicle_msgs/package.xml
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
Для сверки с лимитами ТЗ оставляйте `--rate 1`. Очередь выпускает вход только
до минимума последних штампов тележек и ручки.

Терминал 3:

```bash
source ~/ws/install/setup.bash
ros2 topic type /result/velocity
ros2 interface show tram_vehicle_msgs/msg/VelocitySensor
ros2 topic echo /result/velocity
ros2 topic hz /result/velocity
ros2 topic echo /result/position --field pose.pose.position
ros2 topic echo /result/diagnostics --field status[0].values
```

Что считать нормальным:

- `/result/velocity` появляется с первым выпущенным сообщением тележки;
- `/result/position` — после 3 с от первого валидного фикса любой антенны,
  x порядка 99–103 км, пока `integrity_use_position=true`;
- `gnss` становится `closed`: после окна старта GNSS больше не читается;
- `callback_max_us` — самое долгое время одного входа, микросекунды;
- `slip` — невязка колеса к модели выше порога, не доказанный юз: то же дают задержка привода, уклон, ошибка таблицы тяги, загрузка, сбой штампа или датчика. Статус `DEGRADED` включает сглаженный
  `fault_score`, не один отсчёт;
- режим `MODEL` — этот шаг недостоверен, положение ещё может выходить;
- режим `COMMON_MODE_UNOBSERVABLE` — обе тележки согласны и обе в юзе дольше
  3 с. Две согласные тележки не независимы, поэтому скорость с колёс в состояние не копируется и согласие друг с другом доверие не возвращает. Когда
  выбран слепой бюджет (5 с или 100 м от последнего якоря),
  `integrity_status=LOST`, `integrity_use_position=false`, а `/result/position`
  продолжает выходить с ковариацией σ = 1 км. Скорость остаётся: `velocity_confidence=LOW`. В диагностике `time_to_lost`, `distance_since_last_trusted_anchor` и `common_mode_exit`: `anchor` только после принятого якоря;
- `POSITION_UNTRUSTED` — штамп шагнул назад, обе тележки старше 30 с, или нет
  абсолютного старта. Положение выходит с ковариацией σ = 1 км;
- `notch_input` — `ok`, `no_messages` (ручка не приходила) или `type_absent` (пакет сообщений без `DriverControllerCommand`: работа только по тележкам);
- `order_reason=ORDER_NOT_RESTORED` — один поток отстал больше чем на 1 с. Пока он приходит, выход его ждёт; после 50 чужих входов без него — нет.
  Образец в очереди не выбрасывается;
- `n_behind_out` — сколько штампов пришло уже позади опубликованного выхода.

Пропуск обеих тележек не останавливает модель: пока идёт ручка, выход ведёт таблица тяги.
Положение при этом публикуется всегда; в `LOST` и `POSITION_UNTRUSTED` — с ковариацией σ = 1 км.
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

Очередь входа — 500. На `--rate 10` стена в 10 раз короче штампов, поэтому
задержку для ТЗ снимают в сценарии 2.

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

## Публичный CI

GitHub Actions собирает ядро и ноду против [ci/tram_vehicle_msgs](ci/tram_vehicle_msgs/) — только интерфейса двух сообщений, не пакета организатора — и гоняет юнит-тесты. Записи он не проигрывает. Проверка на записях — в [results.md](docs/solution/results.md).

## Что не запускать как сдачу

`tram_dr_localization` и `docker-compose.yml` — прежний синтетический UKF,
версия `tramDR-0.0.11`. В фильтр сдачи он не входит, GNSS, IMU и lidar там
не используются, таблицы seed 42 не являются результатом маршрута 10.
Проверки того ядра — [`docs/verification.md`](docs/verification.md).

## Передача кода

Отдельной публичной лицензии нет: код передаётся организаторам.
Сторонние компоненты — [NOTICE](NOTICE).
