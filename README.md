<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

<p align="center">
  <a href="docs/RailBreak_МТТЕХ_demo.pdf"><img src="https://img.shields.io/badge/%D0%9F%D1%80%D0%B5%D0%B7%D0%B5%D0%BD%D1%82%D0%B0%D1%86%D0%B8%D1%8F-PDF-D32F2F?style=for-the-badge&logo=adobeacrobatreader&logoColor=white" alt="Презентация PDF"></a>
  &nbsp;
  <a href="docs/RailBreak_МТТЕХ_demo.pptx"><img src="https://img.shields.io/badge/%D0%9F%D1%80%D0%B5%D0%B7%D0%B5%D0%BD%D1%82%D0%B0%D1%86%D0%B8%D1%8F-PowerPoint-B7472A?style=for-the-badge&logo=microsoftpowerpoint&logoColor=white" alt="Презентация PowerPoint"></a>
</p>

<p align="center"><a href="https://github.com/KonkovDV/RailBreak/actions/workflows/ci.yml"><img src="https://github.com/KonkovDV/RailBreak/actions/workflows/ci.yml/badge.svg" alt="CI"></a></p>

# RailBreak

Backup speed and distance for Moscow tram route 10. The submitted package is [`railbreak_backup_odometry`](railbreak_backup_odometry/) on ROS 2 Humble, C++17. Inputs are the two bogie speeds and the driver notch. GNSS places the car in the first 3 s. Rare master-RTK windows stay on by default (`gnss_correction: true`). Published speed is the bogie mean while the bogies agree, resampled every 0.04 s, with the header shifted by +0.105 s. The filter speed is used when the bogies disagree or drop out. Position is the map point at arc `s`, and its header is the bogie stamp. A driver command updates the notch and does not publish a pose under the command header. IMU, lidar and cameras are not used. The research core `tramDR-0.0.11` is not part of this package.

## Run

Docker is required. From the clone root:

```text
scripts/jury.sh play --bag <rosbag2 directory> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <rosbag2 directory> -Msgs <tram_vehicle_msgs>
```

Build against the full `tram_vehicle_msgs` from the dataset. `check-code-with-bag.zip` has no `DriverControllerCommand`. Without that type the node runs on the bogies with notch 0. Bags are not in the repository. The route map is.

## What was measured

Bag `30618_88aea4d9` unless a row says otherwise. Official checker is `hackathon_solution_checker` / `metrics.py` (queue 100, slop 0.05 s). A later row does not replace an earlier one. The submission row is commit `5cd35fa`, 2026-09-27. The node speed on the current tree, 0.024691 m/s, remains above the 0.024 m/s line.

| Run | Rate | 3D, m | max, m | pairs | x | y | z | speed, m/s | max | pairs |
|---|---|---|---|---|---|---|---|---|---|---|
| `5ed4a7c` | — | 6.174 | 19.309 | — | — | — | — | 0.067 | — | — |
| `5cd35fa`, submission, full command | 1 | 2.151 | 5.649 | 50168 | 1.530 | 1.474 | 0.336 | 0.051 | 0.337 | 50481 |
| `1178c67`, full command, one container | 1 | 2.111 | 5.503 | 50124 | 1.481 | 1.467 | 0.337 | 0.0505 | 0.337 | 50508 |
| `e863195`, no command type, 20 Hz hold | 1 | 5.454 | 22.191 | 25488 | 5.110 | 1.872 | 0.366 | 0.044 | 0.307 | 25511 |
| Later tree, no command type | 1 | 1.325 | 4.806 | 25406 | 0.974 | 0.834 | 0.336 | 0.027 | 0.341 | 25429 |
| Later tree, no command type | 10 | 1.315 | — | 12180 | — | — | — | 0.027 | — | — |
| Full command, before the +0.10 s speed stamp | 10 | 1.622 | 4.913 | 49623 | 1.234 | 1.005 | 0.316 | 0.035 | 0.340 | 49807 |
| Full command, before the +0.10 s speed stamp | 1 | 1.615 | 4.899 | 49768 | 1.231 | 0.996 | 0.319 | 0.035 | 0.341 | 50006 |
| Speed stamp +0.10 s, GNSS snaps on | 10 | 1.624 | 4.909 | 49776 | — | — | — | 0.034 | 0.325 | 41383 |
| GNSS snaps off | 10 | 2.093 | 6.467 | 49737 | — | — | — | 0.030 | — | 41364 |
| Wheel-mean speed, before the geodesic arc | 10 | 1.403506 | 4.940951 | 49643 | 0.964917 | 0.963472 | 0.332393 | 0.026695 | 0.289518 | 12117 |
| Wheel-mean speed, before the geodesic arc | 1 | 1.372506 | 4.922128 | 49956 | 0.934009 | 0.949411 | 0.331689 | 0.026633 | 0.289518 | 12205 |
| Geodesic arc, before the seam chord | 10 | 1.164891 | 3.692287 | 49972 | 0.889213 | 0.724400 | 0.203755 | 0.024677 | 0.289518 | 38302 |
| Geodesic arc, before the seam chord | 1 | 1.163029 | 3.682215 | 49940 | 0.888103 | 0.722818 | 0.203576 | 0.024695 | 0.289518 | 38388 |
| After the seam chord | 10 | 1.165573 | 3.692415 | 49914 | 0.889815 | 0.724788 | 0.203648 | 0.024665 | 0.289518 | 38294 |
| `9a954fd`, after the seam chord | 1 | 1.164429 | 3.684696 | 49943 | 0.889361 | 0.723523 | 0.203581 | 0.024692 | 0.289518 | 38392 |
| 2026-10-03, pose still used the command stamp | 10 | 1.165815 | — | 50075 | — | — | — | 0.024692 | 0.289518 | 38348 |
| 2026-10-03, pose uses the bogie stamp, `play_status=0`, `s0` 10943.893 m | 1 | 1.157702 | 3.674567 | 24376 | 0.887955 | 0.714807 | 0.202139 | 0.024691 | 0.289518 | 38396 |
| 2026-10-03, pose uses the bogie stamp, `play_status=0` | 10 | 1.159811 | 3.676738 | 24355 | 0.889699 | 0.716041 | 0.202192 | 0.024686 | 0.289518 | 38306 |

Before the 20 Hz hold, the no-command container was 5.216 m and 0.055 m/s at 9.3 Hz. The `--rate 10` row with 12180 pairs gives that one pair count for 1.315 m and 0.027 m/s. Replaying the recorded velocity at `--rate 10` printed 0.025424 m/s and 26124 pairs after the player reported an empty read queue. That row is not a delivery.

| Other measurement | Result |
|---|---|
| Val, 21 rides, tree `6af0037` | along-track median 1.467 m, p95 5.828 m |
| Val reprint on the same map | median 1.469 m, p95 5.828 m, speed 0.032 m/s |
| Python twin, path carried through the RTK pause | 1.395 m, maximum 4.910 m, speed 0.035 m/s |
| Core harness `tools/v2`, k₀ 1.001234 | 1.009 m, maximum 2.94 m. This is not the node pose and not the checker |
| Offline bogie mean, stamp +0.105 s, previous port | 0.023698 m/s, maximum 0.289518 m/s, bias +0.003012 m/s, 12206 pairs |
| Recorded node velocity, official checker replay, `--rate 1` | 0.024719 m/s, maximum 0.289518 m/s, 38334 pairs |
| Same recording, Humble Python synchronizer, arrival order | 0.024713 m/s, maximum 0.289518 m/s, 38658 pairs, bias +0.002926 m/s |
| Same recording, stamp order | 0.024128 m/s, 38658 pairs |
| `scripts/jury.sh acceptance`, teaching bag `30618_e9a34502` | exit code 0 |
| Memory and CPU, submission tree | 24.1 MB RSS, 0.89% of one core |

| Latency of `/result/position`, `--rate 1` | median | p95 | p99 | max | pairs |
|---|---|---|---|---|---|
| Submission mixed probe | 53 ms | 154 ms | 203 ms | 305 ms | — |
| 2026-10-03, pose still carried the command stamp | 52.6 ms | 154.3 ms | 203.4 ms | 305.8 ms | 50563 after 10 warmup pairs |
| 2026-10-03, pose uses the bogie stamp | 0.20 ms | 0.63 ms | 1.09 ms | 100.7 ms | 24388 |

On the last probe, 2 pairs exceed 100 ms and none exceed 250 ms. Unique position stamps are 9.31 Hz (18.61 Hz by message count, largest gap 0.296 s). Velocity stamps are 29.47 Hz. The along-track bound is an empirical bound, not certified protection level. Coefficient of adhesion is not observable from two wheel speeds and driver command alone.

The Russian runbook below is the same procedure. The long write-up is [results.md](docs/solution/results.md).

# RailBreak: резервная одометрия для автономного трамвая Москвы

Резервный канал скорости и пути для маршрута 10. Сдаётся пакет [`railbreak_backup_odometry`](railbreak_backup_odometry/) на ROS 2 Humble: две тележки, ручка, старт по GNSS за 3 с. По умолчанию редкие окна master RTK остаются (`gnss_correction: true`): организаторы 27 сентября в 13:14 ответили, что за такую поправку оценка не снижается. `gnss_correction: false` снимает подписки после старта. Скорость на выходе — среднее тележек, пока они согласны; между парами оно линейно интерполируется с шагом 0.04 с. Фильтр подставляет скорость, когда тележки расходятся или пропадают. IMU, лидар и камеры не используются. Исследовательское ядро `tramDR-0.0.11` в этот пакет не входит.

## Запуск для жюри

Нужен Docker (Docker Desktop на Windows и macOS, Docker Engine с Compose на Linux). Из корня клона:

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
.\scripts\jury.ps1 play -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

Контейнер собирает пакет вместе с пакетом сообщений организатора и проигрывает запись. Сеть хоста не нужна, первый запуск скачивает образ Humble. Запись и `tram_vehicle_msgs` в репозиторий не входят; карта маршрута уже лежит в пакете.

**Собирать с полным `tram_vehicle_msgs` из датасета.** В архиве `check-code-with-bag.zip` нет `DriverControllerCommand`. Без этого типа нода едет по тележкам с ручкой 0. Числа checker — в [README пакета](railbreak_backup_odometry/README.md).

Что должно появиться:

- `/result/velocity` — с первого сообщения тележки, м/с;
- `/result/position` — через 3 с после первого фикса GNSS: `frame_id` `map`, `child_frame_id` `base_link`, x около 99–103 км;
- в диагностике `gnss` = `correcting` после стартового окна: подписка master жива. `gnss_correction: false` даёт `closed`. `open` значит, что окно ещё не закрыто.

Автоматическая приёмка с кодом возврата:

```text
scripts/jury.sh acceptance --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

Она пишет выход, считает метрики и завершается ненулевым кодом, если нода упала, нет скорости или положения, штамп пошёл назад, частота ниже 10 Гц, разрыв больше 0.25 с, кадры не `map` / `base_link` или GNSS не закрылся.

## Итог проверки

Запись `30618_88aea4d9`, если в строке не названа другая. Официальный checker — `hackathon_solution_checker` / `metrics.py`, очередь 100, допуск 0.05 с. Поздняя строка раннюю не заменяет. Сдача — коммит `5cd35fa`, 2026-09-27. На текущем дереве скорость 0.024691 м/с остаётся выше порога 0.024 м/с.

| Прогон | Rate | 3D, м | max, м | пар | x | y | z | скорость, м/с | max | пар |
|---|---|---|---|---|---|---|---|---|---|---|
| `5ed4a7c` | — | 6.174 | 19.309 | — | — | — | — | 0.067 | — | — |
| `5cd35fa`, сдача, полная ручка | 1 | 2.151 | 5.649 | 50168 | 1.530 | 1.474 | 0.336 | 0.051 | 0.337 | 50481 |
| `1178c67`, полная ручка, один контейнер | 1 | 2.111 | 5.503 | 50124 | 1.481 | 1.467 | 0.337 | 0.0505 | 0.337 | 50508 |
| `e863195`, без типа ручки, удержание 20 Гц | 1 | 5.454 | 22.191 | 25488 | 5.110 | 1.872 | 0.366 | 0.044 | 0.307 | 25511 |
| Позднее дерево, без типа ручки | 1 | 1.325 | 4.806 | 25406 | 0.974 | 0.834 | 0.336 | 0.027 | 0.341 | 25429 |
| Позднее дерево, без типа ручки | 10 | 1.315 | — | 12180 | — | — | — | 0.027 | — | — |
| Полная ручка, до сдвига штампа скорости +0.10 с | 10 | 1.622 | 4.913 | 49623 | 1.234 | 1.005 | 0.316 | 0.035 | 0.340 | 49807 |
| Полная ручка, до сдвига штампа скорости +0.10 с | 1 | 1.615 | 4.899 | 49768 | 1.231 | 0.996 | 0.319 | 0.035 | 0.341 | 50006 |
| Штамп скорости +0.10 с, снимки GNSS включены | 10 | 1.624 | 4.909 | 49776 | — | — | — | 0.034 | 0.325 | 41383 |
| Снимки GNSS выключены | 10 | 2.093 | 6.467 | 49737 | — | — | — | 0.030 | — | 41364 |
| Среднее тележек, до геодезической дуги | 10 | 1.403506 | 4.940951 | 49643 | 0.964917 | 0.963472 | 0.332393 | 0.026695 | 0.289518 | 12117 |
| Среднее тележек, до геодезической дуги | 1 | 1.372506 | 4.922128 | 49956 | 0.934009 | 0.949411 | 0.331689 | 0.026633 | 0.289518 | 12205 |
| Геодезическая дуга, до хорды шва | 10 | 1.164891 | 3.692287 | 49972 | 0.889213 | 0.724400 | 0.203755 | 0.024677 | 0.289518 | 38302 |
| Геодезическая дуга, до хорды шва | 1 | 1.163029 | 3.682215 | 49940 | 0.888103 | 0.722818 | 0.203576 | 0.024695 | 0.289518 | 38388 |
| После хорды шва | 10 | 1.165573 | 3.692415 | 49914 | 0.889815 | 0.724788 | 0.203648 | 0.024665 | 0.289518 | 38294 |
| `9a954fd`, после хорды шва | 1 | 1.164429 | 3.684696 | 49943 | 0.889361 | 0.723523 | 0.203581 | 0.024692 | 0.289518 | 38392 |
| 2026-10-03, поза ещё выходила со штампом ручки | 10 | 1.165815 | — | 50075 | — | — | — | 0.024692 | 0.289518 | 38348 |
| 2026-10-03, поза со штампом тележки, `play_status=0`, `s0` 10943.893 м | 1 | 1.157702 | 3.674567 | 24376 | 0.887955 | 0.714807 | 0.202139 | 0.024691 | 0.289518 | 38396 |
| 2026-10-03, поза со штампом тележки, `play_status=0` | 10 | 1.159811 | 3.676738 | 24355 | 0.889699 | 0.716041 | 0.202192 | 0.024686 | 0.289518 | 38306 |

До удержания 20 Гц контейнер без ручки давал 5.216 м и 0.055 м/с при 9.3 Гц. В строке `--rate 10` с 12180 парами это один счётчик на 1.315 м и 0.027 м/с. Проигрыш записанной скорости на `--rate 10` дал 0.025424 м/с и 26124 пары: плеер написал, что очередь чтения опустела. Это не доставка ряда.

| Другой замер | Итог |
|---|---|
| Val, 21 рейс, дерево `6af0037` | медиана вдоль пути 1.467 м, p95 5.828 м |
| Повтор val на той же карте | медиана 1.469 м, p95 5.828 м, скорость 0.032 м/с |
| Python-двойник, перенос пути через паузу RTK | 1.395 м, максимум 4.910 м, скорость 0.035 м/с |
| Стенд ядра `tools/v2`, k₀ 1.001234 | 1.009 м, максимум 2.94 м. Это не поза ноды и не checker |
| Офлайн-среднее тележек, штамп +0.105 с, прежний порт | 0.023698 м/с, максимум 0.289518 м/с, bias +0.003012 м/с, 12206 пар |
| Записанная скорость ноды, проигрыш в официальный checker, `--rate 1` | 0.024719 м/с, максимум 0.289518 м/с, 38334 пары |
| Та же запись, Python-синхронизатор Humble, порядок прихода | 0.024713 м/с, максимум 0.289518 м/с, 38658 пар, bias +0.002926 м/с |
| Та же запись, порядок штампов | 0.024128 м/с, 38658 пар |
| `scripts/jury.sh acceptance`, учебная запись `30618_e9a34502` | код 0 |
| Память и CPU, дерево сдачи | 24.1 МБ RSS, 0.89 % одного ядра |

| Задержка `/result/position`, `--rate 1` | медиана | p95 | p99 | максимум | пар |
|---|---|---|---|---|---|
| Смешанный проб сдачи | 53 мс | 154 мс | 203 мс | 305 мс | — |
| 2026-10-03, поза ещё несла штамп ручки | 52.6 мс | 154.3 мс | 203.4 мс | 305.8 мс | 50563 после 10 разогревных |
| 2026-10-03, поза со штампом тележки | 0.20 мс | 0.63 мс | 1.09 мс | 100.7 мс | 24388 |

На последней пробе дольше 100 мс две пары, дольше 250 мс ни одной. Уникальные штампы положения — 9.31 Гц, сообщений 18.61 Гц, наибольший разрыв 0.296 с. Штампы скорости — 29.47 Гц. На сдаче частота обоих выходов была 29.3 Гц, разрыв штампов 0.051 с, у скорости в первую секунду 0.196 с, штамп назад 0 раз. Граница вдоль пути — эмпирическая, не protection level. Методика — в [results.md](docs/solution/results.md).

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

Задержка вход→выход: `ros2 bag record` трёх входов, `/result/position` и `/result/velocity` при `--rate 1`, затем `python3 tools/organizer/latency_probe.py --bag <запись> --output-topic /result/position`. Скрипт берёт время приёма выхода минус время приёма входа с тем же `header.stamp`. Штамп скорости сдвинут на 0.105 с, поэтому тот же вызов на `/result/velocity` пары почти не находит. `callback_max_us` в диагностике — время одного колбэка, не эта задержка.

## Контракт

| Вход | Тип |
|---|---|
| `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor`, в записях км/ч; нода делит на 3.6 |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand`, ручка −15…+15 |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix`, окно старта и редкая поправка на оси |

| Выход | Тип | Смысл |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | продольная скорость, м/с; с первого сообщения тележки |
| `/result/position` | `nav_msgs/Odometry` | точка `base_link` в MGRS, скорость в `twist.twist.linear.x`; штамп — штамп тележки. Ручка обновляет рычаг и позу не публикует. Пока обеих тележек нет, новое положение ждёт тележку. Если целостность запрещает им пользоваться, ковариация σ = 1 км |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, целостность, доверие, GNSS, порядок входов |

`base_link` — ось вращения первой тележки в точке касания колеса и рельса. Кадр по умолчанию — UTM 37N минус угол квадрата 300000 м на восток и 6100000 м на север: x — восток, y — север. От антенны master точка сдвинута на +9.873 м вдоль пути и на −3 м по высоте; tf: master `(−9.873, 0, 3)`, rover `(2.563, 0, 3)`. Если в окне старта есть только rover, дуга отступает на 12.436 м к master.

Торможение — отрицательная ручка, отдельного топика тормоза нет. Штамп положения — штамп тележки, штамп скорости — тот же штамп плюс 0.105 с. Часы ноды и `/clock` не используются. При полном пакете сообщений таймера нет. Если типа ручки нет, между тележками выход дополняется экстраполяцией 20 Гц; такой штамп не двигает отметку последнего входа. Три входных потока стоят в одной очереди: вход выпускается, когда все живые потоки дошли до его штампа. Ручка в этот минимум не входит. Отставший, но живой поток ждут; замолчавший перестают ждать после 50 чужих входов. Вход — best effort, очередь 500; выход — reliable, очередь 10.

Ориентиры ТЗ: задержка 100 мс (пик 250 мс), не ниже 10 Гц, не больше 2 ядер и 0.5 ГБ. На прогоне 2026-10-03 пик задержки 100.7 мс, уникальные штампы положения 9.31 Гц. Замер — в таблице выше.

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
