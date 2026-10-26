<p align="center"><img src="../../Logo.png" alt="RailBreak" width="280"></p>

# Три акта для жюри

Сценарий живого прогона. Числа сбоев — строки таблицы `val` в [results.md](results.md), n = 21. Граница целостности эмпирическая, `integrity_certification_claim = false` — это диагностика, не protection level. Записи и их хеши в репозиторий не кладутся.

Экран — три терминала рядом с нодой:

```text
ros2 topic echo /result/diagnostics --field status[0].values
ros2 topic echo /result/position
ros2 topic echo /result/velocity
```

Положение: `frame_id` `map`, `child_frame_id` `base_link`, x и y в абсолютном `mgrs` (на выданных записях x около 99–103 км), дуга кольца — `s_m`. Скорость — `twist.linear.x`, м/с. `/clock` не нужен: штамп выхода — штамп входа.

Подготовка сбоя на машине с записями (каталог остаётся локальным):

```text
python tools/organizer/make_heldout.py files/data/<id> local/heldout/<id>_<fault> --gnss-s 3 --fault <fault>
```

Дальше тот же `scripts/jury.sh play` или `ros2 bag play`, что в [README.md](../../README.md#запуск-для-жюри). Для строгого режима без GNSS после старта нода запускается с параметром `gnss_correction:=false`:

```text
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  --params-file $(ros2 pkg prefix railbreak_backup_odometry)/share/railbreak_backup_odometry/config/params.yaml \
  -p gnss_correction:=false
```

## Что говорить

1. После стартового окна GNSS не ведёт оценку: скорость и путь дают тележки и модель, а редкая пачка RTK на оси только подправляет дугу. В строгом режиме GNSS отписывается полностью.
2. Мы не переключаемся жёстко между датчиками. Недостоверное измерение получает пониженный вес, а модель сохраняет непрерывность.
3. Два согласованных датчика не позволяют наблюдать общую моду. Система это распознаёт: через три секунды режим помечается как degraded, и integrity bound растёт. Следующая абсолютная путевая привязка ограничивает накопленный дрейф.

## Акт 1. GNSS ушёл

Обычная запись без сбоя.

На экране, по времени штампа:

- первые 3 с (`gnss_init_window_s`) `gnss=open`, фиксы входят в выставку, `gnss_fixes_used` растёт;
- затем `gnss=correcting`: master используется только для редких окон RTK на оси, rover путь больше не правит; в строгом режиме — лог `GNSS unsubscribed` и `gnss=closed`, подписки master и rover уничтожены (`sub_gm_.reset()`, `sub_gr_.reset()`);
- `/result/position` продолжается, `output_frame=mgrs`, кадр `map` / `base_link`;
- `/result/velocity` продолжается на сетке 0.04 с.

Фраза — первая из списка. На проверочной записи строгий режим даёт 2.093 м, режим с редкими окнами RTK — 1.624 м при той же ноде (`--rate 10`, [results.md](results.md) §9).

## Акт 2. Отказ одной тележки

Один из двух вводов:

- `drop_front_20s` — передняя тележка молчит 20 с;
- `scale_rear_5pct` — задняя тележка ×1.05 на весь рейс.

На экране:

- оба входа, `/vehicle/front_bogie_velocity` и `/vehicle/rear_bogie_velocity`;
- `slip_front` и `slip_rear` по отдельности;
- `mode`: на колбэке отмеченной тележки `MODEL`, на здоровой — `WHEELS` (или `ZUPT` на стоянке);
- `/result/position` без разрывов.

Фраза — вторая. Отмеченная тележка остаётся в фильтре с `R = 25 (м/с)²` (вес шага по скорости около `Pᵥᵥ/(Pᵥᵥ+25)`), прогноз скорости на этом шаге ведёт ручка, пропуск перекрывают вторая тележка и модель.

| Ввод | RMSE вдоль, м | Флаг в окне |
| --- | ---: | ---: |
| передняя молчит 20 с | 1.60 | 0.05 |
| задняя +5 % весь прогон | 84.0 | 0.19 |

Постоянный масштаб одной тележки — медленный сдвиг, а не юз: флаг редко поднимается, `integrity_status` может остаться `NOMINAL`. На экране видно `wheel_consensus_residual` — тележки разошлись; какая из них ошибается, покажет ближайшая станция.

## Акт 3. Общая мода

Одна и та же ошибка в обе тележки: `slide_both_20pct`, обе ×0.80 на 8 с.

На экране, по штампу от начала окна:

- обе скорости ниже модели, `slip_front=true` и `slip_rear=true`, тележки согласны;
- `integrity_status=DEGRADED_COMMON_MODE_UNOBSERVABLE`, причина `BOGIES_AGREE_MODEL_DISAGREES`, к границе добавлены 32.606 м (`B_mode` этого статуса, 99-й перцентиль запаса на `train`);
- `adhesion_classification=COMMON_MODE_SUSPECTED`, `adhesion_mu_estimate=null`;
- через `recover_s` = 3 с режим `COMMON_MODE_UNOBSERVABLE`: скорость тележек в состояние не копируется, `k` и `bₐ` не подстраиваются;
- `confidence_velocity` не возвращается в `ok` от одного согласия тележек; если за слепой бюджет нет якоря — `LOST`, `use_position` ложен, `/result/position` выходит с ковариацией `σ = 1 км`, скорость остаётся с `velocity_confidence=LOW`;
- `confidence_position` остаётся `degraded`, пока не вырастет `n_anchor`;
- на следующей единственной стоянке `n_anchor` увеличивается, защёлка снимается (`common_mode_exit=anchor`); в режиме по умолчанию то же делает окно RTK на оси.

Фраза — третья, после трёх секунд. Короткий юз одной тележки защёлку положения не ставит.

Таблица `val` для этого ввода: медиана вдоль пути 6.46 м, флаг в 95 % рейсов; без якорей медиана 12.6 м, с ними 1.47 м.

Два согласованных датчика не дают третьего измерения скорости: общий юз, общий масштаб и ошибка таблицы здесь не разделяются. Сценарий показывает, как система это распознаёт и где берёт привязку.
