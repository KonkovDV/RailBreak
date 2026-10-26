<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

# Хакатон Московского транспорта — сдача решения

**Задача:** «Резервная одометрия по модели». Сроки и состав сдачи — в действующем Положении.

Организаторы: Фонд «Транспортные инновации Москвы», ООО «МТТЕХ», ООО «ВСМ-400»,
при поддержке Департамента транспорта города Москвы.

## Что сдаётся

1. **Пакет ROS 2.** [`railbreak_backup_odometry/`](railbreak_backup_odometry/).
2. **Инструкция для жюри.** [`README.md`](README.md).
3. **Модель.** [`docs/solution/model.md`](docs/solution/model.md).
4. **Допущения, параметры, ограничения.** [`docs/solution/assumptions.md`](docs/solution/assumptions.md).
5. **Точность и быстродействие.** [`docs/solution/results.md`](docs/solution/results.md).
6. **Разбор ТЗ.** [`docs/solution/tz-audit.md`](docs/solution/tz-audit.md).

Питч — [`docs/solution/pitch.md`](docs/solution/pitch.md). Текст полей формы — [`docs/solution/form.md`](docs/solution/form.md). Изменения после отбора — [`POSTMORTEM.md`](POSTMORTEM.md).

Сдача к дедлайну — коммит `543a4a5` (тег `submission-2026-09-27`), официальный checker на `5cd35fa`: 2.151 м и 0.051 м/с. Текущее дерево: 1.157702 м и 0.024691 м/с ([`results.md`](docs/solution/results.md)).

Входы решения: `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`,
`/vehicle/driver_position_cmd`; GNSS — стартовое окно 3 с и редкие окна master RTK на оси пути
(`gnss_correction: false` отключает их). Выходы: `/result/velocity` (м/с),
`/result/position` (точка `base_link` в MGRS) и `/result/diagnostics`.

Исследовательский фильтр `tram_dr_localization` (`tramDR-0.0.11`) в комплект сдачи не входит; его таблицы относятся к синтетике.

Код передаётся организаторам по Положению; отдельной публичной лицензии нет. Карта пакета построена по `train` и заменяется каталогом `assets_dir`; полилиния `route_10.yaml` — материал OSM (ODbL) исследовательского стенда. Сторонние компоненты — [`NOTICE`](NOTICE).
