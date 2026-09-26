# Хакатон Московского транспорта — сдача решения

**Задача:** «Резервная одометрия по модели». Сроки и состав сдачи — в действующем Положении.

Организаторы: Фонд «Транспортные инновации Москвы», ООО «МТТЕХ», ООО «ВСМ-400»,
при поддержке Департамента транспорта города Москвы.

## Что сдаётся

1. **Пакет ROS 2 и запуск.** [`railbreak_backup_odometry/`](railbreak_backup_odometry/).
2. **Модель.** [`docs/solution/model.md`](docs/solution/model.md).
3. **Допущения, параметры, ограничения.** [`docs/solution/assumptions.md`](docs/solution/assumptions.md).
4. **Точность и быстродействие.** [`docs/solution/results.md`](docs/solution/results.md).
5. **Текст полей формы.** [`docs/solution/form.md`](docs/solution/form.md).
6. **Питч.** [`docs/solution/pitch.md`](docs/solution/pitch.md).

Исследовательский фильтр `tram_dr_localization` (`tramDR-0.0.11`) в этот комплект не входит.
Его синтетические таблицы не являются результатом на записях маршрута 10.

Входы решения: `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`,
`/vehicle/driver_position_cmd`. Выходы: `/result/velocity` (м/с) и
`/result/position` (точка `base_link` в MGRS). После начальной выставки GNSS в фильтр не входит.

Отдельной публичной лицензии нет: код передаётся организаторам. Сторонние компоненты — [`NOTICE`](NOTICE).
