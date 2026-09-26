# RailBreak — резервная трамвайная одометрия

Пакет для жюри — [`railbreak_backup_odometry`](railbreak_backup_odometry/README.md):
ROS 2 Humble. Входы — скорости двух тележек и положение ручки контроллера.
Выходы — `/result/velocity` (м/с) и `/result/position` (точка `base_link` в MGRS).
GNSS читается только первые 3 с, для начальной выставки.

| Артефакт | Документ |
|---|---|
| Пакет ROS 2 и инструкция запуска | [`railbreak_backup_odometry/`](railbreak_backup_odometry/) |
| Математическая модель | [`docs/solution/model.md`](docs/solution/model.md) |
| Допущения, параметры, ограничения | [`docs/solution/assumptions.md`](docs/solution/assumptions.md) |
| Точность и быстродействие | [`docs/solution/results.md`](docs/solution/results.md) |
| Текст для формы | [`docs/solution/form.md`](docs/solution/form.md) |
| Питч | [`docs/solution/pitch.md`](docs/solution/pitch.md) |

Сборка пакета и проигрывание записи описаны в README пакета.
Офлайн-таблица val воспроизводится так:

```bash
python tools/organizer/eval_odometer.py
```

Карта по умолчанию — та же ось, что в `railbreak_backup_odometry/assets/`.

## Исследовательское ядро

`tram_dr_localization`, версия `tramDR-0.0.11`, в сдачу не входит.
Это синтетический стенд UKF до выдачи записей организатора: GNSS, IMU и lidar
в тот фильтр не входят. Его проверки — [`docs/verification.md`](docs/verification.md),
таблицы seed 42 — [`docs/metrics.md`](docs/metrics.md). Числа стенда не
подставляются вместо таблицы маршрута 10.

```bash
cmake -S standalone -B standalone/build -DCMAKE_BUILD_TYPE=Release
cmake --build standalone/build --parallel
ctest --test-dir standalone/build --output-on-failure
```

На Windows: `cmake --build standalone/build --config Release --parallel`,
`ctest --test-dir standalone/build -C Release`.

## Лицензии

Код — [LICENSE](LICENSE) (MIT). Сторонние уведомления — [NOTICE](NOTICE).
Записи организатора и выданные полилинии в репозиторий не входят.
