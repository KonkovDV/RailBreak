<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

# История

Пакет — `railbreak_backup_odometry` (`package.xml`, версия `0.1.0`). Подробный журнал изменений фильтра — [`POSTMORTEM.md`](POSTMORTEM.md) и [`docs/solution/results.md`](docs/solution/results.md) §9.

## 2026-10-25 — документация и CI

- README и документация решения переписаны под текущее дерево: актуальные числа официального checker (1.157702 м, 0.024691 м/с), позиционирование относительно работ 2025–2026 годов, единый план развития.
- Исправлены расхождения документации с кодом: режим GNSS по умолчанию (`gnss=correcting`) в сценарии демо, геодезическая карта и `k₀ = 1.001378` в модели и питче.
- Ядро пакета (`railbreak_backup_odometry/tools`) добавлено в CI-прогон AddressSanitizer + UndefinedBehaviorSanitizer; в CI добавлены четыре Python-набора `tools/organizer` (`test_ablation`, `test_contract`, `test_independent_generator`, `test_judge`).
- Материалы внешнего аудита перенесены в [`docs/external-audit/`](docs/external-audit/).

## 2026-10-03 — поза на штампе тележки

Официальный checker: 1.157702 м и 0.024691 м/с; задержка вход→публикация 0.20 мс медиана. Сетка сохранённой дуги 0.04 с между штампами тележек.

## 2026-09-27 — сдача

Тег `submission-2026-09-27` (`543a4a5`), checker на `5cd35fa`: 2.151 м и 0.051 м/с.

## Исследовательский стенд

Ядро `tram_dr_localization` имеет версию `tramDR-0.0.11` (`kModelVersion` в `tram_dr_localization/include/tram_dr_localization/types.hpp`); нода сдачи его не использует. Граница проверок — [`docs/verification.md`](docs/verification.md).
