<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

# История

Пакет — `railbreak_backup_odometry` (`package.xml`, версия `0.1.0`). Подробный журнал изменений фильтра — [`POSTMORTEM.md`](POSTMORTEM.md) и [`docs/solution/results.md`](docs/solution/results.md) §9.

## 2026-10-25 — аудит кода, реестр утверждений, документация и CI

Код ([аудит](docs/AUDIT-2026-10-25.md)):

- **A-01.** Проекция фикса GNSS на отрезок кольца в `init_on_ring` исправлена (`(n − ay)·dy` вместо `(e − ay)·dy`): привязка на отгружаемой карте — медиана 0.0000 м и p95 0.0007 м вместо 0.2458 м и 0.4749 м.
- **A-02.** `validate_params` проверяет все параметры, которые нода принимает из конфигурации (`dt_max`, `max_gap_s`, `v_max`, ворота, `load_factor`, коэффициенты Дэвиса и другие); Python-двойник отклоняет те же значения.
- **A-03.** Строгий разбор assets: хвост после числа, NaN в счётчике остановки и в метке позиции ручки — явный отказ конфигурации; CRLF читается.
- **A-04.** Монитор целостности получает `fr_floor` из параметра ноды.
- `test_core`: 472 проверки (было 446), включая проекцию на отрезки пяти курсов, шов кольца, 2000 фиксов на отгружаемой карте и повреждённые копии assets.

Практики ([PORTING.md](docs/PORTING.md)):

- [`CLAIMS_REGISTRY.md`](CLAIMS_REGISTRY.md), [`LIMITS.md`](LIMITS.md), [`MODEL_CARD.md`](MODEL_CARD.md), [`AGENTS.md`](AGENTS.md), [`CITATION.cff`](CITATION.cff), [`BANNED_CLAIMS.txt`](BANNED_CLAIMS.txt).
- `tools/eval/lint_claims.py` в CI: запрещённые формулировки, ссылки на номера реестра, числа реестра против источников, заголовочные числа checker в README, питче и результатах.
- README на русском и [английском](README.en.md).

Документация и CI:

- README и документация решения переписаны под текущее дерево: актуальные числа официального checker (1.157702 м, 0.024691 м/с), позиционирование относительно работ 2025–2026 годов, единый план развития.
- Исправлены расхождения документации с кодом: режим GNSS по умолчанию (`gnss=correcting`) в сценарии демо, геодезическая карта и `k₀ = 1.001378` в модели и питче.
- Ядро пакета (`railbreak_backup_odometry/tools`) добавлено в CI-прогон AddressSanitizer + UndefinedBehaviorSanitizer; в CI добавлены четыре Python-набора `tools/organizer` (`test_ablation`, `test_contract`, `test_independent_generator`, `test_judge`).
- Материалы внешнего аудита перенесены в [`docs/external-audit/`](docs/external-audit/).
- Инженерный журнал `docs/improvement/` получил сводку статуса и итог каждого разбора; документы исследовательского стенда переписаны в утвердительной форме с теми же контрактами и числами.

## 2026-10-03 — поза на штампе тележки

Официальный checker: 1.157702 м и 0.024691 м/с; задержка вход→публикация 0.20 мс медиана. Сетка сохранённой дуги 0.04 с между штампами тележек.

## 2026-09-27 — сдача

Тег `submission-2026-09-27` (`543a4a5`), checker на `5cd35fa`: 2.151 м и 0.051 м/с.

## Исследовательский стенд

Ядро `tram_dr_localization` имеет версию `tramDR-0.0.11` (`kModelVersion` в `tram_dr_localization/include/tram_dr_localization/types.hpp`); нода сдачи его не использует. Граница проверок — [`docs/verification.md`](docs/verification.md).
