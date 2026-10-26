<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# Документация

## С чего начать

| Файл | Содержание |
|---|---|
| [README](../README.md) · [English](../README.en.md) | Проблема, решение, запуск для жюри, официальный checker |
| [`CLAIMS_REGISTRY.md`](../CLAIMS_REGISTRY.md) | Каждое число и сильное утверждение — с источником, командой воспроизведения и границей (C-, M-, L-) |
| [`LIMITS.md`](../LIMITS.md) | Границы утверждений: что делает нода сейчас и что снимает каждую границу |
| [`MODEL_CARD.md`](../MODEL_CARD.md) | Карточка оценщика: входы, выходы, данные калибровки, метрики, назначение |
| [`AUDIT-2026-10-25.md`](AUDIT-2026-10-25.md) | Аудит кода и документации: находки A-01–A-07, исправления, регрессионные тесты |
| [`PORTING.md`](PORTING.md) | Практики, перенесённые из соседних репозиториев автора |
| [`AGENTS.md`](../AGENTS.md) | Правила внесения изменений: проверки перед PR, запреты |

## Решение — `docs/solution/`

| Файл | Содержание |
|---|---|
| [пакет ROS 2](../railbreak_backup_odometry/) | `railbreak_backup_odometry`: входы, выходы, параметры, диагностика |
| [инструкция для жюри](../README.md) | Сборка, запуск, топики, приёмка, официальный checker |
| [`model.md`](solution/model.md) | Математическая модель: состояние `[s, v, k, b_a]`, привод, наблюдаемость, якоря, карта |
| [`assumptions.md`](solution/assumptions.md) | Допущения, источники сведений, полная таблица параметров |
| [`results.md`](solution/results.md) | Официальный checker, `val`, абляции, сбои, реальное время, журнал прогонов |
| [`tz-audit.md`](solution/tz-audit.md) | Соответствие критериям кейса и план развития |
| [`architecture.md`](solution/architecture.md) | Слои вокруг фильтра: FaultScore, диагноз невязки, банк моделей, монитор целостности, очередь входов |
| [`integrity.md`](solution/integrity.md) | Эмпирическая граница ошибки вдоль пути и статусы целостности |
| [`adhesion.md`](solution/adhesion.md) | Карточка невязок колесо–модель вместо оценки `μ` |
| [`references.md`](solution/references.md) | Литература 2012–2026 и что взято из каждой работы |
| [`scorecard.md`](solution/scorecard.md) | Табель по критериям ТЗ на `val`: скорость по режимам, положение, устойчивость |
| [`sigma_reliability.md`](solution/sigma_reliability.md) | Калибровка шкалы `σ_s` на `val` |
| [`hypotheses.md`](solution/hypotheses.md) | Прототип банка путевых гипотез для стрелок и депо |
| [`instrumentation.md`](solution/instrumentation.md) | Инструменты оценки: сопоставление пар, частота, задержка |
| [`pitch.md`](solution/pitch.md), [`demo.md`](solution/demo.md) | Пятиминутный доклад и три акта живой демонстрации |
| [`form.md`](solution/form.md) | Текст полей формы сдачи |

## Инженерный журнал

| Файл | Содержание |
|---|---|
| [`POSTMORTEM.md`](../POSTMORTEM.md) | Изменения после отбора: что вошло в ноду, что проверено и отклонено |
| [`improvement/`](improvement/) | Разборы 2026-10-03: [baseline](improvement/audit_baseline.md), [ADR](improvement/architecture_plan.md), [задержка](improvement/latency_split.md), [скорость](improvement/velocity_gap.md), [положение](improvement/position_gap.md), [отказы](improvement/fault_catalog.md) |
| [`evidence/`](evidence/) | Аудит архива checker организатора (2026-10-02) |
| [`external-audit/`](external-audit/) | Внешний аудит исходного дерева и план воспроизведения |

## Исследовательский стенд `tramDR-0.0.11`

Ранний 12-мерный UKF (`tram_dr_localization/`, `standalone/`) на синтетических данных. Нода сдачи его не использует; таблицы стенда относятся к синтетике, результаты маршрута 10 — в [`solution/results.md`](solution/results.md).

| Файл | Содержание |
|---|---|
| [`brief.md`](brief.md) | Краткое описание стенда |
| [`architecture.md`](architecture.md) | Компоненты и контракты интеграции |
| [`math.md`](math.md), [`estimator-priors.md`](estimator-priors.md) | Физическая модель, UKF, априоры |
| [`integrity-risk.md`](integrity-risk.md), [`checker.md`](checker.md) | Диагностики целостности и независимый checker |
| [`metrics.md`](metrics.md), [`verification.md`](verification.md) | Синтетические метрики и перечень проверок |
| [`refs.md`](refs.md) | Источники стенда |
