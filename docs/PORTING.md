<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# Перенос практик из других репозиториев автора

RailBreak собран на приёмах, которые уже проверены в соседних проектах [KonkovDV](https://github.com/KonkovDV). Код, модели и числа этих проектов в RailBreak не переносились — только инженерные практики.

| Практика | Откуда | Как в RailBreak |
|---|---|---|
| Детерминированная объяснимая карточка вместо вероятностного скора | [open-integrity-twin](https://github.com/KonkovDV/open-integrity-twin) (классы `leak_candidate`, `sensor_fault_suspected`, `transient_or_quality_issue`), [wellguard-os](https://github.com/KonkovDV/wellguard-os), [drillguard-os](https://github.com/KonkovDV/drillguard-os) | `advisory_class` в `/result/diagnostics`: `input_quality`, `common_mode`, `one_bogie_residual`, `both_residual`, `nominal` — плохой вход, невязка одного канала и общая невязка разведены; метка не пишет фильтр |
| Рекомендательный режим без права управления | те же проекты (`automatic_control: false`) | Нода публикует скорость, положение и доверие; тормоз и тягу не замыкает |
| Реестр утверждений и запрещённые формулировки с линтером в CI | [Mandat](https://github.com/KonkovDV/Mandat), [SynAPS-ProgramPlan](https://github.com/KonkovDV/SynAPS-ProgramPlan) | [CLAIMS_REGISTRY.md](../CLAIMS_REGISTRY.md), [BANNED_CLAIMS.txt](../BANNED_CLAIMS.txt), `tools/eval/lint_claims.py`: число в строке реестра сверяется с файлом-источником |
| Отдельный документ границ | [Mandat](https://github.com/KonkovDV/Mandat) `LIMITS.md`, [SynAPS-ProgramPlan](https://github.com/KonkovDV/SynAPS-ProgramPlan) | [LIMITS.md](../LIMITS.md): граница, поведение ноды сейчас и что её снимает |
| Карточка модели | [open-integrity-twin](https://github.com/KonkovDV/open-integrity-twin) `MODEL_CARD.md` | [MODEL_CARD.md](../MODEL_CARD.md) |
| Правила внесения изменений | [Mandat](https://github.com/KonkovDV/Mandat), [OpenRebar](https://github.com/KonkovDV/OpenRebar), [SynAPS](https://github.com/KonkovDV/SynAPS) `AGENTS.md` | [AGENTS.md](../AGENTS.md): проверки перед PR и числа из реестра |
| Машиночитаемая цитата | `CITATION.cff` в Mandat, OpenRebar, SynAPS, AeroBIM | [CITATION.cff](../CITATION.cff) |
| Дата и коммит у каждого опубликованного числа | `evidence.json` в Mandat и SynAPS-ProgramPlan | [evidence/baseline.json](../evidence/baseline.json), [provenance.json](solution/provenance.json), журнал checker в [results.md](solution/results.md) §9 |
| Fail-closed приёмка одной командой | `run-jury.sh` в AeroBIM, `run-demo.sh` в Mandat | `scripts/jury.sh acceptance`: код возврата 0 только при всех проверках выхода |
