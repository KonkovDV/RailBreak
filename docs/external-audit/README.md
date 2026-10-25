<p align="center"><img src="../../Logo.png" alt="RailBreak" width="280"></p>

# Внешний аудит исходного дерева (2026-10-02)

Комплект независимого аудита, выполненного по архиву `RailBreak.git.zip` (HEAD `c6a8c14`). Файлы перенесены сюда из корня репозитория без изменения содержания, чтобы корень оставался картой проекта.

| Файл | Что внутри |
|---|---|
| [`AUDIT_REPORT.md`](AUDIT_REPORT.md) | Что проверено, два исправленных класса дефектов (шов кольцевой карты, валидация assets) и список повторных проверок |
| [`AI_WORKPLAN.md`](AI_WORKPLAN.md) | Пошаговый план воспроизведения: чистая сборка, контракт ROS, время и порядок, карта, GNSS, метрики, sanitizers, release gate |
| [`ARCHIVE_README.md`](ARCHIVE_README.md) | Как распаковать исправленный архив и запустить проверки без ROS |
| [`RELEASE_CHECKSUMS.txt`](RELEASE_CHECKSUMS.txt) | SHA-256 исходного и исправленного архивов |

Что из рекомендаций аудита уже закрыто в основном дереве:

- интерполяция шва кольца и строгая валидация `ring.csv` / `notch.csv` / `stops.csv` — в `TrackMap` и загрузчике assets, покрыто `test_core`;
- ASan + UBSan для ядра сдаваемого пакета — в публичном CI (`railbreak_backup_odometry/tools`, джоб `asan`);
- раздельный учёт исторических и текущих метрик — [`docs/solution/results.md`](../solution/results.md), [`evidence/baseline.json`](../../evidence/baseline.json).
