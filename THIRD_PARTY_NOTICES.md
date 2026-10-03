# Сторонние компоненты

Этот файл не добавляет лицензий и не заменяет [NOTICE](NOTICE). Новых юридических выводов здесь нет.

Из `NOTICE` на 2026-10-03:

- Отдельного публичного LICENSE нет. MIT на код репозитория не объявлена.
- Eigen, `mherb/kalman` и `robot_localization` в ядро не вендорятся.
- ROS 2 Humble используется как upstream (Apache 2.0 у upstream). Исходники Humble в этот репозиторий не скопированы.
- `assets/ring.csv`, `stops.csv`, `notch.csv`, `meta.yaml` — медиана выданных train-рейсов, не OSM.
- `assets/integrity_bound.json` — эмпирическая добавка по train, не сертификат и не SIL.
- `tram_dr_localization/config/route_10.yaml` — полилиния © OpenStreetMap contributors, ODbL, только исследовательское ядро.
- Записи организатора, `files/`, `local/`, `jury_out/`, `*.db3` в публичный git не входят.

Сообщения `tram_vehicle_msgs` приходят с датасетом организатора и в репозиторий не вложены. В архиве `check-code-with-bag.zip` нет `DriverControllerCommand`.

Чужие репозитории финалистов лежат только в локальном `files/` и в публичный коммит не входят. Их код, модели и заявленные метрики сюда не копировались.

Раскрытие помощи модели: аудит `docs/improvement/audit_baseline.md` и этот файл подготовлены coding-моделью в Cursor 2026-10-03. Более ранняя история коммитов построчно на «человек / модель» не размечена; такая разметка из git сама не следует.
