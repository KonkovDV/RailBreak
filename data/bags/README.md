# Слот записи организатора

Сюда кладётся rosbag2 (sqlite3 / mcap), когда его выдадут 25.09. Файлы `*.db3`
и каталоги прогонов в git не входят.

```
python tools/eval/inspect_bag.py data/bags/<run>
python tools/eval/run_bag.py data/bags/<run> --ukf standalone/build/Release/replay_ukf
```

Не подписывать оценщик на NavSatFix / IMU / PointCloud2: в записи это GT, не измерение фильтра.
