# Слот записи организатора

Сюда кладётся rosbag2 (sqlite3 / mcap), когда его выдадут . Файлы `*.db3`
и каталоги прогонов в git не входят.

```
python tools/eval/inspect_bag.py data/bags/<run>
python tools/hackathon/t0.py data/bags/<run> --out reports/t0 \
  --ukf standalone/build/Release/replay_ukf.exe
```

Не подписывать оценщик на NavSatFix / IMU / PointCloud2: в записи это GT, не измерение фильтра.
