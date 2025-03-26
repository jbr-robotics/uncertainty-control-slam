# Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# PGM Metric usage examples

```bash
python3 -m quality_estimator.metric_calculators.pgm.corner_count /path/to/map.pgm
```

```bash
python3 -m quality_estimator.metric_calculators.pgm.enclosed_areas /path/to/map.pgm
```

```bash
python3 -m quality_estimator.metric_calculators.pgm.occupied_proportion /path/to/map.pgm
```

> Note: pass `--help` flag to see more options for each metric.