from pathlib import Path
from typing import Dict, Any, Optional
import csv 

from cartographer_tuner.metrics.metric import Metric

class CsvStatsWriter:
    def __init__(self, csv_file: str):
        self.csv_file = Path(csv_file).open("w")
        self.header_written = False
        self.writer: Optional[csv.DictWriter] = None

    def write(self, in_data: Dict[str, Any], out_data: Dict[str, Metric]):
        if not self.header_written:
            fieldnames = self._make_header(in_data, out_data)
            self.writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.writer.writeheader()
            self.header_written = True
        out_dict = dict()
        for key, metric in out_data.items():
            if metric.uncertainty is not None:
                out_dict[f"{key}_uncertainty"] = metric.uncertainty
            out_dict[key] = metric.value
        self.writer.writerow(in_data | out_dict)

    def _make_header(self, in_data: Dict[str, Any], out_data: Dict[str, Metric]):
        fieldnames = [key for key in in_data.keys()]
        for key, metric in out_data.items():
            if metric.uncertainty is not None:
                fieldnames.append(f"{key}_uncertainty")
            fieldnames.append(f"{key}")
        return fieldnames

    def __del__(self):
        self.csv_file.close()