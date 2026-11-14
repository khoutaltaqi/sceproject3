"""Record the independently measured final distance without using the camera."""
import argparse
import json
import math
from datetime import datetime
from pathlib import Path
from .common import save


def record(path, actual):
    if not math.isfinite(actual) or not 0 < actual < 3:
        raise ValueError('Actual final distance must be in metres, between 0 and 3')
    p = Path(path)/'summary.json'
    s = json.loads(p.read_text())
    if s.get('estimated_final_m') is None:
        raise ValueError('This trial has no final estimate')
    if s.get('actual_final_m') is not None:
        raise ValueError('Actual distance already recorded. Edit summary.json explicitly to correct it.')
    s['actual_final_m'] = actual
    s['measurement_method'] = 'independent tape/ruler, same physical reference, robot not repositioned'
    s['measurement_recorded_at'] = datetime.now().astimezone().isoformat()
    s['error_m'] = s['estimated_final_m'] - actual
    save(p, s)
    print(f"Error = estimated - actual = {s['error_m']:+.4f} m")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    p.add_argument('actual', type=float, help='Independent final tape/ruler distance, metres')
    a = p.parse_args()
    record(a.folder, a.actual)


if __name__ == '__main__':
    main()
