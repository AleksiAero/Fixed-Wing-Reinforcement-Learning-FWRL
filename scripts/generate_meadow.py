"""Generate a repeatable grassy course; save explicitly to avoid overwriting imports."""
import argparse
import json
from pathlib import Path
from fwrl.landscape import make_meadow

parser = argparse.ArgumentParser()
parser.add_argument('--seed', type=int, default=27)
parser.add_argument('--output', default='worlds/training_airfield.json')
args = parser.parse_args()
Path(args.output).write_text(json.dumps(make_meadow(args.seed), indent=2), encoding='utf-8')
