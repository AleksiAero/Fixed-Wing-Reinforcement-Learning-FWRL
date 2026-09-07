"""Generate a long, gentle hoop course with a ground-mounted launch rail."""
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fwrl.landscape import make_meadow, terrain_height
from fwrl.aerodynamics import PARAMETERS, MPH


from fwrl.highspeed import make_highspeed


if __name__ == '__main__':
    path = Path('worlds/training_airfield.json')
    if path.exists() and not Path('worlds/meadow_legacy.json').exists():
        Path('worlds/meadow_legacy.json').write_text(path.read_text())
    path.write_text(json.dumps(make_highspeed(), indent=2))
