"""Export only inference weights; Blender does not need PyTorch installed."""
import argparse
import json
from pathlib import Path
import torch

p = argparse.ArgumentParser()
p.add_argument('checkpoint')
a = p.parse_args()
saved = torch.load(a.checkpoint, map_location='cpu', weights_only=True)
weights = {key: value.tolist() for key, value in saved['model'].items()
           if key.startswith(('body.', 'actor.'))}
Path(a.checkpoint).with_name('blender_policy.json').write_text(json.dumps(weights))
