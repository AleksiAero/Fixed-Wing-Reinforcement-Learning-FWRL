"""Atomic, local telemetry: current training state, never a recorded trajectory."""
import json
import os
import time
from pathlib import Path


def publish(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    data = json.dumps({**payload, 'published_at': time.time()})
    # Windows readers/OneDrive may briefly hold the destination without delete
    # sharing. A missed display packet must never abort a learning session.
    for attempt in range(4):
        try:
            temporary.write_text(data, encoding='utf-8')
            os.replace(temporary, path)
            return True
        except PermissionError:
            time.sleep(.002*(attempt+1))
    return False
