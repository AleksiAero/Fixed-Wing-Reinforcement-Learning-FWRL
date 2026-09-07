import numpy as np
from fwrl.airframe import geometry


def test_shared_airframe_builds_with_valid_parented_meshes():
    parts=geometry()
    seen=set()
    for part in parts:
        assert not part.get('parent') or part['parent'] in seen
        seen.add(part['name'])
        assert np.isfinite(part['vertices']).all()
        assert np.isfinite(part['pivot']).all()
        assert all(len(face)==3 and min(face)>=0 and max(face)<len(part['vertices']) for face in part['faces'])
    assert 'Fixed horizontal stabilizer' in seen and 'Fixed vertical stabilizer' in seen
    assert not any('boom' in name.lower() for name in seen)
