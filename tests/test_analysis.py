import numpy as np
import pytest
from fwrl.analysis import analyze, analyze_files, read_path


def path(t, x, y=None, z=None):
    t = np.array(t, dtype=float)
    return {'t_s': t, 'x_m': np.array(x, dtype=float), 'y_m': t*0 if y is None else np.array(y), 'z_m': t*0+80 if z is None else np.array(z)}


def test_straight_and_roll_constraint():
    d = path([0, 1, 2, 3], [0, 22, 44, 66])
    assert analyze(d)['straight_level_percent'] == 100
    d['roll_deg'] = np.array([10]*4)
    assert analyze(d)['straight_level_percent'] == 0


def test_elapsed_time_weighting():
    d = path([0, 1, 2, 6], [0, 22, 44, 132], z=[80, 80, 80, 88])
    result = analyze(d)
    assert result['straight_level_percent'] == pytest.approx(100/3)
    assert sum(result['percent'].values()) == pytest.approx(100)


def test_heading_wrap_and_loiter():
    t = np.arange(0, 61, .1)
    a = t * .15 + 3
    result = analyze(path(t, 150*np.cos(a), 150*np.sin(a)))
    assert result['straight_level_percent'] == 0
    assert len(result['patterns']) == 1
    assert result['patterns'][0]['pattern'] == 'loiter_candidate'
    assert len(result['segments']) == 1


def test_gaps_and_stationary_not_level():
    result = analyze(path([0, 1, 20, 21], [0, 22, 440, 462]))
    assert result['coverage_percent'] < 10
    assert analyze(path([0, 1, 2], [0, 0, 0]))['straight_level_percent'] == 0


@pytest.mark.parametrize('body', ['t_s,x_m,y_m,z_m\n0,0,0,0\n0,20,0,0\n1,40,0,0', 't_s,x_m,y_m,z_m\n0,0,0,0\n1,nan,0,0\n2,40,0,0'])
def test_bad_csv(tmp_path, body):
    p = tmp_path/'bad.csv'
    p.write_text(body)
    with pytest.raises(ValueError):
        read_path(p)


def test_batch_weighted_by_duration(tmp_path):
    for name, t, climb in [('short', np.arange(3), False), ('long', np.arange(9), True)]:
        np.savetxt(tmp_path/f'{name}.csv', np.column_stack([t, t*22, t*0, t*2 if climb else t*0]), delimiter=',', header='t_s,x_m,y_m,z_m', comments='')
    assert analyze_files(list(tmp_path.glob('*.csv')))['straight_level_percent'] == 20
