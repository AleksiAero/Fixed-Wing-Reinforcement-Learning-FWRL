import argparse
import json
from pathlib import Path
from .analysis import analyze_files, save_report
from .demo import make_examples, rollout, save_flight
from .world import load_world


def main():
    parser = argparse.ArgumentParser(description='SYNTHOSAR fixed-wing navigation workbench')
    sub = parser.add_subparsers(dest='command', required=True)
    a = sub.add_parser('analyze')
    a.add_argument('paths', nargs='+')
    a.add_argument('--output', default='report.json')
    d = sub.add_parser('demo')
    d.add_argument('--world', default='worlds/training_airfield.json')
    d.add_argument('--output', default='runs/baseline.csv')
    e = sub.add_parser('examples')
    e.add_argument('--output', default='data/examples')
    sub.add_parser('app')
    args = parser.parse_args()
    if args.command == 'analyze':
        paths = []
        for p in args.paths:
            paths.extend(sorted(Path(p).glob('*.csv')) if Path(p).is_dir() else [Path(p)])
        report = analyze_files(paths)
        save_report(report, args.output)
        print(json.dumps({'duration_s': report['duration_s'], 'straight_level_percent': report['straight_level_percent'], 'report': args.output}, indent=2))
    elif args.command == 'demo':
        states, result = rollout(load_world(args.world))
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        save_flight(states, args.output)
        print(json.dumps(result, indent=2))
    elif args.command == 'examples':
        make_examples(args.output)
    else:
        from .app import Workbench
        Workbench().mainloop()


if __name__ == '__main__':
    main()
