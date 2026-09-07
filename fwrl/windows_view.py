"""Cross-platform local WebGL view of an existing live training session."""
import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from .landscape import terrain_height, gate_normal
from .launcher import geometry as launcher_geometry
from .airframe import geometry as airframe_geometry

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--live-state',type=Path,help='Telemetry JSON produced by fwrl.training')
    args = parser.parse_args()
    packets = [p for p in (ROOT/'runs/live').glob('mountain-zero-*.json') if not p.name.endswith('-world.json')]
    if not args.live_state and not packets:
        parser.error('No live telemetry found. Start fwrl.training with --live-state first.')
    current = args.live_state or max(packets,key=lambda p:p.stat().st_mtime)
    world = json.loads(current.with_name(current.stem+'-world.json').read_text(encoding='utf-8'))
    run = Path(json.loads(current.read_text(encoding='utf-8'))['run'])
    if os.name == 'nt' and str(run).startswith('\\mnt\\'):
        run = ROOT/'runs'/run.name
    lo,hi = world['bounds']
    nx,ny = 360,300
    vertices = [[lo[0]+(hi[0]-lo[0])*i/nx,lo[1]+(hi[1]-lo[1])*j/ny] for j in range(ny+1) for i in range(nx+1)]
    ground = [[x,y,terrain_height(x,y,world)] for x,y in vertices]
    scene = json.dumps(dict(world=world,ground=ground,nx=nx,ny=ny,launcher=launcher_geometry(world),airframe=airframe_geometry(),
                           normals=[gate_normal(world,i) for i in range(len(world['waypoints']))])).encode()
    cache = {'state': current.read_bytes()}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def do_GET(self):
            if self.path=='/state':
                # WSL atomic replacement can briefly deny Windows readers a
                # handle. Retain the last complete packet rather than break
                # the live connection during that sharing interval.
                try:
                    cache['state'] = current.read_bytes()
                except OSError:
                    pass
                data,kind = cache['state'],'application/json'
            elif self.path=='/scene':
                data,kind = scene,'application/json'
            elif self.path in ('/','/index.html'):
                data,kind = (ROOT/'fwrl/web/index.html').read_bytes(),'text/html; charset=utf-8'
            else:
                self.send_error(404);return
            self.send_response(200)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)

        def do_POST(self):
            if self.headers.get('Origin')!=f'http://127.0.0.1:{args.port}':
                self.send_error(403);return
            if self.path=='/pause':
                pause=run/'PAUSE'
                if pause.exists():pause.unlink()
                else:pause.touch()
            else:
                self.send_error(404);return
            self.send_response(204);self.end_headers()

    print(f'Live flight viewer: http://127.0.0.1:{args.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()


if __name__=='__main__':
    main()
