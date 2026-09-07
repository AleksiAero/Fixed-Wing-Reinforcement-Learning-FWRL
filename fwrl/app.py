"""Native desktop flight lab with a light, Apple-inspired workspace."""
import json
import os
from pathlib import Path
import queue
import random
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import LightSource, LinearSegmentedColormap
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from .analysis import Thresholds, analyze_files, read_path, save_report
from .demo import rollout, save_flight
from .world import load_world
from .landscape import terrain_height, gate_normal, make_meadow
from .highspeed import make_highspeed
from .mountain import make_mountain
from . import ui

ROOT = Path(__file__).resolve().parents[1]


class Workbench(tk.Tk):
    def __init__(self, world_path=None):
        super().__init__()
        self.title('Flight Lab — Meadow workspace')
        self.geometry('1440x940')
        self.minsize(1180, 860)
        self.face = ui.configure(self)
        self.configure(bg=ui.PAGE)
        self.report, self.states = None, None
        self.flights = []
        self.world_path = Path(world_path).resolve() if world_path else ROOT/'worlds/mountain_gauntlet.json'
        self.world = load_world(self.world_path)
        self.events = queue.Queue()
        self.busy = self.closing = False
        self.training_run = None
        self.job_buttons = []
        self.build_shell()
        self.build_world()
        self.build_analysis()
        self.build_training()
        self.select_page(0)
        self.protocol('WM_DELETE_WINDOW', self.close_app)
        self.after(100, self.poll)

    def build_shell(self):
        sidebar = tk.Frame(self, width=218, bg=ui.SIDEBAR)
        sidebar.pack(side='left', fill='y')
        sidebar.pack_propagate(False)
        brand = tk.Frame(sidebar, bg=ui.SIDEBAR)
        brand.pack(fill='x', padx=24, pady=(34, 35))
        ui.label(brand, '◈', size=32, color=ui.BLUE).pack(anchor='w')
        ui.label(brand, 'Flight Lab', size=20, weight='bold').pack(anchor='w', pady=(5, 0))
        ui.label(brand, 'SYNTHOSAR', size=9, color=ui.MUTED).pack(anchor='w', pady=(2, 0))
        ui.label(sidebar, 'WORKSPACE', size=8, color=ui.MUTED).pack(anchor='w', padx=25, pady=(0, 10))
        self.nav = []
        for index, title in enumerate(('◇   Environment', '⌁   Flight analysis', '◉   Training')):
            button = ui.Button(sidebar, title, lambda i=index: self.select_page(i), width=182, height=39)
            button.pack(padx=18, pady=3)
            self.nav.append(button)
        bottom = tk.Frame(sidebar, bg=ui.SIDEBAR)
        bottom.pack(side='bottom', fill='x', padx=24, pady=26)
        ui.label(bottom, 'RESEARCH AIRCRAFT', size=8, color=ui.MUTED).pack(anchor='w')
        ui.label(bottom, '5 kg  ·  1.65 m', size=12, weight='bold').pack(anchor='w', pady=(6, 5))
        ui.label(bottom, 'Fixed wing · Forward camera', size=8, color=ui.MUTED).pack(anchor='w')
        ui.label(bottom, 'LOCAL SIMULATION', size=8, color=ui.MUTED).pack(anchor='w', pady=(28, 6))
        ui.label(bottom, 'WSL2  /  ROS 2  /  MATLAB', size=8).pack(anchor='w')
        content = tk.Frame(self, bg=ui.PAGE)
        content.pack(side='left', fill='both', expand=True, padx=28)
        header = tk.Frame(content, bg=ui.PAGE)
        header.pack(fill='x', pady=(28, 20))
        self.heading = tk.StringVar()
        self.subtitle = tk.StringVar()
        ui.label(header, variable=self.heading, size=27, weight='bold').pack(anchor='w')
        ui.label(header, variable=self.subtitle, size=10, color=ui.MUTED).pack(anchor='w', pady=(4, 0))
        self.tabs = ttk.Notebook(content, width=1, height=1)
        self.world_tab = tk.Frame(self.tabs, bg=ui.PAGE)
        self.analysis_tab = tk.Frame(self.tabs, bg=ui.PAGE)
        self.train_tab = tk.Frame(self.tabs, bg=ui.PAGE)
        for tab in (self.world_tab, self.analysis_tab, self.train_tab):
            self.tabs.add(tab)
        footer = tk.Frame(content, bg=ui.PAGE)
        footer.pack(fill='x', pady=(10, 14))
        self.progress = ttk.Progressbar(footer, mode='indeterminate', length=72)
        self.status = tk.StringVar(value='Ready. Explore the meadow or load your flight data.')
        ui.label(footer, variable=self.status, size=9, color=ui.MUTED, anchor='w', wraplength=940).pack(side='left', fill='x', expand=True)
        self.tabs.pack(fill='both', expand=True, before=footer)

    def select_page(self, index):
        self.tabs.select(index)
        for i, button in enumerate(self.nav):
            button.selected = i == index
            button.draw()
        titles = [('Environment', 'A new perspective on your next flight.'),
                  ('Flight analysis', 'See where your time in the air goes.'),
                  ('Training', 'Turn a course into a repeatable experiment.')]
        self.heading.set(titles[index][0])
        self.subtitle.set(titles[index][1])

    def button(self, parent, text, command, primary=False, lock=True):
        button = ui.Button(parent, text, command, primary=primary)
        button.pack(side='left', padx=(0, 8), pady=3)
        if lock:
            self.job_buttons.append(button)
        return button

    def build_world(self):
        bar = tk.Frame(self.world_tab, bg=ui.PAGE)
        bar.pack(fill='x', pady=(0, 14))
        self.button(bar, 'Run simulation', lambda: self.background(self.run_baseline), primary=True)
        self.button(bar, 'Shuffle course', self.shuffle_course)
        self.button(bar, 'Import world', self.import_world)
        self.button(bar, 'Save world', self.export_world)
        self.button(bar, 'Build Blender scene', self.build_blender)
        self.world_metrics = ui.metric_row(self.world_tab, [('Course', '', 'Upright hoops · Route aligned'), ('Landscape', '', 'Rolling grass · Wild foliage'), ('Course seed', '', 'A repeatable random layout')])
        card = ui.Card(self.world_tab, padding=16)
        card.pack(fill='both', expand=True)
        heading = tk.Frame(card.body, bg=ui.SURFACE)
        heading.pack(fill='x')
        self.world_name = tk.StringVar(value=self.world['name'])
        ui.label(heading, variable=self.world_name, size=15, weight='bold').pack(side='left')
        self.button(heading, 'Reset view', self.reset_view, lock=False).pack_configure(side='right')
        self.world_summary = tk.StringVar(value='Drag to orbit · Scroll to zoom')
        ui.label(card.body, variable=self.world_summary, size=9, color=ui.MUTED).pack(anchor='w', pady=(3, 0))
        self.world_fig = Figure(figsize=(10, 5), facecolor=ui.SURFACE)
        self.world_ax = self.world_fig.add_subplot(111, projection='3d', computed_zorder=False)
        self.world_fig.subplots_adjust(left=0, right=1, bottom=.14, top=.97)
        self.world_canvas = FigureCanvasTkAgg(self.world_fig, master=card.body)
        self.world_canvas.get_tk_widget().configure(bg=ui.SURFACE, highlightthickness=0, width=1, height=1)
        self.world_canvas.get_tk_widget().pack(fill='both', expand=True)
        self.world_canvas.mpl_connect('scroll_event', self.zoom_world)
        ui.label(card.body, '●  Course guide     ●  Tree canopy     —  Simulated flight       |       Hoops are route markers', size=9, color=ui.MUTED).pack(anchor='w', pady=(0, 3))
        self.draw_world()

    def draw_world(self):
        ax = self.world_ax
        ax.clear()
        ui.style_axes(ax, True)
        lo, hi = self.world['bounds']
        x, y = np.meshgrid(np.linspace(lo[0], hi[0], 100), np.linspace(lo[1], hi[1], 100))
        z = terrain_height(x, y, self.world, np)
        cmap = LinearSegmentedColormap.from_list('meadow', ['#91a76b', '#a8b97b', '#bdc797'])
        rgb = LightSource(azdeg=305, altdeg=42).shade(z, cmap=cmap, vert_exag=1.4, fraction=.45, dx=(hi[0]-lo[0])/99, dy=(hi[1]-lo[1])/99, blend_mode='soft')
        ax.plot_surface(x, y, z, facecolors=rgb, rcount=100, ccount=100, linewidth=0, antialiased=False, shade=False, zorder=1)
        trees = [o for o in self.world['obstacles'] if o.get('kind') == 'tree']
        if trees:
            c = np.array([o['center'] for o in trees])
            size = np.array([o['size'] for o in trees])
            segments = np.stack([c-np.column_stack([c[:, 0]*0, c[:, 0]*0, size[:, 2]/2]), c+np.column_stack([c[:, 0]*0, c[:, 0]*0, size[:, 2]*.27])], axis=1)
            ax.add_collection3d(Line3DCollection(segments, colors='#75604b', linewidths=.7, zorder=2))
            ax.scatter(c[:, 0], c[:, 1], c[:, 2]+size[:, 2]*.27, s=size[:, 0]*2.6, c=[['#365f36', '#567536', '#718d42'][o.get('variant', 0)%3] for o in trees], alpha=.92, edgecolors='none', depthshade=True, zorder=3)
        for o in self.world['obstacles']:
            if o.get('kind') != 'tree':
                c, s = np.array(o['center']), np.array(o['size'])
                ax.bar3d(*(c-s/2), *s, color='#93a1b0', alpha=.7)
        points = np.array([self.world['start'], *self.world['waypoints']])
        ax.plot(*points.T, color='#ec9b47', linestyle=(0, (3, 5)), linewidth=1, alpha=.6, zorder=4)
        angle = np.linspace(0, 2*np.pi, 70)
        for i, point in enumerate(self.world['waypoints']):
            normal = gate_normal(self.world, i)
            side = np.array([-normal[1], normal[0], 0.])
            radius = self.world.get('gate_radii_m',[self.world['goal_radius_m']]*len(self.world['waypoints']))[i]
            ring = np.array(point) + radius*(np.cos(angle)[:, None]*side + np.sin(angle)[:, None]*np.array([0, 0, 1]))
            ax.plot(*ring.T, color='#007aff' if i%3 == 0 else '#e89036', linewidth=2, zorder=5)
            ax.text(point[0], point[1], point[2]+radius+8, f'{i+1:02d}', fontsize=8, color='#51535a', ha='center', zorder=6)
        ax.scatter(*points[0], color=ui.BLUE, s=35, marker='>', zorder=5)
        if self.states is not None:
            ax.plot(*self.states[:, :3].T, color=ui.BLUE, linewidth=1.8, zorder=6)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]), xlabel='East · m', ylabel='North · m', zlabel='Altitude · m')
        ax.set_box_aspect((1, 1, .28), zoom=1.35)
        ax.view_init(elev=29, azim=-63)
        # Mplot3D enforces a square axes box even in a wide canvas. Permit the
        # wider projected landscape outside that box, within the canvas itself.
        for artist in [*ax.collections, *ax.lines]:
            artist.set_clip_on(False)
        self.world_canvas.draw_idle()
        self.world_metrics[0].set(f'{len(self.world["waypoints"])} hoops')
        self.world_metrics[1].set(f'{len(trees)} trees')
        self.world_metrics[2].set(str(self.world.get('seed', 'Custom')))
        self.world_name.set(self.world['name'])

    def reset_view(self):
        lo, hi = self.world['bounds']
        self.world_ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]))
        self.world_ax.view_init(elev=29, azim=-63)
        self.world_canvas.draw_idle()

    def zoom_world(self, event):
        if event.inaxes != self.world_ax:
            return
        factor = .9 if event.button == 'up' else 1.1
        for get_limit, set_limit in ((self.world_ax.get_xlim, self.world_ax.set_xlim), (self.world_ax.get_ylim, self.world_ax.set_ylim)):
            lo, hi = get_limit()
            middle, span = (lo+hi)/2, (hi-lo)*factor/2
            set_limit(middle-span, middle+span)
        self.world_canvas.draw_idle()

    def shuffle_course(self):
        seed = random.SystemRandom().randrange(1, 999999)
        world = make_mountain(seed) if self.world.get('control_mode') in ('direct','elevons') else (make_highspeed(seed) if 'launcher' in self.world else make_meadow(seed))
        path = ROOT/'runs'/'worlds'/f'meadow-{seed}.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(world, indent=2), encoding='utf-8')
        self.world, self.world_path, self.states = load_world(path), path, None
        self.world_summary.set('New course · Ready to simulate')
        self.draw_world()
        self.status.set(f'Course {seed} saved. Blender and training will use this selected layout.')

    def import_world(self):
        p = filedialog.askopenfilename(filetypes=[('World JSON', '*.json')])
        if p:
            try:
                world = load_world(p)
                self.world, self.world_path, self.states = world, Path(p), None
                self.world_summary.set('Imported world · Ready to simulate')
                self.draw_world()
            except Exception as exc:
                messagebox.showerror('Invalid world', str(exc), parent=self)

    def export_world(self):
        p = filedialog.asksaveasfilename(defaultextension='.json', initialfile=f'meadow-{self.world.get("seed", "custom")}.json', filetypes=[('World JSON', '*.json')])
        if p:
            Path(p).write_text(json.dumps(self.world, indent=2), encoding='utf-8')
            self.status.set(f'World saved to {Path(p).name}.')

    def run_baseline(self):
        if self.world.get('control_mode') in ('direct','elevons'):
            self.run_process(self.wsl_command('bash','scripts/wsl_python.sh','-m','fwrl.session','--world',self.linux_path(self.world_path)))
            return
        states, result = rollout(self.world)
        self.events.put(('flight', (states, result)))

    def export_flight(self):
        if self.states is None:
            messagebox.showinfo('No flight yet', 'Run a simulation first.', parent=self)
            return
        p = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('Flight CSV', '*.csv')])
        if p:
            save_flight(self.states, p)

    def build_analysis(self):
        bar = tk.Frame(self.analysis_tab, bg=ui.PAGE)
        bar.pack(fill='x', pady=(0, 14))
        self.button(bar, 'Import flights', self.import_flights, primary=True)
        self.button(bar, 'Load examples', lambda: self.analyze(list((ROOT/'data/examples').glob('*.csv'))))
        self.button(bar, 'Analyze simulation', self.analyze_simulation)
        self.button(bar, 'Export report', self.export_report)
        self.button(bar, 'Export flight', self.export_flight)
        self.analysis_metrics = ui.metric_row(self.analysis_tab, [('Flights', '—', 'Individual recordings'), ('Flight time', '—', 'Weighted by elapsed time'), ('Straight & level', '—', 'Trajectory classification')])
        settings = tk.Frame(self.analysis_tab, bg=ui.PAGE)
        settings.pack(fill='x', pady=(0, 10))
        self.turn, self.vertical = tk.StringVar(value='3.0'), tk.StringVar(value='0.5')
        for text, var in [('Turn threshold · °/s', self.turn), ('Vertical threshold · m/s', self.vertical)]:
            ui.label(settings, text, size=9, color=ui.MUTED).pack(side='left', padx=(0, 8))
            ttk.Entry(settings, textvariable=var, width=6).pack(side='left', padx=(0, 20))
        self.button(settings, 'Apply', self.reanalyze)
        self.summary = tk.StringVar(value='Import CSV files to explore trajectories and flight patterns.')
        card = ui.Card(self.analysis_tab, padding=12)
        card.pack(fill='both', expand=True, pady=(0, 10))
        ui.label(card.body, variable=self.summary, size=10, color=ui.MUTED).pack(anchor='w', padx=8)
        self.fig = Figure(figsize=(10, 3), facecolor=ui.SURFACE, constrained_layout=True)
        self.ax_path = self.fig.add_subplot(121, projection='3d')
        self.ax_share = self.fig.add_subplot(122)
        ui.style_axes(self.ax_path, True)
        ui.style_axes(self.ax_share)
        self.ax_path.set_title('Flight paths', fontsize=11, loc='left', color=ui.INK)
        self.ax_share.set_title('Time in each pattern', fontsize=11, loc='left', color=ui.INK)
        self.canvas = FigureCanvasTkAgg(self.fig, master=card.body)
        self.canvas.get_tk_widget().configure(width=1, height=1)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        table_card = ui.Card(self.analysis_tab, padding=12)
        table_card.pack(fill='x')
        self.table = ttk.Treeview(table_card.body, columns=('duration', 'straight', 'coverage', 'patterns'), show='tree headings', height=4)
        self.table.heading('#0', text='Flight')
        self.table.column('#0', width=170, minwidth=120)
        for key, title, width in [('duration', 'Duration', 100), ('straight', 'Straight & level', 130), ('coverage', 'Coverage', 110), ('patterns', 'Pattern candidates', 240)]:
            self.table.heading(key, text=title)
            self.table.column(key, width=width, minwidth=70)
        scroll = ttk.Scrollbar(table_card.body, orient='vertical', command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.table.pack(fill='x', expand=True)

    def import_flights(self):
        paths = filedialog.askopenfilenames(filetypes=[('Flight CSV', '*.csv')])
        if paths:
            self.analyze(paths)

    def reanalyze(self):
        if self.flights:
            self.analyze(self.flights)

    def analyze_simulation(self):
        if self.states is None:
            messagebox.showinfo('No flight yet', 'Run a simulation in Environment first.', parent=self)
            return
        path = ROOT/'runs'/'last-simulation.csv'
        path.parent.mkdir(parents=True, exist_ok=True)
        save_flight(self.states, path)
        self.analyze([path])

    def analyze(self, paths):
        try:
            cfg = Thresholds(turn_rate_deg_s=float(self.turn.get()), vertical_speed_m_s=float(self.vertical.get()))
            report = analyze_files(paths, cfg)
            self.report, self.flights = report, list(paths)
            self.ax_path.clear()
            self.ax_share.clear()
            ui.style_axes(self.ax_path, True)
            ui.style_axes(self.ax_share)
            self.table.delete(*self.table.get_children())
            palette = ['#007aff', '#34a878', '#f0a03e', '#a078d4', '#de6b76']
            for index, flight in enumerate(report['flights']):
                data = read_path(flight['file'])
                self.ax_path.plot(data['x_m'], data['y_m'], data['z_m'], label=Path(flight['file']).stem, color=palette[index%len(palette)], linewidth=1.7)
                self.table.insert('', 'end', text=Path(flight['file']).name, values=(f"{flight['duration_s']:.1f} s", f"{flight['straight_level_percent']:.1f}%", f"{flight['coverage_percent']:.1f}%", ', '.join(p['pattern'].replace('_', ' ') for p in flight['patterns']) or '—'))
            self.ax_path.set(xlabel='East · m', ylabel='North · m', zlabel='Altitude · m')
            self.ax_path.legend(fontsize=7, frameon=False, loc='upper left', ncol=2)
            shares = sorted(report['percent'].items(), key=lambda item: -item[1])
            labels = [k.replace('_', ' ').capitalize() for k, _ in shares]
            values = [v for _, v in shares]
            self.ax_share.barh(labels, values, color=['#007aff' if k == 'straight_level' else '#a9caff' for k, _ in shares], height=.57)
            for i, value in enumerate(values):
                self.ax_share.text(min(value+1, 94), i, f'{value:.1f}%', va='center', fontsize=8, color=ui.MUTED)
            self.ax_share.set(xlabel='Elapsed flight time · %', xlim=(0, 110))
            self.ax_share.set_xticks([0, 25, 50, 75, 100])
            self.ax_share.invert_yaxis()
            self.canvas.draw_idle()
            self.analysis_metrics[0].set(str(len(paths)))
            self.analysis_metrics[1].set(f"{report['duration_s']/60:.1f} min")
            self.analysis_metrics[2].set(f"{report['straight_level_percent']:.1f}%")
            self.summary.set('Flight paths and duration-weighted patterns. Position alone does not verify wings-level attitude.')
            self.status.set('Analysis complete. Adjust the thresholds and select Apply to recalculate.')
        except Exception as exc:
            messagebox.showerror('Cannot analyze flights', str(exc), parent=self)

    def export_report(self):
        if self.report:
            path = filedialog.asksaveasfilename(defaultextension='.json', filetypes=[('Analysis JSON', '*.json')])
            if path:
                save_report(self.report, path)

    def build_training(self):
        ui.metric_row(self.train_tab, [('Compute', 'NVIDIA GPU', 'CUDA · WSL2 runtime'), ('Learning', 'PPO', 'Direct control from zero' if self.world.get('control_mode') in ('direct','elevons') else 'Residual waypoint guidance'), ('Observations', 'World state', 'Vehicle, wind and obstacles')])
        settings = ui.Card(self.train_tab, padding=18)
        settings.pack(fill='x', pady=(0, 12))
        ui.label(settings.body, 'Training session', size=14, weight='bold').pack(anchor='w')
        ui.label(settings.body, 'Uses the world selected in Environment. Every run gets its own checkpoint folder.', size=9, color=ui.MUTED).pack(anchor='w', pady=(5, 12))
        form = tk.Frame(settings.body, bg=ui.SURFACE)
        form.pack(fill='x')
        ui.label(form, 'Transitions', size=10).pack(side='left', padx=(0, 10))
        self.steps = tk.StringVar(value='10000000' if self.world.get('control_mode') in ('direct','elevons') else '100000')
        ttk.Entry(form, textvariable=self.steps, width=12).pack(side='left', padx=(0, 18))
        self.button(form, 'Start training', self.start_training, primary=True)
        self.stop_button = self.button(form, 'Stop after checkpoint', self.stop_training, lock=False)
        self.stop_button.set_enabled(False)
        tools = tk.Frame(settings.body, bg=ui.SURFACE)
        tools.pack(fill='x', pady=(12, 0))
        self.button(tools, 'Evaluate checkpoint', self.evaluate_checkpoint)
        self.button(tools, 'Check GPU', lambda: self.background(lambda: self.run_process(self.wsl_command('nvidia-smi'))))
        self.button(tools, 'Open runs', lambda: self.open_folder(ROOT/'runs'), lock=False)
        self.button(tools, 'Setup guide', lambda: self.open_file(ROOT/'README.md'), lock=False)
        card = ui.Card(self.train_tab, padding=18)
        card.pack(fill='both', expand=True)
        ui.label(card.body, 'Activity', size=14, weight='bold').pack(anchor='w', pady=(0, 12))
        self.log = tk.Text(card.body, bg='#f8f9fb', fg='#51545c', insertbackground=ui.BLUE,
                           font=('Consolas', 10), wrap='word', bd=0, padx=15, pady=15, height=1, highlightthickness=1, highlightbackground=ui.LINE)
        self.log.pack(fill='both', expand=True)
        self.log.insert('end', 'Your training activity will appear here.\n\nCheckpoints are saved after each update.\nEvaluation reports and flight recordings are saved beside the checkpoint.\n')
        self.log.configure(state='disabled')
        ui.label(card.body, 'State-based navigation · Camera-image learning is not enabled', size=9, color=ui.MUTED).pack(anchor='w', pady=(12, 0))

    def wsl_command(self, *args):
        return ['wsl', '-d', 'Ubuntu-24.04', '-u', 'root', '--cd', str(ROOT), '--', *args] if os.name == 'nt' else list(args)

    def linux_path(self, path):
        if os.name == 'nt':
            return subprocess.check_output(['wsl', '-d', 'Ubuntu-24.04', '--', 'wslpath', '-a', str(path)], text=True, creationflags=subprocess.CREATE_NO_WINDOW).strip()
        return str(path)

    def start_training(self):
        try:
            steps = int(self.steps.get())
            if steps <= 0:
                raise ValueError('Transitions must be positive')
            if self.busy:
                return
            if self.world.get('control_mode') in ('direct','elevons'):
                command = self.wsl_command('bash','scripts/wsl_python.sh','-m','fwrl.session','--world',self.linux_path(self.world_path),'--steps',str(steps))
                self.background(lambda: self.run_process(command))
                self.status.set('Opening live learning in Blender. Pause/stop in the Flight Lab sidebar.')
                return
            self.training_run = ROOT/'runs'/f'ppo-{time.time_ns()}'
            command = self.wsl_command('bash', 'scripts/wsl_python.sh', '-m', 'fwrl.training', '--device', 'cuda', '--steps', str(steps), '--world', self.linux_path(self.world_path), '--output', self.linux_path(self.training_run))
            self.stop_button.set_enabled(True)
            self.background(lambda: self.run_process(command))
        except Exception as exc:
            self.stop_button.set_enabled(False)
            messagebox.showerror('Training', str(exc), parent=self)

    def stop_training(self):
        if self.training_run:
            # The directory may not exist until Python finishes importing CUDA.
            if self.training_run.exists():
                (self.training_run/'STOP').touch()
            else:
                self.after(300, self.stop_training)
            self.status.set('Stopping after the next checkpoint…')

    def evaluate_checkpoint(self):
        checkpoint = filedialog.askopenfilename(initialdir=ROOT/'runs', filetypes=[('FWRL policy checkpoint', '*.pt')])
        if checkpoint:
            try:
                command = self.wsl_command('bash', 'scripts/wsl_python.sh', '-m', 'fwrl.evaluate', self.linux_path(Path(checkpoint)), '--device', 'cuda')
                self.background(lambda: self.run_process(command))
            except Exception as exc:
                messagebox.showerror('Evaluation', str(exc), parent=self)

    def build_blender(self):
        try:
            output = ROOT/'runs'/'world-build'/f'meadow-{self.world.get("seed", "custom")}-{time.time_ns()}.blend'
            command = self.wsl_command('blender', '--background', '--python', 'blender/build_world.py', '--', '--world', self.linux_path(self.world_path), '--output', self.linux_path(output), '--render')
            self.background(lambda: self.run_process(command))
        except Exception as exc:
            messagebox.showerror('Blender', str(exc), parent=self)

    def background(self, task):
        if self.busy:
            return
        self.busy = True
        for button in self.job_buttons:
            button.set_enabled(False)
        self.progress.start(12)
        self.progress.pack(side='right', padx=8)
        self.status.set('Working…')
        def worker():
            try:
                task()
                self.events.put(('done', 'Finished. Your results are ready.'))
            except Exception as exc:
                self.events.put(('done', f'Error: {exc}'))
        threading.Thread(target=worker, daemon=True).start()

    def run_process(self, command):
        flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        with subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace', creationflags=flags) as proc:
            for line in proc.stdout:
                self.events.put(('log', line))
            if proc.wait():
                raise RuntimeError(f'Process exited {proc.returncode}; see Activity in Training.')

    def poll(self):
        for _ in range(200):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self.log.configure(state='normal')
                self.log.insert('end', value)
                if int(self.log.index('end-1c').split('.')[0]) > 1500:
                    self.log.delete('1.0', '300.0')
                self.log.see('end')
                self.log.configure(state='disabled')
            elif kind == 'flight':
                self.states, result = value
                self.world_summary.set(f"{result['result'].capitalize()}  ·  {result['waypoints_reached']} / {len(self.world['waypoints'])} waypoints  ·  {result['time_s']:.1f} seconds")
                self.draw_world()
            else:
                self.busy = False
                self.training_run = None
                self.stop_button.set_enabled(False)
                self.progress.stop()
                self.progress.pack_forget()
                for button in self.job_buttons:
                    button.set_enabled(True)
                self.status.set(value)
                if self.closing:
                    self.destroy()
                    return
        self.after(100, self.poll)

    def close_app(self):
        if self.busy:
            self.closing = True
            if self.training_run:
                self.stop_training()
            self.status.set('Finishing the current job before closing…')
        else:
            self.destroy()

    @staticmethod
    def open_file(path):
        if os.name == 'nt':
            os.startfile(path)
        else:
            subprocess.Popen(['xdg-open', str(path)])

    def open_folder(self, folder):
        folder.mkdir(parents=True, exist_ok=True)
        self.open_file(folder)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Flight Lab desktop workspace')
    parser.add_argument('--world', help='Open a saved world JSON at startup')
    args = parser.parse_args()
    Workbench(args.world).mainloop()


if __name__ == '__main__':
    main()
