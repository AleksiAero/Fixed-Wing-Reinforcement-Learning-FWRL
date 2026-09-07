"""Small, dependency-free desktop components with a quiet, light visual style."""
import tkinter as tk
from tkinter import ttk, font

PAGE = '#f5f5f7'
SURFACE = '#ffffff'
SIDEBAR = '#ededf0'
INK = '#1d1d1f'
MUTED = '#737378'
BLUE = '#007aff'
LINE = '#e5e5ea'
GREEN = '#248a3d'


def rounded(canvas, x1, y1, x2, y2, radius, **kwargs):
    r = min(radius, (x2-x1)/2, (y2-y1)/2)
    return canvas.create_polygon(x1+r, y1, x2-r, y1, x2, y1, x2, y1+r,
                                 x2, y2-r, x2, y2, x2-r, y2, x1+r, y2,
                                 x1, y2, x1, y2-r, x1, y1+r, x1, y1,
                                 smooth=True, splinesteps=24, **kwargs)


def configure(root):
    families = set(font.families(root))
    face = next((f for f in ('SF Pro Text', 'Segoe UI Variable Text', 'Segoe UI') if f in families), 'Helvetica')
    root.option_add('*Font', (face, 10))
    root.option_add('*Background', PAGE)
    root.option_add('*Foreground', INK)
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', background=PAGE, foreground=INK, font=(face, 10))
    style.configure('TFrame', background=PAGE)
    style.configure('TLabel', background=PAGE, foreground=INK)
    style.configure('TNotebook', background=PAGE, borderwidth=0, tabmargins=0, lightcolor=PAGE, darkcolor=PAGE, bordercolor=PAGE)
    style.layout('TNotebook.Tab', [])
    style.configure('TEntry', fieldbackground=SURFACE, foreground=INK, bordercolor=LINE, lightcolor=LINE, darkcolor=LINE, padding=8)
    style.map('TEntry', bordercolor=[('focus', BLUE)])
    style.configure('Treeview', background=SURFACE, fieldbackground=SURFACE, foreground=INK, borderwidth=0, rowheight=30)
    style.configure('Treeview.Heading', background='#fafafa', foreground=MUTED, font=(face, 9), relief='flat', borderwidth=0, padding=8)
    style.map('Treeview', background=[('selected', '#e3efff')], foreground=[('selected', '#0057b8')])
    style.map('Treeview.Heading', background=[('active', '#f2f2f7')])
    style.configure('Horizontal.TProgressbar', background=BLUE, troughcolor=LINE, borderwidth=0, thickness=3)
    return face


class Button(tk.Canvas):
    def __init__(self, parent, text, command, *, primary=False, width=None, background=None, height=36):
        self.text, self.command, self.primary = text, command, primary
        self.hover = self.selected = False
        self.enabled = True
        face = parent.winfo_toplevel().face
        self.text_font = font.Font(family=face, size=10, weight='normal')
        super().__init__(parent, width=width or self.text_font.measure(text)+30, height=height,
                         bg=background or parent.cget('bg'), highlightthickness=0, bd=0, takefocus=True, cursor='hand2')
        self.bind('<Configure>', lambda _: self.draw())
        self.bind('<Enter>', lambda _: self.set_hover(True))
        self.bind('<Leave>', lambda _: self.set_hover(False))
        self.bind('<Button-1>', lambda _: self.invoke())
        self.bind('<Return>', lambda _: self.invoke())
        self.bind('<space>', lambda _: self.invoke())
        self.bind('<FocusIn>', lambda _: self.draw())
        self.bind('<FocusOut>', lambda _: self.draw())

    def draw(self):
        self.delete('all')
        w, h = max(self.winfo_width(), 10), max(self.winfo_height(), 10)
        fill = ('#0064d6' if self.hover else BLUE) if self.primary else ('#e5e5eb' if self.hover else SURFACE)
        text = '#ffffff' if self.primary else INK
        if self.selected:
            fill, text = '#dce9fb', '#0066d7'
        if not self.enabled:
            fill, text = '#ececef', '#aaaab0'
        rounded(self, 1, 1, w-1, h-1, 10, fill=fill, outline=BLUE if self.focus_get() == self else '' if self.primary or self.selected else LINE, width=1)
        self.create_text(w/2, h/2, text=self.text, fill=text, font=self.text_font)

    def set_hover(self, value):
        self.hover = value
        self.draw()

    def set_enabled(self, value):
        self.enabled = value
        self.configure(cursor='hand2' if value else 'arrow', takefocus=value)
        self.draw()

    def invoke(self):
        if self.enabled:
            self.focus_set()
            self.command()


class Card(tk.Frame):
    def __init__(self, parent, padding=16, **kwargs):
        super().__init__(parent, bg=PAGE, **kwargs)
        self.backdrop = tk.Canvas(self, bg=PAGE, highlightthickness=0, bd=0)
        self.backdrop.place(x=0, y=0, relwidth=1, relheight=1)
        self.body = tk.Frame(self, bg=SURFACE)
        self.body.pack(fill='both', expand=True, padx=padding, pady=padding)
        self.bind('<Configure>', self.draw)

    def draw(self, event=None):
        w, h = self.winfo_width(), self.winfo_height()
        self.backdrop.delete('all')
        if w > 10 and h > 10:
            rounded(self.backdrop, 1, 3, w-1, h-1, 18, fill='#e8e8ed', outline='')
            rounded(self.backdrop, 1, 1, w-1, h-3, 18, fill=SURFACE, outline=LINE, width=.5)


def label(parent, text=None, *, variable=None, size=10, color=INK, weight='normal', **kwargs):
    return tk.Label(parent, text=text, textvariable=variable,
                    font=(parent.winfo_toplevel().face, size, weight), bg=parent.cget('bg'), fg=color, **kwargs)


def metric_row(parent, specs):
    row = tk.Frame(parent, bg=PAGE)
    row.pack(fill='x', pady=(0, 12))
    values = []
    for i, (title, value, detail) in enumerate(specs):
        row.columnconfigure(i, weight=1, uniform='metric')
        card = Card(row, padding=15)
        card.grid(row=0, column=i, sticky='nsew', padx=(0 if i == 0 else 6, 0 if i == len(specs)-1 else 6))
        label(card.body, title.upper(), size=8, color=MUTED).pack(anchor='w')
        variable = tk.StringVar(value=value)
        label(card.body, variable=variable, size=23, weight='bold').pack(anchor='w', pady=(3, 2))
        label(card.body, detail, size=9, color=MUTED).pack(anchor='w')
        values.append(variable)
    return values


def style_axes(ax, three_d=False):
    ax.set_facecolor(SURFACE)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)
    for name in ('xaxis', 'yaxis', 'zaxis') if three_d else ('xaxis', 'yaxis'):
        axis = getattr(ax, name)
        axis.label.set_color(MUTED)
        axis.label.set_fontsize(9)
        if three_d:
            axis.set_pane_color((.98, .985, .99, .18))
            axis.line.set_color('#dedee3')
            axis._axinfo['grid']['color'] = (.85, .87, .90, .28)
    if not three_d:
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.grid(axis='x', color=LINE, linewidth=.7, zorder=0)
        ax.set_axisbelow(True)
