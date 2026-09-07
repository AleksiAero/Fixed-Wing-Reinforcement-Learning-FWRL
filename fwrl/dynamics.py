"""Coordinated-turn point-mass model with first-order inner-loop response.

State: x,y,z,heading,airspeed,bank,flight-path-angle (m, rad, m/s).
Command: desired bank, desired flight-path-angle, desired airspeed.
This is an idealized autopilot model, not aerodynamic surface control.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Aircraft:
    dt: float = 0.05
    min_speed: float = 12.0
    max_speed: float = 35.0
    max_bank: float = 0.7
    max_gamma: float = 0.25
    bank_tau: float = 0.6
    gamma_tau: float = 1.0
    speed_tau: float = 3.0

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("Aircraft parameters must be positive and finite")
        if self.min_speed >= self.max_speed or self.dt > min(self.bank_tau, self.gamma_tau, self.speed_tau):
            raise ValueError("Invalid speed bounds or integration timestep")
        if self.max_bank >= np.pi / 2 or self.max_gamma >= np.pi / 2:
            raise ValueError("Bank and climb-angle limits must be below pi/2")


def step(state, command, cfg=None, wind=(0., 0., 0.)):
    cfg = cfg or Aircraft()
    s = np.array(state, dtype=float, copy=True)
    command = np.asarray(command, dtype=float)
    if s.shape != (7,) or command.shape != (3,) or not np.isfinite(s).all() or not np.isfinite(command).all():
        raise ValueError("Finite state[7] and command[3] required")
    bank, gamma, speed = np.clip(command, [-cfg.max_bank, -cfg.max_gamma, cfg.min_speed], [cfg.max_bank, cfg.max_gamma, cfg.max_speed])
    s[5] += cfg.dt * (bank - s[5]) / cfg.bank_tau
    s[6] += cfg.dt * (gamma - s[6]) / cfg.gamma_tau
    s[4] += cfg.dt * (speed - s[4]) / cfg.speed_tau
    s[3] += cfg.dt * 9.81 * np.tan(s[5]) / max(s[4], cfg.min_speed)
    s[3] = np.arctan2(np.sin(s[3]), np.cos(s[3]))
    s[:3] += cfg.dt * (s[4] * np.array([np.cos(s[6]) * np.cos(s[3]), np.cos(s[6]) * np.sin(s[3]), np.sin(s[6])]) + np.asarray(wind))
    return s


def guidance(state, target, cfg=None):
    cfg = cfg or Aircraft()
    d = np.asarray(target) - state[:3]
    error = np.arctan2(np.sin(np.arctan2(d[1], d[0]) - state[3]), np.cos(np.arctan2(d[1], d[0]) - state[3]))
    return np.array([np.clip(error * 1.2, -cfg.max_bank, cfg.max_bank), np.clip(np.arctan2(d[2], max(np.linalg.norm(d[:2]), 30)), -cfg.max_gamma, cfg.max_gamma), 22.])
