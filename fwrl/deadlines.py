"""Generous training deadlines based on slow flight, climb and maneuver margin."""
import math


def gate_budgets(world):
    slow_speed = world['aerodynamics'].get('stall_speed_mps',50*.44704)
    points=world['waypoints']
    return [20.]+[max(120.,30.+2*math.dist(a,b)/slow_speed+max(0,b[2]-a[2])/3.)
                  for a,b in zip(points,points[1:])]
