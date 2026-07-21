"""Fixtures for thruster parameter loader smoke checks."""

from __future__ import annotations


def thruster_param_maps():
    return {
        "global": {},
        "scale": {"t1": 9.0, "t2": 9.0},
        "direct": {"t1": 9.0, "t2": 9.0},
        "reverse": {"t1": 0.5, "t2": 0.5},
        "tau_up": {"t1": 0.5, "t2": 0.5},
        "tau_down": {"t1": 0.5, "t2": 0.5},
    }


def thruster_param_payload():
    return {
        "global": {"gain_scale_all": 2.0, "direct_gain_scale_all": 3.0, "deadzone": 0.04},
        "per_thruster": {
            "t1": {"gain_scale": 1.5, "direct_gain_scale": 0.5, "reverse_asymmetry": 0.8, "tau_up": 0.2},
            "missing": {"gain_scale": 10.0},
        },
    }


__all__ = ["thruster_param_maps", "thruster_param_payload"]
