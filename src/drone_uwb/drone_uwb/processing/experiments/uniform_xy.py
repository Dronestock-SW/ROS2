"""Baseline A's strict four-anchor API; shared arithmetic lives in range_solver."""
from drone_uwb.processing.range_solver import RangeFit as UniformFit, solve_slant_xy


def solve_uniform_xy(anchors, ranges, z_m, max_iterations=40, step_tol_m=1e-7,
                     condition_max=1e6):
    return solve_slant_xy(anchors, ranges, z_m, max_iterations, step_tol_m,
                          condition_max, required_count=4)
