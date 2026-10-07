"""Shared offline error statistics; absent observations remain absent."""
import numpy as np


def error_stats(values, target_m=None):
    data = np.asarray([v for v in values if v is not None], dtype=float)
    result = dict(count=len(data), missing_count=len(values)-len(data))
    if not len(data):
        return dict(result, median_m=None, rmse_m=None, p95_m=None, max_m=None,
                    at_or_above_target_count=None, at_or_above_target_ratio=None)
    result.update(median_m=float(np.median(data)), rmse_m=float(np.sqrt(np.mean(data**2))),
                  p95_m=float(np.quantile(data, .95)), max_m=float(np.max(data)))
    if target_m is not None:
        count = int(np.count_nonzero(data >= target_m))
        result.update(target_m=target_m, at_or_above_target_count=count,
                      at_or_above_target_ratio=count/len(data))
    return result
