import numpy as np

def track_length(track: np.ndarray)-> np.ndarray:

    track_xy= track[:, :2]
    diff_xy= np.power(np.diff(track_xy, axis=0), 2)
    s_segment= np.sqrt(np.sum(diff_xy, axis=1))

    s_cumsum= np.cumsum(s_segment)

    return s_cumsum, s_segment