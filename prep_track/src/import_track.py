import numpy as np
from parameters import trk

def import_track (file_path: str) -> np.ndarray:

    track_csv_data= np.loadtxt(file_path, comments='#', delimiter=',')

    center_line= track_csv_data[:, 0:2]
    width_right= track_csv_data[:, 2]
    width_left= track_csv_data[:, 3]
    grad= track_csv_data[:, 4]

    center_line= np.tile(center_line, (trk.lap, 1))
    width_right= np.tile(width_right, trk.lap)
    width_left= np.tile(width_left, trk.lap)
    grad= np.tile(grad, trk.lap)

    track_raw= np.column_stack((center_line, width_right, width_left, grad)) # the track is open

    if trk.reverse:
        track_raw= np.flipud(track_raw)

    return track_raw