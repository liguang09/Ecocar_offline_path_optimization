#### import_track
- Convert csv data to array for calculation
- return: raw data of track (track_raw)

#### smooth_track
- inputs: raw track data
- Linear interpolation of the raw track
- Spline interpolation after linear interpolation
- return: smoothed track

#### track_length
- inputs: track data
- calculate the euclidean distance between each two points
- accumulate the euclidean distances
- return: s_seg, s_cumsum

#### cal_curvature
- inputs: track data, track lengths
- calculate the curvature dtheta/ds



