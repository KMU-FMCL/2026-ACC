# PH quintic curve segment defined by G1 Hermite data.
# A subscriber reconstructs the full curve via Quintic::Interpolate with these fields.


# G1 Hermite data: start/end positions in map frame [m] (z unused, planar curve)
geometry_msgs/Point start_point
geometry_msgs/Point end_point

# Unit tangent vectors in map frame (z unused, planar curve)
geometry_msgs/Vector3 start_tangent
geometry_msgs/Vector3 end_tangent

# Solution branch (1-4); 0 = auto-selected (shortest arc length)
int32 branch

# Exact polynomial arc length [m]
float64 arc_length
