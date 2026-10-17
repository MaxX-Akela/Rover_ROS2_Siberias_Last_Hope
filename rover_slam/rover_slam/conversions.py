"""ENU/FLU (ROS, 'map' frame) -> NED/FRD (PX4) for the SLAM pose (pure functions, no ROS imports).

Inverse of rover_fcu_bridge.conversions.ned_to_enu / attitude_enu_flu. Quaternions are (w, x, y, z).
"""
from rover_fcu_bridge.conversions import Q_ENU_NED, Q_FRD_FLU, quat_conj, quat_mul


def enu_to_ned(v):
    return (v[1], v[0], -v[2])


def attitude_ned_frd(q_enu_flu):
    """Body FLU -> world ENU quaternion  =>  body FRD -> world NED quaternion."""
    return quat_mul(quat_mul(quat_conj(Q_ENU_NED), q_enu_flu), quat_conj(Q_FRD_FLU))
