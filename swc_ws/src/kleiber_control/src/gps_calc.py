"""
For all GPS related calculations
"""

from math import sin, cos, atan2, radians, pi, sqrt

def measure_gps(lat1, lon1, lat2, lon2):
    """ Haversine formula for finding distance between two GPS points """
    R = 6378.137; # Radius of earth in KM
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)
    a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    return d * 1000 # meters