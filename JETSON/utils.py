# utils.py
import math
R = 6378137.0

def haversine_m(lat1, lon1, lat2, lon2):
    phi1,phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2-lat1)
    dl = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dl/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def meters_to_latlon(lat0, lon0, dx_east, dy_north):
    lat = lat0 + (dy_north / R) * (180.0/math.pi)
    lon = lon0 + (dx_east / (R * math.cos(math.radians(lat0)))) * (180.0/math.pi)
    return lat, lon

def local_to_global_latlon(q_lat, q_lon, theta_deg, local_list):
    """local_list: list of (x, z) where z=forward (m), x=right (m).
       theta_deg: yaw where 0 = north, positive clockwise (east)."""
    out=[]
    th = math.radians(theta_deg)
    st,ct = math.sin(th), math.cos(th)
    for x,z in local_list:
        # dx East, dy North
        dx = z*st + x*ct
        dy = z*ct - x*st
        out.append(meters_to_latlon(q_lat, q_lon, dx, dy))
    return out
