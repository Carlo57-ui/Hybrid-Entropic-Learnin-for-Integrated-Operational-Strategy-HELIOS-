# utils.py

# Este módulo contiene funciones auxiliares para convertir entre coordenadas
# geográficas (latitud/longitud) y coordenadas locales (metros),
# así como para calcular distancias geodésicas.
# Son herramientas esenciales para que el robot pueda:
# - medir la distancia entre dos puntos GPS,
# - proyectar desplazamientos en metros sobre coordenadas geográficas, y
# - convertir coordenadas locales (x, z) del robot a coordenadas GPS globales.

import math
R = 6378137.0

def haversine_m(lat1, lon1, lat2, lon2):
    """
    Calcula la distancia entre dos puntos geográficos (lat1, lon1) y (lat2, lon2)
    utilizando la fórmula de Haversine.
    Retorna la distancia en metros.

    Parámetros:
    - lat1, lon1: coordenadas del punto 1 (en grados)
    - lat2, lon2: coordenadas del punto 2 (en grados)

    Fórmula:
        a = sin²(Δφ/2) + cos φ₁ * cos φ₂ * sin²(Δλ/2)
        c = 2 * atan2(√a, √(1−a))
        d = R * c
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)  # Conversión a radianes
    dphi = math.radians(lat2 - lat1)                     # Diferencia de latitud
    dl = math.radians(lon2 - lon1)                       # Diferencia de longitud

    # Fórmula de Haversine para calcular la distancia sobre una esfera
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))  # Distancia final en metros


def meters_to_latlon(lat0, lon0, dx_east, dy_north):
    """
    Convierte un desplazamiento local en metros (dx, dy) a coordenadas GPS (lat, lon)
    relativas a un punto base (lat0, lon0).

    Parámetros:
    - lat0, lon0: coordenadas base en grados.
    - dx_east: desplazamiento hacia el ESTE en metros.
    - dy_north: desplazamiento hacia el NORTE en metros.

    Retorna:
    - (lat, lon): nuevas coordenadas GPS después del desplazamiento.
    """
    # Cambio en latitud: proporcional al desplazamiento norte y al radio terrestre
    lat = lat0 + (dy_north / R) * (180.0 / math.pi)

    # Cambio en longitud: depende de la latitud actual (corrección con cos(lat))
    lon = lon0 + (dx_east / (R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)

    return lat, lon


def local_to_global_latlon(q_lat, q_lon, theta_deg, local_list):
    """
    Convierte una lista de coordenadas locales (x, z) del robot a coordenadas GPS globales.

    Parámetros:
    - q_lat, q_lon: posición actual del robot en GPS (grados).
    - theta_deg: orientación YAW del robot (0° = norte, positivo en sentido horario → este).
    - local_list: lista de puntos [(x, z), ...] donde:
        x = desplazamiento lateral (derecha positiva) en metros,
        z = desplazamiento frontal (adelante positiva) en metros.

    Retorna:
    - out: lista de coordenadas GPS [(lat, lon), ...] transformadas desde el marco local.

    Explicación:
    Se realiza una rotación y traslación del marco local al marco global usando la orientación
    del robot. Luego, los desplazamientos se transforman a coordenadas geográficas reales.
    """
    out = []
    th = math.radians(theta_deg)  # Convertir orientación a radianes
    st, ct = math.sin(th), math.cos(th)

    for x, z in local_list:
        # Transformación de coordenadas locales a desplazamientos globales (E/N)
        # dx → eje Este, dy → eje Norte
        dx = z * st + x * ct
        dy = z * ct - x * st

        # Convertir desplazamientos locales (en metros) a latitud/longitud global
        out.append(meters_to_latlon(q_lat, q_lon, dx, dy))

    return out
