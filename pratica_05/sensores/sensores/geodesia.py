"""Conversão de coordenadas geográficas (GPS) para um plano local X, Y, Z.

O GPS mede (latitude, longitude, altitude) sobre o elipsoide WGS84. Para
controlar o robô precisamos de metros num referencial local. O caminho
usado aqui é o padrão em navegação:

    (lat, lon, alt)  ──►  ECEF (X, Y, Z)  ──►  ENU (Leste, Norte, Cima)

- ECEF (Earth-Centered, Earth-Fixed): cartesiano com origem no centro da Terra.
- ENU: plano tangente à Terra num ponto de referência (a origem do mundo).
  Com o heading do mundo igual a 0, Leste = eixo X e Norte = eixo Y do Gazebo,
  então o resultado fica no mesmo referencial do /odom.

Python puro (sem ROS), para poder ser testado isoladamente.
"""
import math

# Elipsoide WGS84
WGS84_A = 6378137.0                    # semi-eixo maior (m)
WGS84_F = 1.0 / 298.257223563          # achatamento
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)   # excentricidade ao quadrado


def geodetic_to_ecef(lat_deg, lon_deg, alt):
    """(lat, lon em graus; alt em m) -> (X, Y, Z) ECEF em metros."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    # Raio de curvatura no primeiro vertical
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat ** 2)
    x = (n + alt) * cos_lat * math.cos(lon)
    y = (n + alt) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt) * sin_lat
    return x, y, z


def ecef_to_enu(ecef, ref_lat_deg, ref_lon_deg, ref_ecef):
    """Ponto ECEF -> (leste, norte, cima) em relação ao ponto de referência."""
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    dx = ecef[0] - ref_ecef[0]
    dy = ecef[1] - ref_ecef[1]
    dz = ecef[2] - ref_ecef[2]
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


class LocalFrame:
    """Referencial local ENU com origem em (lat0, lon0, alt0)."""

    def __init__(self, lat0_deg, lon0_deg, alt0):
        self.lat0 = lat0_deg
        self.lon0 = lon0_deg
        self.alt0 = alt0
        self.ref_ecef = geodetic_to_ecef(lat0_deg, lon0_deg, alt0)

    def to_xyz(self, lat_deg, lon_deg, alt):
        """(lat, lon, alt) -> (x = leste, y = norte, z = cima) em metros."""
        ecef = geodetic_to_ecef(lat_deg, lon_deg, alt)
        return ecef_to_enu(ecef, self.lat0, self.lon0, self.ref_ecef)
