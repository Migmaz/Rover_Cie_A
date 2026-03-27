"""
SENSOR
Gestion des capteurs.

Rôle :
- Lire les données du LiDAR (scan)
- Lire les données IMU (yaw / orientation)
- (Optionnel) Estimer la position (odometry)

Sorties standard :
- scan : liste de distances
- yaw : orientation du robot
"""

def get_lidar_scan():
    """
    Retourne un scan LiDAR.

    Returns:
        scan (list of float):
            Liste de distances en mètres.
            Taille typique : 180 à 360 points.
            Chaque index correspond à un angle relatif au robot.
            Exemple: scan[0] = -90°, scan[len/2] = 0°, scan[-1] = +90°
    """
    pass


def get_yaw():
    """
    Retourne l'orientation du robot.

    Returns:
        yaw (float):
            Angle en radians.
            0 = direction initiale du robot.
            Positif = rotation gauche (CCW)
            Négatif = rotation droite (CW)
    """
    pass