"""
SENSOR
Gestion des capteurs.

Rôle :
- Lire les données du LiDAR (scan)
- Lire les données IMU (yaw / orientation)

Sorties standard :
- scan : np.ndarray [distance, angle] Nx2
- yaw  : orientation du robot (rad)
- a    : accélération linéaire (3x1)
"""

import numpy as np

# Guard pour permettre les tests sans matériel
try:
    import busio
    import board
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GAME_ROTATION_VECTOR,
    )
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
 
 
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
 
 
# ==================================================
# IMU
# ==================================================

def IMU(bno) -> tuple[np.ndarray, float]:
    """
    Lit l'accélération linéaire et le cap (yaw) depuis le BNO08x.

    Args:
        bno : instance BNO08X_I2C déjà initialisée.

    Returns:
        tuple:
            - a   (np.ndarray): Accélération linéaire [ax, ay, az] (3x1), m/s²
            - yaw (float)     : Orientation (rad), dans [-pi, pi]
                                Retourne 0.0 en cas d'erreur.
    """
    try:
        ax, ay, az = bno.linear_acceleration
        qi, qj, qk, qr = bno.game_quaternion   # GAME_ROTATION_VECTOR, sans magnéto

    except OSError as e:
        print(f"[IMU] Erreur I2C : {e}")
        return np.zeros((3, 1)), 0.0

    except RuntimeError as e:
        print(f"[IMU] Données non disponibles : {e}")
        return np.zeros((3, 1)), 0.0

    # Yaw depuis quaternion (rotation autour de Z)
    yaw = np.arctan2(
        2.0 * (qr * qk + qi * qj),
        1.0 - 2.0 * (qj**2 + qk**2)
    )

    a = np.array([[ax], [ay], [az]])

    return a, float(yaw)
 