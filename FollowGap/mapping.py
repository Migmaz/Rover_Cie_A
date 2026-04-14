"""
MAPPING
Mémoire et position du robot.
 
Rôle :
- Estimer la position du robot par odométrie IMU
- Enregistrer la trajectoire (path memory)
 
Limites :
- Odométrie IMU uniquement → dérive progressive
- Acceptable pour missions courtes (< 5m, < 60s)
"""
 
import numpy as np
 
 
# ==================================================
# ÉTAT INTERNE (vitesse intégrée)
# ==================================================
 
# La vitesse est maintenue entre les itérations pour
# éviter de recalculer depuis l'accélération brute à chaque frame.
_vx = 0.0
_vy = 0.0
 
# Coefficient de friction virtuelle : réduit la dérive
# due au bruit de l'IMU quand le rover est quasi-immobile.
FRICTION = 0.98
 
 
# ==================================================
# POSITION
# ==================================================
 
def update_position(
    x: float,
    y: float,
    yaw: float,
    ax: float,
    dt: float,
    ay: float = 0.0
) -> tuple[float, float]:
    """
    Met à jour la position estimée du rover par odométrie IMU.
 
    Étapes :
    1. Intégration accélération → vitesse (repère rover)
    2. Rotation vitesse dans le repère monde (yaw)
    3. Intégration vitesse → position
 
    Args:
        x   (float) : Position actuelle X (m)
        y   (float) : Position actuelle Y (m)
        yaw (float) : Orientation du rover (rad)
        ax  (float) : Accélération longitudinale (m/s²) — axe avant du rover
        dt  (float) : Temps écoulé depuis la dernière mise à jour (s)
        ay  (float) : Accélération latérale (m/s²) — optionnel, défaut 0.0
 
    Returns:
        tuple[float, float] : Nouvelle position (new_x, new_y)
    """
    global _vx, _vy
 
    # Étape 1 : intégration accélération → vitesse (repère rover)
    v_forward = ax * dt
    v_lateral = ay * dt
 
    # Friction virtuelle pour limiter la dérive au repos
    _vx = (_vx + v_forward) * FRICTION
    _vy = (_vy + v_lateral) * FRICTION
 
    # Étape 2 : rotation dans le repère monde
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
 
    vx_world = cos_yaw * _vx - sin_yaw * _vy
    vy_world = sin_yaw * _vx + cos_yaw * _vy
 
    # Étape 3 : intégration vitesse → position
    new_x = x + vx_world * dt
    new_y = y + vy_world * dt
 
    return new_x, new_y
 
 
def reset_velocity():
    """
    Remet la vitesse interne à zéro.
    À appeler si le rover est connu immobile (ex: au démarrage).
    """
    global _vx, _vy
    _vx = 0.0
    _vy = 0.0
 
 
# ==================================================
# TRAJECTOIRE
# ==================================================
 
def save_path(path: list, x: float, y: float) -> list:
    """
    Enregistre la position courante dans la trajectoire.
 
    Args:
        path (list of tuple) : Trajectoire existante [(x, y), ...]
        x, y (float)         : Position courante
 
    Returns:
        list : Trajectoire mise à jour
    """
    path.append((x, y))
    return path