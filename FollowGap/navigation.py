"""
NAVIGATION
Fonctions algorithmiques pures.

Rôle :
- Calculer la direction d’évitement (Follow-the-Gap)
- Calculer la direction vers la cible (angle cible)
- Fournir des outils de navigation réutilisables

Important :
- Ne contrôle PAS directement le robot
- Ne dépend PAS des états (FSM)

Entrées : données capteurs
Sorties : angles (directions)
"""

def follow_the_gap(scan, threshold=0.8):
    """
    Calcule la meilleure direction pour éviter les obstacles.

    Args:
        scan (list of float):
            Distances LiDAR en mètres.
        threshold (float):
            Distance minimale pour considérer un espace libre.

    Returns:
        angle_gap (float):
            Angle en radians relatif au robot.
            0 = devant, + = gauche, - = droite
    """
    pass


def compute_target_angle(yaw, direction_initiale=0.0):
    """
    Calcule l'angle vers la direction cible.

    Args:
        yaw (float):
            Orientation actuelle du robot (rad).
        direction_initiale (float):
            Direction souhaitée au départ (rad).

    Returns:
        angle_cible (float):
            Angle relatif au robot pour revenir vers la cible.
    """
    pass