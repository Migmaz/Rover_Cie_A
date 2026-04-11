"""
CONTROL (FSM)
Gestion des états du robot.
 
Rôle :
- Décider quel comportement utiliser
- Gérer les transitions entre états
 
Exemples d’états :
- NAVIGATE
- SCAN
- ESCAPE
- RETURN
 
Entrées : données capteurs
Sortie : nom du behavior à exécuter
"""
 
# Ludo : À modifier les schémas de fonctions car peu clair et pas adapter
 
import numpy as np
from dataclasses import dataclass, field
from collections import deque
 
 
# =========================
# ===== ROBOT STATE =======
# =========================
 
@dataclass
class RobotState:
    prev_state: str = "NAVIGATE"
 
    # Historique
    theta_history: deque = field(default_factory=lambda: deque(maxlen=10))
    # FIX #2 — progrès accumulé sur fenêtre glissante (indépendant de la fréquence d'appel)
    progress_history: deque = field(default_factory=lambda: deque(maxlen=10))
    prev_dist_to_goal: float = None
 
    # Stuck detection
    stuck_counter: int = 0
 
    # Timers
    escape_timer: int = 0
    scan_timer: int = 0
 
 
# =========================
# ===== MAIN FSM ==========
# =========================
 
def update_state(
    scan: np.ndarray,
    theta_goal: float,
    robot_pos: tuple,
    goal_pos: tuple,
    state: RobotState,
    has_valid_gap: bool = True
) -> str:
 
    distances = scan[:, 0]
    angles = scan[:, 1]
 
    # =========================
    # === MASQUE LOCAL =========
    # =========================
 
    front_mask = np.abs(angles) < np.pi / 2
    local_mask = front_mask & (distances < 1.2)
 
    front_dist = distances[front_mask]
    local_dist = distances[local_mask]
 
    if len(front_dist) == 0:
        return "STOP"
 
    # =========================
    # === STOP (sécurité) ======
    # =========================
 
    if np.percentile(front_dist, 10) < 0.2:
        state.prev_state = "STOP"
        state.escape_timer = 8
        return "STOP"
 
    # =========================
    # === ESCAPE (prioritaire)
    # =========================
 
    if state.escape_timer > 0 or not has_valid_gap:
        state.escape_timer = max(state.escape_timer, 6)
        state.escape_timer -= 1
        state.prev_state = "ESCAPE"
        return "ESCAPE"
 
    # =========================
    # === PROGRÈS VERS GOAL ===
    # =========================
 
    dist_to_goal = np.linalg.norm(np.array(goal_pos) - np.array(robot_pos))
 
    if state.prev_dist_to_goal is None:
        state.prev_dist_to_goal = dist_to_goal
 
    progress = state.prev_dist_to_goal - dist_to_goal
    state.prev_dist_to_goal = dist_to_goal
 
    # FIX #2 — on accumule le progrès dans un historique glissant.
    # La somme représente la distance totale parcourue sur la fenêtre,
    # ce qui est invariant à la fréquence d'appel (contrairement au delta par frame).
    state.progress_history.append(progress)
    cumulative_progress = sum(state.progress_history)
 
    # =========================
    # === HISTORIQUE ANGLE ====
    # =========================
 
    if theta_goal is not None:
        state.theta_history.append(theta_goal)
 
    # FIX #1 — variance circulaire.
    # np.var sur des angles est invalide près de ±π : deux angles quasi-identiques
    # comme -π+0.1 et π-0.1 produisent une variance maximale au lieu de zéro.
    # La variance circulaire (1 - |mean(e^{jθ})|) est toujours dans [0, 1]
    # et traite correctement le repliement à ±π.
    if len(state.theta_history) > 1:
        thetas = np.array(state.theta_history)
        theta_var = 1.0 - np.abs(np.mean(np.exp(1j * thetas)))
    else:
        theta_var = 0.0
 
    # =========================
    # === ENVIRONNEMENT LOCAL ==
    # =========================
 
    if len(local_dist) > 0:
        free_ratio = np.sum(local_dist > 0.8) / len(local_dist)
    else:
        free_ratio = 0.0
 
    is_blocked = np.percentile(front_dist, 30) < 0.5
 
    # =========================
    # === CUL-DE-SAC ==========
    # =========================
 
    # FIX #4 — la condition originale exigeait theta_var élevée, ce qui exclut
    # le blocage frontal simple (mur droit devant → theta_var ≈ 0).
    # On retire theta_var de la condition nécessaire et on utilise le progrès
    # cumulé (FIX #2) comme indicateur principal.
    if cumulative_progress < 0.05 and is_blocked:
        state.stuck_counter += 1
    else:
        # FIX #3 — décrément graduel au lieu d'un reset brutal.
        # Un reset total sur une seule frame favorable efface toute l'accumulation
        # dans un environnement bruité. Le décrément graduel est bien plus robuste.
        state.stuck_counter = max(0, state.stuck_counter - 1)
 
    if state.stuck_counter > 5:
        state.escape_timer = 10
        state.stuck_counter = 0  # reset pour éviter un re-déclenchement immédiat
        state.prev_state = "CUL-DE-SAC"
        return "CUL-DE-SAC"
 
    # =========================
    # === SCAN (mission) ======
    # =========================
 
    # FIX #5 — initialisation et décrément séparés.
    # Dans l'original, le décrément était à l'intérieur du bloc `dist_to_goal < 1.0`,
    # donc si le robot dérivait au-delà de 1 m pendant le scan, le timer se gelait
    # et scan_timer == 0 ne se produisait jamais → RETOUR_BASE jamais déclenché.
    if dist_to_goal < 1.0 and state.scan_timer == 0:
        state.scan_timer = 20
 
    if state.scan_timer > 0:
        state.scan_timer -= 1
        state.prev_state = "SCAN"
        return "SCAN"
 
    # =========================
    # === RETOUR BASE =========
    # =========================
 
    if state.prev_state == "SCAN" and state.scan_timer == 0:
        state.prev_state = "RETOUR_BASE"
        return "RETOUR_BASE"
 
    # =========================
    # === NAVIGATE ============
    # =========================
 
    state.prev_state = "NAVIGATE"
    return "NAVIGATE"