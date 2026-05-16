import matplotlib
# Mode Headless pour éviter l'erreur "Connection refused"
matplotlib.use('Agg') 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# ==========================================
# 1. ARBRES SOURCÈS ORIENTÉS (GAUCHE ET DROITE)
# ==========================================
ORIGINAL_TSTART = [
    (0, 5),     # 0: Racine Gauche (Départ Robot)
    (10, 8),    # 1: Enfant de 0
    (8, 1),     # 2: Enfant de 0 (Branche morte bas)
    (18, 9),    # 3: Enfant de 1
    (15, 4),    # 4: Enfant de 1 (Branche morte milieu)
    (25, 6)     # 5: Point de jonction final côté Start
]
Tstart_idx = [(0, 0), (0, 1), (0, 2), (1, 3), (1, 4), (3, 5)]

ORIGINAL_TGOAL = [
    (50, 5),    # 0: Racine Droite (But final)
    (40, 3),    # 1: Enfant de 0
    (42, 9),    # 2: Enfant de 0 (Branche morte haut)
    (32, 4),    # 3: Enfant de 1
    (34, 8),    # 4: Enfant de 1 (Branche morte milieu)
    (25, 6)     # 5: Point de jonction final côté Goal
]
Tgoal_idx = [(0, 0), (0, 1), (0, 2), (1, 3), (1, 4), (3, 5)]


# ==========================================
# 2. LOGIQUE RECONSTRUCT_PATH STRICTE
# ==========================================
def search_path_stepbystep(T, T_idx, reverse_input=False):
    if reverse_input:
        T = list(reversed(T))
        T_idx = list(reversed(T_idx)) # Tu inverses bien les deux ici !
        
    path = list(T)
    idx = list(T_idx)
    
    optimal_path = [path[-1], path[-2]]
    optimal_path_idx = [idx[-1], idx[-2]]
    path = path[:-2]
    idx = idx[:-2]
    
    states = [(list(path), list(idx), list(optimal_path))]
    
    while optimal_path_idx[-1] != (0, 0):
        len_p = len(path) - 1
        parent_trouve = False
        
        for i in range(len_p, -1, -1):
            if idx[i][1] == optimal_path_idx[-1][0]:
                optimal_path.append(path[i])
                optimal_path_idx.append(idx[i])
                states.append((list(path), list(idx), list(optimal_path)))
                
                seuil_id = idx[i][0]
                
                for j in range(len_p, -1, -1):
                    if idx[j][0] >= seuil_id and idx[j] != (0, 0):
                        del path[j]
                        del idx[j]
                        states.append((list(path), list(idx), list(optimal_path)))
                
                parent_trouve = True
                break
                
        if not parent_trouve:
            break
            
    states.append((list(path), list(idx), list(optimal_path)))
    return states


# ==========================================
# 3. PRÉPARATION DU SCÉNARIO DE L'ANIMATION
# ==========================================
states_start = search_path_stepbystep(ORIGINAL_TSTART, Tstart_idx, reverse_input=True)
states_goal = search_path_stepbystep(ORIGINAL_TGOAL, Tgoal_idx, reverse_input=False)

frames_data = []
Tstart_reversed = list(reversed(ORIGINAL_TSTART))

# Phase 1 : Élagage de Tstart (avec son référentiel inversé cohérent)
for p, i, opt in states_start:
    frames_data.append({
        'current_start_ref': Tstart_reversed,
        'start_idx': i, 'start_path': p,
        'goal_idx': list(Tgoal_idx[:-2]), 'goal_path': list(ORIGINAL_TGOAL[:-2]),
        'optimal': opt
    })

# Phase 2 : Élagage de Tgoal
final_start_p = states_start[-1][0]
final_start_i = states_start[-1][1]
base_optimal = states_start[-1][2]

for p, i, opt in states_goal:
    frames_data.append({
        'current_start_ref': Tstart_reversed,
        'start_idx': final_start_i, 'start_path': final_start_p,
        'goal_idx': i, 'goal_path': p,
        'optimal': base_optimal + opt
    })


# ==========================================
# 4. TRACÉ ET RENDU GRAPHIQUE DYNAMIQUE
# ==========================================
fig, ax = plt.subplots(figsize=(12, 6))
ax.set_title("RRT-Connect : Animation de la réduction de chemin", fontsize=12, fontweight='bold')
ax.set_xlim(-5, 55)
ax.set_ylim(-2, 12)
ax.grid(True, linestyle='--', alpha=0.5)

scat_start = ax.scatter([], [], color='dodgerblue', s=80, edgecolors='black', label='Noeuds actifs Tstart', zorder=3)
scat_goal = ax.scatter([], [], color='crimson', s=80, edgecolors='black', label='Noeuds actifs Tgoal', zorder=3)
scat_opt = ax.scatter([], [], color='gold', s=120, marker='*', edgecolors='black', label='Noeuds du chemin', zorder=5)
ax.legend(loc="upper right")

def update(frame_idx):
    ax.lines.clear()
    frame = frames_data[frame_idx]
    
    # 1. Dessin des lignes de Tstart en se basant dynamiquement sur l'état des index restants
    start_ref = frame['current_start_ref']
    for p_idx_in_frame, (p_id, n_id) in enumerate(frame['start_idx']):
        if p_id != n_id:
            # On récupère le point courant (qui correspond à la position actuelle dans la liste filtrée)
            n_coor = frame['start_path'][p_idx_in_frame]
            
            # On cherche les coordonnées du parent dans la liste actuelle
            p_coor = None
            for lookup_idx, (_, lookup_nid) in enumerate(frame['start_idx']):
                if lookup_nid == p_id:
                    p_coor = frame['start_path'][lookup_idx]
                    break
            
            # Si le parent n'a pas été élagué, on trace le segment
            if p_coor is not None:
                ax.plot([p_coor[0], n_coor[0]], [p_coor[1], n_coor[1]], color='skyblue', linewidth=2, alpha=0.6, zorder=2)
            # Si le parent a été élagué mais qu'il fait partie de la racine stable, on se rabat sur le référentiel d'origine
            elif p_id < len(start_ref):
                p_coor = start_ref[p_id]
                ax.plot([p_coor[0], n_coor[0]], [p_coor[1], n_coor[1]], color='skyblue', linewidth=2, alpha=0.6, zorder=2)
            
    # 2. Dessin des lignes de Tgoal
    for p_idx_in_frame, (p_id, n_id) in enumerate(frame['goal_idx']):
        if p_id != n_id:
            n_coor = frame['goal_path'][p_idx_in_frame]
            p_coor = None
            for lookup_idx, (_, lookup_nid) in enumerate(frame['goal_idx']):
                if lookup_nid == p_id:
                    p_coor = frame['goal_path'][lookup_idx]
                    break
            if p_coor is not None:
                ax.plot([p_coor[0], n_coor[0]], [p_coor[1], n_coor[1]], color='lightcoral', linewidth=2, alpha=0.6, zorder=2)
            elif p_id < len(ORIGINAL_TGOAL):
                p_coor = ORIGINAL_TGOAL[p_id]
                ax.plot([p_coor[0], n_coor[0]], [p_coor[1], n_coor[1]], color='lightcoral', linewidth=2, alpha=0.6, zorder=2)

    # 3. Sauvegarde et tracé robuste du chemin conservé (Ligne continue verte)
    current_opt_nodes = frame['optimal']
    if len(current_opt_nodes) > 1:
        x_coords = [pt[0] for pt in current_opt_nodes]
        y_coords = [pt[1] for pt in current_opt_nodes]
        ax.plot(x_coords, y_coords, color='limegreen', linewidth=4, linestyle='-', zorder=4)

    # 4. Actualisation des positions des points (Ronds & Étoiles)
    if frame['start_path']: scat_start.set_offsets(frame['start_path'])
    else: scat_start.set_offsets(np.empty((0, 2)))
        
    if frame['goal_path']: scat_goal.set_offsets(frame['goal_path'])
    else: scat_goal.set_offsets(np.empty((0, 2)))
        
    if current_opt_nodes: scat_opt.set_offsets(current_opt_nodes)
    else: scat_opt.set_offsets(np.empty((0, 2)))
        
    return scat_start, scat_goal, scat_opt

# Compilation finale
ani = animation.FuncAnimation(fig, update, frames=len(frames_data), interval=400, repeat=False)
output_filename = "/home/benjamin/ros2_ws/src/IN425/in425_nav/in425_nav/rrt_connect_test.gif"

ani.save(output_filename, writer='pillow', fps=3)
print(f"\n[SUCCÈS] L'animation synchronisée avec ton double 'reversed' est prête : {output_filename}")