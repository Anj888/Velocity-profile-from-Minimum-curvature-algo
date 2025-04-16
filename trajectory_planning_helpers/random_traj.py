import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
from calc_splines import calc_splines
from interp_splines import interp_splines
from calc_spline_lengths import calc_spline_lengths
from calc_vel_profile import calc_vel_profile
from calc_head_curv_an import calc_head_curv_an
from import_veh_dyn_info import import_veh_dyn_info

df = pd.read_csv('C:\\Users\\maila\\OneDrive\\Desktop\\AGV\\Really AGV\\trajectory_planning_helpers\\trajectory_planning_helpers\\Spielberg_waypoints.csv', header=None)
coord=df.values

def calculate_element_lengths(coords: np.ndarray) -> np.ndarray:
    
    deltas = np.diff(coords, axis=0)
    
    
    distances = np.sqrt(np.sum(deltas**2, axis=1))
    
    return distances
el_lengths=calculate_element_lengths(coord)
start_vec = coord[1] - coord[0]
end_vec = coord[-1] - coord[-2]
psi_s = np.arctan2(start_vec[1], start_vec[0])
psi_e = np.arctan2(end_vec[1], end_vec[0])
coeffs_x, coeffs_y, M, normvec_normalized=calc_splines(coord , el_lengths,psi_s, psi_e,True) 
spline_lengths=calc_spline_lengths(coeffs_x, coeffs_y)
path_interp, spline_inds, t_values, dists_interp=interp_splines(coeffs_x, coeffs_y, spline_lengths,stepsize_approx=0.1, stepnum_fixed=None) 

psi, kappa= calc_head_curv_an(coeffs_x,
                      coeffs_y,
                      spline_inds,
                      t_values)

ggv, ax_max_machines=import_veh_dyn_info( 'C:\\Users\\maila\\OneDrive\\Desktop\\AGV\\Really AGV\\trajectory_planning_helpers\\trajectory_planning_helpers\\ggv.csv',
                        'C:\\Users\\maila\\OneDrive\\Desktop\\AGV\\Really AGV\\trajectory_planning_helpers\\trajectory_planning_helpers\\ax_max_machines.csv')
el_lengths_interp = calculate_element_lengths(path_interp)

print(kappa.size)
print(el_lengths_interp.size)
vx_profile =calc_vel_profile(ax_max_machines,
                     kappa,
                     el_lengths_interp,
                     closed=False,
                     drag_coeff=0.5,
                     m_veh= 3,
                     ggv=ggv ,
                     loc_gg=None,
                     v_max= 6.3,
                     dyn_model_exp = 1.0,
                     mu = None,
                     v_start= 0,
                     v_end= 6.3,
                     filt_window = None) 

cum_distances = np.insert(np.cumsum(el_lengths_interp), 0, 0) 

plt.figure(figsize=(10, 6))
plt.plot(cum_distances, vx_profile, marker='o', linestyle='-', color='b')
plt.xlabel('Element Lengths (Distances between coordinates)')
plt.ylabel('Velocity Profile (vx_profile)')
plt.title('Velocity Profile vs. Element Lengths')
plt.grid(True)
plt.show()