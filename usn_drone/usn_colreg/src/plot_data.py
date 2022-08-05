#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

path_boat = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/LOS_0_0/boat.csv'
#algorithm configurations for comparison
path_los_0_0 = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/LOS_0_0/seadrone.csv'
path_los_0_line_static = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/LOS_0_line_static/seadrone.csv'
path_los_line_dynamic_line_static = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/LOS_line_dynamic_line_static/seadrone.csv'
path_mpc_no_traj = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/MPC_no_traj/seadrone.csv'

data_boat = np.array(pd.read_csv(path_boat))
line_destination = np.linspace([0, 0], [50, 0], 1000)


"get data from different los configurations"
los_0_0 = np.array(pd.read_csv(path_los_0_0))
los_0_line_static = np.array(pd.read_csv(path_los_0_line_static))
los_dynamic_static = np.array(pd.read_csv(path_los_line_dynamic_line_static))
mpc_no_traj = np.array(pd.read_csv(path_mpc_no_traj))


fig, ax = plt.subplots() 
"plot trajectory of oncoming boat and start-end position"
ax.plot(data_boat[:, 0], data_boat[:, 1], alpha=1, color='k')
ax.plot(line_destination[:, 0], line_destination[:, 1], ':', color='g')

w = 1

"plot trajectories of los configurations"
ax.plot(los_0_0[:, 0], los_0_0[:, 1], '-', linewidth=w, color='r', label='LOS: (0, 0) -> (0, 0)')
ax.plot(los_0_line_static[:, 0], los_0_line_static[:, 1],'-', linewidth=w, color='b', label='LOS: (0,0) -> init')
ax.plot(los_dynamic_static[:, 0], los_dynamic_static[:, 1],'-', linewidth=w, color='c', label='LOS: current / current')
#ax.plot(mpc_no_traj[:, 0], mpc_no_traj[:, 1], color='m', label='MPC: standard')
ax.grid()



"draw arrows showing direction of oncoming boat"
for i in range(-20, 20, 8):
    ax.arrow(20, i, 0, 1, width=0.2, color='k', alpha=1)

ax.set_ylim([-20, 20])
ax.legend()
plt.show(ax)
