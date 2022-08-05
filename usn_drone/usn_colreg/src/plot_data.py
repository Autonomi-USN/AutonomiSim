#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

path_boat = '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/collision_boat.csv'

#algorithm configurations for comparison
path_los_0_0 =                      '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/LOS_0_0/seadrone.csv'
path_los_0_line_static =            '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/LOS_0_line_static/seadrone.csv'
path_los_still =                    '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/LOS_line_dynamic_line_static/seadrone.csv'
path_los_heavy =                    '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/heavy_waters/LOS_line_dynamic_line_static/seadrone.csv'

path_mpc_no_traj_still =                  '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/MPC_no_traj/seadrone.csv'
path_mpc_no_traj_heavy =                  '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/heavy_waters/MPC_no_traj/seadrone.csv'

path_mpc_traj_0 =                   '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/MPC_traj_0/seadrone.csv'
path_mpc_traj_init =                '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/still_waters/MPC_traj_init/seadrone.csv'




data_boat = np.array(pd.read_csv(path_boat))
line_destination = np.linspace([0, 0], [50, 0], 1000)


"get data from different los configurations"
#still waters
los_0_0 = np.array(pd.read_csv(path_los_0_0))
los_0_line_static = np.array(pd.read_csv(path_los_0_line_static))
mpc_traj_0 = np.array(pd.read_csv(path_mpc_traj_0))
mpc_traj_init = np.array(pd.read_csv(path_mpc_traj_init))

los_still = np.array(pd.read_csv(path_los_still))
los_heavy = np.array(pd.read_csv(path_los_heavy))

mpc_no_traj_still = np.array(pd.read_csv(path_mpc_no_traj_still))
mpc_no_traj_heavy = np.array(pd.read_csv(path_mpc_no_traj_heavy))



"plot trajectory of oncoming boat and start-end position"
fig, ax = plt.subplots() 
ax.plot(data_boat[:, 0], data_boat[:, 1], alpha=1, color='k')
#ax.plot(line_destination[:, 0], line_destination[:, 1], ':', color='g')
start = plt.Circle((0, 0), 0.4, color='k')
goal = plt.Circle((50, 0), 0.4, color='r')


"plot trajectories of los configurations"
w = 1 #linewidth
#ax.plot(los_0_0[:, 0], los_0_0[:, 1], '-.', linewidth=w, color='r', label='LOS: (0, 0) -> (0, 0)')
#ax.plot(los_0_line_static[:, 0], los_0_line_static[:, 1],'-.', linewidth=w, color='b', label='LOS: (0,0) -> init')
#ax.plot(mpc_traj_0[:, 0], mpc_traj_0[:, 1], label='MPC: trajectory (0, 0)')
#ax.plot(mpc_traj_init[:, 0], mpc_traj_boat[:, 1], label='MPC: trajectory boat init')

ax.plot(los_still[:, 0], los_still[:, 1],'-.', linewidth=w, color='k', label='LOS: still waters')
ax.plot(los_heavy[:, 0], los_heavy[:, 1],'-', linewidth=w, color='c', label='LOS: heavy waters')

ax.plot(mpc_no_traj_still[:, 0], mpc_no_traj_still[:, 1],'-.', color='k', label='mpc: still waters')
ax.plot(mpc_no_traj_heavy[:, 0], mpc_no_traj_heavy[:, 1],'-', color='r', label='mpc: heavy waters')






"draw arrows showing direction of oncoming boat"
for i in range(-20, 20, 8):
    ax.arrow(20, i, 0, 1, width=0.2, color='k', alpha=1)

ax.add_patch(start)
ax.add_patch(goal)
# ax.grid()


"display plot"
ax.set_ylim([-20, 20])
ax.legend()
plt.show(ax)
