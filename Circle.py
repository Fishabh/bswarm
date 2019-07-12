#%%
import sys
import os
sys.path.insert(0, os.getcwd())

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import bswarm.trajectory as tgen
import bswarm.formation as form
import bswarm
import json

#%% Takeoff formation.

scale = 1
Takeoff = np.array([
    [1, 0, 1],
    [1, 1, 1],
    [-1, 1, 1],
    [-1, 0, 1],
    [-1, -1, 1],
    [1, -1, 1],
]).T
Takeoff[:, ]
plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(Takeoff[0, :], Takeoff[1, :], Takeoff[2, :], 'ro')
plt.title('Takeoff')
plt.show()

Land = np.array([
    [1, 0, .5],
    [1, 1, .5],
    [-1, 1, .5],
    [-1, 0, .5],
    [-1, -1, .5],
    [1, -1, .5],
]).T

#%% A circle formation.

theta = 0
circle = np.array([
    [np.cos(np.deg2rad(theta)), np.sin(np.deg2rad(theta)), 1],
    [np.cos(np.deg2rad(theta+60)), np.sin(np.deg2rad(theta+60)), 1],
    [np.cos(np.deg2rad(theta+120)), np.sin(np.deg2rad(theta+120)), 1],
    [np.cos(np.deg2rad(theta+180)), np.sin(np.deg2rad(theta+180)), 1],
    [np.cos(np.deg2rad(theta+240)), np.sin(np.deg2rad(theta+240)), 1],
    [np.cos(np.deg2rad(theta+300)), np.sin(np.deg2rad(theta+300)), 1],
]).T

plt.figure()
ax = plt.axes(projection='3d')
#ax.plot3D(P[0, :], P[1, :], P[2, :], 'ro')
ax.plot3D(circle[0, :], circle[1, :], circle[2, :], 'bo')
plt.title('circle')
plt.show()

#%% Create waypoints for flat P -> slanted P -> rotating slanted P -> flat P
waypoints = [Takeoff]

while theta <= 720:
    circle = np.array([
        [np.cos(np.deg2rad(theta)), np.sin(np.deg2rad(theta)), 1],
        [np.cos(np.deg2rad(theta+60)), np.sin(np.deg2rad(theta+60)), 1],
        [np.cos(np.deg2rad(theta+120)), np.sin(np.deg2rad(theta+120)), 1],
        [np.cos(np.deg2rad(theta+180)), np.sin(np.deg2rad(theta+180)), 1],
        [np.cos(np.deg2rad(theta+240)), np.sin(np.deg2rad(theta+240)), 1],
        [np.cos(np.deg2rad(theta+300)), np.sin(np.deg2rad(theta+300)), 1],
    ]).T
    theta= theta+40
    waypoints.append(circle)
waypoints.append(Takeoff)
waypoints.append(Land)
waypoints = np.array(waypoints)

plt.figure()
ax = plt.axes(projection='3d')
for point in range(waypoints.shape[2]):
    ax.plot3D(waypoints[:, 0, point], waypoints[:, 1, point], waypoints[:, 2, point], '-')
    ax.view_init(azim=0, elev=40)
plt.title('waypoints')
plt.show()

#%% plan trajectories
dist = np.linalg.norm(waypoints[1:, :, :] - waypoints[:-1, :, :], axis=1)
dist_max = np.max(dist, axis=1)
dist_max

trajectories = []

T = 10*np.ones(len(dist_max))

origin = np.array([1.5, 2, 0])

for drone in range(waypoints.shape[2]):
    pos_wp = waypoints[:, :, drone] + origin
    yaw_wp = np.zeros((pos_wp.shape[0], 1))
    traj = tgen.min_snap_4d(
        np.hstack([pos_wp, yaw_wp]), T, stop=True)
    trajectories.append(traj)

tgen.plot_trajectories_3d(trajectories)
tgen.trajectories_to_json(trajectories, 'scripts/data/circle_form.json')
plt.show()


#%%
tgen.plot_trajectory_derivatives(trajectories[0])
print('T', T)

#%%
