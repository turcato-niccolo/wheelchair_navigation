import pickle as pkl
import matplotlib.pyplot as plt

files = ['move_base_wheelchair_trajectory.pkl', 'MPC_wheelchair_trajectory.pkl']
lbls = ['move_base', 'MPC']

trajectories = [pkl.load(open(file, 'rb')) for file in files]

plt.figure('2d trajectory comparison')
# for trj_np, lbl in zip(trajectories, lbls):
#     plt.scatter(trj_np[:500, 1], trj_np[:500, 2], label=lbl)
plt.scatter(trajectories[0][320:400+320, 1], trajectories[0][320:400+320, 2], label=lbls[0])
plt.scatter(trajectories[1][:400, 1], trajectories[1][:400, 2], label=lbls[1])
plt.grid()
plt.legend()


fig, axes = plt.subplots(2, 1, sharex=True)

axes[0].plot(trajectories[0][320:, 1], label=lbls[0])
axes[0].plot(trajectories[1][:, 1], label=lbls[1])
axes[0].set_ylabel('y')

axes[1].plot(trajectories[0][320:, 2], label=lbls[0])
axes[1].plot(trajectories[1][:, 2], label=lbls[1])
axes[1].set_ylabel('y')
axes[1].set_xlabel('x')

plt.grid()
plt.legend()

plt.show()
