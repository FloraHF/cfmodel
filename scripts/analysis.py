import matplotlib.pyplot as plt
import numpy as np

from exp_replay import ReplayPool

data = ReplayPool('cf0', res_dir='cf0_yaw2/')
fig, ax = plt.subplots()

ts = np.linspace(data.t_start, data.t_end, 1000)


# rolls = np.asarray([data.roll(t)*57.3+2.3 for t in ts])
# roll_cmds = np.asarray([data.cmd_roll(t) for t in ts])
# ax.plot(ts, rolls)
# ax.plot(ts, roll_cmds)
# plt.show()

# pitchs = np.asarray([data.pitch(t)*57.3-0.1 for t in ts])
# pitch_cmds = np.asarray([data.cmd_pitch(t) for t in ts])
# ax.plot(ts, pitchs)
# ax.plot(ts, pitch_cmds)
# plt.show()

yaws = np.asarray([data.yaw(t)*57.3 for t in ts])
yaw_cmds = np.asarray([data.cmd_yaw(t) for t in ts])
ax.plot(ts, yaws)
ax.plot(ts, yaw_cmds)
plt.show()

# zs = np.asarray([data.z(t) for t in ts])
# thrust_cmds = np.asarray([data.cmd_thrust(t)/80000+0.032 for t in ts])
# ax.plot(ts, zs)
# ax.plot(ts, thrust_cmds)
# plt.show()