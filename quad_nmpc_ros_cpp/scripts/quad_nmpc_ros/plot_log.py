"""Render the flight log written by nmpc_node into a PNG.

Kept out of nmpc_node on purpose. The node's job is to fly the drone and
record what happened; turning that record into a picture is a separate job
that must not be able to disturb the control loop, and must still work if the
node was killed rather than shut down cleanly.

    ros2 run quad_nmpc_ros_cpp plot_log                  # tracking_log.csv -> .png
    ros2 run quad_nmpc_ros_cpp plot_log some_other.csv
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')          # write a file, never open a window
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

from .paths import default_log_path       # noqa: E402  one source of truth


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    argv = [a for a in argv if not a.startswith('--ros-args')]

    csv_path = os.path.abspath(argv[0]) if argv else default_log_path()
    if not os.path.exists(csv_path):
        print(f'[plot_log] no such file: {csv_path}')
        print('[plot_log] run nmpc_node first; it writes the log there')
        return 1

    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    if data.size == 0:
        print(f'[plot_log] {csv_path} is empty')
        return 1

    t   = data['t']
    pos = np.column_stack([data['x'], data['y'], data['z']])
    ref = np.column_stack([data['ref_x'], data['ref_y'], data['ref_z']])
    err = data['err_cm']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    ax1.plot(t, err, lw=1.2)
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('3D tracking error [cm]')
    ax1.set_title(f'mean {err.mean():.1f} cm    max {err.max():.1f} cm')
    ax1.grid(alpha=0.3)

    ax2.plot(ref[:, 0], ref[:, 1], '--', lw=1.0, label='reference')
    ax2.plot(pos[:, 0], pos[:, 1], lw=1.2, label='flown')
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('y [m]')
    ax2.set_aspect('equal')
    ax2.set_title('ground track')
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    out = os.path.splitext(csv_path)[0] + '.png'
    fig.savefig(out, dpi=130)
    plt.close(fig)

    print(f'[plot_log] {len(t)} samples, {t[-1] - t[0]:.1f} s of flight')
    print(f'[plot_log] mean {err.mean():.1f} cm    max {err.max():.1f} cm')
    print(f'[plot_log] saved {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
