#!/usr/bin/env python3
"""Expected vs actual trajectory of one nmpc_node log, top view + 3D, with the error numbers.

  python plot_track.py <log.csv> [settle_s] [out.png]   (default out: <log>_track.png next to the log)

The log is nmpc_node's log_file (t,x,y,z,ref_x,ref_y,ref_z,err_cm); err_cm is the 3D distance.
Max and mean error are given for the whole flight and after settle_s (default 2 s), because the
start transient (the reference starts at 2.5 m/s) sets the whole-flight max. Needs numpy + matplotlib.
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

path = sys.argv[1]
settle = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
d = np.genfromtxt(path, delimiter=",", names=True)
late = d["t"] > settle

text = (f"whole flight: max {d['err_cm'].max():.2f} cm, mean {d['err_cm'].mean():.2f} cm\n"
        f"after {settle:g} s:    max {d['err_cm'][late].max():.2f} cm, mean {d['err_cm'][late].mean():.2f} cm")

fig = plt.figure(figsize=(13, 6))
top = fig.add_subplot(1, 2, 1)
top.plot(d["x"], d["y"], "-", color="#2a78d6", lw=2.5, label="actual")
top.plot(d["ref_x"], d["ref_y"], "--", color="#eb6834", lw=1.5, label="expected (reference)")
top.set(title="Top view", xlabel="x [m]", ylabel="y [m]", aspect="equal")
top.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2)
top.text(0.5, -0.2, text, transform=top.transAxes, ha="center", va="top", family="monospace", fontsize=9,
         bbox=dict(facecolor="white", alpha=0.85, edgecolor="#cccccc"))

d3 = fig.add_subplot(1, 2, 2, projection="3d")
d3.plot(d["x"], d["y"], d["z"], "-", color="#2a78d6", lw=2.5, label="actual")
d3.plot(d["ref_x"], d["ref_y"], d["ref_z"], "--", color="#eb6834", lw=1.5, label="expected (reference)")
d3.set(title="3D", xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")

fig.suptitle(f"Expected vs actual trajectory: {path}  ({d['t'][-1]:.0f} s)")
out = sys.argv[3] if len(sys.argv) > 3 else path.rsplit(".", 1)[0] + "_track.png"
fig.savefig(out, dpi=130, bbox_inches="tight")
print(text)
print("saved", out)
