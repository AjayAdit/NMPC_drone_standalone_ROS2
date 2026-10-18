"""Where the package puts the files it writes.

Its own module so that nmpc_node and plot_log can agree on the location
without either importing the other's dependencies: the node would drag acados
and MuJoCo into the plotter, and the plotter would drag matplotlib into the
control loop.
"""

import os


def default_log_path():
    """One fixed, absolute location for the flight log.

    Deliberately not the working directory: nmpc_node and plot_log are usually
    started from different terminals, and a log you cannot find is a log you
    do not have.
    """
    return os.path.join(os.path.expanduser('~'), '.quad_nmpc_ros',
                        'tracking_log.csv')
