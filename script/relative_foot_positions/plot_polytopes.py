import numpy as np

# from hpp_centroidal_dynamics import *
# from hpp_spline import *
from numpy import array

from constants_and_tools import genPolytope

import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull

# from hpp_bezier_com_traj import *
# from qp import solve_lp

# import cdd


def plot_hull_in_subplot(hull, pts, apts, ax, color="r", just_pts=False):
    # Plot defining corner points
    ax.plot(apts.T[0], apts.T[1], apts.T[2], "ko")
    if not just_pts:
        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            ax.plot(apts[s, 0], apts[s, 1], apts[s, 2], color + "-")


def plot_hull(
    hull, pts, apts, color="r", just_pts=False, plot=False, fig=None, ax=None
):
    if fig is None:
        fig = plt.figure()
    if ax is None:
        print("ax is none")
        ax = fig.add_subplot(111, projection="3d")
    plot_hull_in_subplot(hull, pts, array(pts), ax, color, just_pts)
    if plot:
        print(" PLOT")
        plt.show(block=False)
    return ax


def plot_polytope_H_rep(A_in, b_in, color="r", just_pts=False):
    hull, pts, apts, cd = genPolytope(A_in, b_in)
    plot_hull(hull, pts, apts, color, just_pts)


def plot_polytope_V_rep(pts, color="r", just_pts=False):
    pts = [array(el) for el in pts]
    apts = array(pts)
    hull = ConvexHull(apts, qhull_options="Q12")
    plot_hull(hull, pts, apts, color, just_pts)


def plot_polytope_CDD_PolyHeron(H, color="r", just_pts=False):
    g = H.get_generators()
    pts = [array(g[el][1:]) for el in range(g.row_size)]
    plot_polytope_V_rep(pts, color, just_pts)
