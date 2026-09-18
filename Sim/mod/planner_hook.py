"""
mod/planner_hook.py — Risk-Aware Path Planning Helpers
=======================================================
Provides convenience functions for integrating MoD risk into
path planning / task allocation.

Behind the ``MAP_OF_DYNAMICS["planner_hook_enabled"]`` flag (default OFF).
Does NOT alter any existing planning behaviour unless the flag is on.
"""


def risk_at(mod, x_px, y_px):
    """
    Query the MoD risk value at a world pixel position.

    Parameters
    ----------
    mod : MapOfDynamics
    x_px, y_px : float
        World position in pixel coordinates.

    Returns
    -------
    float : risk in [0, 1)
    """
    return mod.risk_at(x_px, y_px)


def edge_cost(length, risk, k_risk=2.0):
    """
    Compute a risk-augmented edge cost for path planning.

    cost = length × (1 + k_risk × risk)

    Parameters
    ----------
    length : float
        Edge length (in any unit: pixels, metres, cells).
    risk : float
        MoD risk value for this edge's cell, in [0, 1).
    k_risk : float
        Risk cost multiplier (from config).

    Returns
    -------
    float : augmented cost ≥ length
    """
    return length * (1.0 + k_risk * risk)
