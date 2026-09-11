"""CLI parser checks."""

from rocket_control.cli import build_parser


def test_landing_degree_flags():
    ns = build_parser().parse_args(["landing", "--theta-deg", "10", "--omega-deg", "-2", "--no-anim"])
    assert ns.theta_deg == 10.0
    assert ns.omega_deg == -2.0
    assert ns.no_anim is True


def test_attitude_mode():
    ns = build_parser().parse_args(["attitude", "--mode", "dual", "--theta0-deg", "30"])
    assert ns.mode == "dual"
    assert ns.theta0_deg == 30.0


def test_plots_flag_shapes():
    off = build_parser().parse_args(["landing", "--no-anim"])
    assert off.plots is None
    window = build_parser().parse_args(["landing", "--plots"])
    assert window.plots == ""
    saved = build_parser().parse_args(["landing", "--plots", "results/diag.png"])
    assert saved.plots == "results/diag.png"
