"""Analytic checks for the slender-rod CoM / MoI model."""

import pytest

from rocket_control.core import com_and_inertia, default_vehicle
from rocket_control.core.vehicle import VehicleSpec


def test_dry_mass_is_uniform_rod():
    v = default_vehicle()
    d_com, i_z = com_and_inertia(v.m_dry, v)
    assert d_com == pytest.approx(v.rocket_height / 2)
    assert i_z == pytest.approx((1.0 / 12.0) * v.m_dry * v.rocket_height**2)


def test_full_tank_com_is_mass_weighted_average():
    v = default_vehicle()
    d_com, _ = com_and_inertia(v.m0, v)
    expected = (
        v.m_dry * v.dry_cm_from_nozzle + v.m_fuel * v.fuel_cm_from_nozzle
    ) / v.m0
    assert d_com == pytest.approx(expected)
    # Fuel in the lower half pulls the CoM down from the geometric centre.
    assert d_com < v.dry_cm_from_nozzle


def test_casadi_matches_numpy():
    casadi = pytest.importorskip("casadi")
    from rocket_control.core.mass_properties import com_and_inertia_ca

    v = default_vehicle()
    m = casadi.SX.sym("m")
    d_sym, i_sym = com_and_inertia_ca(m, v)
    f = casadi.Function("mass_props", [m], [d_sym, i_sym])
    for mass in (v.m_dry, 0.5 * (v.m_dry + v.m0), v.m0):
        d_np, i_np = com_and_inertia(mass, v)
        d_ca, i_ca = f(mass)
        assert float(d_ca) == pytest.approx(d_np, rel=1e-12, abs=1e-12)
        assert float(i_ca) == pytest.approx(i_np, rel=1e-12, abs=1e-12)


def test_rejects_non_positive_mass():
    v = default_vehicle()
    with pytest.raises(ValueError):
        com_and_inertia(0.0, v)


def test_vehicle_validation():
    with pytest.raises(ValueError):
        VehicleSpec(
            m_dry=-1,
            m_fuel=1,
            T_max=1,
            T_min=0,
            I_sp=1,
            rocket_height=1,
            fuel_tank_height=0.5,
        )
