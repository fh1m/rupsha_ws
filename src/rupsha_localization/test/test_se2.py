"""SE(2) identities, checked against matrix algebra rather than against themselves."""
import math

import numpy as np
import pytest

from rupsha_localization import se2


def _random_taus(n=200, seed=1):
    rng = np.random.default_rng(seed)
    taus = rng.uniform([-20, -20, -math.pi + 1e-6], [20, 20, math.pi - 1e-6], (n, 3))
    edge = np.array([[1.0, 2.0, 0.0], [1.0, -2.0, 1e-12], [3.0, 0.5, -1e-10],
                     [0.0, 0.0, 0.0], [5.0, 5.0, math.pi - 1e-9]])
    return np.vstack([taus, edge])


def test_exp_log_round_trip():
    for tau in _random_taus():
        assert np.allclose(se2.log(se2.exp(tau)), tau, atol=1e-9), tau


def test_exp_matches_the_matrix_exponential():
    """Closed form against a brute series of the 3x3 algebra element."""
    rng = np.random.default_rng(7)
    for _ in range(20):
        rho_x, rho_y, th = rng.uniform(-3, 3, 3)
        A = np.array([[0.0, -th, rho_x], [th, 0.0, rho_y], [0.0, 0.0, 0.0]])
        E, term = np.eye(3), np.eye(3)
        for k in range(1, 60):
            term = term @ A / k
            E = E + term
        assert np.allclose(se2.exp([rho_x, rho_y, th]), E, atol=1e-10)


def test_adjoint_identity():
    rng = np.random.default_rng(3)
    for _ in range(50):
        T = se2.exp(rng.uniform(-10, 10, 3))
        tau = rng.uniform(-1, 1, 3)
        lhs = T @ se2.exp(tau) @ se2.inverse(T)
        assert np.allclose(lhs, se2.exp(se2.adjoint(T) @ tau), atol=1e-10)


def test_compose_and_inverse():
    A = se2.make(1.0, 2.0, 0.7)
    B = se2.make(-3.0, 0.5, -2.0)
    assert np.allclose(se2.compose(A, se2.inverse(A)), np.eye(3), atol=1e-12)
    C = se2.compose(A, B)
    assert np.allclose(C, A @ B, atol=1e-12)
    assert se2.yaw_of(C) == pytest.approx(se2.wrap_pi(0.7 - 2.0))


def test_wrap_pi():
    assert se2.wrap_pi(math.pi) == pytest.approx(-math.pi)
    assert se2.wrap_pi(3 * math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert se2.wrap_pi(-0.2) == pytest.approx(-0.2)
    assert math.isnan(se2.wrap_pi(float('nan')))


def test_compass_and_enu_yaw():
    """North is compass 0 and ENU pi/2; east is compass 90 and ENU 0."""
    assert se2.yaw_from_compass_deg(0.0) == pytest.approx(math.pi / 2)
    assert se2.yaw_from_compass_deg(90.0) == pytest.approx(0.0)
    assert se2.yaw_from_compass_deg(270.0) == pytest.approx(-math.pi)
    for c in (0.0, 45.0, 181.0, 359.5):
        assert se2.compass_deg_from_yaw(se2.yaw_from_compass_deg(c)) == pytest.approx(c)
    assert math.isnan(se2.compass_deg_from_yaw(float('nan')))
