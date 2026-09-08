#!/usr/bin/env python3
"""Selected Polach equations and returned wheel-state/force contracts.

Reference: O. Polach, Wear 258 (2005), 992-1000, equations (4), (9), (11),
(12), doi:10.1016/j.wear.2004.03.046. These finite-input witnesses do not
validate vehicle parameters, full 3D contact or the coupled body integrator.
"""
import math
import unittest

import plant_ref as model


class PolachEquationTests(unittest.TestCase):
    def test_eq9_known_values_and_evenness(self):
        for mu0 in (0.2, 0.55):
            for decay in (0.2, 0.6):
                for slip, factor in ((0.0, 1.0), (math.log(2) / decay, 0.7),
                                     (50.0 / decay, 0.4)):
                    for sign in (-1.0, 1.0):
                        with self.subTest(mu0=mu0, decay=decay, slip=sign * slip):
                            self.assertAlmostEqual(
                                model.polach9_mu(mu0, sign * slip, 0.4, decay),
                                mu0 * factor, delta=1e-14)

    def test_eq11_at_unit_epsilon(self):
        load, mu, speed = 50000.0, 0.3, 10.0
        for ka, ks in ((1.0, 0.4), (0.3, 0.1)):
            p = model.PlantParams(polach_kA=ka, polach_kS=ks)
            stiffness = p.shear_mod_pa * p.hertz_a_m * p.hertz_b_m * p.kalker_c11
            slip = 4.0 * load * mu * speed / (math.pi * stiffness)
            expected = 2.0 * load * mu / math.pi * (ka / (1.0 + ka * ka) + math.atan(ks))
            with self.subTest(ka=ka, ks=ks):
                self.assertAlmostEqual(model.polach11_force_n(load, mu, slip, speed, p),
                                       expected, delta=1e-9)

    def test_eq12_small_creep_slope_with_code_regularization(self):
        for ka, ks in ((1.0, 0.4), (0.3, 0.1)):
            p = model.PlantParams(polach_kA=ka, polach_kS=ks)
            stiffness = p.shear_mod_pa * p.hertz_a_m * p.hertz_b_m * p.kalker_c11
            for speed in (-10.0, 0.0, 10.0):
                denominator = max(abs(speed), p.v_eps)
                for load, mu in ((20000.0, 0.06), (80000.0, 0.55)):
                    # Set epsilon=1e-6, avoiding a parameter-dependent FD step.
                    slip = 1e-6 * 4.0 * load * mu * denominator / (math.pi * stiffness)
                    actual = model.polach11_force_n(load, mu, slip, speed, p) / slip
                    expected = 0.5 * (ka + ks) * stiffness / denominator
                    with self.subTest(ka=ka, speed=speed, load=load, mu=mu):
                        self.assertAlmostEqual(actual / expected, 1.0, delta=1e-10)

    def test_combined_contact_bound_symmetry_and_dissipation(self):
        load, mu0 = 50000.0, 0.35
        for ka, ks in ((1.0, 0.4), (0.3, 0.1)):
            p = model.PlantParams(polach_kA=ka, polach_kS=ks)
            for speed in (-15.0, 0.0, 15.0):
                for slip in (-30.0, -1.0, 0.0, 1e-8, 1.0, 30.0):
                    with self.subTest(ka=ka, speed=speed, slip=slip):
                        force = model.adhesion_force_n(load, mu0, slip, speed, p)
                        opposite = model.adhesion_force_n(load, mu0, -slip, speed, p)
                        mu = model.polach9_mu(mu0, slip, p.polach_A, p.polach_B_s_per_m)
                        self.assertTrue(math.isfinite(force))
                        self.assertLessEqual(abs(force), load * mu * (1.0 + 1e-12))
                        self.assertAlmostEqual(force, -opposite, delta=1e-9)
                        # Contact-pair power: F*v - r*F*omega = -F*slip <= 0.
                        # Not an energy-stability proof for the time integrator.
                        self.assertLessEqual(-force * slip, 0.0)


class WheelReturnTests(unittest.TestCase):
    def assert_force_matches_state(self, omega, force, speed, p):
        expected = model.adhesion_force_n(50000.0, 0.35, 0.35 * omega - speed, speed, p)
        self.assertAlmostEqual(force, expected, delta=1e-9)

    def test_converged_clipped_state_recomputes_force(self):
        for law in ('polach11', 'tanh'):
            p = model.PlantParams(creep_force=law)
            for sign in (-1.0, 1.0):
                # Stress the accepted out-of-range initial state: the initial
                # residual is zero, but clipping changes the contact slip.
                initial = sign * 100.0
                speed = 0.35 * initial
                with self.subTest(law=law, sign=sign):
                    omega, force = model._implicit_wheel_omega(
                        initial, speed, 0.35, 0.0, 0.02, 60.0, 50000.0, 0.35, p)
                    self.assertEqual(omega, sign * model.OMEGA_MAX)
                    self.assert_force_matches_state(omega, force, speed, p)

    def test_in_range_equilibrium_is_unchanged(self):
        p = model.PlantParams()
        for initial in (-80.0, -20.0, 0.0, 20.0, 80.0):
            speed = 0.35 * initial
            with self.subTest(initial=initial):
                omega, force = model._implicit_wheel_omega(
                    initial, speed, 0.35, 0.0, 0.02, 60.0, 50000.0, 0.35, p)
                self.assertEqual(omega, initial)
                self.assertEqual(force, 0.0)

    def test_nonpositive_dt_retains_existing_identity_contract(self):
        p = model.PlantParams()
        for dt in (0.0, -0.02):
            for initial in (-100.0, 100.0):
                with self.subTest(dt=dt, initial=initial):
                    omega, force = model._implicit_wheel_omega(
                        initial, 10.0, 0.35, 500.0, dt, 60.0, 50000.0, 0.35, p)
                    self.assertEqual(omega, initial)
                    self.assert_force_matches_state(omega, force, 10.0, p)

    def test_saturated_fallback_force_is_consistent_not_an_ode_root(self):
        p = model.PlantParams()
        for sign in (-1.0, 1.0):
            initial, torque, dt = sign * 79.0, sign * 1e6, 0.2
            with self.subTest(sign=sign):
                omega, force = model._implicit_wheel_omega(
                    initial, 0.0, 0.35, torque, dt, 60.0, 50000.0, 0.35, p)
                self.assertEqual(omega, sign * model.OMEGA_MAX)
                self.assert_force_matches_state(omega, force, 0.0, p)
                residual = omega - initial - dt / 60.0 * (torque - 0.35 * force)
                self.assertGreater(abs(residual), 1.0)


if __name__ == '__main__':
    unittest.main()
