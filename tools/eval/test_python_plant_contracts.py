"""Analytic Python physics regressions, not vehicle-validation evidence."""
from __future__ import annotations

import math
import unittest
from dataclasses import asdict
from unittest.mock import patch

import plant_ref as plant
from plant_ref import G, PlantParams, VehicleState, drive_force_cmd, plant_step


class DriveMemoryTests(unittest.TestCase):
    def test_no_memory_is_algebraic(self) -> None:
        for tau, jerk in ((0.0, 0.0), (0.1, 0.0), (0.0, 0.5), (0.1, 0.5)):
            for force in (-28000.0, 28000.0):
                with self.subTest(tau=tau, jerk=jerk, force=force):
                    p = PlantParams(tau_drv_s=tau, j_max_mps3=jerk)
                    self.assertEqual(drive_force_cmd(force, 0.02, p, 28000.0, None), force)

    def test_stateful_pt1_uses_backward_euler_and_actual_dt(self) -> None:
        p = PlantParams(tau_drv_s=0.3)
        memory = [123.0]
        expected = memory[0]
        for dt, force in ((0.001, 100.0), (0.007, 500.0), (0.02, -1000.0), (0.2, 300.0)):
            expected = (0.3 * expected + dt * force) / (0.3 + dt)
            got = drive_force_cmd(force, dt, p, 28000.0, memory)
            self.assertAlmostEqual(got, expected, delta=1e-10)
            self.assertEqual(memory, [got])

    def test_stateful_jerk_preserves_force_memory(self) -> None:
        p = PlantParams(j_max_mps3=0.5)
        memory = [0.0]
        # Delta force = m*j*dt = 280 N; this is not a body-jerk bound.
        for command, expected in ((28000.0, 280.0), (28000.0, 560.0), (-28000.0, 280.0)):
            self.assertEqual(drive_force_cmd(command, 0.02, p, 28000.0, memory), expected)
            self.assertEqual(memory, [expected])

    def test_empty_buffer_is_a_stateful_cold_start(self) -> None:
        memory: list[float] = []
        got = drive_force_cmd(28000.0, 0.02, PlantParams(tau_drv_s=0.1), 28000.0, memory)
        self.assertAlmostEqual(got, 28000.0 * 0.02 / 0.12, delta=1e-9)
        self.assertEqual(memory, [got])


class FilterTwinStopTests(unittest.TestCase):
    def test_passive_coast_integrates_to_zero_in_both_directions(self) -> None:
        p = PlantParams(A_d=28000.0, B_d=0.0, C_d=0.0)
        for sign in (-1.0, 1.0):
            with self.subTest(sign=sign):
                x = plant_step(VehicleState(v_mps=sign * 0.15), 0.0, 0.0, 0.2, p)
                self.assertEqual(x.v_mps, 0.0)
                self.assertAlmostEqual(x.s_m, sign * 0.15**2 / 2.0, delta=1e-14)

    def test_braking_stop_matches_existing_cpp_event_contract(self) -> None:
        p = PlantParams(A_d=0.0, B_d=0.0, C_d=0.0, a_svc=1.2)
        for sign in (-1.0, 1.0):
            with self.subTest(sign=sign):
                x = plant_step(VehicleState(v_mps=sign * 0.15), 0.0, 1.0, 0.2, p)
                self.assertEqual(x.v_mps, 0.0)
                self.assertAlmostEqual(x.s_m, sign * 0.15**2 / (2.0 * 1.2), delta=1e-14)

    def test_no_crossing_retains_full_constant_acceleration_step(self) -> None:
        p = PlantParams(A_d=28000.0, B_d=0.0, C_d=0.0)
        x = plant_step(VehicleState(v_mps=0.15), 0.0, 0.0, 0.01, p)
        self.assertAlmostEqual(x.v_mps, 0.14, delta=1e-14)
        self.assertAlmostEqual(x.s_m, 0.15 * 0.01 - 0.5 * 0.01**2, delta=1e-14)

    def test_rotational_inertia_is_in_the_stop_acceleration(self) -> None:
        p = PlantParams(A_d=35000.0, B_d=0.0, C_d=0.0, gamma_rot=0.25)
        x = plant_step(VehicleState(v_mps=0.15), 0.0, 0.0, 0.2, p)
        self.assertEqual(x.v_mps, 0.0)
        self.assertAlmostEqual(x.s_m, 0.15**2 / 2.0, delta=1e-14)

    def test_external_forces_are_not_replaced_by_static_holding(self) -> None:
        cases = (
            (PlantParams(A_d=0.0, B_d=0.0, C_d=0.0, i_grade=0.04), 0.0, None, -G * 0.04),
            (PlantParams(A_d=0.0, B_d=0.0, C_d=0.0), 14000.0, None, -0.5),
            (PlantParams(A_d=0.0, B_d=0.0, C_d=0.0, tau_drv_s=1.0), 0.0, [-28000.0], -1.0 / 1.2),
        )
        for p, bias, memory, acceleration in cases:
            with self.subTest(grade=p.i_grade, bias=bias, tau=p.tau_drv_s):
                x = plant_step(VehicleState(v_mps=0.05, f_bias_n=bias), 0.0, 0.0, 0.2, p, memory)
                self.assertLess(x.v_mps, 0.0)
                self.assertAlmostEqual(x.v_mps, 0.05 + acceleration * 0.2, delta=1e-12)
                self.assertAlmostEqual(x.s_m, 0.05 * 0.2 + 0.5 * acceleration * 0.2**2, delta=1e-12)

    def test_nonpositive_dt_does_not_mutate_state_or_memory(self) -> None:
        for dt in (0.0, -0.02):
            x, memory = VehicleState(v_mps=1.0), [17.0]
            before = asdict(x)
            got = plant_step(x, 1.0, 1.0, dt, PlantParams(tau_drv_s=0.1), memory)
            self.assertIs(got, x)
            self.assertEqual(asdict(x), before)
            self.assertEqual(memory, [17.0])


class ImplicitWheelTests(unittest.TestCase):
    def test_linear_contact_has_correct_chain_rule_and_fast_newton(self) -> None:
        # F = K*(r*omega-v) gives a linear backward-Euler residual with
        # a closed-form root. Only the contact law is replaced deliberately;
        # the real solver runs. This fixture is not a calibrated rail law.
        K, dt, J = 100000.0, 0.02, 60.0
        for r in (0.25, 0.35, 0.7, 1.0):
            for sign in (-1.0, 1.0):
                with self.subTest(r=r, sign=sign):
                    omega, velocity, torque = sign * 20.0, sign * 5.0, sign * 2000.0
                    calls = []

                    def linear_force(Q, mu, slip, v, p):
                        calls.append(slip)
                        return K * slip

                    with patch.object(plant, "adhesion_force_n", linear_force):
                        got, force = plant._implicit_wheel_omega(
                            omega, velocity, r, torque, dt, J, 1e6, 0.5, PlantParams()
                        )
                    expected = (omega + dt / J * (torque + r * K * velocity)) / (1 + dt / J * r*r*K)
                    self.assertAlmostEqual(got, expected, delta=1e-8)
                    self.assertAlmostEqual(force, K * (r * got - velocity), delta=1e-6)
                    self.assertLessEqual(len(calls), 10, "linear residual should not need the bisection fallback")

    def test_zero_timestep_is_an_identity(self) -> None:
        p = PlantParams()
        got, force = plant._implicit_wheel_omega(20.0, 5.0, 0.35, 2000.0, 0.0, 60.0, 1e5, 0.35, p)
        self.assertEqual(got, 20.0)
        self.assertEqual(force, plant.adhesion_force_n(1e5, 0.35, 0.35*20.0-5.0, 5.0, p))

    def test_real_contact_force_and_backward_euler_residual(self) -> None:
        p = PlantParams()
        r, dt, J, Q, mu = 0.35, 0.02, 60.0, 28000.0*G/4, 0.35
        for omega, v, torque in ((20.0, 5.0, 2000.0), (-20.0, -5.0, -2000.0), (0.0, 0.0, 0.0), (5.0, 5.0, 0.0)):
            with self.subTest(omega=omega, v=v, torque=torque):
                got, force = plant._implicit_wheel_omega(omega, v, r, torque, dt, J, Q, mu, p)
                self.assertTrue(math.isfinite(got) and math.isfinite(force))
                self.assertEqual(force, plant.adhesion_force_n(Q, mu, r*got-v, v, p))
                self.assertAlmostEqual(got - omega - dt/J*(torque-r*force), 0.0, delta=1e-6)


if __name__ == "__main__":
    unittest.main()
