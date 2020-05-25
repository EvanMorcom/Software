#!/usr/bin/env python
"""
This file contains the unit tests for the DifferenceEquation class

"""

import numpy as np
import control as ct
from control.matlab import *
import unittest
import difference_equation as de
import collections


class TestDifferenceEquation(unittest.TestCase):
    def test_step_function_first_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (1st order)
        using the python control toolbox
        """

        sample_time = 0.1  # [s]
        end_time = 5

        J = 0.001
        B = 0.001
        K = 1 / 1000

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s + B)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        for i in range(0, num_points):
            difference_equation.tick(step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

    def test_step_function_second_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (2nd order)
        using the python control toolbox
        """

        sample_time = 0.01  # [s]
        end_time = 20

        J = 0.01
        B = 0.001
        K = 1

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s ** 2 + B * s + 1.5)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        for i in range(0, num_points):
            difference_equation.tick(step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

    def test_run_for_ticks(self):
        """
        This test checks the run_for_ticks member function to ensure that the difference equation is ran for exactly the
        number of specified iterations
        """

        sample_time = 0.1  # [s]
        end_time = 20

        J = 0.01
        B = 0.001
        K = 1

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s ** 2 + B * s + 1.5)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        difference_equation.run_for_ticks(num_points, step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

        self.assertEqual(num_points, difference_equation.get_timestep_count())

    def test_get_time_step_count(self):
        """
        This test checks the get_time_step_count() member function to ensure it returns the exact numer of steps
        the difference equation has iterated through
        """

        denominator = [1, 2, 4]
        numerator = [1]

        difference_equation = de.DifferenceEquation(denominator, numerator)

        for i in range(0, 100):
            self.assertEqual(difference_equation.get_timestep_count(), i)
            difference_equation.tick(0)

    #########################################333
    def test_difference_equation_init(self):
        numerator = [1, 0]
        denominator = [1, 1, 1]
        interpolation_period = 0.1

        transfer_function = ct.tf(numerator, denominator, interpolation_period)

        difference_equation = de.Difference_Equation(transfer_function)

        self.assertEqual(difference_equation.get_output_order(), len(denominator))
        self.assertEqual(difference_equation.get_input_order(), len(numerator))

        for i in range(0, len(numerator)):
            self.assertEqual(
                difference_equation.get_input_coefficients()[i], numerator[i]
            )

        self.assertEqual(difference_equation.get_output_coefficients()[0], 0)
        for i in range(1, len(denominator)):
            self.assertEqual(
                difference_equation.get_output_coefficients()[i], denominator[i]
            )

        numerator = [1, 0]
        denominator = [2, 1, 1]
        interpolation_period = 0.1

        transfer_function = ct.tf(numerator, denominator, interpolation_period)

        difference_equation = de.Difference_Equation(transfer_function)

        self.assertEqual(difference_equation.get_input_coefficients()[0], 0.5)
        self.assertEqual(difference_equation.get_input_coefficients()[1], 0)

        self.assertEqual(difference_equation.get_output_coefficients()[0], 0)
        self.assertEqual(difference_equation.get_output_coefficients()[1], 0.5)
        self.assertEqual(difference_equation.get_output_coefficients()[2], 0.5)

    def test_difference_equation_output(self):
        """
        Test that the difference equation follows the correct output sequence
        as a human-checked sequence with constant input
        """

        numerator = [1, 1]
        denominator = [1, 1, 1]
        interpolation_period = 0.1
        input_value = 1

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=2)
        outputs = collections.deque(maxlen=3)

        for i in range(0, len(denominator)):
            inputs.append(0)
        for i in range(0, len(denominator)):
            outputs.append(0)

        transfer_function = tf(numerator, denominator, interpolation_period)
        difference_equation = de.Difference_Equation(transfer_function)
        inputs.appendleft(input_value)

        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))
        self.assertEqual(outputs[0], 0)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 1)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 1)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 0)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], -1)

    def test_output_of_difference_equation_against_discrete_tf(self):

        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (1st order)
        using the python control toolbox
        """

        sample_time = 0.1  # [s]
        end_time = 5

        J = 0.001
        B = 0.001
        K = 1 / 1000

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s + B)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.Difference_Equation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=difference_equation.get_output_order()+1)
        outputs = collections.deque(maxlen=difference_equation.get_output_order())
        
        full_output_history = []

        for i in range(0, difference_equation.get_output_order()+1):
            inputs.append(0)
        for i in range(0, difference_equation.get_output_order()):
            outputs.append(0)

        for i in range(0, num_points):
            inputs.appendleft(step_input)
            output = difference_equation.calculate_output(outputs, inputs)
            outputs.appendleft(output)
            full_output_history.append(output)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], full_output_history[i], 4)


if __name__ == "__main__":
    unittest.main()
