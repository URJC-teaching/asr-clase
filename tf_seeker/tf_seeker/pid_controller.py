# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

from math import fabs
from typing import Optional


class PIDController:
    def __init__(self, min_ref: float, max_ref: float, min_output: float, max_output: float):
        self.min_ref = min_ref  # Below this ref, output is 0.0
        self.max_ref = max_ref  # Above this ref, output is max_output
        self.min_output = min_output
        self.max_output = max_output

        self.KP = 0.41
        self.KI = 0.06
        self.KD = 0.53

        self.prev_error = 0.0
        self.int_error = 0.0

    def set_pid(self, kp: float, ki: float, kd: float):
        self.KP = kp
        self.KI = ki
        self.KD = kd

    def get_output(self, new_reference: float) -> float:
        ref = new_reference
        output = 0.0

        # Proportional
        direction = ref / fabs(ref) if ref != 0.0 else 0.0

        if fabs(ref) < self.min_ref:
            output = 0.0
        elif fabs(ref) > self.max_ref:
            output = direction * self.max_output
        else:
            output = direction * self.min_output + ref * (self.max_output - self.min_output)

        # Integral
        self.int_error = (self.int_error + output) * 2.0 / 3.0

        # Derivative
        deriv_error = output - self.prev_error
        self.prev_error = output

        pid_output = (
            self.KP * output +
            self.KI * self.int_error +
            self.KD * deriv_error
        )

        return max(-self.max_output, min(pid_output, self.max_output))
