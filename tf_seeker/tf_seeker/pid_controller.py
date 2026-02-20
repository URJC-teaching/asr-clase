# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

from math import fabs
from typing import Optional


class PIDController:
    def __init__(self, min_ref: float, max_ref: float, min_output: float, max_output: float,
                 kp: float = 0.41, ki: float = 0.06, kd: float = 0.53):
        self.min_ref = min_ref  # Below this ref, output is 0.0
        self.max_ref = max_ref  # Above this ref, output is max_output
        self.min_output = min_output
        self.max_output = max_output

        self.KP = kp
        self.KI = ki
        self.KD = kd

        self.prev_error = 0.0
        self.int_error = 0.0

    def set_pid(self, kp: float, ki: float, kd: float):
        self.KP = kp
        self.KI = ki
        self.KD = kd

    def get_output(self, error: float, dt: float) -> float:
        """
        PID estándar simple: u[n] = Kp*e[n] + Ki*sum(e[k]) + Kd*(e[n]-e[n-1])
        
        Parámetros:
            error: error actual (setpoint - valor_actual)
            dt: intervalo de tiempo entre llamadas (requerido para I y D)
        """

        # Término Proporcional
        p_term = self.KP * error

        # Término Integral (con saturación simple)
        self.int_error += error * dt
        # Limitar integral para evitar windup
        max_int = 10.0  # límite razonable
        self.int_error = max(-max_int, min(self.int_error, max_int))
        i_term = self.KI * self.int_error

        # Término Derivativo
        d_term = self.KD * (error - self.prev_error) / dt
        self.prev_error = error

        # Salida PID
        output = p_term + i_term + d_term

        # Saturación de salida
        output = max(self.min_output, min(output, self.max_output))

        return output
