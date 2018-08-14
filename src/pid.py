#!/usr/bin/env python

from math import copysign

class PID:
    def __init__(self, settings):
        self.e_sum = self.e_old = 0
        self.kp = self.ki = self.kd = self.clamp = 0
        self.set_params(settings)

    def set_params(self, settings):
        self.kp = settings['kp']
        self.ki = settings['ki']
        self.kd = settings['kd']
        self.clamp = settings['clamp']

    def update(self, error):
        p = self.kp * error
        self.e_sum += self.ki * error
        if abs(self.e_sum) > self.clamp:
            self.e_sum = copysign(self.clamp, self.e_sum)
        d = self.kd * (error - self.e_old)
        self.e_old = error
        return p + self.e_sum + d
