#!/usr/bin/env python

import random as r
import math as m
import numpy as np
from copy import deepcopy
from read_config import read_config


class TexSensor():
    def __init__(self, robot):
        """Read config file and setup ROS things"""
        self.config = read_config()
        r.seed(self.config['seed'])
        self.grid = robot.mapServer
        self.robot = robot

    def sense(self):
        """Callback function for the texture service."""
        self.robot.mapServer.addEnergy(self.config['texture_cost'], self.robot)
        texture = self.take_measurement()
        noisy_texture = self.add_noise(texture)
        return noisy_texture

    def take_measurement(self):
        """Get the texture of the current square."""
        tex_response = self.grid.handle_data_request('tex', self.robot.pos)
        return tex_response

    def add_noise(self, measurement):
        """Flip measurement with some probability."""
        roll = r.uniform(0,1)
        if roll < self.config['prob_tex_correct']:
            return measurement
        else:
            if measurement == 'R':
                return 'S'
            elif measurement == 'S':
                return 'R'


if __name__ == '__main__':
    ts = TexSensor()
