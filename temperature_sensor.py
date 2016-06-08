#!/usr/bin/env python

import random
import math as m
import numpy as np
from copy import deepcopy
from read_config import read_config


class TempSensor():
    def __init__(self, robot):
        """Read config file and setup ROS things"""
        self.config = read_config()
        
        self.temp_dict = {
                'H': 40.0,
                'C': 20.0,
                '-': 25.0
        }
        self.sensor_on = False
        random.seed(self.config['seed'])
        self.grid = robot.mapServer
        self.robot = robot

    def sense(self):
        """Get the temperature of the current square."""
        self.robot.mapServer.addEnergy(self.config['texture_cost'], self.robot)
        temperature = self.grid.handle_data_request('temp', self.robot.pos)
        temp = self.temp_dict[temperature]
        noisy_measurement = self.add_noise(temp)
        return noisy_measurement

    def add_noise(self, true_val):
        """Returns measurement after adding Gaussian noise."""
        noise = m.ceil(random.gauss(0, self.config['temp_noise_std_dev'])*100.)/100.
        noisy_measurement = true_val + noise
        return noisy_measurement