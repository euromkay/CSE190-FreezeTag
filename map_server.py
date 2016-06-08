#!/usr/bin/env python
import random as r
import math as m
import numpy as np
from copy import deepcopy
from read_config import read_config
from temperature_sensor import TempSensor
from texture_sensor import TexSensor
from robot import Robot

from visual import *

width = 17
height = 13

IT = True
RUNNER = False

class MapServer():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()

        self.height = len(self.config['walls_map'])
        self.width  = len(self.config['walls_map'][0])

        self.grid = Grid((self.width, self.height))
        self.grid.commands = self.config['possible_moves']
        self.robots = []
        self.it = None
        self.runners = []
        self.frozenRunners = []
        
        
        r.seed(self.config['seed'])



        for p in self.grid.getPoints():
            x, y = p.x, p.y
            p.prob = {}
            p.prob_new = {}
            p.isWall = (self.config['walls_map'][y][x] == 'W')
            p.temp = self.config['pipe_map'][y][x]
            p.text = self.config['texture_map'][y][x]

        self.vis = Visual(self)

    def getRobot(self, it):
        r = Robot(self)
        r.grid = self.grid
        r.setupConfig()
        r.it = it
        r.frozen = False
        if it:
            self.it = r
        else:
            self.runners.append(r)
        self.vis.addRobot(r)
        return r

    def handle_data_request(self, request, pos):
        if request == "temp":
            return self.grid.get(pos.x, pos.y).temp
        if request == "tex":
            return self.grid.get(pos.x, pos.y).text

    def addDistanceError(self, distance):
        return r.gauss(distance, self.config['distance_error'])

    def addSlopeError(self, slope):
        tan_slope = r.gauss(m.atan(slope), self.config['angle_error'])
        return m.tan(tan_slope)

    def handle_move_request(self, move, robot):
        move = list(move)
        if self.config['uncertain_motion']:
            roll = r.uniform(0,1)
            if roll >= self.config["prob_move_correct"]:
                possible_moves = deepcopy(self.config['possible_moves'])
                possible_moves.remove(move)
                move = r.choice(possible_moves)
        
        prePos = robot.pos
        prePos.robots.remove(robot)
        robot.pos = prePos.move(move[0], move[1])
        robot.energy += robot.pos.energy
        robot.pos.energy = 0
        robot.pos.robots.append(robot)



        if robot.it:
            for runner in self.runners:
                if robot.pos is runner.pos:
                    runner.frozen = True
                    self.runners.remove(runner)
                    self.frozenRunners.append(runner)
                    robot.energy += runner.energy * self.config['freeze_reward']
                    runner.energy *= (1 - self.config['freeze_reward'])
        else:
            for frozen in self.frozenRunners:
                #print frozen.pos
                if frozen.pos is robot.pos:
                    frozen.frozen = False
                    self.runners.append(frozen)
                    self.frozenRunners.remove(frozen)
                    robot.energy += self.it.energy * self.config['un_freeze_reward']
                    frozen.energy += self.it.energy * self.config['un_freeze_startup']
                    self.it.energy *= 1 - (self.config['un_freeze_reward'] + self.config['un_freeze_startup'])


    def addEnergy(self, amount, robot):
        robot.energy -= amount
        self.distributeEnergy(amount)

    def distributeEnergy(self, amount):
        point = r.choice([p for p in self.grid.getPoints() if not p.isWall])
        point.energy += amount


class Grid():
    def __init__(self, size):
        self.width  = size[0]
        self.height = size[1]
        self.base   = [[Point(x,y, self) for y in range(self.height)] for x in range(self.width)]

    def switch(self, n):
        if n.isWall:
            return 'WALL'
        if n.isPit:
            return 'PIT'
        if n.isGoal:
            return n.prevValue
        return round(n.value, 2)

    def printGrid(self):
        for h in range(self.height):
            print [self.switch(c[h]) for c in self.base]
        print '\n'

    def get(self, x, y):
        return self.base[x % self.width][y % self.height]

    def heuristic(self, p0, p1):
        return math.sqrt((p0.x - p1.x)**2 + (p0.y - p1.y)**2)

    def getPoints(self):
        return [x for sublist in self.base for x in sublist]

class Point():
    def __init__(self, x, y, grid):
        self.x = x
        self.y = y
        self.grid = grid
        self.isWall = False
        self.isPit  = False
        self.isGoal = False
        self.edges = []
        self.parent = None
        self.energy = 0.0
        self.robots = []

        
    def isSpecial(self):
        return self.isWall or self.isPit or self.isGoal

    def getPossibleMoves(self):
        ret = []
        for x,y in self.grid.commands:
            if x == 0 and y == 0:
                ret.append([x,y])
                continue
            if self is not self.move(x,y):
                ret.append([x,y])
        return ret

    def move(self, x, y):
        m = self.grid.get(self.x + x, self.y + y)
        if m.isWall:
            return self
        return m

    def parentNodeCount(self):
        if self.parent == None:
            return 0
        else:
            return 1 + self.parent.parentNodeCount()

    def __repr__(self):
        return str((self.x, self.y))

if __name__ == '__main__':
    ms = MapServer()