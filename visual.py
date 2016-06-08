import pygame
from read_config import read_config
import random

scale = 53

WALL_COLOR = (0, 0, 0)

class Visual:
	def __init__(self,mapServer):
		pygame.init()
		self.mapServer = mapServer
		self.screen = pygame.display.set_mode([mapServer.width * scale, mapServer.height * scale], pygame.RESIZABLE, 0)
		
		self.walls = [[p.y,p.x] for p in mapServer.grid.getPoints() if p.isWall]
		self.robots = []

		self.refresh()

	def addRobot(self, robot):
		self.robots.append(robot)

	def refresh(self):
		self.screen.fill((37,56,60))
		
		if self.mapServer.it != None:
			for n in self.mapServer.it.grid.getPoints():
				self.drawEnergy(n)
		for w in self.walls:
			self.drawWall(w)
		for r in self.robots:
			self.drawRobot(r)
		
		pygame.display.flip()

	def drawEnergy(self, n):
		xStart = n.x*scale + random.randint(0,scale-1)
		yStart = n.y*scale + random.randint(0,scale-1)
		for i in range(int(n.energy)):
			pygame.draw.polygon(self.screen, (0,255,0), [(xStart, yStart ),(xStart + 2, yStart ),(xStart, yStart + 2)], 0)

	def drawRobot(self, robot):
		pygame.draw.polygon(self.screen, getColor(robot), getRobotPoints(robot), 0)

	def drawWall(self, wall):
		p1 = wall[1] * scale, wall[0] * scale
		p2 = p1[0] + scale, p1[1]
		p3 = p1[0], p1[1] + scale
		p4 = p2[0], p2[1] + scale
		point_list = [p1, p2, p4, p3]
		pygame.draw.polygon(self.screen, WALL_COLOR, point_list, 0)

top = 10
bottom = 5
side = (top + bottom) / 2

def getRobotPoints(robot):
	x = (robot.pos.x * scale) + scale/2
	y = (robot.pos.y * scale)+ scale/2
	if robot.lastMove == [1,0]:
		top_point  = x + top, y
		bot_point  = x - bottom, y + side
		bot_point2 = x - bottom, y - side
	elif robot.lastMove == [0, 1]:
		top_point  = x, y + top
		bot_point  = x - side, y - bottom
		bot_point2 = x + side, y - bottom
	elif robot.lastMove == [0, -1]:
		top_point  = x, y - top
		bot_point  = x - side, y + bottom
		bot_point2 = x + side, y + bottom
	elif robot.lastMove == [-1, 0]:
		top_point  = x - top, y
		bot_point  = x + bottom, y + side
		bot_point2 = x + bottom, y - side

	return [top_point, bot_point, bot_point2]


def getColor(robot):
	if robot.it:
		return (255, 0, 0)
	if robot.frozen:
		return (0, 0, 255)
	return (255, 255, 0)

