import sys
import time, thread
import math
import random

class LaserSensor():
	def __init__(self, robot):
		self.mapServer = robot.mapServer
		self.robot = robot

	def scan(self):
		if self.robot.it:
			return self.getClosestRobot()

		return self.mapServer.it


	def getClosestRobot(self):
		if len(self.mapServer.runners) == 0:
			return None
		closestRobot = self.mapServer.runners[0]
		minDistance = gridDistance(closestRobot, self.robot.pos)
		for runner in self.mapServer.runners[1:]:
			dist = gridDistance(runner, self.robot.pos)
			if(dist < minDistance):
				closestRobot = runner
				minDistance = dist
		return closestRobot


	def itScan(self):
		runner = self.getClosestRobot(self.robot.it) #gets closest moving runner

		xdist = (float(runner.pos.x) - self.robot.pos.x)
		ydist = (float(runner.pos.y) - self.robot.pos.y)
		if int(xdist) != 0:
			m = (float(runner.pos.y) - self.robot.pos.y)/xdist
			m = self.mapServer.addSlopeError(m)
		else:
			m = abs(ydist)/ydist * sys.maxint

		
		distance = robot_distance(self.robot.pos, runner.pos)
		distance = self.mapServer.addDistanceError(distance)

		x = distance/math.sqrt(m**2 + 1)
		y = m * x #x and y are distances from the robot

		if runner.pos.x > self.robot.pos.x:
			x += self.robot.pos.x
		else:
			x -= self.robot.pos.x

		if runner.pos.y > self.robot.pos.y:
			y += self.robot.pos.y
		else:
			y += self.robot.pos.y


		runnerLoc = Point(myFloor(x), myFloor(y))

		minMove = self.robot.possible_moves[0]
		firstLoc = Point(self.robot.pos.x + minMove[0], self.robot.pos.y + minMove[1])
		minDistance = robot_distance(firstLoc, runnerLoc)


		for move in self.robot.possible_moves[1:]:
			moveLoc = Point(self.robot.pos.x + move[0], self.robot.pos.y + move[1])
			distance = robot_distance(moveLoc, runnerLoc)
			if distance < minDistance:
				minDistance = distance
				minMove = move

		return minMove


	def runnerScan(self):
		return random.choice(self.robot.possible_moves)


def myFloor(x):
	sign = x / abs(x)
	return int(math.floor(x)) * int(sign)

def gridDistance(r1, r2):
	width = r1.width
	height = r1.height
	x1 = min(r1.pos.x, r2.x)
	x2 = max(r1.pos.x, r2.x)
	y1 = min(r1.pos.y, r2.y)
	y2 = max(r1.pos.y, r2.y)

	x = min(abs(x2 - x1), abs(x2 - (x1 + width)))
	y = min(abs(y2 - y1), abs(y2 - (y1 + width)))
	return (x**2 + y**2) ** .5


class Point():
	def __init__(self, x, y):
		self.x = x
		self.y = y
	def inc(self, m, b):
		inc_x = Point(self.x+1, (self.x+1) * m + b)
		if m == 0 or m == 0.0:
			inc_y = None
		else:
			inc_y = Point((self.y+1 - b) / m, self.y+1)
		return inc_x, inc_y
	def print_to_screen(self):
		print str(self.x) + " , " + str(self.y)

	def clean(self):
		return Point(int(self.x), int(self.y))





class Subscriber():
	def __init__(self, x, y, z):
		if x == "/map":
			thread.start_new_thread(initMap, (z,))
		elif x == "/base_scan":
			thread.start_new_thread(initLaser, (z,))


def spin():
	while running:
		continue

def get_pose(x, d, s):
	return None


def move_function (angle, dist):
	l.theta += angle
	l.theta %= 2 * math.pi
	l.robot_loc.x += math.cos(l.theta) * dist
	l.robot_loc.y += math.sin(l.theta) * dist
	l.robot_loc.print_to_screen()
	calibrateScan()

def nearest_obstacle(theta):
	slope = math.tan(theta)
	
	star = l.robot_loc.clean()
	closest_grid = l.robot_loc.clean()

	inc_x, inc_y = l.robot_loc.inc(slope, l.robot_loc.y)
	closest_grid, star = closest(star, inc_x, inc_y)
	
	try:
		while l.m.grid[closest_grid.x][closest_grid.y] != 1.:
			inc_x, inc_y = star.inc(slope, l.robot_loc.y)
			closest_grid, star = closest(star, inc_x, inc_y)
	except IndexError:
		print 'out of bounds'
		closest_grid.print_to_screen()
		running = False
		return None

	return distance(closest_grid, l.robot_loc)


def closest(star, inc_x, inc_y):
	d1 = distance(inc_x, star)
	if inc_y == None:
		d2 = None
	else:
		d2 = distance(inc_y, star)


	if d2 == None or d1 < d2:
		star = inc_x
	else:
		star = inc_y

	closest_grid = star.clean()
	return closest_grid, star



def robot_distance(p0, p1):
    return math.sqrt((p0.x - p1.x)**2 + (p0.y - p1.y)**2)


def initMap(z):
	time.sleep(0.1)
	o = OccupancyGrid()
	f = open('occupancygrid.txt', 'r').read()
	o.data = json.loads(f)
	o.info.height = 194
	o.info.width = 259
	o.info.resolution = 1.0
	l.m = Map(o)
	for i in range(l.m.width):
		l.m.grid[0][i] = 1.
		l.m.grid[l.m.height-1][i] = 1.
	for i in range(l.m.height):
		l.m.grid[i][0] = 1.
		l.m.grid[i][l.m.width-1] = 1.
	calibrateScan()
	z(o)

def calibrateScan(robot):
	scan = LaserScan()
	scan.angle_min = 0.
	scan.angle_max = math.pi * 2
	scan.angle_increment = (scan.angle_max - scan.angle_min)/100.
	scan.ranges = []
	for i in range(100):
		t = i * scan.angle_increment + l.theta
		scan.ranges.append(nearest_obstacle(t))
	return scan

def initLaser(z):
	time.sleep(0.2)
	while running:
		z(l.scan)
