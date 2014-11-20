"""
	Shortest path motion planning
"""

"""
	Convex Hull Algorithm Demo: Graham Scan, O(nlogn)
	http://en.wikipedia.org/wiki/Graham_scan
	http://www.dcs.gla.ac.uk/~pat/52233/slides/Hull1x1.pdf
"""
import random
import functools
import copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

class MotionPlanning:

	def __init__(self):
		self.anchor = [0, 0]
		self.points = []
		# initialize with three points
		for i in range(3):
			self.points.append([random.random(), random.random()])
			#points.append([0.2*i, 0.2*i])	

	def run(self):	
		fig = plt.figure()
		fig.canvas.mpl_connect('button_press_event', self.on_click)
		self.update()
		plt.show()

	def update(self):
		plt.clf()
		x, y = zip(*self.points)
		#area = [10*a for a in range(len(points))]
		plt.scatter(x, y, s = 60, c = 'b')

		cpoints = self.graham_scan(self.points)
		plt.scatter(self.anchor[0], self.anchor[1], s = 20, color = 'r')

		x, y = zip(*cpoints)
		plt.plot(x, y, 'g')
		plt.plot((x[0], x[-1]), (y[0], y[-1]), 'g')

		plt.draw()

	def on_click(self, event):
		if (event.xdata != None) and (event.ydata != None):
			self.points.append([event.xdata, event.ydata])
			self.update()

	def swap(self, points, idx1, idx2):
		temp = points[idx1]
		points[idx1] = points[idx2]
		points[idx2] = temp

	def ccw(self, p, q, r):
		val = (q[0] - p[0]) * (r[1] - q[1]) - \
			(q[1] - p[1]) * (r[0] - q[0]);
		if val == 0:
			# colinear
			return 0
		elif val > 0:
			# ccw
			return 1
		else:
			# cw
			return -1

	def compare_angle(self, pt1, pt2):
		ret = self.ccw(self.anchor, pt1, pt2)
		if ret == 0:
			dist1 = (self.anchor[0] - pt1[0]) ** 2 + (self.anchor[1] - pt1[1]) ** 2
			dist2 = (self.anchor[0] - pt2[0]) ** 2 + (self.anchor[1] - pt2[1]) ** 2
			if dist1 >= dist2:
				# discard closer point
				return -1
			else:
				return 1
		return -ret

	def graham_scan(self, pts):
		""" Graham Scan Algorithm main routine
			Args: 
				points: n points
			Returns:
				m convex hull points
		"""
		
		points = copy.deepcopy(pts)
		n = len(points)
		if n < 3:
			print 'at least three points are needed!'
			return None

		# step one: find the lowest anchor point, O(n)
		anchor_i = 0
		for i in xrange(1, n):
			if points[i][1] < points[anchor_i][1] or \
				(points[i][1] == points[anchor_i][1] and \
					points[i][0] < points[anchor_i][0]):
				anchor_i = i
		# swap
		self.swap(points, 0, anchor_i)
		
		# step two: sort
		# TODO: no globals
		self.anchor = points[0]
		points[1:] = sorted(points[1:], key = functools.cmp_to_key(self.compare_angle))

		# step three: find convex hull points
		epoints = [points[n-1]]
		epoints.extend(points)
		m = 1
		for i in xrange(2, n+1):
			while self.ccw(epoints[m-1], epoints[m], epoints[i]) <= 0:
				if m > 1:
					m -= 1
				elif i == n:
					break
				else:
					i += 1
			m += 1
			self.swap(epoints, m, i)

		return epoints[1:m+1]

	def draw_polygon(self, ax, points, color = 'g', alpha = 0.5):
		ax.add_patch(Polygon(points, color = color, alpha = alpha))
		#x,y = zip(*points)
		#ax.plot(x, y, 'g')
		#ax.plot((x[0], x[-1]), (y[0], y[-1]), 'g')
		#ax.scatter(points[0][0], points[0][1], s = 20, c = 'r')

	def line_intersections(self, pt1, pt2, pt3, pt4):
		"""
			Calculate intersections of two line segments
			http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
		"""
		x1 = float(pt1[0]); y1 = float(pt1[1]);
		x2 = float(pt2[0]); y2 = float(pt2[1]);
		x3 = float(pt3[0]); y3 = float(pt3[1]);
		x4 = float(pt4[0]); y4 = float(pt4[1]);

		d = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1)
		na = (x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)
		nb = (x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)

		if d == 0:
			# parallel
			if (na == 0) and (nb == 0):
				# TODO: coincident
				return None
		else:
			ua = na/d
			ub = nb/d
			if ua >= 0.0 and ua <= 1.0 and ub >= 0.0 and ub <= 1.0:
				x = x1 + ua*(x2 - x1)
				y = y1 + ua*(y2 - y1)
				return (x, y)
			else:
				return None

	def is_point_in_cpolygon(self, pt, points):
		"""
			Check if point is in convex polygon
		"""
		c0 = self.ccw(pt, points[0], points[1])
		if c0 == 0:
			return False
		for i in range(1, len(points)-1):
			c1 = self.ccw(pt, points[i], points[i+1])
			if c1 != c0:
				return False
		c1 = self.ccw(pt, points[len(points)-1], points[0])
		if c1 != c0:
			return False
		else:
			return True

	def gen_cspace(self, obstacle, robot):

		# mirror robot
		vrobot = [[robot[0][0] - p[0], robot[0][1] - p[1]] for p in robot]

		vspace = []
		for p in obstacle:
			for r in vrobot:
				vspace.append([p[0] + r[0], p[1] + r[1]])
		vspace = self.graham_scan(vspace)

		return vspace, vrobot

if __name__ == '__main__':
	#mp = MotionPlanning()
	#mp.run()

	fig = plt.figure()
	ax = fig.add_subplot(111)

	mp = MotionPlanning();


	#p1 = (1,1); p2 = (2,2); p3 = (3,3); p4 = (4,4)
	#p1 = (1,1); p2 = (3,3); p3 = (1,3); p4 = (3,1)
	#print mp.line_intersections(p1, p2, p3, p4)

	obstacles = []
	for o in range(3):
		obstacle = []
		for i in range(4):
			obstacle.append([random.random(), random.random()])
		obstacle = mp.graham_scan(obstacle)
		obstacles.append(obstacle)


	for obstacle in obstacles:
		mp.draw_polygon(ax, obstacle, 'r')

	# [pt1, pt2, obstacle_id]
	segments = []
	# [pt, obstacle_id, obstacle_id]
	control_points = []
	for i in range(len(obstacles)):
		ob = obstacles[i]
		for j in range(len(ob)-1):
			control_points.append((ob[j][0], ob[j][1], i, i))
			segments.append((ob[j], ob[j+1], i))
		segments.append((ob[-1], ob[0], i))
		control_points.append((ob[-1][0], ob[-1][1], i, i))

	# [pt1, pt2, obstacle_id1, obstacle_id2]
	intersections = []
	for i in range(len(segments)):
		for j in range(i+1, len(segments)):
			if segments[i][2] == segments[j][2]:
				# sage obstacle
				continue;
			p = mp.line_intersections(segments[i][0], segments[i][1], \
				segments[j][0], segments[j][1])
			if p != None:
				intersections.append((p[0], p[1], segments[i][2], segments[j][2]))

	m_intersections = []
	for p in intersections:
		inside = False
		for i in range(len(obstacles)):
			ob = obstacles[i]
			if p[2] == i or p[3] == i:
				continue
			if mp.is_point_in_cpolygon(p, ob):
				inside = True
				break
		if not inside:
			m_intersections.append(p)

	m_control_points = []
	for p in control_points:
		inside = False
		for i in range(len(obstacles)):
			ob = obstacles[i]
			if p[2] == i or p[3] == i:
				continue
			if mp.is_point_in_cpolygon(p, ob):
				inside = True
				break
		if not inside:
			m_control_points.append(p)

	x,y,id1,id2 = zip(*m_control_points)
	ax.scatter(x,y, s = 50, c = 'g')

	x,y,id1,id2 = zip(*m_intersections)
	ax.scatter(x,y, s = 20, c = 'r')

	# generate graph edge candidates
	edges = []
	for i in range(len(m_intersections)):
		p1 = m_intersections[i]
		for j in range(i+1, len(m_intersections)):
			p2 = m_intersections[j]
			if (p1[2] == p2[2] and p1[3] == p2[3]) or \
				(p1[2] == p2[3] and p1[3] == p2[2]):
				# intersections are from two obstacles only
				continue
			edges.append((p1[0:2], p2[0:2]))

	for edge in edges:
		x,y = zip(*edge)
		ax.plot(x, y, 'g--')


	"""
	poly = [(1,1), (3,1), (3,5), (1,5)]
	pts = [(0,0), (8,8), (-1,3),(2,2), (1,1), (2,1)]
	for p in pts:
		print p
		inside = mp.is_point_in_cpolygon(p, poly)
		print 'inside =', inside
	"""
	"""
	robot = []
	for i in range(5):
		robot.append([random.random(), random.random()])
	robot = mp.graham_scan(robot)
	mp.draw_polygon(ax, robot, 'b')

	vspace, vrobot = mp.gen_cspace(obstacle, robot)
	#mp.draw_polygon(ax, vrobot, 'g')
	mp.draw_polygon(ax, vspace, 'g', alpha = 0.3)
	"""

	plt.show()




