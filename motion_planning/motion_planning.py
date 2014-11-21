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
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math

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

	mp = MotionPlanning();

	robot = []
	for i in range(4):
		robot.append([random.random()*0.1, random.random()*0.1])
	x, y = zip(*robot)
	cx = sum(x)/len(x)
	cy = sum(y)/len(y)
	robot = [(p[0] - cx, p[1] - cy) for p in robot]
	robot = mp.graham_scan(robot)

	raw_obstacles = []
	obstacles = []
	for o in range(5):
		obstacle = []
		x = random.random()
		y = random.random()
		for i in range(5):
			obstacle.append([x + 0.4*random.random(), y + 0.4*random.random()])
		
		raw_obstacle = mp.graham_scan(obstacle)
		raw_obstacles.append(raw_obstacle)
		
		obstacle, vrobot = mp.gen_cspace(raw_obstacle, robot)
		obstacles.append(obstacle)

	# [pt1, pt2, obstacle_id]
	segments = []
	# [x, y, [obstacle_id1, seg_id], [obstacle_id2, seg_id2], id]
	control_points = []
	for i in range(len(obstacles)):
		ob = obstacles[i]
		start_index = len(control_points)
		start_seg_index = len(segments)
		for j in range(len(ob)-1):
			segments.append((ob[j], ob[j+1], i))
			control_points.append([ob[j][0], ob[j][1], [i, len(segments)-1], [i, len(segments)-2]])
		segments.append((ob[-1], ob[0], i))
		control_points.append([ob[-1][0], ob[-1][1], [i, len(segments)-1], [i, len(segments)-2]])
		control_points[start_index] = [ob[0][0], ob[0][1], [i, start_seg_index], [i, len(segments)-1]]

	# [x, y, [obstacle_id1, seg_id], [obstacle_id2, seg_id2], id]
	intersections = []
	for i in range(len(segments)):
		for j in range(i+1, len(segments)):
			if segments[i][2] == segments[j][2]:
				# same obstacle
				continue;
			p = mp.line_intersections(segments[i][0], segments[i][1], \
				segments[j][0], segments[j][1])
			if p != None:
				intersections.append([p[0], p[1], [segments[i][2], i], [segments[j][2], j]])

	# robot id is 0
	point_id = 1
	m_intersections = []
	for p in intersections:
		inside = False
		for i in range(len(obstacles)):
			ob = obstacles[i]
			if p[2][0] == i or p[3][0] == i:
				continue
			if mp.is_point_in_cpolygon(p, ob):
				inside = True
				break
		if not inside:
			p.append(point_id)
			point_id += 1
			m_intersections.append(p)

	m_control_points = []
	for p in control_points:
		inside = False
		for i in range(len(obstacles)):
			ob = obstacles[i]
			if p[2] == i:
				continue
			if mp.is_point_in_cpolygon(p, ob):
				inside = True
				break
		if not inside:
			p.append(point_id)
			point_id += 1
			m_control_points.append(p)

	edges_i = []
	# generate graph edge candidates between intersection points
	for i in range(len(m_intersections)):
		p1 = m_intersections[i]
		for j in range(i+1, len(m_intersections)):
			p2 = m_intersections[j]
			obs = set()
			obs.add(p1[2][0]); obs.add(p1[3][0]);
			obs.add(p2[2][0]); obs.add(p2[3][0]);
			# number of obstacles
			if len(obs) == 2:
				continue
			elif len(obs) == 3:
				# find the common obstacle
				ps = (p1[2], p1[3], p2[2], p2[3])
				for p in ps:
					if p[0] in obs:
						obs.remove(p[0])
					else:
						break
				ob_id = p[0]
				if p1[2][0] == ob_id:
					seg1 = p1[2][1]
				else:
					seg1 = p1[3][1]

				if p2[2][0] == ob_id:
					seg2 = p2[2][1]
				else:
					seg2 = p2[3][1]
				
				# if two points are on different segements
				if seg1 != seg2:
					continue

			crossed = False
			for i in range(len(segments)):
				if i == p1[2][1] or i == p1[3][1] or \
					i == p2[2][1] or i == p2[3][1]:
					continue
				if mp.line_intersections(p1, p2, segments[i][0], segments[i][1]) != None:
					crossed = True
					break
			if not crossed:
				edges_i.append(((p1[-1],p1[0],p1[1]), (p2[-1],p2[0],p2[1])))

	edges_c = []
	# generate graph edge candidates between control points
	for i in range(len(m_control_points)):
		p1 = m_control_points[i]
		for j in range(i + 1, len(m_control_points)):
			p2 = m_control_points[j]
			if p1[2][0] == p2[2][0]:
				# same obstacle, different segments
				if p1[2][1] != p2[2][1] and p1[2][1] != p2[3][1] and \
					p1[3][1] != p2[2][1] and p1[3][1] != p2[3][1]:
					continue

			crossed = False
			for s in range(len(segments)):
				if s == p1[2][1] or s == p1[3][1] or \
					s == p2[2][1] or s == p2[3][1]:
					continue
				seg = segments[s]
				if mp.line_intersections(p1, p2, seg[0], seg[1]) != None:
					crossed = True
					break
			if not crossed:
				edges_c.append(((p1[-1],p1[0],p1[1]), (p2[-1],p2[0],p2[1])))

	edges_ic = []
	# generate graph edge candidates between a control point and intersection point
	for i in range(len(m_control_points)):
		p1 = m_control_points[i]
		for j in range(len(m_intersections)):
			p2 = m_intersections[j]
			if p1[2][0] == p2[2][0] or p1[2][0] == p2[3][0]:
				# same obstacle, different segments
				if p1[2][1] != p2[2][1] and p1[2][1] != p2[3][1] and \
					p1[3][1] != p2[2][1] and p1[3][1] != p2[3][1]:
					continue

			crossed = False
			for s in range(len(segments)):
				if s == p1[2][1] or s == p1[3][1] or \
					s == p2[2][1] or s == p2[3][1]:
					continue
				seg = segments[s]
				if mp.line_intersections(p1, p2, seg[0], seg[1]) != None:
					crossed = True
					break
			if not crossed:
				edges_ic.append(((p1[-1],p1[0],p1[1]), (p2[-1],p2[0],p2[1])))

	edges_r = []
	# generate graph edge candidates between robot and any other point
	p1 = [robot[0][0], robot[0][1], 0]
	for i in range(len(m_control_points)):
		p2 = m_control_points[i]
		crossed = False
		for s in range(len(segments)):
			if s == p2[2][1] or s == p2[3][1]:
				continue
			seg= segments[s]
			if mp.line_intersections(p1, p2, seg[0], seg[1]) != None:
				crossed = True
				break
		if not crossed:
			edges_r.append(((p1[-1],p1[0],p1[1]), (p2[-1],p2[0],p2[1])))


	edges_d = []
	# generate graph edge candidates between desination and any other point
	p1 = [1.4, 1.4, point_id]
	point_id += 1
	valid_destination = True
	for ob in obstacles:
		if mp.is_point_in_cpolygon(p1, ob):
			valid_destination = False
			break
	for i in range(len(m_control_points)):
		p2 = m_control_points[i]
		crossed = False
		for s in range(len(segments)):
			if s == p2[2][1] or s == p2[3][1]:
				continue
			seg= segments[s]
			if mp.line_intersections(p1, p2, seg[0], seg[1]) != None:
				crossed = True
				break
		if not crossed:
			edges_d.append(((p1[-1],p1[0],p1[1]), (p2[-1],p2[0],p2[1])))

	fig = plt.figure()
	ax = fig.add_subplot(111, axisbg = 'black')
	plt.xlim((-0.2, 1.5))
	plt.ylim((-0.2, 1.5))

	for obstacle in raw_obstacles:
		mp.draw_polygon(ax, obstacle, 'r')

	for obstacle in obstacles:
		mp.draw_polygon(ax, obstacle, 'y')

	mp.draw_polygon(ax, robot, 'w', alpha = 0.5)
	ax.scatter(robot[0][0],robot[0][1], s = 20, c = 'r')

	x,y,osid1,osid2,pid = zip(*m_control_points)
	ax.scatter(x,y, s = 50, c = 'g')

	if len(m_intersections) > 0:
		x,y,osid1,osid2,pid = zip(*m_intersections)
		ax.scatter(x,y, s = 20, c = 'r')

	for edge in edges_i:
		pid,x,y = zip(*edge)
		ax.plot(x, y, 'b--', alpha = 0.5)

	for edge in edges_c:
		pid,x,y = zip(*edge)
		ax.plot(x, y, 'g--', alpha = 0.5)

	for edge in edges_ic:
		pid,x,y = zip(*edge)
		ax.plot(x, y, 'm--', alpha = 0.5)	

	for edge in edges_r:
		pid,x,y = zip(*edge)
		ax.plot(x, y, 'r--', alpha = 0.5)

	for edge in edges_d:
		pid,x,y = zip(*edge)
		ax.plot(x, y, 'w--', alpha = 0.5)

	all_edges = edges_i + edges_c + edges_ic + edges_r + edges_d
	
	import graph
	g = graph.Graph()
	for edge in all_edges:
		v1 = edge[0][0]
		v2 = edge[1][0]
		dist = math.sqrt((edge[0][1] - edge[1][1])**2 + (edge[0][2] - edge[1][2])**2)
		g.insert(v1, v2, dist)

	#g.dijkstra(0)

	plt.show()




