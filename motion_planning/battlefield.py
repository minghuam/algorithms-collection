import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math

from convexhull import *
from point import *
from segment import *
from graph import *

class BattleField:
	def __init__(self):
		# make robot
		self.robot = self.make_random_robot(0, 4)

		# make obstacles
		self.obstacles = self.make_random_obstacles(self.robot, 5, 10)

		self.__gen_base_points()
		self.__gen_base_edges()

	def run(self):
		fig = plt.figure()
		self.ax = fig.add_subplot(111, axisbg = 'black')
		fig.canvas.mpl_connect('button_press_event', self.on_click)
		
		self.__update(1.4, 1.4)

		plt.show()

	def on_click(self, event):
		if (event.xdata != None) and (event.ydata != None):
			self.__update(event.xdata, event.ydata)

	def __update(self, dx, dy):
		self.__update_destination(dx, dy)
		self.__search_shortest_path()

		self.ax.cla()
		plt.xlim((-0.2, 1.5))
		plt.ylim((-0.2, 1.5))

		self.__draw()
		plt.draw()

	def __draw(self):
		# draw robot
		self.ax.add_patch(Polygon(self.robot.cs_points, color = 'w', alpha = 0.5))
		self.ax.scatter(self.robot.anchor[0], self.robot.anchor[1], s = 20, c = 'r')

		# draw obstacles
		for ob in self.obstacles:
			self.ax.add_patch(Polygon(ob.raw_points, color = 'r', alpha = 0.5))
			self.ax.add_patch(Polygon(ob.cs_points, color = 'y', alpha = 0.45))

		# draw control points
		#for p in self.control_points:
		for p in self.m_control_points:
			self.ax.scatter(p.pos[0], p.pos[1], s = 30, c = 'g')

		# draw intersections
		#for p in self.intersections:
		for p in self.m_intersections:
			self.ax.scatter(p.pos[0], p.pos[1], s = 30, c = 'r')

		for e in self.edges_i:
			x = (e[0].pos[0], e[1].pos[0])
			y = (e[0].pos[1], e[1].pos[1])
			self.ax.plot(x, y, 'r--', alpha = 0.75)

		for e in self.edges_c:
			x = (e[0].pos[0], e[1].pos[0])
			y = (e[0].pos[1], e[1].pos[1])
			self.ax.plot(x, y, 'g--', alpha = 0.75)

		for e in self.edges_ic:
			x = (e[0].pos[0], e[1].pos[0])
			y = (e[0].pos[1], e[1].pos[1])
			self.ax.plot(x, y, 'y--', alpha = 0.75)

		for e in self.edges_r:
			x = (e[0].pos[0], e[1].pos[0])
			y = (e[0].pos[1], e[1].pos[1])
			self.ax.plot(x, y, 'r--', alpha = 0.5)

		for e in self.edges_d:
			x = (e[0].pos[0], e[1].pos[0])
			y = (e[0].pos[1], e[1].pos[1])
			self.ax.plot(x, y, 'w--', alpha = 0.5)	

		# draw start and end
		self.ax.scatter(self.start.pos[0], self.start.pos[1], s = 50, c = 'r')
		self.ax.scatter(self.end.pos[0], self.end.pos[1], s = 50, c = 'w')

		x = [p.pos[0] for p in self.shortest_path]
		y = [p.pos[1] for p in self.shortest_path]
		self.ax.plot(x, y, 'y-', alpha = 0.75, linewidth = 3)

	def __search_shortest_path(self):
		"""
			Graphs search
		"""
		self.shortest_path = []

		all_edges = self.edges_i + self.edges_c + self.edges_ic + self.edges_r + self.edges_d
		g = Graph()
		for e in all_edges:
			dist = math.sqrt((e[0].pos[0] - e[1].pos[0])**2 + (e[0].pos[1] - e[1].pos[1])**2)
			g.insert(e[0], e[1], dist)

		dist, previous = g.dijkstra(self.start)
		self.shortest_path.append(self.end)
		v = self.end
		while previous[v] != None:
			self.shortest_path.append(previous[v])
			v = previous[v]

	def __update_destination(self, x, y):
		
		# validation
		for ob in self.obstacles:
			if ob.is_inside((x,y)):
				print "Invalid destination!"
				return

		self.end.pos = (x,y)
		self.edges_d = []
		# generate edges between destination and all other points
		p1 = self.end
		all_points = self.m_intersections + self.m_control_points
		for i in range(len(all_points)):
			p2 = all_points[i]
			crossed = False
			for seg in self.segments:
				if seg.convex_seg[1] == p2.convex_segs[0][1] or \
					seg.convex_seg[1] == p2.convex_segs[1][1]:
					continue
				temp = Segment(p1.pos, p2.pos)
				if Segment.intersections(temp, seg) != None:
					crossed = True
					break
			if not crossed:
				self.edges_d.append((p1, p2))
		
		# between start and end
		p1 = self.start
		p2 = self.end
		crossed = False
		for seg in self.segments:
			temp = Segment(p1.pos, p2.pos)
			if Segment.intersections(temp, seg) != None:
				crossed = True
				break
		if not crossed:
			self.edges_d.append((p1, p2))


	def __gen_base_edges(self):
		"""
			Generate edges from all points
		"""
		self.edges_i = []
		self.edges_c = []
		self.edges_ic = []
		self.edges_r = []
		self.edges_d = []

		# generate graph edge candidates between intersection points
		for i in range(len(self.m_intersections)):
			p1 = self.m_intersections[i]
			for j in range(i+1, len(self.m_intersections)):
				p2 = self.m_intersections[j]
				ob_ids = set()
				ob_ids.add(p1.convex_segs[0][0]); ob_ids.add(p1.convex_segs[1][0]);
				ob_ids.add(p2.convex_segs[0][0]); ob_ids.add(p2.convex_segs[1][0]);
				# number of obstacles
				if len(ob_ids) == 2:
					continue
				elif len(ob_ids) == 3:
					# find the common obstacle
					ids = (p1.convex_segs[0][0], p1.convex_segs[1][0], p2.convex_segs[0][0], p2.convex_segs[1][0])
					for ob_id in ids:
						if ob_id in ob_ids:
							ob_ids.remove(ob_id)
						else:
							break
					common_id = ob_id
					if p1.convex_segs[0][0] == common_id:
						seg_id1 = p1.convex_segs[0][1]
					else:
						seg_id1 = p1.convex_segs[1][1]
					if p2.convex_segs[0][0] == common_id:
						seg_id2 = p2.convex_segs[0][1]
					else:
						seg_id2 = p2.convex_segs[1][1]
					
					# two points are on different segments
					if seg_id1 != seg_id2:
						continue

				# cross test
				crossed = False
				for seg in self.segments:
					if seg.convex_seg[1] == p1.convex_segs[0][1] or \
						seg.convex_seg[1] == p1.convex_segs[1][1] or \
						seg.convex_seg[1] == p2.convex_segs[0][1] or \
						seg.convex_seg[1] == p2.convex_segs[1][1]:
						continue
					temp = Segment(p1.pos, p2.pos)
					if Segment.intersections(temp, seg) != None:
						crossed = True
						break
				if not crossed:
					self.edges_i.append((p1, p2))

		# generate graph edge candidates between control points
		for i in range(len(self.m_control_points)):
			p1 = self.m_control_points[i]
			for j in range(i+1, len(self.m_control_points)):
				p2 = self.m_control_points[j]
				# same obstacle
				if p1.convex_segs[0][0] == p2.convex_segs[0][0]:
					# different segments
					if p1.convex_segs[0][1] != p2.convex_segs[0][1] and \
						p1.convex_segs[0][1] != p2.convex_segs[1][1] and \
						p1.convex_segs[1][1] != p2.convex_segs[0][1] and \
						p1.convex_segs[1][1] != p2.convex_segs[1][1]:
						continue
				# cross test
				crossed = False
				for seg in self.segments:
					if seg.convex_seg[1] == p1.convex_segs[0][1] or \
						seg.convex_seg[1] == p1.convex_segs[1][1] or \
						seg.convex_seg[1] == p2.convex_segs[0][1] or \
						seg.convex_seg[1] == p2.convex_segs[1][1]:
						continue
					temp = Segment(p1.pos, p2.pos)
					if Segment.intersections(temp, seg) != None:
						crossed = True
						break
				if not crossed:
					self.edges_c.append((p1, p2))

		# generate graph edge candidates between control points and intersections
		for i in range(len(self.m_control_points)):
			p1 = self.m_control_points[i]
			for j in range(len(self.m_intersections)):
				p2 = self.m_intersections[j]
				# same obstacle
				if p1.convex_segs[0][0] == p2.convex_segs[0][0] or \
					p1.convex_segs[0][0] == p2.convex_segs[1][0]:
					# different segments
					if p1.convex_segs[0][1] != p2.convex_segs[0][1] and \
						p1.convex_segs[0][1] != p2.convex_segs[1][1] and \
						p1.convex_segs[1][1] != p2.convex_segs[0][1] and \
						p1.convex_segs[1][1] != p2.convex_segs[1][1]:
						continue
				# cross test
				crossed = False
				for seg in self.segments:
					if seg.convex_seg[1] == p1.convex_segs[0][1] or \
						seg.convex_seg[1] == p1.convex_segs[1][1] or \
						seg.convex_seg[1] == p2.convex_segs[0][1] or \
						seg.convex_seg[1] == p2.convex_segs[1][1]:
						continue
					temp = Segment(p1.pos, p2.pos)
					if Segment.intersections(temp, seg) != None:
						crossed = True
						break
				if not crossed:
					self.edges_ic.append((p1, p2))

		all_points = self.m_control_points + self.m_intersections
		# generate edge candidates between robot and other points
		p1 = self.start
		for i in range(len(all_points)):
			p2 = all_points[i]
			crossed = False
			for seg in self.segments:
				if seg.convex_seg[1] == p2.convex_segs[0][1] or \
					seg.convex_seg[1] == p2.convex_segs[1][1]:
					continue
				temp = Segment(p1.pos, p2.pos)
				if Segment.intersections(temp, seg) != None:
					crossed = True
					break
			if not crossed:
				self.edges_r.append((p1, p2))

	def __gen_base_points(self):
		""" Merge obstacles and calculate key points: 
				segments, control points, intersection points
		"""
		self.segments = []
		self.control_points = []
		self.intersections = []
		self.m_control_points = []
		self.m_intersections = []

		point_id = 0

		self.start = Point(self.robot.anchor, point_id)
		point_id += 1

		self.end = Point(self.robot.anchor, point_id)
		point_id += 1

		# segments and control points
		for ob in self.obstacles:
			start_cp_id = len(self.control_points)
			start_seg_id = len(self.segments)
			seg_id = start_seg_id
			previous_seg_id = -1
			for j in range(len(ob.cs_points) - 1):
				# start, end, (convex_id, seg_id)
				seg = Segment(ob.cs_points[j], ob.cs_points[j+1], (ob.convex_id, seg_id))
				self.segments.append(seg)
				# pos, ((convex_id, seg_id1), (convex_id, seg_id2))
				cp = Point(ob.cs_points[j], point_id, ((ob.convex_id, previous_seg_id),(ob.convex_id, seg_id)))
				self.control_points.append(cp)
				point_id += 1

				previous_seg_id = seg_id
				seg_id += 1

			seg = Segment(ob.cs_points[-1], ob.cs_points[0], (ob.convex_id, seg_id))
			self.segments.append(seg)
			cp = Point(ob.cs_points[-1], point_id, ((ob.convex_id, previous_seg_id),(ob.convex_id, seg_id)))
			self.control_points.append(cp)
			point_id += 1

			cp = Point(ob.cs_points[0], point_id, ((ob.convex_id, seg_id),(ob.convex_id, start_seg_id)))
			self.control_points[start_cp_id] = cp
			point_id += 1

		# segments intersections
		for i in range(len(self.segments)):
			for j in range(i+1, len(self.segments)):
				if self.segments[i].convex_seg[0] == self.segments[j].convex_seg[0]:
					# same obstacle
					continue
				seg1 = self.segments[i]
				seg2 = self.segments[j]
				pos = Segment.intersections(seg1, seg2)
				if pos != None:
					self.intersections.append(Point(pos, point_id, (seg1.convex_seg, seg2.convex_seg)))
				point_id += 1

		# merge control points
		for p in self.control_points:
			inside = False
			for ob in self.obstacles:
				if p.convex_segs[0][0] == ob.convex_id:
					continue
				if ob.is_inside(p.pos):
					inside = True
					break
			if not inside:
				self.m_control_points.append(p)

		# merge intersections
		for p in self.intersections:
			inside = False
			for ob in self.obstacles:
				if p.convex_segs[0][0] == ob.convex_id or \
					p.convex_segs[1][0] == ob.convex_id:
					continue
				if ob.is_inside(p.pos):
					inside = True
					break
			if not inside:
				self.m_intersections.append(p)


	def make_random_robot(self, robot_id, size):
		points = []
		for i in range(size):
			points.append([random.random()*0.1, random.random()*0.1])
		x, y = zip(*points)
		cx = sum(x)/len(x)
		cy = sum(y)/len(y)
		points = [(p[0] - cx, p[1] - cy) for p in points]

		return ConvexHull(robot_id, points)

	def make_random_obstacles(self, robot, count, size):
		obstacles = []
		convex_id = robot.convex_id + 1
		for o in range(count):
			points = []
			x = random.random()
			y = random.random()
			for i in range(size):
				points.append([x + 0.4*random.random(), y + 0.4*random.random()])
			
			obstacles.append(ConvexHull(convex_id, points, robot))
			convex_id += 1

		return obstacles

if __name__ == '__main__':
	bf = BattleField()
	bf.run()