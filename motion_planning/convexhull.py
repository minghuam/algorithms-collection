"""	Convex Hull Utilitites, Graham Scan, O(nlogn)
	http://en.wikipedia.org/wiki/Graham_scan
"""
import copy
import functools

class ConvexHull:

	def __init__(self, convex_id, points, robot = None):
		self.convex_id = convex_id
		self.raw_points = self.__graham_scan(points)
		if robot != None:
			self.cs_points = self.__build_cspace(points, robot.cs_points)
		else:
			self.cs_points = self.raw_points
		self.anchor = self.cs_points[0]

	def __str__(self):
		s = "id: {0}, anchor: {1}\n".format(self.convex_id, self.anchor)
		s += "raw: {0}\n".format(self.raw_points)
		s += "cs: {0}".format(self.cs_points)
		return s 

	def is_inside(self, pt, raw = False):
		"""
			Check if point is in convex polygon
		"""
		points = self.cs_points
		if raw == True:
			points = self.raw_points
			
		c0 = self.__ccw(pt, points[0], points[1])
		if c0 == 0:
			return False
		for i in range(1, len(points)-1):
			c1 = self.__ccw(pt, points[i], points[i+1])
			if c1 != c0:
				return False
		c1 = self.__ccw(pt, points[len(points)-1], points[0])
		if c1 != c0:
			return False
		else:
			return True		

	def __build_cspace(self, points, robot_points):
		"""
			Generate configuration space for our lovely robot
		"""
		# mirror robot
		vrobot_points = [[robot_points[0][0] - p[0], robot_points[0][1] - p[1]] for p in robot_points]

		vspace = []
		for p in points:
			for r in vrobot_points:
				vspace.append([p[0] + r[0], p[1] + r[1]])
		vspace = self.__graham_scan(vspace)

		return vspace	

	def __graham_scan(self, pts):
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
		self.__swap(points, 0, anchor_i)
		
		# step two: sort
		# TODO: no globals
		self.anchor = points[0]
		points[1:] = sorted(points[1:], key = functools.cmp_to_key(self.__compare_angle))

		# step three: find convex hull points
		epoints = [points[n-1]]
		epoints.extend(points)
		m = 1
		for i in xrange(2, n+1):
			while self.__ccw(epoints[m-1], epoints[m], epoints[i]) <= 0:
				if m > 1:
					m -= 1
				elif i == n:
					break
				else:
					i += 1
			m += 1
			self.__swap(epoints, m, i)

		return epoints[1:m+1]

	def __swap(self, points, idx1, idx2):
		temp = points[idx1]
		points[idx1] = points[idx2]
		points[idx2] = temp

	def __ccw(self, p, q, r):
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

	def __compare_angle(self, pt1, pt2):
		ret = self.__ccw(self.anchor, pt1, pt2)
		if ret == 0:
			dist1 = (self.anchor[0] - pt1[0]) ** 2 + (self.anchor[1] - pt1[1]) ** 2
			dist2 = (self.anchor[0] - pt2[0]) ** 2 + (self.anchor[1] - pt2[1]) ** 2
			if dist1 >= dist2:
				# discard closer point
				return -1
			else:
				return 1
		return -ret

