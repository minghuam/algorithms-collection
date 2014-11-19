"""
	Convex Hull Algorithm Demo: Graham Scan, O(nlogn)
	http://en.wikipedia.org/wiki/Graham_scan
	http://www.dcs.gla.ac.uk/~pat/52233/slides/Hull1x1.pdf
"""
import random
import functools
import numpy as np
import matplotlib.pyplot as plt

class GrahamScan:

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

	def graham_scan(self, points):
		""" Graham Scan Algorithm main routine
			Args: 
				points: n points
			Returns:
				m convex hull points
		"""
		
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

if __name__ == '__main__':
	gs = GrahamScan()
	gs.run()