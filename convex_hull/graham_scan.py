"""
	Convex Hull Algorithm: Graham Scan, O(nlogn)
	http://en.wikipedia.org/wiki/Graham_scan
	http://www.dcs.gla.ac.uk/~pat/52233/slides/Hull1x1.pdf
"""
import random
import functools
import numpy as np
import matplotlib.pyplot as plt


def draw_points(points):
	plt.scatter(points[0,:], points[1,:])
	plt.show()

def swap(points, idx1, idx2):
	temp = points[idx1]
	points[idx1] = points[idx2]
	points[idx2] = temp

def orientation(p, q, r):
	val = (q[1] - p[1]) * (r[0] - q[0]) - \
		(q[0] - p[0]) * (r[1] - q[1]);
	if val == 0:
		# colinear
		return 0
	elif val > 0:
		# cw
		return -1
	else:
		# ccw
		return 1
pt0 = [0,0]

def compare_angle(pt1, pt2):
	ort = orientation(pt0, pt1, pt2)
	if ort == 0:
		dist1 = (pt0[0] - pt1[0]) ** 2 + (pt0[1] - pt1[1]) ** 2
		dist2 = (pt0[0] - pt2[0]) ** 2 + (pt0[1] - pt2[1]) ** 2
		if dist1 >= dist2:
			# discard closer point
			return -1
		else:
			return 1
	return -ort

def graham_scan(points):
	"""
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
	anchor = 0
	for i in xrange(1, n):
		if points[i][1] < points[anchor][1] or \
			(points[i][1] == points[anchor][1] and \
				points[i][0] < points[anchor][0]):
			anchor = i
	# swap
	swap(points, 0, anchor)
	
	# step two: sort
	pt0 = points[0]
	#points[1:] = sorted(points[1:], key = functools.cmp_to_key(compare_angle))
	
	return anchor

if __name__ == '__main__':

	points = []
	for i in range(10):
		points.append([random.random(), random.random()])
		
	x = [p[0] for p in points]
	y = [p[1] for p in points]
	plt.scatter(x, y, color = 'b')
	a = graham_scan(points)
	plt.scatter(points[a][0], points[a][1], color = 'r')

	plt.show()
