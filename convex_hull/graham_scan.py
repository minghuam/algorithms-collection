"""
	Convex Hull Algorithm: Graham Scan, O(nlogn)
	http://en.wikipedia.org/wiki/Graham_scan
	http://www.dcs.gla.ac.uk/~pat/52233/slides/Hull1x1.pdf
"""
import numpy as np
import matplotlib.pyplot as plt

def draw_points(points):
	plt.scatter(points[0,:], points[1,:])
	plt.show()

def graham_scan(points):
	"""
		Args: 
			points: n points' coordinates in 2xn numpy array
		Returns:
			2xm convex hull points
	"""

	n = points.shape[1]
	if n < 3:
		print 'at least three points are needed!'
		return None

	# step one: find the lowest anchor point, O(n)
	anchor = 0
	for i in xrange(1, n):
		if points[1,i] < points[1,anchor] or \
			(points[1,i] == points[1,anchor] and \
				points[0,i] < points[0,anchor]):
			anchor = i
	return anchor

	# step two: compute angle and sort



if __name__ == '__main__':
	points = np.random.rand(2, 10)
	
	plt.scatter(points[0,:], points[1,:], color = 'b')
	a = graham_scan(points)
	plt.scatter(points[0,a], points[1,a], color = 'r')

	plt.show()