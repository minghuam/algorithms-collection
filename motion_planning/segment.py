"""
	Line segment data structure
"""

from point import *

class Segment:
	def __init__(self, start_pos, end_pos, convex_seg = None):
		self.start = Point(start_pos, convex_seg)
		self.end = Point(end_pos, convex_seg)
		self.convex_seg = convex_seg

	def __str__(self):
		s = "cs: {0}, {1} => {2}".format(self.convex_seg, self.start, self.end)
		return s

	@staticmethod
	def intersections(seg1, seg2):
		"""
			Calculate intersections of two line segments
			http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
		"""
		x1 = float(seg1.start.pos[0]); y1 = float(seg1.start.pos[1]);
		x2 = float(seg1.end.pos[0]); y2 = float(seg1.end.pos[1]);
		x3 = float(seg2.start.pos[0]); y3 = float(seg2.start.pos[1]);
		x4 = float(seg2.end.pos[0]); y4 = float(seg2.end.pos[1]);

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