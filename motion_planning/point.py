class Point:
	def __init__(self, pos, pid = None, convex_segs = None):
		self.pos = pos
		self.pid = pid
		self.convex_segs = convex_segs

	def __str__(self):
		s = "xy: ({0}, {1}), cs: ".format(self.pos[0], self.pos[1])
		if self.convex_segs:
			for cs in self.convex_segs:
				s += "({0}, {1}), ".format(cs[0], cs[1])
			return s[0:-2]
		else:
			s += " None"
			return s