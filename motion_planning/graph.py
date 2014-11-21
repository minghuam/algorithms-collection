class Graph:
	def __init__(self):
		# key: vertex, value: neighbors
		self.vertices = dict()
		self.edges = dict()

	def insert(self, v1, v2, distance):
		""" Insert an edge to graph
		"""
		if v1 not in self.vertices:
			self.vertices[v1] = []
		if v2 not in self.vertices:
			self.vertices[v2] = []

		self.vertices[v1].append((v2, distance))
		self.vertices[v2].append((v1, distance))

	def print_graph(self):
		for v in self.vertices:
			print v, len(self.vertices[v]), self.vertices[v]

	def dijkstra(self, source):
		dist = dict()
		previous = dict()
		unvisited = set()
		
		for v in self.vertices:
			dist[v] = float('Inf')
			previous[v] = None
			unvisited.add(v)

		dist[source] = 0

		while len(unvisited) > 0:
			# find u with minimum dist
			min_dist = float('Inf')
			for v in unvisited:
				if dist[v] <= min_dist:
					min_dist = dist[v]
					u = v
			# remove u
			unvisited.remove(u)

			neighbors = self.vertices[u]
			for vw in neighbors:
				v = vw[0]; w = vw[1];
				d = dist[u] + w
				if d < dist[v]:
					dist[v] = d
					previous[v] = u

		return dist, previous

		