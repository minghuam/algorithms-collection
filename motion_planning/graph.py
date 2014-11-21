class Graph:
	def __init__(self):
		# key: vertex, value: neighbors
		self.vertices = dict()

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

	def dijkstra(self, source, dest):
		dist = dict()

		for v in self.vertices:
			dist[v] = float('Inf')
			unvisited.append(v)

		dist[source] = 0.0

		for i in range(len(self.vertices)):
			# find the closest & unvisited vertex
			min_dist = float('Inf')
			for v in self.vertices:
				if visited[v] == False and dist[v] <= min_dist:
					min_dist = dist[v]
					u = v
			# mark visited
			visited[u] = True

			# update distance of neighbors
			neighbors = self.vertices[u]
			for vd in neighbors:
				v = vd[0]; w = vd[1];
				d = dist[u] + w;
				if visited[v] == False and d < dist[v]:
					dist[v] = d
		