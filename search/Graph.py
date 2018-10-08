class Graph():
	def __init__(self, initial_node=False, visited = [], *args, **kwargs):
		super(Graph, self).__init__(*args, **kwargs)
		if initial_node:
			visited.append(initial_node)
			self.visited = visited
			findSolution

	def findSolution(self, current_node):
		for neighbor in current_node.neighbors:
			if neighbor not in visited:
				neighbor_node = Node(neighbor, current_node)
				self.visited.append(neighbor_node)
				if(problem.isSolution(neighbor_node)):
					node_aux = neighbor_node
					path = [node_aux]
					while node_aux != False:
						path.append(node_aux.getPredecesor().getMovement())
						node_aux = node_aux.getPredecesor()

	class Node():
		def __init__(self, node_info, node_predecesor = False, *args, **kwargs):
			super(Node, self).__init__(*args, **kwargs)
			self.info = node_info
			self.predecesor = node_predecesor

		def getMovement(self):
			return self.info[1]
		def getPosition(self):
			return self.info[0]
		def getPredecesor(self):
			return predecesor