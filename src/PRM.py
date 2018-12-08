import numpy as np
from networkx import Graph
import Utils


class PRM():
    def __init__(self, map, max_edge_distance, num_vertices):
        self.map = map
        self.max_edge_distance = max_edge_distance
        self.num_vertices = num_vertices
        self.graph = None

        self.generate_prm_graph ()

    ''' 
    Check provided nodes for collision with obstacles in provided map
    In:
        node: Node to check as list of x, y points
    Out:
        True if valid node, False otherwise
    '''
    def check_node_validity (self, node):
        if node[0] < 0 or node[0] >= self.map.info.width:
            return False

        if node[1] < 0 or node[1] >= self.map.info.height:
            return False

        return self.map.data[node[0] + node[1] * self.map.info.width] == 0

    ''' 
    Check edge between provided nodes for collisions with obstacles in provided map
    In:
        n1: Source node as list of x, y points
        n2: Target node as list of x, y points
    Out:
        True if valid edge, False otherwise
    '''
    def check_edge_validity (self, n1, n2):
        line = Utils.bresenham_get_line (n1, n2)

        for node in line:
            if not self.check_node_validity (node):
                return False

        return True

    ''' 
    Add vertex to graph
    In:
        vertex: Node as list of x, y points to add to graph
    Out:
        True if addition of vertex succeeded, False otherwise
    '''
    def add_vertex_to_graph (self, vertex):
        if not self.check_node_validity (vertex):
            return False

        vertex = tuple (vertex)

        if vertex in self.graph:
            return True

        self.graph.add_node (tuple (vertex))

        for neighbor in self.graph:       
            euclidean_distance = Utils.euclidean_cost (vertex, neighbor)

            if euclidean_distance < self.max_edge_distance:
                if not self.check_edge_validity (vertex, neighbor):
                    continue
                
                self.graph.add_edge (vertex, neighbor, weight = euclidean_distance)

        return True

    ''' 
    Generate probablistic roadmap
    '''
    def generate_prm_graph (self):
        if self.map is None or self.num_vertices == 0 or self.max_edge_distance == 0:
            return

        min_vertex = [self.map.info.origin.position.x, self.map.info.origin.position.y]
        max_vertex = [self.map.info.width, self.map.info.height]

        self.graph = Graph ()

        num_vertices = self.num_vertices

        while num_vertices > 0:
            vertex = [np.random.randint (low = self.map.info.origin.position.x, high = self.map.info.width), \
                      np.random.randint (low = self.map.info.origin.position.y, high = self.map.info.height)]

            if not self.check_node_validity (vertex):
                continue

            self.graph.add_node (tuple (vertex))

            num_vertices -= 1

        for node in self.graph:
            for neighbor in [n for n in self.graph if n is not node]:
                euclidean_distance = Utils.euclidean_cost (node, neighbor)

                if euclidean_distance < self.max_edge_distance:
                    if not self.check_edge_validity (node, neighbor):
                        continue

                    self.graph.add_edge (node, neighbor, weight = euclidean_distance)
