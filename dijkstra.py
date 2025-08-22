# Author: Mohammadhossein Bagheri (@PyBagheri)
# License: MIT

import math

class Dijkstra:
    def __init__(self, graph, start_vertex):
        self.graph = graph
        self.start_vertex = start_vertex
        self.vertices = list(set(graph.keys()) | {v for neighbors in graph.values() for v in neighbors})

        # distance: minimum distance from start vertex
        self.vertex_labels = {vertex: {'distance': math.inf, 'prev': None} for vertex in self.vertices}
        self.vertex_labels[start_vertex]['distance'] = 0

    def _get_edge_weight(self, vertex1, vertex2):
        return self.graph.get(vertex1, {}).get(vertex2, math.inf)

    def _set_label(self, vertex, weight, prev=None):
        self.vertex_labels[vertex]['distance'] = weight
        self.vertex_labels[vertex]['prev'] = prev

    def _get_label(self, vertex):
        return self.vertex_labels[vertex]

    def dijkstra(self):
        visited = set()

        while len(visited) < len(self.vertices):
            current = min(
                (v for v in self.vertices if v not in visited),
                key=lambda v: self.vertex_labels[v]['distance'],
                default=None
            )

            if current is None or self.vertex_labels[current]['distance'] == math.inf:
                break

            visited.add(current)

            for neighbor, weight in self.graph.get(current, {}).items():
                alt = self.vertex_labels[current]['distance'] + weight
                if alt < self.vertex_labels[neighbor]['distance']:
                    self._set_label(neighbor, alt, current)

    def build_path(self, vertex):
        if vertex not in self.vertex_labels or self.vertex_labels[vertex]['prev'] is None:
            return [vertex] if vertex == self.start_vertex else []

        path = []
        while vertex is not None:
            path.insert(0, vertex)
            vertex = self.vertex_labels[vertex]['prev']
        return path
