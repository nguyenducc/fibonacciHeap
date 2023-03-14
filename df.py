import heapq

class FibonacciHeap:
    class Node:
        def __init__(self, key, val):
            self.key = key
            self.val = val
            self.degree = 0
            self.marked = False
            self.parent = None
            self.child = None
            self.left = None
            self.right = None

        def __lt__(self, other):
            return self.key < other.key

    def __init__(self):
        self.min_node = None
        self.node_count = 0

    def is_empty(self):
        return self.min_node is None

    def insert(self, key, val):
        node = self.Node(key, val)
        self.node_count += 1
        if self.min_node is None:
            self.min_node = node
            node.left = node
            node.right = node
        else:
            self.min_node.right.left = node
            node.right = self.min_node.right
            self.min_node.right = node
            node.left = self.min_node
            if node.key < self.min_node.key:
                self.min_node = node
        return node

    def merge(self, other_heap):
        if self.min_node is None:
            self.min_node = other_heap.min_node
            self.node_count = other_heap.node_count
        elif other_heap.min_node is not None:
            self.min_node.right.left = other_heap.min_node.left
            other_heap.min_node.left.right = self.min_node.right
            self.min_node.right = other_heap.min_node
            other_heap.min_node.left = self.min_node
            if other_heap.min_node.key < self.min_node.key:
                self.min_node = other_heap.min_node
            self.node_count += other_heap.node_count

    def extract_min(self):
        node = self.min_node
        if node is not None:
            if node.child is not None:
                child = node.child
                while True:
                    next_child = child.right
                    self.min_node.right.left = child
                    child.right = self.min_node.right
                    self.min_node.right = child
                    child.left = self.min_node
                    child.parent = None
                    child = next_child
                    if child == node.child:
                        break
            node.left.right = node.right
            node.right.left = node.left
            if node == node.right:
                self.min_node = None
            else:
                self.min_node = node.right
                self._consolidate()
            self.node_count -= 1
        return node

    def decrease_key(self, node, new_key):
        node.key = new_key
        parent = node.parent
        if parent is not None and node.key < parent.key:
            self._cut(node, parent)
            self._cascading_cut(parent)
        if node.key < self.min_node.key:
            self.min_node = node

    def _cut(self, node, parent):
        node.left.right = node.right
        node.right.left = node.left
        parent.degree -= 1
        if parent.child == node:
            parent.child = node.right
        if node == node.right:
            node.parent = None
        else:
            node.right.parent = parent
            node.parent = None
            node.right.left = node.left
            node.left.right = node.right

        node.marked = False

    def _cascading_cut(self, node):
        parent = node.parent
        if parent is not None:
            if node.marked == False:
                node.marked = True
            else:
                self._cut(node, parent)
                self._cascading_cut(parent)

    def _consolidate(self):
        A = [None] * self.node_count
        nodes = []
        nodes.append(self.min_node)
        while nodes:
            node = nodes.pop()
            degree = node.degree
            while A[degree] is not None:
                other = A[degree]
                if node.key > other.key:
                    node, other = other, node
                self._link(other, node)
                A[degree] = None
                degree += 1
            A[degree] = node

        self.min_node = None
        for i in range(len(A)):
            if A[i] is not None:
                if self.min_node is None:
                    self.min_node = A[i]
                    A[i].left = A[i]
                    A[i].right = A[i]
                else:
                    self.min_node.right.left = A[i]
                    A[i].right = self.min_node.right
                    self.min_node.right = A[i]
                    A[i].left = self.min_node
                    if A[i].key < self.min_node.key:
                        self.min_node = A[i]

    def _link(self, node1, node2):
        node1.left.right = node1.right
        node1.right.left = node1.left
        node1.parent = node2
        if node2.child is None:
            node2.child = node1
            node1.right = node1
            node1.left = node1
        else:
            node1.right = node2.child.right
            node2.child.right.left = node1
            node2.child.right = node1
            node1.left = node2.child
        node2.degree += 1
        node1.marked = False

def dijkstra_heap(adj_list, start):
    n = len(adj_list)
    dist = {node: float('inf') for node in adj_list}
    dist[start] = 0

    pq = FibonacciHeap()
    nodes = {node: None for node in adj_list}
    nodes[start] = pq.insert(0, start)
    visited = [False]*n

    while not pq.is_empty():
        min_node = pq.extract_min()
        u = min_node.val
        visited[u] = True
        for v, weight in adj_list[u].items():
            if visited[v]:
                continue
            new_dist = dist[u] + weight
            if new_dist < dist[v]:
                dist[v] = new_dist
                if nodes[v] is not None:
                    pq.decrease_key(nodes[v], new_dist)
                else:
                    nodes[v] = pq.insert(new_dist, v)

    return dist

def test_dijkstra():
    # test graph
    graph = {
    'A': {'B': 2, 'C': 1},
    'B': {'C': 2, 'D': 3},
    'C': {'D': 1},
    'D': {}
}
    # run algorithm on graph
    distances = dijkstra_heap(graph, 'A')
    # expected distances
    expected = {'A': 0, 'B': 2, 'C': 1, 'D': 2}
    # compare actual and expected distances
    assert distances == expected, f"Expected {expected}, but got {distances}"
    print("Test passed")

test_dijkstra()
