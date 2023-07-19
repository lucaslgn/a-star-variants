from grid import Node, NodeGrid
from math import inf
import heapq
import queue


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

        # variables used in bidirectional search algorithm
        self.fcost = 0
        self.bcost = 0
        self.bpath = []
        self.fpath = []

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []

        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent

        return reversed_path[::-1]  # This syntax creates the reverse list

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []

        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal = self.node_grid.get_node(goal_position[0], goal_position[1])

        start.g = 0
        start.f = start.distance_to(goal.i, goal.j)

        heapq.heappush(pq, (start.f, start))

        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            if node == goal:
                path, f_path = self.construct_path(node), f
                break
            node.closed = True
            node_tuple = (node.i, node.j)
            for successor_tuple in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(successor_tuple[0], successor_tuple[1])
                if not successor.closed and successor.f > node.g + self.node_grid.cost_map.get_edge_cost(node_tuple, successor_tuple) + successor.distance_to(goal.i, goal.j):
                    successor.g = node.g + self.node_grid.cost_map.get_edge_cost(node_tuple, successor_tuple)
                    successor.f = successor.g + successor.distance_to(goal.i, goal.j)
                    successor.parent = node
                    heapq.heappush(pq, (successor.f, successor))

        self.node_grid.reset()
        return path, f_path

    def weighted_a_star(self, start_position, goal_position):
        """
        Plans a path using Weighted A* (WA*).
        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        :hyper-param epsilon: value of weight
        :type: float
        """

        pq = []
        epsilon = 2.5
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal = self.node_grid.get_node(goal_position[0], goal_position[1])

        start.g = 0
        start.f = start.distance_to(goal.i, goal.j)

        heapq.heappush(pq, (start.f, start))

        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            if node == goal:
                path, f_path = self.construct_path(node), f
                break
            node.closed = True
            node_tuple = (node.i, node.j)
            for successor_tuple in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(successor_tuple[0], successor_tuple[1])
                h = self.node_grid.cost_map.get_edge_cost(node_tuple,
                                                          successor_tuple) + successor.distance_to(goal.i, goal.j)

                if not successor.closed and successor.f > node.g + epsilon * h:
                    successor.parent = node
                    successor.g = node.g + self.node_grid.cost_map.get_edge_cost(node_tuple, successor_tuple)
                    successor.f = successor.g + epsilon * successor.distance_to(goal.i, goal.j)
                    heapq.heappush(pq, (successor.f, successor))

        self.node_grid.reset()
        return path, f_path
