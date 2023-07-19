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

    def example(self, start_position, goal_position):
        """
        Example method.

        :param start_position: position where the planning starts as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        path = []
        f_path = 0

        self.node_grid.reset()
        return path, f_path

