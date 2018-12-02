import random
from pathfinding.path_finder import ConstraintAstar
from pathfinding.maze import Maze
import math


"""
    Rudimentary class that converts a ccbsMap text file into a proper object that can be used by the solver.
"""


class ConformantProblem:

    """
    map_file_path - the path to the map file
    map - a 2 dimensional array representing the map
    edges_and_weights - a dictionary representing the edges. Key - vertex id, value - list of tuples,
    each tuple is of the form (u,cost,t1,t2) where u is the other vertex that comprises the edge, cost is the
    weight, t1 is the minimal time to traverse the edge and t2 is the maximum time.
    """
    def __init__(self, map_file_path=None):
        self.map = []
        self.edges_and_weights = None
        self.start_positions = {}
        self.goal_positions = {}
        self.heuristic_table = None
        self.width = -1
        self.height = -1
        if map_file_path:
            self.map_file_path = map_file_path
            with open(self.map_file_path) as map_text:
                if self.map_file_path[-4:] == ".map":  # It's a moving-ai map
                    self.__extract_moving_ai_map(map_text)
                else:
                    self.__extract_map(map_text)  # extract ccbs map

    @staticmethod
    def generate_rectangle_map(height, width, min_time_range=(1, 1), max_time_range=(1, 1),
                               agent_num=2, is_eight_connected=False):
        """
        Generates a map according to the given input. Returns a ccbsMap object
        """
        new_map = ConformantProblem()
        new_map.width = width
        new_map.height = height
        solvable = False
        while not solvable:
            new_map.map = ConformantProblem.__generate_map(height, width)
            new_map.generate_edges_and_weights(min_time_range, max_time_range, is_eight_connected)
            new_map.generate_agents(agent_num)
            solver = ConstraintAstar(new_map)
            solvable = solver.trivial_solution(new_map.start_positions, new_map.goal_positions)
        new_map.fill_heuristic_table()
        return new_map

    @staticmethod
    def __generate_map(height, width):
        maze = Maze.generate(width, height)._to_str_matrix()
        return maze

    """
    The main function, adds to the ccbs problem the agents, vertices/edges and the weights.
    """
    def generate_problem_instance(self, agent_num=2, min_time_range=(1, 1), max_time_range=(1, 1), eight_connected=False):
        if self.map_file_path[-4:] == ".map":  # It's a moving-ai map
            self.generate_edges_and_weights(min_time_range, max_time_range, eight_connected)
            self.generate_agents(agent_num)
        else:
            with open(self.map_file_path) as map_text:
                self.__extract_weights_and_time(map_text)  # extract edges, weights and traversal time
                self.width = len(self.map[0])
                self.__extract_agents(map_text)  # extract the agents' start and goal positions.

    def __extract_weights_and_time(self, map_text):
        """
    curr_line: the current line in the file.
    map_text: the map text file.
    return: returns the last line read from the file. Also sets self.edges_weights_and_timeSteps to be a dictionary
    containing all of the vertices and the edges protruding from them
    """
        curr_line = map_text.readline()
        curr_vertex = -1
        while curr_line[0] == 'V':
            curr_vertex += 1
            self.edges_and_weights[curr_vertex] = []

            edges = curr_line.split(':')[1]
            if edges != '\n':
                edges = edges[:-1].split('|')
                for edge in edges:
                    edge_tuple = tuple(edge.split(','))
                    self.edges_and_weights[curr_vertex].append(tuple(edge_tuple))

            curr_line = map_text.readline()

        # little trick to make the pointer go one line up
        pos = map_text.tell()
        pos -= len(curr_line)
        map_text.seek(pos)

    """
    map_text: The text file containing the map
    return: the last line pointed to in the file.
    Also sets self.map to be a 2 dimensional array representing the map. 0 is empty and 1 is blocked
    """
    def __extract_map(self, map_text):
        curr_line = map_text.readline()
        while curr_line[0] != 'V':
            self.map.append(list(curr_line[:-1]))
            curr_line = map_text.readline()

        # little trick to make the pointer go one line up
        pos = map_text.tell()
        pos -= len(curr_line)
        map_text.seek(pos)

    def __extract_agents(self, map_text):

        agent_id = 1
        for line in map_text:
            #       line = line[:-1]  # remove the \n
            positions = tuple(line.split(':')[1].split(','))
            self.start_positions[agent_id] = int(positions[0])
            self.goal_positions[agent_id] = int(positions[1])
            agent_id += 1

    def vertex_id_to_coordinate(self, v_id):
        """
        A function that receives a vertex ID and transforms it into an actual coordinate.
        Example map and vertex IDs:
        0  1  2  3
        4  5  6  7
        8  9  10 11
        12 13 14 15

        v_id: the input vertex ID
        return - the coordinate as a tuple (row,col)

        e.g, vertex 9 would be (9/4, 9%4) => (2,1)

        """
        row = int(v_id / self.width)
        col = v_id % self.width
        return row, col

    def coordinate_to_vertex_id(self, coord):
        """
        Converts coordinates to the proper vertex id
        """
        vid = coord[0] * self.width + coord[1]
        return vid

    def calc_heuristic(self, start_pos, goal_pos):
        """
        calculates the heuristic value for given start and goal coordinates. Implemented here since
        the way that heuristic calculated depends on the map.
        Currently used a precomputed table
        """

        if self.heuristic_table:
            if start_pos in self.heuristic_table[goal_pos]:
                return self.heuristic_table[goal_pos][start_pos]
            else:
                return math.inf
        else:  # Useful for the first run (find trivial solution)
            return abs(start_pos[0] - goal_pos[0]) + abs(start_pos[1] - goal_pos[1])

    def generate_edges_and_weights(self, min_time_range=(1, 1), max_time_range=(1, 1), is_eight_connected=False):
        """
        With a given map, generates all the edges and assigns semi-random time ranges.

        general formula:

        X-width-1 X-width X-width+1
        X-1          X        X+1
        X+width-1 X+width X+width+1

        (0,0) (0,1) (0,2) ...
        (1,0) (1,1) (1,2) ...
        (2,0) (2,1) (2,2) ...
        """
        self.edges_and_weights = {}
        for row in range(self.height):
            for col in range(self.width):
                if self.map[row][col] == 1:  # current index is a wall.
                    continue
                curr_node = (row, col)
                self.edges_and_weights[(row, col)] = []
                if is_eight_connected:
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            if i == 0 and j == 0:
                                continue
                            if row + i < 0 or row + i == self.height or col + j < 0 or col + j == self.width:
                                continue
                            if self.map[row + i][col + j] == 0:
                                edge = self.__generate_edge((row, col), i, j, min_time_range, max_time_range)
                                self.edges_and_weights[curr_node].append(edge)
                else:  # 4-connected
                    if row > 0 and self.map[row - 1][col] == 0:  # moving up
                        edge = self.__generate_edge((row, col), -1, 0, min_time_range, max_time_range)
                        self.edges_and_weights[curr_node].append(edge)
                    if col > 0 and self.map[row][col - 1] == 0:  # moving left
                        edge = self.__generate_edge((row, col), 0, -1, min_time_range, max_time_range)
                        self.edges_and_weights[curr_node].append(edge)
                    if col < self.width - 1 and self.map[row][col + 1] == 0:  # moving right
                        edge = self.__generate_edge((row, col), 0, 1, min_time_range, max_time_range)
                        self.edges_and_weights[curr_node].append(edge)
                    if row < self.height - 1 and self.map[row + 1][col] == 0:  # moving down
                        edge = self.__generate_edge((row, col), 1, 0, min_time_range, max_time_range)
                        self.edges_and_weights[curr_node].append(edge)

    @staticmethod
    def __generate_edge(coordinate, i, j, min_time_range, max_time_range):

        min_time = random.randint(min_time_range[0], min_time_range[1])
        max_time = random.randint(max_time_range[0], max_time_range[1])
        node = (coordinate[0] + i, coordinate[1] + j)
        edge = (node, min_time, max_time)

        return edge

    def generate_agents(self, agent_num):
        """
        generates a number of agents with random start and goal positions. Agents are guaranteed to not have the same
        start and goal positions.
        """
        start_set = set()
        goal_set = set()

        width = len(self.map[0])
        height = len(self.map)
        for agent_id in range(1, agent_num + 1):
            x = random.randint(0, height - 1)
            y = random.randint(0, width - 1)
            while self.map[x][y] == 1 or (x, y) in start_set or (x, y) in goal_set:
                x = random.randint(0, height - 1)
                y = random.randint(0, width - 1)

            self.start_positions[agent_id] = (x, y)
            start_set.add((x, y))

            x = random.randint(0, height - 1)
            y = random.randint(0, width - 1)

            while self.map[x][y] == 1 or (x, y) in start_set or (x, y) in goal_set:
                x = random.randint(0, height - 1)
                y = random.randint(0, width - 1)

            self.goal_positions[agent_id] = (x, y)
            goal_set.add((x, y))

    def __extract_moving_ai_map(self, map_text):
        """
        Parses a moving AI map.
        """

        curr_line = map_text.readline()
        curr_line = map_text.readline()
        self.height = int(curr_line.split(' ')[1][:-1])
        curr_line = map_text.readline()
        self.width = int(curr_line.split(' ')[1][:-1])
        curr_line = map_text.readline()
        for line in map_text:
            if line == '\n':
                break
            row = []
            for char in line:
                if char == '.':
                    row.append(0)
                elif char == '\n':
                    continue
                else:
                    row.append(1)

            self.map.append(row)

    def fill_heuristic_table(self):

        solver = ConstraintAstar(self)
        self.heuristic_table = {}
        for agent, goal in self.goal_positions.items():
            solution = solver.dijkstra_solution(goal)
            self.heuristic_table[goal] = solution
