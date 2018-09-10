import random
from pathfinding.path_finder import constraint_Astar
from pathfinding.conformant_cbs import constraint_node

class ccbsMap:
    """
    Rudimentary class that converts a ccbsMap text file into a proper object that can be used by the solver.
    """

    def __init__(self, map_file_path=None):
        """
        map_file_path - the path to the map file
        map - a 2 dimensional array representing the map
        edges_weights_and_timeSteps - a dictionary representing the edges. Key - vertex id, value - list of tuples,
        each tuple is of the form (u,cost,t1,t2) where u is the other vertex that comprises the edge, cost is the
        weight, t1 is the minimal time to traverse the edge and t2 is the maximum time.
        """
        if map_file_path:
            self.map_file_path = map_file_path
        self.map = []
        self.edges_weights_and_timeSteps = {}
        self.start_positions = {}
        self.goal_positions = {}
        self.width = -1
        self.height = -1

    @staticmethod
    def generate_rectangle_map(height, width, min_time_range, max_time_range, agent_num):
        """
        Generates a map according to the given input. Returns a ccbsMap object
        """
        new_map = ccbsMap()
        new_map.width = width
        new_map.height = height
        solvable = False
        while not solvable:
            new_map.map = ccbsMap.__generate_map(height, width)
            new_map.__generate_edges_and_timeSteps(min_time_range, max_time_range)
            new_map.__generate_agents(agent_num)
            solver = constraint_Astar(new_map.start_positions, new_map.goal_positions, new_map)
            root = constraint_node()
            root.constraints = {}
            solvable = solver.compute_individual_paths(root, new_map.start_positions, set(), time_limit=1000 * 10 * 60)
        return new_map

    @staticmethod
    def __generate_map(height, width):
        mx = width
        my = height  # width and height of the maze
        maze = [[0 for x in range(mx)] for y in range(my)]
        dx = [0, 1, 0, -1]
        dy = [-1, 0, 1, 0]  # 4 directions to move in the maze
        # start the maze from a random cell
        stack = [(random.randint(0, mx - 1), random.randint(0, my - 1))]
        while len(stack) > 0:
            (cx, cy) = stack[-1]
            maze[cy][cx] = 1
            # find a new cell to add
            nlst = []  # list of available neighbors
            for i in range(4):
                nx = cx + dx[i]
                ny = cy + dy[i]
                if nx >= 0 and nx < mx and ny >= 0 and ny < my:
                    if maze[ny][nx] == 0:
                        # of occupied neighbors must be 1
                        ctr = 0
                        for j in range(4):
                            ex = nx + dx[j]
                            ey = ny + dy[j]
                            if ex >= 0 and ex < mx and ey >= 0 and ey < my:
                                if maze[ey][ex] == 1: ctr += 1
                        if ctr == 1:
                            nlst.append(i)
            # if 1 or more neighbors available then randomly select one and move
            if len(nlst) > 0:
                ir = nlst[random.randint(0, len(nlst) - 1)]
                cx += dx[ir]
                cy += dy[ir]
                stack.append((cx, cy))
            else:
                stack.pop()
        return maze

    def parse_file(self):
        """
        The main function, parses the map text file and turns it into a manageable object that contains the actual map,
        the vertices/edges, the weights and the time steps.
        """
        with open(self.map_file_path) as map_text:
            self.__extract_map(map_text)  # extract map
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
            self.edges_weights_and_timeSteps[curr_vertex] = []

            edges = curr_line.split(':')[1]
            if edges != '\n':
                edges = edges[:-1].split('|')
                for edge in edges:
                    edge_tuple = tuple(edge.split(','))
                    self.edges_weights_and_timeSteps[curr_vertex].append(tuple(edge_tuple))

            curr_line = map_text.readline()

        # little trick to make the pointer go one line up
        pos = map_text.tell()
        pos -= len(curr_line)
        map_text.seek(pos)

    def __extract_map(self, map_text):
        """
        map_text: The text file containing the map
        return: the last line pointed to in the file.
        Also sets self.map to be a 2 dimensional array representing the map. 0 is empty and 1 is blocked
        """
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

    def calc_heuristic(self, start_pos, goal_pos):
        """
        calculates the heuristic value for given start and goal coordinates. Implemented here since
        the way that heuristics are calculates depends on the map.
        Currently a simple manhattan distance
        """
        start_cord = self.vertex_id_to_coordinate(start_pos)
        goal_cord = self.vertex_id_to_coordinate(goal_pos)
        return abs(start_cord[0] - goal_cord[0]) + \
               abs(start_cord[1] - goal_cord[1])

    def __generate_edges_and_timeSteps(self, min_time_range, max_time_range):
        """
        With a given map, generates all the edges and assigns semi-random time ranges.

        general formula:

        X-width-1 X-width X-width+1
        X-1          X        X+1
        X+width-1 X+width X+width+1

        (0,0) (0,1) (0,2) ...
        (1,0) (1,1) (1,2) ...
        (2,0) (2,1) (2,2) ...


        self.map = [[0, 0, 1, 0, 0],
                    [1, 0, 1, 1, 0],
                    [1, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0]]  # ToDo: remove these lines!
        self.width = 5
        self.height = 5
        """
        curr_vertex = -1
        for row in range(self.height):
            for col in range(self.width):
                curr_vertex += 1
                if self.map[row][col] == 1:  # current index is a wall.
                    continue
                self.edges_weights_and_timeSteps[curr_vertex] = []
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        if i == 0 and j == 0:
                            continue
                        if row + i < 0 or row + i == self.width or col + j < 0 or col + j == self.height:
                            continue
                        if self.map[row + i][col + j] == 0:
                            edge = self.__generate_edge((row, col), i, j, min_time_range, max_time_range)
                            self.edges_weights_and_timeSteps[curr_vertex].append(edge)

    def __generate_edge(self, coordinate, i, j, min_time_range, max_time_range):

        vertex_id = (coordinate[0] + i) * self.width + coordinate[1] + j
        min_time = random.randint(min_time_range[0], min_time_range[1])
        max_time = random.randint(max_time_range[0], max_time_range[1])
        edge = (vertex_id, min_time, max_time)

        return edge

    def __generate_agents(self, agent_num):
        """
        generates a number of agents with random start and goal positions. Agents are guaranteed to not have the same
        start and goal positions.
        """
        start_set = set()
        goal_set = set()

        for agent_id in range(1, agent_num + 1):
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)

            while self.map[x][y] == 1 or (x, y) in start_set or (x, y) in goal_set:
                x = random.randint(0, self.width - 1)
                y = random.randint(0, self.height - 1)

            self.start_positions[agent_id] = x * self.width + y
            start_set.add((x, y))

            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)

            while self.map[x][y] == 1 or (x, y) in start_set or (x, y) in goal_set:
                x = random.randint(0, self.width - 1)
                y = random.randint(0, self.height - 1)

            self.goal_positions[agent_id] = x * self.width + y
            goal_set.add((x, y))
