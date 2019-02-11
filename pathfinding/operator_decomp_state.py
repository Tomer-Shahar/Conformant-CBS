"""
The class that represents a state in the Operator-Decomposition A* algorithm.
Each state consists of a dictionary of each agents current location and time interval, and a pointer to the previous
state for backtracking.
"""

STAY_STILL_COST = 1


class ODState:
    """
    Class for representing standard states where each agent hasn't been assigned a movement yet.
    curr_positions: The current positions of all agents. Dictionary mapping agent id to vertex
    goals: Goal vertices.
    prev_node: the previous node.
    prev_op: The operate that led to this node.
    g_val: g_val of this node. Calculating it depends on the desired goal function.
    grid_map: The map we are searching on.
    """

    def __init__(self, curr_positions, goals, prev_node, prev_op, g_val, conf_map):
        if prev_node:
            self.curr_positions = curr_positions
        else:  # Root node
            self.curr_positions = {}
            for agent, location in curr_positions.items():
                self.curr_positions[agent] = {'location': location, 'time': (0, 0)}

        self.goals = goals
        self.prev_node = prev_node
        self.prev_op = prev_op
        self.h_val = self.calc_heuristic(self.curr_positions, goals, conf_map)
        self.g_val = g_val  # The time range to reach the node.
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val  # heuristic val + cost of predecessor

    def expand(self, grid_map, sic=True):
        """
        Generates all child nodes of the state. This is different than regular A*, since it expands only according to
        the current agent that makes a move. The current moving agent depends on the last one that moved.
        :param sic: How to compute the g value of newly generated states. If sic equals true, use the sum of individual
        costs metric. Otherwise the g value is simply the maximum time between all agents.
        :param grid_map: the map being searched. Used for iterating over edges
        :return: A list of legal states.
        """
        successors = []

        agent_to_move = self.get_next_agent()
        curr_location = self.curr_positions[agent_to_move]['location']
        curr_time = self.curr_positions[agent_to_move]['time']
        for edge_tuple in grid_map.edges_and_weights[curr_location]:
            vertex = edge_tuple[0]
            operator = Operation(agent_to_move, curr_location, vertex, curr_time, edge_tuple[1])
            if self.legal_move(operator):
                new_positions = self.copy_and_update_positions(agent_to_move, operator)
                successor_g_val = self.compute_successor_g_val(edge_tuple, sic)  # ToDO: Verify this
                successor = ODState(new_positions, self.goals, self, operator, successor_g_val, grid_map)
                successors.append(successor)

        stay_still = Operation(agent_to_move, curr_location, curr_location,
                               curr_time, (STAY_STILL_COST, STAY_STILL_COST))
        if self.legal_move(stay_still):  # Add the option of not moving.
            successors.append(ODState(self.curr_positions, self.goals, self, stay_still, stay_still.time, grid_map))

        return successors

    def calc_solution(self):

        curr_node = self
        paths = {}
        for agent in self.curr_positions:
            paths[agent] = []
        while curr_node:
            vertex = curr_node.prev_op.edge[1]
            agent = curr_node.prev_op.agent
            agent_time = curr_node.curr_positions[agent]['time']
            paths[agent].insert(0, (vertex, agent_time))

            curr_node = curr_node.prev_node

        return paths, self.g_val

    def get_next_agent(self):
        """
        :return: The next agent that needs to move.
        """
        if self.prev_op:
            return (self.prev_op.agent % len(self.curr_positions)) + 1
        else:
            return 1

    def legal_move(self, new_op):
        """
        Check whether the given OPERATION is valid. We basically compare the new operation to all agent locations in the
        current state. We traverse the branch up towards the root, until there is no chance of finding a new conflict.
        We maintain a set of agents that have been validated up to the point where their minimum time is larger than
        the max time of the given operation.
        :param new_op: The operation being validated
        :return: True if it's legal, otherwise False.
        """

        curr_node = self
        # ToDO: Make sure all agents are safe, i.e we went so far back that their max time is less than the min time of
        # The operation being tested.
        safe_agents = {new_op.agent}  # Agents who cannot conflict with the new operation. Starts with only the moving agent

        while curr_node:
            # We aren't interested in nodes that were reached by operations performed by the current moving agent
            if curr_node.prev_op and curr_node.prev_op.agent == new_op.agent:
                curr_node = curr_node.prev_node
                continue

            if curr_node.creates_vertex_conflict(new_op):
                return False

            last_op = curr_node.prev_op

            if last_op and curr_node.creates_edge_conflict(last_op, new_op):
                return False

            if last_op and new_op.time[1] < last_op.time[0]:
                safe_agents.add(last_op.agent)
                if len(safe_agents) == len(self.curr_positions):
                    return True
            else:
                curr_node = curr_node.prev_node

        return True

    def creates_edge_conflict(self, curr_op, new_op):
        """

        :param curr_op: The current operation, that was already done
        :param new_op: The new operation that is being validated
        :return: True if it creates an edge conflict, otherwise false
        """
        for agent, position in self.curr_positions.items():
            if agent != new_op.agent and \
                    self.overlapping(new_op.time, self.prev_op.time) and \
                    new_op.normalize_edge() == curr_op.normalize_edge():
                return True
        return False

    def creates_vertex_conflict(self, new_op):
        """
        Tests whether the given operation results in a vertex conflict for the current state. i.e., for different
        agents, after executing the operation, they are in the same location at overlapping times.
        :param new_op: The new operation
        :return: True if there is a conflict, otherwise False
        """
        for agent, position in self.curr_positions.items():
            if agent != new_op.agent and \
                    position['location'] == new_op.edge[1] and \
                    self.overlapping(position['time'], new_op.time):
                return True  # Vertex conflict.
        return False

    @staticmethod
    def overlapping(time_1, time_2):
        """ Returns true if the time intervals in 'time_1' and 'time_2' overlap.

        1: A<-- a<-->b -->B
        2: a<-- A -->b<---B>
        3: A<-- a<--- B --->b
        4: a<-- A<--->B --->b
        5: A<-->B
           a<-->b
        ===> A <= a <= B or a <= A <= b
        """

        if (time_1[0] <= time_2[0] <= time_1[1]) or (time_2[0] <= time_1[0] <= time_2[1]):
            return True

        return False

    @staticmethod
    def calc_heuristic(curr_positions, goals, grid_map):

        heuristic_sum = 0

        for agent, state in goals.items():
            heuristic_sum += grid_map.calc_heuristic(curr_positions[agent]['location'], state)

        return heuristic_sum

    def create_tuple(self):
        """
        Converts the node to a tuple that can be used as a key in a dictionary. The key consists of a tuple for every
        agent and its location. For example if agent a is at location (x,y) and agent b is at location (v,t), the key
        would be ( (a,(x,y), (b,(v,t) )
        :return: A tuple representing the state.
        """
        key_tuple = ()
        for agent, position in self.curr_positions.items():
            key_tuple = key_tuple + (agent, position['location'])

        return key_tuple

    def copy_and_update_positions(self, agent_to_move, op):
        """
        Copies the current positions of self into a new dictionary, where the chosen move is updated accordingly.
        :param op: The operation being done.
        :param agent_to_move: The agent that is moving
        :return: A dictionary mapping between agents to location and time.
        """
        new_positions = {}
        for agent, position in self.curr_positions.items():
            if agent != agent_to_move:
                new_positions[agent] = position
            else:
                new_positions[agent_to_move] = {'location': op.edge[1],
                                                'time': op.time}

        return new_positions

    def compute_successor_g_val(self, edge_tuple, sic=True):
        if sic:
            return self.g_val[0] + edge_tuple[1][0], self.g_val[1] + edge_tuple[1][1]
        else:
            max_time = -1
            max_min_time = -1
            for agent, position in self.curr_positions.items():
                if position['time'][0] > max_min_time:
                    max_min_time = position['time'][0]
                if position['time'][1] > max_time:
                    max_time = position['time'][1]

            return max_min_time, max_time


class Operation:
    """
    The class that represents a movement. Contains the agent that performed it, and the edge being traversed.
    """

    def __init__(self, agent, prev_node, next_node, curr_time, move_time):
        """
        :param agent: Agent that is moving
        :param prev_node: Previous coordinate
        :param next_node: Next coordinate
        :param time_range: The time range the agent will be at the next location. Note that this is NOT a "global" time,
        but is only relevant for the agent making the operation.
        """

        self.agent = agent
        self.edge = prev_node, next_node
        self.time = curr_time[0] + move_time[0], curr_time[1] + move_time[1]

    def normalize_edge(self):
        return min(self.edge[0], self.edge[1]), max(self.edge[0], self.edge[1])
