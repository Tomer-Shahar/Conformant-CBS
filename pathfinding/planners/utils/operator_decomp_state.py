"""
The class that represents a state in the Operator-Decomposition A* algorithm.
Each state consists of a dictionary of each agents current location and time interval, and a pointer to the previous
state for backtracking.
"""
import math

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

    def expand(self, grid_map, sic=True, min_time_policy=True):
        """
        Generates all child nodes of the state. This is different than regular A*, since it expands only according to
        the current agent that makes a move. The current moving agent depends on the last one that moved.
        :param min_time_policy: Determines if to choose the next agent to move in a serial fashion or choose the agent
        with the lowest minimum time.
        :param sic: How to compute the g value of newly generated states. If sic equals true, use the sum of individual
        costs metric. Otherwise the g value is simply the maximum time between all agents.
        :param grid_map: the map being searched. Used for iterating over edges
        :return: A list of legal states.
        """
        successors = []

        agent_to_move = self.get_next_agent(min_time_policy)
        curr_location = self.curr_positions[agent_to_move]['location']
        curr_time = self.curr_positions[agent_to_move]['time']
        for edge_tuple in grid_map.edges_and_weights[curr_location]:
            vertex = edge_tuple[0]
            operator = self.create_movement_op(agent_to_move, curr_location, vertex, curr_time, edge_tuple[1])
            if self.legal_move(operator):
                new_positions = self.copy_and_update_positions(agent_to_move, operator)
                move_cost = self.compute_move_cost(operator, edge_tuple[1])
                successor_g_val = self.compute_successor_g_val(move_cost, sic)  # ToDO: Verify this
                successor = ODState(new_positions, self.goals, self, operator, successor_g_val, grid_map)
                successors.append(successor)
        # INCREASE THE AGENT TIME WHEN STAYING STILL AT GOAL, BUT DO NOT INCREASE G VALUE! THIS IS TO DISCOVER
        # CONFLICTS. MAKE SURE THAT THE AGENT TIME DOES NOT AFFECT G VALUE
        stay_still = self.create_stay_still_op(agent_to_move, curr_location, curr_time)
        if self.legal_move(stay_still):  # Add the option of not moving.
            stay_still_g_val = self.compute_successor_stay_g_val(agent_to_move, curr_location)
            new_positions = self.copy_and_update_positions(agent_to_move, stay_still)
            successors.append(ODState(new_positions, self.goals, self, stay_still, stay_still_g_val, grid_map))

        return successors

    def compute_successor_stay_g_val(self, agent_to_stay, curr_location):
        """
        Computes the g val of the successor node, where the successor resulted in waiting.
        Returns the parents' g_val plus the cost of moving. Usually the cost is 1, except in the case where the agent is
        staying still at the goal location and then it's 0.
        :param agent_to_stay: The agent that is staying still
        :param curr_location: Current location.
        :return: The proper g_val of the successor.
        """
        if curr_location == self.goals[agent_to_stay]:
            stay_still_g_val = self.compute_successor_g_val((0, 0))
        else:
            stay_still_g_val = self.compute_successor_g_val((STAY_STILL_COST, STAY_STILL_COST))

        return stay_still_g_val

    def compute_move_cost(self, operator, move_cost):
        """
        Computes the proper move cost for a given operator. This is useful when the agent stayed for a long time in its
        goal location and then moved.
        :param operator: Given operator.
        :param move_cost: The cost of the edge being traversed. This will be added to a possible wait "penalty"
        :return: The real cost of the move, to be used to calculate a proper g value.
        """
        wait_time = self.calc_wait_time(operator.agent)
        wait_time *= STAY_STILL_COST

        return move_cost[0] + wait_time, move_cost[1] + wait_time

    @staticmethod
    def create_movement_op(agent_to_move, curr_location, next_location, curr_time, move_time):
        """
        Creates an operation for moving. This function is needed, in case the agent is currently at its goal location
        and is moving. For example, if the agent has stayed at the goal vertex for 5 time steps (meaning a cost of 0
         since waiting at the goal has a cost of 0) and suddenly moves, we add 5 time steps to the cost.
        :param agent_to_move: The agent that's moving
        :param curr_location: Current location
        :param next_location: Next location
        :param curr_time: Current time for the agent
        :param move_time: Time to traverse the edge.
        :return: The appropriate operator
        """
        return Operation(agent_to_move, curr_location, next_location, curr_time, move_time)

    def calc_wait_time(self, agent_to_move):
        """
        Calculates how much time the given agent has been waiting at its goal state.
        :param agent_to_move:
        :return: Number of waiting time steps.
        """
        curr_node = self
        goal_node = self.goals[agent_to_move]
        wait_time = 0
        while curr_node.prev_op and curr_node.curr_positions[agent_to_move]['location'] == goal_node:
            if curr_node.prev_op.agent != agent_to_move:
                curr_node = curr_node.prev_node
                continue

            if curr_node.prev_op and curr_node.prev_op.edge == (goal_node, goal_node):
                wait_time += 1

            curr_node = curr_node.prev_node

        return wait_time

    def create_stay_still_op(self, agent_to_move, curr_location, curr_time):
        """
        Creates an operation for staying still. This function is needed, since if an agent is staying at its goal, we
        consider the cost to be 0. This is the case in most maps, where the agent reaches the goal before other agents
        and the cost shouldn't increase. HOWEVER, if the agent DOES move later, the cumulative cost is added.
        :param agent_to_move: The agent that will stay still
        :param curr_location: The current location
        :param curr_time: Current time.
        :return: An appropriate operator
        """

        return Operation(agent_to_move, curr_location, curr_location, curr_time, (STAY_STILL_COST, STAY_STILL_COST))

    def calc_solution(self, start_positions, sic):

        curr_node = self
        paths = {}
        for agent in self.curr_positions:
            paths[agent] = []
        while curr_node.prev_op:
            vertex = curr_node.prev_op.edge[1]
            agent = curr_node.prev_op.agent
            agent_time = curr_node.curr_positions[agent]['time']
            paths[agent].insert(0, (vertex, agent_time))

            curr_node = curr_node.prev_node

        for agent, start in start_positions.items():
            paths[agent].insert(0, (start, (0, 0)))

        sol_cost = self.comp_sol_cost(paths, sic)
        return paths, sol_cost

    def comp_sol_cost(self, paths, sic):
        """
        Computes final solution cost according to the sic heuristic.
        """
        if sic:
            return self.g_val
        else:
            total_min_cost = 0
            total_max_cost = 0
            for agent, path in paths.items():
                agent_cost = self.compute_path_cost(self.goals[agent], path)
                total_min_cost += agent_cost[0]
                total_max_cost += agent_cost[1]

            return total_min_cost, total_max_cost

    @staticmethod
    def compute_path_cost(goal, path):
        """
        Computes the cost of a single path. Necessary for when an agent just stays at same location in the end.
        Basically subtract 1 from the time of the path for every time step the agent just waited at the goal.
        :param goal: The agent's goal.
        :param path: Some agents' path.
        :return: The minimum and maximum cost of the path.
        """

        path_min_cost = path[-1][1][0]
        path_max_cost = path[-1][1][1]
        for i in reversed(range(len(path)-1)):
            if path[i][0] != goal:
                break

            path_min_cost -= 1
            path_max_cost -= 1

        return path_min_cost, path_max_cost

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

            if last_op and last_op.time[1] < new_op.time[0]:
                safe_agents.add(last_op.agent)
                if len(safe_agents) == len(self.curr_positions):
                    return True
            curr_node = curr_node.prev_node

        return True

    def creates_edge_conflict(self, curr_op, new_op):
        """
        :param curr_op: The current operation, that was already done
        :param new_op: The new operation that is being validated
        :return: True if it creates an edge conflict, otherwise false
        """
        new_op_start_time = self.curr_positions[new_op.agent]['time']

        for agent, position in self.curr_positions.items():
            new_op_time = (new_op_start_time[0], new_op.time[1]-1)
            curr_op_start = self.prev_node.curr_positions[curr_op.agent]['time'][0]
            prev_op_time = (curr_op_start, curr_op.time[1]-1)
            if agent != new_op.agent and self.overlapping(new_op_time, prev_op_time) and (
                curr_op.edge == new_op.edge or (curr_op.edge[0] == new_op.edge[1] and curr_op.edge[1] == new_op.edge[0])):
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

    def get_next_agent(self, min_time_policy):
        """
        :return: The next agent that needs to move.
        """
        if min_time_policy:
            earliest_agent = 1
            min_time = math.inf
            for agent, position in self.curr_positions.items():
                if position['time'][0] < min_time:
                    min_time = position['time'][0]
                    earliest_agent = agent

            return earliest_agent
        else:
            if self.prev_op:
                return (self.prev_op.agent % len(self.curr_positions)) + 1
            else:
                return 1

    @staticmethod
    def calc_heuristic(curr_positions, goals, grid_map):

        heuristic_sum = 0

        for agent, goal in goals.items():
            heuristic_sum += grid_map.calc_heuristic(curr_positions[agent]['location'], goal)

        return heuristic_sum

    def create_tuple(self):
        """
        Converts the node to a tuple that can be used as a key in a dictionary. The key consists of a tuple for every
        agent and its location + time. For example if agent a is at location (x,y) at time (t1,t2) and agent b is at
        location (v,t) at time (t3, t4), the key would be ( (a,(x,y), (t1, t2)), (b,(v,t), (t3, t4)).
        We also add the last agent that moved, 0 for root node.
        -- This is mainly important for nodes where an agent stays still and then doesn't --
        :return: A tuple representing the state.
        """
        key_tuple = ()
        for agent, position in self.curr_positions.items():
            key_tuple = key_tuple + (agent, position['location'], position['time'])

        return key_tuple

        if self.prev_op:
            return key_tuple + (self.prev_op.agent, )
        else:
            return key_tuple + (0, )

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

    def compute_successor_g_val(self, move_cost, sic=True):
        if sic:  # Simply add the cost of moving to the current g value.
            return self.g_val[0] + move_cost[0], self.g_val[1] + move_cost[1]
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
        :param time: The time range the agent will be at the next location. Note that this is NOT a "global" time,
        but is only relevant for the agent making the operation.
        """

        self.agent = agent
        self.edge = prev_node, next_node
        self.time = curr_time[0] + move_time[0], curr_time[1] + move_time[1]

    def normalize_edge(self):
        # ToDo: Why does this take so long?
        return min(self.edge[0], self.edge[1]), max(self.edge[0], self.edge[1])
