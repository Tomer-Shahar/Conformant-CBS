"""Implementation of a conformant variation of the Conlift-Based Search and Meta-agent Conflict based
search algorithms from

Meta-agent Conflict-Based Search for Optimal Multi-Agent Path Finding by
Guni Sharon, Roni Stern, Ariel Felner and Nathan Sturtevant

This implementation uses code from gswagner's github at: https://github.com/gswagner/mstar_public

Basic idea is independent planning, then check for pairwise collisions.  Branch
into two separate searches, which each require that one robot avoid that
particular space-time position.  The search over the conflict tree continues
until a set of collision free paths are found.

constraints will be of the form:
((robot_ids),(disallowed states,...))
where each disallowed state will have the form (time,coord,[coord2]), where the optional second coord indicates that
this is an edge constraint.
The time for an edge constraint refers to the time at which edge traversal is finished.  Not using objects so I can
use constraints in dictionary keys. Constraints will be ordered by time.

This project adds two aspects to the planner:
1) The edges are weighted.
2) The time that it takes to pass an edge is not static, meaning that it won't necessarily be completed in 1 time-step,
but it can be anywhere between (for example) 2-5.

*Note that a weight of 1 for all edges and a time range of (1,1) is simply the regular CBS.

The goal is to find an optimal solution that guarantees that no two agents will collide.
"""
import time
import math

class conformant_cbs_planner:
    """
    The class that represents the cbs solver. The input is a conformedMap - a map that also contains the weights
    of the edges and the time step range to complete the transfer.

    map - the map the search is conducted on

    edge_weights_and_timeSteps - a 2 dimensional array where each cell A[i,j] contains the weight of the edge from
    i to j, and the time range in the format of (i,j,(t1,t2)) where t1 is the minimal time and t2 is the maximal time.
    A null cell means the movement is impossible.

    startPositions - a list containing all the start positions.
    goalPositions - a list containing all goal positions.
    (Each position is represented as a (x,y) tuple.
    """

    def __init__(self,conformedMap, startPositions, goalPositions):

        self.map = conformedMap.map
        self.edge_weights_and_timeSteps = conformedMap.edges
        self.paths = {}
        self.closed = {}
        self.sub_search = {}
        self.startPositions = startPositions
        self.goalPositions = goalPositions

    def find_solution(self, timeLimit=5*60*1000):
        """
        The main function - returns the solution for each agents and the total cost of the solution.

        root - the root node. Contains no constraints.
        open - the open list.
        """
        startTime = time.time()
        root = constraint_node()
        root.solution = self.__compute_individual_paths(initPositions=self.startPositions,constraints=None)
        root.cost = self.__compute_paths_cost(root.solution)
        open = [] #Consider using a sorted list or something, ordered by cost
        open.append(root)

        while open:
            if(time.time() - startTime > timeLimit):
                raise OutOfTimeError('Ran out of time :-(')

            best_node = open.pop() #ToDo: Get the best node
            new_constraints = self.__validate_solution(best_node.solution)

            if not new_constraints:
                cost = self.__compute_paths_cost(best_node.solution)
                return (best_node.solution, cost)

            for new_con in new_constraints:
                new_node = constraint_node(best_node.constraints, best_node.solution, best_node)
                new_node.solution = self.__compute_agent_path(new_node,new_con[0])
                new_node.cost = self.__compute_paths_cost(new_node.solution)

                if(new_node.cost < math.inf):
                    open.append(new_node)

            open = sorted(open, key=lambda k: k.cost, reverse=False)




    def __compute_individual_paths(self, initPositions, constraints, timeLimit = 5*60):
        return []

    def __compute_paths_cost(self, solution, timeLimit = 5*60):
        """
        A function that computes the cost for each agent to reach their goal given a solution.
        Useful for the SIC heuristic.
        """
        return -1

    def __validate_solution(self, solution):
        return []

    def __compute_agent_path(self, new_node, agent):
        """
        Computes the path for a particular agent.
        """
        pass


class constraint_node:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do not need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.
    """

    def __init__(self,constraints=None, solution=None, parent=None):

        if(parent):
            self.constraints = self.__append_constraints(parent.constraints, constraints)
        else:
            self.constraints = constraints

        self.solution = solution
        self.parent = parent
        self.cost = math.inf # Default value higher than any possible int

    def __append_constraints(self, parentConstraints, new_constraint):
        pass

class OutOfTimeError(Exception):
    def __init__(self, value=None):
        self.value = value

    def __str__(self):
        return repr(self.value)









