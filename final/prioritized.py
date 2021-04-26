import time as timer
import math
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, push_node


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        #stop054 = {'agent': 1,'loc': (1,1),'timestep': 2}
        #stop154 = {'agent': 0,'loc': (1,3),'timestep': 2}
        #stop0542 = {'agent': 1,'loc': (1,4),'timestep': 2}
        #stop12131 = {'agent': 1,'loc': [(1,3),(1,4)],'timestep': 2}
        #stop12121 = {'agent': 1,'loc': [(1,3),(1,3)],'timestep': 2}
        #stop132 = {'agent': 1,'loc': (1,3),'timestep': 2}
        #stop155 = {'agent': 0,'loc': (1,5),'timestep': 10}

        #stop12100 = {'agent': 1,'loc': [(1,2),(1,1)],'timestep': 1}
        #stop12101 = {'agent': 1,'loc': [(1,2),(1,2)],'timestep': 1}
        #stop12131 = {'agent': 1,'loc': [(1,3),(1,4)],'timestep': 2}
        #stop12121 = {'agent': 1,'loc': [(1,3),(1,3)],'timestep': 2}
        #stop12120 = {'agent': 1,'loc': [(1,3),(1,2)],'timestep': 2}
        #constraints.append(stop054)
        #constraints.append(stop154)
        #constraints.append(stop0542)
        #constraints.append(stop132)
        #constraints.append(stop12121)
        #constraints.append(stop155)

        #constraints.append(stop12100)
        #constraints.append(stop12101)
        #constraints.append(stop12131)
        #constraints.append(stop12121)
        #constraints.append(stop12120)
        #print(self.my_map)
        #1.2 VERTEX CONSTRAINT
        #constraints.append({'agent': 0,'loc': (1,5),'timestep': 4})
        #1.3 EDGE CONSTRAINT
        #constraints.append({'agent': 1,'loc': [(1,2),(1,3)],'timestep': 1})
        #1.4 
        #constraints.append({'agent': 0,'loc': (1,5),'timestep': 10})
        #1.5
        # constraints.append({'agent': 1,'loc': (1,2),'timestep': 1})
        # constraints.append({'agent': 1,'loc': (1,3),'timestep': 2})
        # constraints.append({'agent': 1,'loc': (1,4),'timestep': 3})
        # constraints.append({'agent': 1,'loc': (1,5),'timestep': 4})
        # constraints.append({'agent': 1,'loc': [(1,2),(1,1)],'timestep': 1})
        # constraints.append({'agent': 1,'loc': [(1,2),(1,2)],'timestep': 1})
        # constraints.append({'agent': 1,'loc': [(1,3),(1,2)],'timestep': 2})
        # constraints.append({'agent': 1,'loc': [(1,3),(1,3)],'timestep': 2})
        # constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 3})
        # constraints.append({'agent': 1,'loc': [(1,5),(1,4)],'timestep': 4})

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            #stop10000 = {'agent': 0,'loc': [(1,1),(1,2)],'timestep': 1}
            #constraints.append(stop10000)
            #print(path)
            z=0
            maxrange=10;
            final=(0,0)

            # constraints.append({'agent': 1,'loc': (1,2),'timestep': 1})
            # constraints.append({'agent': 1,'loc': (1,3),'timestep': 2})
            # constraints.append({'agent': 1,'loc': (1,4),'timestep': 3})
            # constraints.append({'agent': 1,'loc': (1,5),'timestep': 4})
            # constraints.append({'agent': 1,'loc': [(1,2),(1,1)],'timestep': 1})
            # constraints.append({'agent': 1,'loc': [(1,3),(1,2)],'timestep': 2})
            # constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 3})
            #FINAL CONSTRAINTS

            for x in range(len(path)):
                #print({'agent': 1,'loc': path[x+1],'timestep': x})
                #2.1 
                constraints.append({'agent': 1,'loc': path[x],'timestep': x})
                #2.2
                constraints.append({'agent': 1,'loc': [path[x],path[x-1]],'timestep': x})
                #2.3
                constraints.append({'agent': 1,'loc': [path[x-1],path[x]],'timestep': x})
                constraints.append({'agent': 1,'loc': [path[x-1],path[x-1]],'timestep': x})
                constraints.append({'agent': 1,'loc': [path[x-2],path[x-2]],'timestep': x})
                constraints.append({'agent': 1,'loc': [path[x],path[x]],'timestep': x})
                constraints.append({'agent': 1,'loc': [path[x-1],path[x-1]],'timestep': x+1})
                constraints.append({'agent': 1,'loc': [path[x-1],path[x-2]],'timestep': x+1})
                constraints.append({'agent': 1,'loc': [path[x],path[x-1]],'timestep': x+1})
                constraints.append({'agent': 1,'loc': [path[x-2],path[x-2]],'timestep': x+1})
                constraints.append({'agent': 1,'loc': [path[x],path[x]],'timestep': x+1})
                final=path[x]
                curr=path[x-1]
                prev=path[x-2]
                z=z+1   
            
            constraints.append({'agent': 1,'loc': [self.starts[1],self.starts[1]],'timestep': 1})
            constraints.append({'agent': 1,'loc': [curr,final],'timestep': len(path)})

            for y in range(maxrange):
                constraints.append({'agent': 1,'loc': final,'timestep': y})


            ###############
            #constraints.append({'agent': 1,'loc': [(1,2),(1,1)],'timestep': 2})
            #constraints.append({'agent': 1,'loc': [(1,2),(1,2)],'timestep': 2})
            # constraints.append({'agent': 1,'loc': [(1,3),(1,4)],'timestep': 3})
            # constraints.append({'agent': 1,'loc': [(1,3),(1,3)],'timestep': 3})
            # constraints.append({'agent': 1,'loc': [(1,3),(1,2)],'timestep': 3})
            # constraints.append({'agent': 1,'loc': [(2,3),(1,3)],'timestep': 4})
            # constraints.append({'agent': 1,'loc': [(2,3),(2,3)],'timestep': 4})
            # constraints.append({'agent': 1,'loc': [(2,4),(2,3)],'timestep': 5})
            # constraints.append({'agent': 1,'loc': [(2,4),(2,4)],'timestep': 5})
            # constraints.append({'agent': 1,'loc': [(2,4),(1,4)],'timestep': 5})
            
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
