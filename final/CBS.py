import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, a_star
import copy
from paths_violate_constraint import paths_violate_constraint
from epea_star import epea_star
from ID import iterative_deepening_a_star
from EPEIDA import EPEIDE_a_star


def detect_collision(path1, path2):
   
    result = dict()
    longer_path = max(len(path1), len(path2))
    for j in range(longer_path):
       
        if get_location(path1,j) == get_location(path2,j):
            result = {'loc': [get_location(path1,j)], 'timestep': j}
            return result
       
        if j<longer_path-1:
            if get_location(path1,j)==get_location(path2,j+1) and get_location(path1,j+1)==get_location(path2,j):
                result = {'loc': [get_location(path1,j), get_location(path1,j+1)], 'timestep': j+1}
                return result


    return None


def detect_collisions(paths):
    
    collision = dict()
    temp = dict()
    collisions = []
    for i in range(len(paths)):
        for k in range(i+1, len(paths)):
            temp = detect_collision(paths[i], paths[k])
            if temp is not None:
                collision = {'a1': i, 'a2': k, 'loc': temp['loc'], 'timestep': temp['timestep']}
                collisions.append(collision)
    return collisions


def standard_splitting(collision):
   
    result = []
    temp1 = dict()
    temp2 = dict()
    
    if len(collision['loc']) == 1:
        temp1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
        result.append(temp1)
        temp2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
        result.append(temp2)
   
    if len(collision['loc']) == 2:
        temp1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
        result.append(temp1)
        temp2 = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False}
        result.append(temp2)
    return result




def disjoint_splitting(collision):
 

    result = []
    temp1 = dict()
    temp2 = dict()
    random_agent = random.randint(0,1)

    
    if random_agent == 0:
       
        if len(collision['loc']) == 1:
            temp1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True}
            result.append(temp1)
            temp2 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
            result.append(temp2)
        
        if len(collision['loc']) == 2:
            temp1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True}
            result.append(temp1)
            temp2 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
            result.append(temp2)

    
    if random_agent == 1:
       
        if len(collision['loc']) == 1:
            temp1 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True}
            result.append(temp1)
            temp2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
            result.append(temp2)
       
        if len(collision['loc']) == 2:
            temp1 = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': True}
            result.append(temp1)
            temp2 = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False}
            result.append(temp2)
    return result


class CBSSolver(object):
    """The high-level search of IDCBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    # def find_solution(self, disjoint=True):
    #     """ Finds paths for all agents from their start locations to their goal locations

    #     disjoint    - use disjoint splitting or not
    #     """

    #     self.start_time = timer.time()

        
    #     root = {'cost': 0,
    #             'constraints': [],
    #             'paths': [],
    #             'collisions': []}
    #     for i in range(self.num_of_agents):  
    #         path = epea_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
    #                       i, root['constraints'])
    #         if path is None:
    #             raise BaseException('No solutions')
    #         root['paths'].append(path)

    #     root['cost'] = get_sum_of_cost(root['paths'])
    #     root['collisions'] = detect_collisions(root['paths'])
    #     self.push_node(root) # generate node

        
    #     print("Task 3.1: Testing")
    #     print(root['collisions'])

        
    #     print("Task 3.2: Testing")
    #     for collision in root['collisions']:
    #         print(standard_splitting(collision))

        
    #     print("Task 4.2: Testing")
    #     for collision in root['collisions']:
    #         print(disjoint_splitting(collision))

       
    #     while len(self.open_list)>0:

    #         curr = self.pop_node() # expand node

    #         if len(curr['collisions']) == 0:
    #             print("final paths")
    #             print(curr['paths'])
    #             #print(curr['constraints'])
    #             self.print_results(curr)
    #             return curr['paths']

    #         curr_collision = curr['collisions'][0]
    #         curr_constraints = standard_splitting(curr_collision)
            
    #         curr_node_constraints_copy = copy.deepcopy(curr['constraints'])
    #         curr_node_child_paths_copy = copy.deepcopy(curr['paths'])

    #         for constraint_ele in curr_constraints:
    #             child_constraints = curr_node_constraints_copy
    #             child_constraints.append(constraint_ele)
               
    #             child = {'cost': 0,
    #                      'constraints': child_constraints,
    #                      'paths': curr['paths'],
    #                      'collisions': []}
    #             curr_node_constraints_copy = curr['constraints']
    #             curr['paths'] = curr_node_child_paths_copy
    

    #             ai = constraint_ele['agent']
    #             child_path = epea_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
    #             if child_path is not None:
    #                 child['paths'][ai] = child_path
    #                 child['collisions'] = detect_collisions(child['paths'])
    #                 child['cost'] = get_sum_of_cost(child['paths'])
    #                 self.push_node(child) # generate node
               
                

    #     return None

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = EPEIDE_a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))
        

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            curr = self.pop_node()
            # if not curr['collisions']:
            #     return curr['paths']
            if len(curr['collisions']) == 0:
                print("final paths")
                print(curr['paths'])
                #print(curr['constraints'])
                self.print_results(curr)
                return curr['paths']
            curr_collision=curr['collisions'][0]
            curr_constraints=standard_splitting(curr_collision)
            curr_node_constraints_copy = copy.deepcopy(curr['constraints'])
            curr_node_child_paths_copy = copy.deepcopy(curr['paths'])

            for constraint_ele in curr_constraints:
                child_constraints = curr_node_constraints_copy
                child_constraints.append(constraint_ele)
               
                child = {'cost': 0,
                         'constraints': child_constraints,
                         'paths': curr['paths'],
                         'collisions': []}
                curr_node_constraints_copy = curr['constraints']
                curr['paths'] = curr_node_child_paths_copy
    

                ai = constraint_ele['agent']
                child_path = EPEIDE_a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
                if child_path is not None:
                    child['paths'][ai] = child_path
                    child['collisions'] = detect_collisions(child['paths'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child) # generate node
            # for constraint in currConstrait:
            #     child = {'cost': 0,
            #     'constraints': [],
            #     'paths': [],
            #     'collisions': []}
            #     #child['constraints']=curr['constraints']
            #     child['constraints']=currConstrait
            #     child['paths']=curr['paths']
            #     for i in range(self.num_of_agents):  # Find initial path for each agent
            #         path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
            #               i, child['constraints'])
            #         if path:
            #             child['paths']=path
            #             child['collisions']=detect_collisions(child['paths'])
            #             child['cost'] = get_sum_of_cost(child['paths'])
            #             self.push_node(child)

            #         root['paths'].append(path)




        self.print_results(root)
        return None
        #return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
