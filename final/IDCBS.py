import time as timer
import heapq
import random
#from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, a_star
from single_agent_planner import a_star, move, get_sum_of_cost, compute_heuristics, build_constraint_table, get_location, get_path, is_constrained, push_node, pop_node, compare_nodes
import copy
from paths_violate_constraint import paths_violate_constraint
from epea_star import epea_star
from ID import iterative_deepening_a_star
from EPEIDE import EPEIDE_a_star_rec


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


class IDCBSSolver(object):
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
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        self.low_level_node_generate = 0
        h_values=0

        
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  
            path, node_generate_count_temp = epea_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            h_values=h_values+self.heuristics[i][self.starts[i]]
            self.low_level_node_generate=self.low_level_node_generate+node_generate_count_temp
            print(h_values)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root) # generate node

        
        print("Task 3.1: Testing")
        print(root['collisions'])
        print(h_values)

        
        print("Task 3.2: Testing")
        for collision in root['collisions']:
            print(standard_splitting(collision))

        
        print("Task 4.2: Testing")
        for collision in root['collisions']:
            print(disjoint_splitting(collision))

        threshold=h_values
        # while True:
        #     #print("Iteration with threshold: " + str(threshold))
        #     result, distance = iterative_deepening_a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
        #     if result is not None:
        #         # We've found the goal node while doing DFS with this max depth
        #         return result
        #     if distance == float("inf"):
        #         # Node not found and no more nodes to visit
        #         return None
        #     elif distance < 0:
        #         # if we found the node, the function returns the negative distance
        #         #print("Found the node we're looking for!")
        #         return result
        #     else:
        #         # if it hasn't found the node, it returns the (positive) next-bigger threshold
        #         threshold = distance
       
        while True:
            try:
                curr = self.pop_node() # expand node
            except IndexError as error:
                self.push_node(root)
                curr = self.pop_node()

            if len(curr['collisions']) == 0:
                print("final paths")
                print(curr['paths'])
                #print(curr['constraints'])
                self.print_results(curr)
                return curr['paths']

            curr_collision = curr['collisions'][0]
            curr_constraints = standard_splitting(curr_collision)
            
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
                #threshold = self.heuristics[ai][self.starts[ai]]
                
                closed_list = dict()
                node_generate_count = 0
                constraint_table = build_constraint_table(child['constraints'], ai)
                earliest_goal_timestep = 0
                h_value = self.heuristics[ai][self.starts[ai]]
                first = {'loc': self.starts[ai], 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0, 'f_val': h_value}
                closed_list[(first['loc'],first['time_step'])] = first
                terminal_time = 10
                if len(child['constraints']) != 0:
                    terminal_time = 4
                    for j in range(len(child['constraints'])):
                        if len(child['constraints'][j]['loc']) == 2:
                            terminal_time = terminal_time + 1
                    pre_start = child['constraints'][0]['loc'][0]
                    pre_goal = child['constraints'][len(child['constraints'])-1]['loc'][0]
                    terminal_time = terminal_time + abs(int(self.starts[ai][0])-int(pre_start[0])) + abs(int(self.starts[ai][1])-int(pre_start[1])) + abs(int(self.goals[ai][0])-int(pre_goal[0])) + abs(int(self.goals[ai][1])-int(pre_goal[1]))+threshold
                #child_path, distance = EPEIDE_a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
                child_path, distance, node_generate_count_temp= EPEIDE_a_star_rec(self.my_map, child['constraints'], constraint_table, closed_list, ai, node_generate_count, self.heuristics[ai], first, self.goals[ai], 0, threshold, terminal_time)
                self.low_level_node_generate=self.low_level_node_generate+node_generate_count_temp
                if child_path is not None:
                    child['paths'][ai] = child_path
                    child['collisions'] = detect_collisions(child['paths'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child) # generate node
                    print("what")
                # result, distance = iterative_deepening_a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
                # if result is not None:
                #     # We've found the goal node while doing DFS with this max depth
                #     return result
                if distance == float("inf"):
                    # Node not found and no more nodes to visit
                    return None
                elif distance < 0:
                    # if we found the node, the function returns the negative distance
                    #print("Found the node we're looking for!")
                    # child['paths'][ai] = child_path
                    # child['collisions'] = detect_collisions(child['paths'])
                    # child['cost'] = get_sum_of_cost(child['paths'])
                    # self.push_node(child)
                    print("fail")
                else:
                    # if it hasn't found the node, it returns the (positive) next-bigger threshold
                    threshold = threshold+distance
                    print(distance)
                    print(threshold)
                    print("hey")
                    # child = {'cost': 0,
                    #      'constraints': child_constraints,
                    #      'paths': curr['paths'],
                    #      'collisions': []}
                    #self.push_node(child)

        # while len(self.open_list)>0:

        #     curr = self.pop_node() # expand node

        #     if len(curr['collisions']) == 0:
        #         print("final paths")
        #         print(curr['paths'])
        #         #print(curr['constraints'])
        #         self.print_results(curr)
        #         return curr['paths']

        #     curr_collision = curr['collisions'][0]
        #     curr_constraints = standard_splitting(curr_collision)
            
        #     curr_node_constraints_copy = copy.deepcopy(curr['constraints'])
        #     curr_node_child_paths_copy = copy.deepcopy(curr['paths'])

        #     for constraint_ele in curr_constraints:
        #         child_constraints = curr_node_constraints_copy
        #         child_constraints.append(constraint_ele)
               
        #         child = {'cost': 0,
        #                  'constraints': child_constraints,
        #                  'paths': curr['paths'],
        #                  'collisions': []}
        #         curr_node_constraints_copy = curr['constraints']
        #         curr['paths'] = curr_node_child_paths_copy
    

        #         ai = constraint_ele['agent']
        #         child_path = epea_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, child['constraints'])
        #         if child_path is not None:
        #             child['paths'][ai] = child_path
        #             child['collisions'] = detect_collisions(child['paths'])
        #             child['cost'] = get_sum_of_cost(child['paths'])
        #             self.push_node(child) # generate node
               
                

        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("Low Level nodes: {}".format(self.low_level_node_generate))
