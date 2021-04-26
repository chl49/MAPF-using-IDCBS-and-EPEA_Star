import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

  
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
   

    constraint_table = dict()
    for i in range(len(constraints)):   
        if constraints[i]['agent'] == agent :
           
            if constraints[i]['positive'] == False:
                if len(constraints[i]['loc']) == 1:   # vertex constraint
                    constraint_table[(constraints[i]['timestep'], 'v')] = (agent, constraints[i]['loc'], constraints[i]['timestep'], False)
                if len(constraints[i]['loc']) == 2:   # edge constraint
                    constraint_table[(constraints[i]['timestep'], 'e')] = (agent, constraints[i]['loc'], constraints[i]['timestep'], False)

           
            if constraints[i]['positive'] == True:
                if len(constraints[i]['loc']) == 1:   # vertex constraint
                    constraint_table[(constraints[i]['timestep'], 'v')] = (agent, constraints[i]['loc'], constraints[i]['timestep'], True)
                if len(constraints[i]['loc']) == 2:   # edge constraint
                    constraint_table[(constraints[i]['timestep'], 'e')] = (agent, constraints[i]['loc'], constraints[i]['timestep'], True)

            
            
    
    
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    
    result = False
    if (next_time, 'v') in constraint_table:
       
        if constraint_table[(next_time, 'v')][3]==False:
            if next_loc == constraint_table[(next_time, 'v')][1][0]: 
                result = True
        
        if constraint_table[(next_time, 'v')][3]==True:
            if next_loc != constraint_table[(next_time, 'v')][1][0]: 
                result = True

    if (next_time, 'e') in constraint_table:
       
        if constraint_table[(next_time, 'e')][3]==False:
            if curr_loc == constraint_table[(next_time, 'e')][1][0] and next_loc == constraint_table[(next_time, 'e')][1][1]: 
                result = True
       
        if constraint_table[(next_time, 'e')][3]==True:
            if curr_loc != constraint_table[(next_time, 'e')][1][0] or next_loc != constraint_table[(next_time, 'e')][1][1]: 
                result = True

    return result

    


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()
    node_generate_count = 0


    constraint_table = build_constraint_table(constraints, agent)

    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0, 'f_val': h_value}
    push_node(open_list, root)
    closed_list[(root['loc'],root['time_step'])] = root
    terminal_time = 10
    if len(constraints) != 0:
        terminal_time = 4
        for j in range(len(constraints)):
            if len(constraints[j]['loc']) == 2:
                terminal_time = terminal_time + 1
        pre_start = constraints[0]['loc'][0]
        pre_goal = constraints[len(constraints)-1]['loc'][0]
        terminal_time = terminal_time + abs(int(start_loc[0])-int(pre_start[0])) + abs(int(start_loc[1])-int(pre_start[1])) + abs(int(goal_loc[0])-int(pre_goal[0])) + abs(int(goal_loc[1])-int(pre_goal[1]))

    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        print('pop_node loc: ', curr['loc'])
        
        if len(constraints) == 0:
            if curr['loc'] == goal_loc:
                print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with a_star !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                return get_path(curr)

        else:
            if curr['loc'] == goal_loc and curr['time_step'] >= constraints[len(constraints)-1]['timestep']:
                print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with a_star!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                return get_path(curr)

            if curr['time_step'] >= terminal_time:
                break

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            print('dir: ', dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue            
            if my_map[child_loc[0]][child_loc[1]]:
                continue         
            if is_constrained(curr['loc'], child_loc, curr['time_step']+1, constraint_table):          
                continue       
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step']+1,
                    'f_val': curr['g_val'] + 1 + h_values[child_loc]}
            if (child['loc'], child['time_step']) in closed_list: #######
                existing_node = closed_list[(child['loc'], child['time_step'])]
               
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
                    node_generate_count = node_generate_count + 1
                    print('push 1: ', child['loc'], '  f_val: ', child['f_val'])
                    #print(child['time_step'])        
            else:
                 closed_list[(child['loc'], child['time_step'])] = child
                 push_node(open_list, child)
                 node_generate_count = node_generate_count + 1
                 print('push 2: ', child['loc'], '  f_val: ', child['f_val'])
                 
                 #print(child['time_step'])
                 

    return None  # Failed to find solutions


