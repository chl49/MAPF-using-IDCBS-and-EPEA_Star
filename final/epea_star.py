import heapq
from single_agent_planner import move, get_sum_of_cost, compute_heuristics, build_constraint_table, get_location, get_path, is_constrained, push_node, pop_node, compare_nodes
import copy


def epea_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    open_list = []
    closed_list = dict()
    node_generate_count = 0
 


    constraint_table = build_constraint_table(constraints, agent)
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0, 'f_val': h_value}
    push_node(open_list, root)
    closed_list[(root['loc'],root['time_step'])] = root

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
                print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with epea_star!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

                return get_path(curr), node_generate_count

        else:
            #if curr['loc'] == goal_loc and curr['time_step'] >= constraints[len(constraints)-1]['timestep']:
            if curr['loc'] == goal_loc and curr['time_step']:
                print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with epea_star!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                return get_path(curr), node_generate_count

            if curr['time_step'] >= terminal_time:
                break

        all_child_locs = []
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue    
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['time_step']+1, constraint_table):
                continue
            all_child_locs.append(child_loc)
            print('dir: ', dir, ' child_loc: ', child_loc)
                

        good_child_locs = []
        new_root_loc = []
        for i in range(3):
            good_child_locs, new_root_loc = OSF(curr, h_values, all_child_locs, i)     
            if len(good_child_locs) != 0:
                break

        
        if len(new_root_loc) != 0:
            child = {'loc': new_root_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[new_root_loc],
                     'parent': curr,
                     'time_step': curr['time_step']+1,
                     'f_val': curr['g_val'] + 1 + h_values[new_root_loc]}
            push_node(open_list, child)
            node_generate_count = node_generate_count + 1


        for ele in good_child_locs:
            child = {'loc': ele,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[ele],
                    'parent': curr,
                    'time_step': curr['time_step']+1,
                    'f_val': curr['g_val'] + 1 + h_values[ele]}
        

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

    return None, None  # Failed to find solutions


def OSF(curr_node, h_values, all_child_locs, delta_small_f):
    good_child_locs = []
    not_good_child_locs = []
    new_root_loc = []
    for ele in all_child_locs:
        child_f_val = curr_node['g_val'] + 1 + h_values[ele] 
        if (child_f_val - curr_node['f_val']) == delta_small_f:
            good_child_locs.append(ele)
            print('delta_small_f is: ', delta_small_f, ' good_child_loc: ', ele)
        else:
            not_good_child_locs.append(ele)
            print('delta_small_f is: ', delta_small_f, ' not good_child_loc: ', ele)

    
    if len(good_child_locs) != 0:
        delta_big_F_next_1 = []
        delta_big_F_next_2 = []
        if delta_small_f != 2:
            for ele in not_good_child_locs:
                ele_f_val = curr_node['g_val'] + 1 + h_values[ele] 
                if ele_f_val - curr_node['f_val'] == delta_small_f + 1:
                    delta_big_F_next_1.append(ele)
                if delta_small_f != 1 and (ele_f_val - curr_node['f_val']) == delta_small_f + 2:
                    delta_big_F_next_2.append(ele)

        if len(delta_big_F_next_1) != 0:
            new_root_loc = delta_big_F_next_1[0]
        elif len(delta_big_F_next_2) != 0:
            new_root_loc = delta_big_F_next_2[0]
        else:
            new_root_loc = []

    return good_child_locs, new_root_loc