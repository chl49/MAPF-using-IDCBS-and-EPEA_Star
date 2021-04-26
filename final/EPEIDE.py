import heapq
from single_agent_planner import move, get_sum_of_cost, compute_heuristics, build_constraint_table, get_location, get_path, is_constrained, push_node, pop_node, compare_nodes
import copy


def EPEIDE_a_star_rec(my_map, constraints, constraint_table, closed_list, agent, node_generate_count, h_values, curr, goal_loc, distance, threshold, terminal_time):
    """
    Performs DFS up to a depth where a threshold is reached (as opposed to interative-deepening DFS which stops at a fixed depth).
    Can be modified to handle graphs by keeping track of already visited nodes.
    :param tree:      An adjacency-matrix-representation of the tree where (x,y) is the weight of the edge or 0 if there is no edge.
    :param heuristic: An estimation of distance from node x to y that is guaranteed to be lower than the actual distance. E.g. straight-line distance.
    :param node:      The node to continue from.
    :param goal:      The node we're searching for.
    :param distance:  Distance from start node to current node.
    :param threshold: Until which distance to search in this iteration.
    :return: number shortest distance to the goal node. Can be easily modified to return the path.
     """
    #print("Visiting Node " + str(curr['loc']))
    open_list = []
    #node_generate_count = 0

    if len(constraints) == 0:
        if curr['loc'] == goal_loc:
            #print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with a_star !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            return get_path(curr), -distance, node_generate_count

    else:
        if curr['loc'] == goal_loc and curr['time_step'] >= constraints[len(constraints)-1]['timestep']:
            #print('agent ', agent, ' generate ', node_generate_count, ' in single path finding with a_star!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            return get_path(curr), -distance, node_generate_count

        if curr['time_step'] >= terminal_time:
            return None, distance, node_generate_count

    estimate = distance + h_values[curr['loc']]
    if estimate > threshold:
        #print("Breached threshold with heuristic: " + str(estimate))
        return None, estimate, node_generate_count

    # ...then, for all neighboring nodes....
    min = float("inf")
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
        #print('dir: ', dir, ' child_loc: ', child_loc)

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

                    #print('push 1: ', child['loc'], '  f_val: ', child['f_val'])
                    #print(child['time_step'])
                      
            else:
                 closed_list[(child['loc'], child['time_step'])] = child
                 push_node(open_list, child)
                 node_generate_count = node_generate_count + 1

                 #print('push 2: ', child['loc'], '  f_val: ', child['f_val'])
        if child['g_val'] != 0:
            result, t, node_generate_count_temp= EPEIDE_a_star_rec(my_map, constraints, constraint_table, closed_list, agent, node_generate_count, h_values, child, goal_loc, distance + child['g_val'], threshold, terminal_time+1)
            if t < 0:
                # Node found

                return result, t, node_generate_count_temp
            elif t < min:
                min = t
    # for ele in good_child_locs:
    #     child = {'loc': ele,
    #             'g_val': curr['g_val'] + 1,
    #             'h_val': h_values[ele],
    #             'parent': curr,
    #             'time_step': curr['time_step']+1,
    #             'f_val': curr['g_val'] + 1 + h_values[ele]}
    #     if child['g_val'] != 0:
    #         result, t = iterative_deepening_a_star_rec(my_map, constraints, constraint_table, closed_list, agent, node_generate_count, h_values, child, goal_loc, distance + child['g_val'], threshold, terminal_time+1)
    #         if t < 0:
    #             # Node found

    #             return result, t
    #         elif t < min:
    #             min = t

    return None, min, node_generate_count


def OSF(curr_node, h_values, all_child_locs, delta_small_f):
    good_child_locs = []
    not_good_child_locs = []
    new_root_loc = []
    for ele in all_child_locs:
        child_f_val = curr_node['g_val'] + 1 + h_values[ele] 
        if (child_f_val - curr_node['f_val']) == delta_small_f:
            good_child_locs.append(ele)
            #print('delta_small_f is: ', delta_small_f, ' good_child_loc: ', ele)
        else:
            not_good_child_locs.append(ele)
            #print('delta_small_f is: ', delta_small_f, ' not good_child_loc: ', ele)

    
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