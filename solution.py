#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import *
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state

#SOKOBAN HEURISTICS


def heur_displaced(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


def heur_manhattan_distance(state):
    #IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    result_dist = 0
    for box in state.boxes:
        distances = []
        for goal in state.storage:
            if state.restrictions is None or goal in state.restrictions[state.boxes[box]]:
                distances.append(abs(box[0] - goal[0]) + abs(box[1] - goal[1]))
        if distances:
            result_dist += min(distances)
    return result_dist


def heur_alternate(state):
    #IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    result = 0
    robot_box_dists = []  # distance from robot to each box
    for box in state.boxes:
        distances = []
        # calculate distance from robot to this box
        robot_box_dists.append(abs(state.robot[0] - box[0]) + abs(state.robot[1] - box[1]))
        # check if this state is deadlocked
        if trapped(box[0], box[1], state.height, state.width, state.obstacles, state.storage):
            result += 2
        for goal in state.storage:
            if state.restrictions is None or goal in state.restrictions[state.boxes[box]]:
                distances.append(abs(box[0] - goal[0]) + abs(box[1] - goal[1]))
        if distances:
            result += min(distances)

    result += min(robot_box_dists)
    return result

#up: y - 1, right: x + 1, down: y + 1, left: x - 1
def trapped(x, y, height, width, obstacles, storage):
    corners = ((0, 0), (0, height - 1), (width - 1, 0), (width - 1, height - 1))
    return ((x, y) in corners) or \
           (x == 0 and (x, y) not in storage) or \
           (y == 0 and (x, y) not in storage) or \
           (x == width - 1 and (x, y) not in storage) or \
           (y == height - 1 and (x, y) not in storage)


def fval_function(sN, weight):
    #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval


def anytime_gbfs(initial_state, heur_fn, timebound=10):
    #IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn, fval_function=None)

    solution = None
    costbound = None

    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound
    while os.times()[0] <= search_stop_time:
        final_state = se.search(search_stop_time - os.times()[0], costbound)
        if final_state:
            solution = final_state
            # subtract one so search compares >= for costbound, not >
            # send in infinity for hval since solution.hval is always zero
            costbound = (solution.gval - 1, float('inf'), float('inf'))
        else:
            break
    return solution


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound=10):
    #IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn,
                       lambda sN: fval_function(sN, weight))

    solution = None
    costbound = None

    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound
    while os.times()[0] <= search_stop_time:
        final_state = se.search(search_stop_time - os.times()[0], costbound)
        if final_state:
            solution = final_state
            # subtract one so search compares >= for costbound, not >
            costbound = (solution.gval - 1, float('inf'), solution.gval - 1)
        else:
            break
    return solution


if __name__ == "__main__":
    #TEST CODE
    # solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
    # print("*************************************")
    # print("Running A-star")
    #
    # for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.
    #
    #     print("*************************************")
    #     print("PROBLEM {}".format(i))
    #
    #     s0 = PROBLEMS[i] #Problems will get harder as i gets bigger
    #
    #     se = SearchEngine('astar', 'full')
    #     se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    #
    #     if final:
    #         final.print_path()
    #         solved += 1
    #     else:
    #         unsolved.append(i)
    #     counter += 1
    #
    # if counter > 0:
    #     percent = (solved/counter)*100
    #
    # print("*************************************")
    # print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
    # print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
    # print("*************************************")

    solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit
    print("Running Anytime Weighted A-star")

    for i in range(0, 10):
        print("*************************************")
        print("PROBLEM {}".format(i))

        s0 = PROBLEMS[i] #Problems get harder as i gets bigger
        weight = 10
        final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

        if final:
            final.print_path()
            solved += 1
        else:
            unsolved.append(i)
        counter += 1

    if counter > 0:
        percent = (solved/counter)*100

    print("*************************************")
    print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
    print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
    print("*************************************")



