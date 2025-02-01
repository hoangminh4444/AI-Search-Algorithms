from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def get_h(state, goal):
    delx = abs(state.get_x() - goal.get_x())
    dely = abs(state.get_y() - goal.get_y())

    h_score = 1.5 * min (delx, dely) + abs(delx - dely)

    return h_score


def A_star(S_init, S_goal, T):
    CLOSED = {}
    OPEN = []
    S_init.set_cost(S_init.get_g() + get_h(S_init, S_goal))
    heapq.heappush(OPEN, S_init)
    CLOSED[S_init.state_hash()] = S_init
    while len(OPEN) > 0:
        n = heapq.heappop(OPEN)
        if n == S_goal:
            return n.get_g(), len(CLOSED)
        for n1 in T.successors(n):
            n1.set_cost(n1.get_g() + get_h(n1, S_goal))
            if n1.state_hash() not in CLOSED:
                heapq.heappush(OPEN, n1)
                CLOSED[n1.state_hash()] = n1
            if n1.state_hash() in CLOSED and n1.get_cost() < CLOSED[n1.state_hash()].get_cost():
                CLOSED[n1.state_hash()].set_g(n1.get_g())
                CLOSED[n1.state_hash()].set_cost(n1.get_cost())
                heapq.heapify(OPEN)
    return -1, len(CLOSED)

def Bi_AStar (start, goal, T):
    OPEN_F = []
    OPEN_B = []
    CLOSED_F = {}
    CLOSED_B = {}
    start.set_cost(start.get_g() + get_h(start, goal))
    goal.set_cost(goal.get_g() + get_h(goal, start))
    heapq.heappush(OPEN_F, start)
    heapq.heappush(OPEN_B, goal)
    CLOSED_F[start.state_hash()] = start
    CLOSED_B[goal.state_hash()] = goal 
    u = float("inf")
    while len(OPEN_F) > 0 and len(OPEN_B) > 0:
        if u <= OPEN_F[0].get_cost() or u <=  OPEN_B[0].get_cost():
            x = [x for x in CLOSED_F]
            y = [y for y in CLOSED_B]
            l = set(x + y)
            return u, len(l)
        if OPEN_F[0].get_cost() < OPEN_B[0].get_cost():
            n = heapq.heappop(OPEN_F)
            for n1 in T.successors(n):
                n1.set_cost(n1.get_g() + get_h(n1, goal))
                if n1.state_hash() in CLOSED_B:
                    u = min(u, n1.get_g() + CLOSED_B[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_F:
                    heapq.heappush(OPEN_F, n1)
                    CLOSED_F[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_F and n1.get_cost() < CLOSED_F[n1.state_hash()].get_cost():
                    CLOSED_F[n1.state_hash()].set_g(n1.get_g())
                    CLOSED_F[n1.state_hash()].set_cost(n1.get_cost())
                    heapq.heapify(OPEN_F)
        else: 
            n = heapq.heappop(OPEN_B)
            for n1 in T.successors(n):
                n1.set_cost(n1.get_g() + get_h(n1, start))
                if n1.state_hash() in CLOSED_F:
                    u = min(u, n1.get_g() + CLOSED_F[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_B:
                    heapq.heappush(OPEN_B, n1)
                    CLOSED_B[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_B and n1.get_cost() < CLOSED_B[n1.state_hash()].get_cost():
                    CLOSED_B[n1.state_hash()].set_g(n1.get_g())
                    CLOSED_B[n1.state_hash()].set_cost(n1.get_cost())
                    heapq.heapify(OPEN_B)
    x = [x for x in CLOSED_F]
    y = [y for y in CLOSED_B]
    l = set(x + y)
    return -1, len(l)

def MM(start, goal, T):
    OPEN_F = []
    OPEN_B = []
    CLOSED_F = {}
    CLOSED_B = {}
    start.set_cost(start.get_g() + get_h(start, goal))
    goal.set_cost(goal.get_g() + get_h(goal, start))
    heapq.heappush(OPEN_F, start)
    heapq.heappush(OPEN_B, goal)
    CLOSED_F[start.state_hash()] = start
    CLOSED_B[goal.state_hash()] = goal 
    u = float("inf")
    while len(OPEN_F) > 0 and len(OPEN_B) > 0:
        if u <= OPEN_F[0].get_cost() or u <=  OPEN_B[0].get_cost():
            x = [x for x in CLOSED_F]
            y = [y for y in CLOSED_B]
            l = set(x + y)
            return u, len(l)
        if OPEN_F[0].get_cost() < OPEN_B[0].get_cost():
            n = heapq.heappop(OPEN_F)
            for n1 in T.successors(n):
                n1.set_cost(max(n1.get_g() + get_h(n1, goal), 2*n1.get_g()))
                if n1.state_hash() in CLOSED_B:
                    u = min(u, n1.get_g() + CLOSED_B[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_F:
                    heapq.heappush(OPEN_F, n1)
                    CLOSED_F[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_F and n1.get_cost() < CLOSED_F[n1.state_hash()].get_cost():
                    CLOSED_F[n1.state_hash()].set_g(n1.get_g())
                    CLOSED_F[n1.state_hash()].set_cost(n1.get_cost())
                    heapq.heapify(OPEN_F)
        else: 
            n = heapq.heappop(OPEN_B)
            for n1 in T.successors(n):
                n1.set_cost(max(n1.get_g() + get_h(n1, start), 2*n1.get_g()))
                if n1.state_hash() in CLOSED_F:
                    u = min(u, n1.get_g() + CLOSED_F[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_B:
                    heapq.heappush(OPEN_B, n1)
                    CLOSED_B[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_B and n1.get_cost() < CLOSED_B[n1.state_hash()].get_cost():
                    CLOSED_B[n1.state_hash()].set_g(n1.get_g())
                    CLOSED_B[n1.state_hash()].set_cost(n1.get_cost())
                    heapq.heapify(OPEN_B)
    x = [x for x in CLOSED_F]
    y = [y for y in CLOSED_B]
    l = set(x + y)
    return -1, len(l)

        

def main():
    """
    Function for testing your implementation. Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, _ in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
    test_instances = "test-instances/testinstances.txt"
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_biastar = []   
    nodes_expanded_astar = []   
    nodes_expanded_mm = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):   

        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_astar = A_star(start, goal, gridded_map) # Replace None, None with a call to your implementation of A*
        nodes_expanded_astar.append(expanded_astar)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_mm = MM(start, goal, gridded_map)
        nodes_expanded_mm.append(expanded_mm)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by MM and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_biastar = Bi_AStar(start, goal, gridded_map) # Replace None, None with a call to your implementation of Bi-A*
        nodes_expanded_biastar.append(expanded_biastar)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    print('Finished running all tests. The implementation of an algorithm is likely correct if you do not see mismatch messages for it.')

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_astar, "Nodes Expanded (MM)", "Nodes Expanded (A*)", "nodes_expanded_mm_astar")
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_biastar, "Nodes Expanded (MM)", "Nodes Expanded (Bi-A*)", "nodes_expanded_mm_biastar")
        

if __name__ == "__main__":
    main()