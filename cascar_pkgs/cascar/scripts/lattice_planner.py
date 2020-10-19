import numpy as np
from prio_q import PriorityQueue
import define_world


def lattice_planner(num_nodes, mission, f_next, heuristic):
    unvis_node = -1
    previous = np.full(num_nodes, dtype=np.int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, 2), dtype=np.int)

    startNode = mission['start']['id']
    goalNode = mission['goal']['id']
    q = PriorityQueue()
    q.insert(cost_to_come[startNode] + heuristic(startNode, goalNode), startNode)
    foundPlan = False


    while not q.IsEmpty():
        priority, x = q.pop() #priority temp variable
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node or cost_to_come[xi] > cost_to_come[x] + di:
                previous[xi] = x
                cost_to_come[xi] = cost_to_come[x] + di
                q.insert(cost_to_come[xi] + heuristic(xi, goalNode),xi)
                control_to_come[xi] = ui

    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_visited_nodes': np.sum(previous != unvis_node),
                'name': 'Astar',
                'control': control,
                'visited_nodes': previous[previous != unvis_node]}
    pass
