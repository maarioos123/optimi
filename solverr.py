from matplotlib import pyplot as plt

import SolutionDrawer
from VRP_Model import *


class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []


class Saving:
    def __init__(self, n1, n2, sav):
        self.n1 = n1
        self.n2 = n2
        self.score = sav


class Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.matrix
        self.capacity = m.capacity
        self.sol = None
        self.bestSolution = None


model = Model()
model.BuildModel()
solver = Solver(model)
routes = []
used = {solver.depot.ID}


def find_node(r, n):
    dist = solver.distanceMatrix[n.ID]
    nearest_v = 10000000
    nearest_index = 0
    for i in range(0, len(dist) - 1):
        if dist[i] < nearest_v and i not in used and r.load + solver.allNodes[i].demand < r.capacity:
            nearest_index = i
            nearest_v = dist[i]
    used.add(nearest_index)
    return nearest_index, nearest_v


def nearestneighbor():
    for i in range(0, 14):
        routes.append(Route(solver.depot, solver.capacity))
    j = 0
    for i in range(1, len(solver.allNodes) - 1):
        route = routes[j % 14]
        node = route.sequenceOfNodes[-1]
        nearest_possible, value = find_node(route, node)
        solver.allNodes[nearest_possible].isRouted = True
        if(len(route.sequenceOfNodes)==1):
            solver.allNodes[nearest_possible].waitingtime=value
            print("here1")
        else:
            solver.allNodes[nearest_possible].waitingtime=route.sequenceOfNodes[-1].waitingtime+value
            print("here2")
        route.load += solver.allNodes[nearest_possible].demand
        route.cost += solver.allNodes[nearest_possible].waitingtime
        solver.allNodes[nearest_possible].waitingtime+=10
        route.sequenceOfNodes.append(solver.allNodes[nearest_possible])



        j += 1


def distance(from_node, to_node):
    dx = from_node.x - to_node.x
    dy = from_node.y - to_node.y
    dist = math.sqrt(dx ** 2 + dy ** 2)
    return dist


def calculate_route_details(nodes_sequence):
    rt_load = 0
    rt_cumulative_cost = 0
    tot_time = 0
    for i in range(len(nodes_sequence) - 1):
        from_node = nodes_sequence[i]
        to_node = nodes_sequence[i + 1]
        tot_time += distance(from_node, to_node)
        rt_cumulative_cost += tot_time
        tot_time += 10
        rt_load += from_node.demand
    return rt_cumulative_cost, rt_load

nearestneighbor()
sol = Solution()
sol.routes = routes
# plot solution
# for r in range(0, len(sol.routes)):
#     rt = sol.routes[r]
#     for i in range(0, len(rt.sequenceOfNodes) - 1):
#         c0 = rt.sequenceOfNodes[i]
#         c1 = rt.sequenceOfNodes[i + 1]
#         plt.plot([c0.x, c1.x], [c0.y, c1.y])
#
# plt.show()
tot_cost =0
true_total_cost = 0
for k in routes:
    for n in k.sequenceOfNodes:
        print(str(n.ID) + ", ", end="")
    print()
    tot_cost+=k.cost
    rt_cost, load = calculate_route_details(k.sequenceOfNodes)
    true_total_cost+=rt_cost

print(tot_cost)
print(true_total_cost)