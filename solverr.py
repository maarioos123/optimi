from matplotlib import pyplot as plt
import copy

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
    for i in range(0, len(dist)):
        if dist[i] < nearest_v and i not in used and r.load + solver.allNodes[i].demand < r.capacity:
            nearest_index = i
            nearest_v = dist[i]
    used.add(nearest_index)
    return nearest_index, nearest_v


def nearestneighbor():
    for i in range(0, 14):
        routes.append(Route(solver.depot, solver.capacity))
    j = 0
    for i in range(1, len(solver.allNodes)):
        route = routes[j % 14]
        node = route.sequenceOfNodes[-1]
        nearest_possible, value = find_node(route, node)
        solver.allNodes[nearest_possible].isRouted = True
        if (len(route.sequenceOfNodes) == 1):
            solver.allNodes[nearest_possible].waitingtime = value
        else:
            solver.allNodes[nearest_possible].waitingtime = route.sequenceOfNodes[-1].waitingtime + value
        route.load += solver.allNodes[nearest_possible].demand
        route.cost += solver.allNodes[nearest_possible].waitingtime
        solver.allNodes[nearest_possible].waitingtime += 10
        route.sequenceOfNodes.append(solver.allNodes[nearest_possible])
        solver.allNodes[nearest_possible].positionInRoute = len(route.sequenceOfNodes) - 1

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


def plot_solution(solution):
    SolutionDrawer.SolDrawer.drawRoutes(solution)


def report_solution():
    tot_cost = 0
    true_total_cost = 0
    for k in routes:
        for n in k.sequenceOfNodes:
            print(str(n.ID) + ", ", end="")
        print()
        tot_cost += k.cost
        # rt_cost, load = calculate_route_details(k.sequenceOfNodes)
        # true_total_cost += rt_cost
    print(tot_cost)


nearestneighbor()
sol = Solution()
sol.routes = routes
solver.sol = sol

# show solution and plot the solution
report_solution()
plot_solution(sol)


# local search attempt
# two opt

class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9


def FindBestTwoOptMove(top):
    # combination of all the possible routes
    for rtInd1 in range(0, len(solver.sol.routes)):
        rt1: Route = solver.sol.routes[rtInd1]
        for rtInd2 in range(rtInd1, len(solver.sol.routes)):
            rt2: Route = solver.sol.routes[rtInd2]
            for nodeInd1 in range(0, len(rt1.sequenceOfNodes) - 1):
                start2 = 0
                if (rt1 == rt2):
                    start2 = nodeInd1 + 2
                for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                    moveCost = 10 ** 9
                    # trying to do 2-opt move between A and K
                    A = rt1.sequenceOfNodes[nodeInd1]
                    B = rt1.sequenceOfNodes[nodeInd1 + 1]  # the successor of A
                    K = rt2.sequenceOfNodes[nodeInd2]
                    L = rt2.sequenceOfNodes[nodeInd2 + 1]  # the successorof K
                    if rt1 == rt2:
                        if nodeInd1 == 0 and nodeInd2 == len(rt1.sequenceOfNodes) - 1:
                            continue
                        # cost added not calculated this way
                        cost_now = rt1.cost

                        testrt = copy.deepcopy(rt1)
                        reversedSegment = reversed(
                            testrt.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1])
                        # lst = list(reversedSegment)
                        # lst2 = list(reversedSegment)
                        testrt.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1] = reversedSegment
                        cost_new,ld = calculate_route_details(testrt.sequenceOfNodes)
                        moveCost = cost_new - cost_now
                    else:
                        if nodeInd1 == 0 and nodeInd2 == 0:
                            continue
                        if nodeInd1 == len(rt1.sequenceOfNodes) - 1 and nodeInd2 == len(rt2.sequenceOfNodes) - 1:
                            continue

                        if CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                            continue
                        cost_old = rt1.cost + rt2.cost
                        test1 = copy.deepcopy(rt1)
                        test2 = copy.deepcopy(rt2)
                        relocatedSegmentOfRt1 = test1.sequenceOfNodes[nodeInd1 + 1:]

                        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
                        relocatedSegmentOfRt2 = test2.sequenceOfNodes[nodeInd2 + 1:]

                        del test1.sequenceOfNodes[nodeInd1 + 1:]
                        del test2.sequenceOfNodes[nodeInd2 + 1:]

                        test1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
                        test2.sequenceOfNodes.extend(relocatedSegmentOfRt1)
                        # cost added not calculated this way
                        c1,ld = calculate_route_details(test1.sequenceOfNodes)
                        c2,ld = calculate_route_details(test2.sequenceOfNodes)
                        cost_new = c1+c2
                        moveCost = cost_new - cost_old
                    if moveCost < top.moveCost:
                        StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)


def CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
    rt1FirstSegmentLoad = 0
    for i in range(0, nodeInd1 + 1):
        n = rt1.sequenceOfNodes[i]
        rt1FirstSegmentLoad += n.demand
    rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

    rt2FirstSegmentLoad = 0
    for i in range(0, nodeInd2 + 1):
        n = rt2.sequenceOfNodes[i]
        rt2FirstSegmentLoad += n.demand
    rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

    if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
        return True
    if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
        return True

    return False


def StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
    top.positionOfFirstRoute = rtInd1
    top.positionOfSecondRoute = rtInd2
    top.positionOfFirstNode = nodeInd1
    top.positionOfSecondNode = nodeInd2
    top.moveCost = moveCost


def ApplyTwoOptMove(top):
    rt1: Route = solver.sol.routes[top.positionOfFirstRoute]
    rt2: Route = solver.sol.routes[top.positionOfSecondRoute]

    if rt1 == rt2:
        # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
        reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
        # lst = list(reversedSegment)
        # lst2 = list(reversedSegment)
        rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegment

        # reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
        # rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList
        rt1.cost += top.moveCost

    else:
        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]

        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

        del rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]
        del rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

        rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
        rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

        UpdateRouteCostAndLoad(rt1)
        UpdateRouteCostAndLoad(rt2)

    sol.cost += top.moveCost


def UpdateRouteCostAndLoad(rt: Route):
    tl, tc = calculate_route_details(rt.sequenceOfNodes)
    rt.load = tl
    rt.cost = tc


def localsearch():
    top = TwoOptMove()
    terminationCondition = False
    localSearchIterator = 0
    while terminationCondition is False:
        top.Initialize()
        FindBestTwoOptMove(top)
        if top.positionOfFirstRoute is not None:
            if top.moveCost < 0:
                print(top.moveCost)
                ApplyTwoOptMove(top)
                report_solution()
            else:
                terminationCondition = True


def ApplyTwoOptMove1(rt1, rt2, top):
    rt1: Route
    rt2: Route
    if rt1 == rt2:
        # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
        reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
        # lst = list(reversedSegment)
        # lst2 = list(reversedSegment)
        rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegment

        # reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
        # rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

        rt1.cost, rt1.load = calculate_route_details(rt1.sequenceOfNodes)

    else:
        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]

        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

        del rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]
        del rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

        rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
        rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)
        rt1.cost, rt1.load = calculate_route_details(rt1.sequenceOfNodes)
        rt2.cost, rt2.load = calculate_route_details(rt2.sequenceOfNodes)
    return rt1, rt2


# swap move
def testcommulative(rt1: Route,rt2:Route):
    # Node 1 postioning
    firstNodeIndex = 7
    secondNodeIndex = 6
    a1: Node = rt1.sequenceOfNodes[firstNodeIndex - 1]
    b1: Node = rt1.sequenceOfNodes[firstNodeIndex]
    c1: Node = rt1.sequenceOfNodes[firstNodeIndex + 1]
    n1 = len(rt1.sequenceOfNodes) - a1.positionInRoute - 1
    n2 = len(rt1.sequenceOfNodes) - b1.positionInRoute - 1

    a2: Node = rt2.sequenceOfNodes[secondNodeIndex - 1]
    b2: Node = rt2.sequenceOfNodes[secondNodeIndex]
    c2: Node = rt2.sequenceOfNodes[secondNodeIndex + 1]
    n3 = len(rt2.sequenceOfNodes) - a2.positionInRoute - 1
    n4 = len(rt2.sequenceOfNodes) - b2.positionInRoute - 1

    costRemoved1 = n1 * solver.distanceMatrix[a1.ID][b1.ID] + n2 * solver.distanceMatrix[b1.ID][c1.ID]
    costAdded1 = n1 * solver.distanceMatrix[a1.ID][b2.ID] + n2 * solver.distanceMatrix[b2.ID][c1.ID]
    costRemoved2 = n3 * solver.distanceMatrix[a2.ID][b2.ID] + n4 * solver.distanceMatrix[b2.ID][c2.ID]
    costAdded2 = n3 * solver.distanceMatrix[a2.ID][b1.ID] + n4 * solver.distanceMatrix[b1.ID][c2.ID]
    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
    c1 = calculatecost2(rt1) + calculatecost2(rt2)
    rt1.sequenceOfNodes[firstNodeIndex] = b2
    rt2.sequenceOfNodes[secondNodeIndex] = b1
    c2 = calculatecost2(rt1)+calculatecost2(rt2)
    print(c2)


def two_opt_check(rt1: Route, rt2: Route):
    return


def calculatecost2(rt: Route):
    cost = 0
    for i in range(0, len(rt.sequenceOfNodes) - 1):
        n = len(rt.sequenceOfNodes) - i - 1
        cost += n * solver.distanceMatrix[rt.sequenceOfNodes[i].ID][rt.sequenceOfNodes[i + 1].ID]
    n = len(rt.sequenceOfNodes) - 2
    cummulative = n * (n + 1) / 2
    cost += cummulative * 10
    return cost


#two_opt_check(sol.routes[0], sol.routes[8])
#testcommulative(sol.routes[0],sol.routes[8])#swap move
#testcommulative(sol.routes[0],sol.routes[8])#swap move


def CalculateTotalCost(self, sol):
    c = 0
    for i in range(0, len(sol.routes)):
        rt = sol.routes[i]
        c += self.calculate_cost(rt)
    return c

def test2():
    firstNodeIndex = 6
    secondNodeIndex = 6
    rt1 = sol.routes[3]
    rt2 = sol.routes[8]
    a1: Node = rt1.sequenceOfNodes[firstNodeIndex - 1]
    cut_of_a1_b1 = len(rt1.sequenceOfNodes) - a1.positionInRoute - 1
    b1 = rt1.sequenceOfNodes[firstNodeIndex]
    c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
    cut_of_b1_c1 = len(rt1.sequenceOfNodes) - b1.positionInRoute - 1

    a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
    cut_of_a2_b2 = len(rt2.sequenceOfNodes) - a2.positionInRoute - 1
    b2 = rt2.sequenceOfNodes[secondNodeIndex]
    cut_of_b2_c2 = len(rt2.sequenceOfNodes) - b2.positionInRoute - 1
    c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
    costRemoved1 = cut_of_a1_b1 * solver.distanceMatrix[a1.ID][b1.ID] + cut_of_b1_c1 * solver.distanceMatrix[b1.ID][c1.ID]
    costAdded1 = cut_of_a1_b1 * solver.distanceMatrix[a1.ID][b2.ID] + cut_of_b1_c1 * solver.distanceMatrix[b2.ID][c1.ID]
    costRemoved2 = cut_of_a2_b2 * solver.distanceMatrix[a2.ID][b2.ID] + cut_of_b2_c2 * solver.distanceMatrix[b2.ID][c2.ID]
    costAdded2 = cut_of_a2_b2 * solver.distanceMatrix[a2.ID][b1.ID] + cut_of_b2_c2 * solver.distanceMatrix[b1.ID][c2.ID]
    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
    print(moveCost)
    rt1.sequenceOfNodes[firstNodeIndex] = b2
    rt2.sequenceOfNodes[secondNodeIndex] = b1
    rt1.sequenceOfNodes[firstNodeIndex].positionInRoute = firstNodeIndex
    rt1.sequenceOfNodes[secondNodeIndex].positionInRoute = secondNodeIndex
    c11 = calculatecost2(rt1) + calculatecost2(rt2)
    firstNodeIndex = 6
    secondNodeIndex = 6
    rt1 = sol.routes[3]
    rt2 = sol.routes[8]
    a1: Node = rt1.sequenceOfNodes[firstNodeIndex - 1]
    cut_of_a1_b1 = len(rt1.sequenceOfNodes) - a1.positionInRoute - 1
    b1 = rt1.sequenceOfNodes[firstNodeIndex]
    c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
    cut_of_b1_c1 = len(rt1.sequenceOfNodes) - b1.positionInRoute - 1

    a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
    cut_of_a2_b2 = len(rt2.sequenceOfNodes) - a2.positionInRoute - 1
    b2 = rt2.sequenceOfNodes[secondNodeIndex]
    cut_of_b2_c2 = len(rt2.sequenceOfNodes) - b2.positionInRoute - 1
    c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
    costRemoved1 = cut_of_a1_b1 * solver.distanceMatrix[a1.ID][b1.ID] + cut_of_b1_c1 * solver.distanceMatrix[b1.ID][
        c1.ID]
    costAdded1 = cut_of_a1_b1 * solver.distanceMatrix[a1.ID][b2.ID] + cut_of_b1_c1 * solver.distanceMatrix[b2.ID][c1.ID]
    costRemoved2 = cut_of_a2_b2 * solver.distanceMatrix[a2.ID][b2.ID] + cut_of_b2_c2 * solver.distanceMatrix[b2.ID][
        c2.ID]
    costAdded2 = cut_of_a2_b2 * solver.distanceMatrix[a2.ID][b1.ID] + cut_of_b2_c2 * solver.distanceMatrix[b1.ID][c2.ID]
    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
    rt1.sequenceOfNodes[firstNodeIndex] = b2
    rt2.sequenceOfNodes[secondNodeIndex] = b1
    rt1.sequenceOfNodes[firstNodeIndex].positionInRoute = firstNodeIndex
    rt1.sequenceOfNodes[secondNodeIndex].positionInRoute = secondNodeIndex
    c22 = calculatecost2(rt1) + calculatecost2(rt2)
    print(moveCost)
    print(c22-c11)

def test():
    firstNodeIndex = 7
    secondNodeIndex = 6
    rt1 = sol.routes[0]
    rt2 = sol.routes[8]
    print("first change")
    old_cost = calculatecost2(rt1) + calculatecost2(rt2)
    b1 = rt1.sequenceOfNodes[firstNodeIndex]
    b2 = rt2.sequenceOfNodes[secondNodeIndex]
    rt1.sequenceOfNodes[firstNodeIndex] = b2
    rt2.sequenceOfNodes[secondNodeIndex] = b1
    new_cost = calculatecost2(rt1) + calculatecost2(rt2)
    print(old_cost)
    print(new_cost)
    print(new_cost - old_cost)
    print("-----------------------------")
    print()

# print(routes[8].sequenceOfNodes[7].ID)
test2()
# calculatecost2(routes[0])
# print(calculate_route_details(routes[0].sequenceOfNodes))
#localsearch()
#report_solution()
#plot_solution(sol)
# now looping throw same two opt move because
# we dont update the solution by applying the best 2-opt move
# TODO update the solution with applying the 2-opt move
# TODO calculate the cost on a smart way
