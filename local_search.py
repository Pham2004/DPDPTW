from solution import Solution
from graph import Graph
from vehicle import Vehicle
import random
import numpy as np
import copy


class LCS(object):
    def __init__(self, graph: Graph, sol: Solution):
        self.sol = sol
        self.graph = graph
    
    def init_solution(self):
        pickup_list = []
        for node in self.graph.nodes:
            if node.demand > 0:
                pickup_list.append(node.id)
        random.shuffle(pickup_list)
        sol = Solution(self.graph)
        vehicle_index = 0
        for id in pickup_list:
            p = np.random.rand()
            if p < 0.5:
                vehicle_index += 1
                vehicle_index %= self.graph.vehicle_num
            vehicle = sol.vehicle_list[vehicle_index]
            vehicle.move_to_next_index(id)
            vehicle.move_to_next_index(self.graph.nodes[id].did)
        for vehicle in sol.vehicle_list:
            vehicle.move_to_next_index(0)
        print(vehicle_index)
        return sol
                

    def imp_local_search(self, init_solution: Solution):
        best_solution = init_solution
        best_cost = Solution.cal_cost_of_all_vehicle(self.graph, best_solution.vehicle_list)
        improved = True
        while improved:
            improved = False
            cur_solution = copy.deepcopy(best_solution.vehicle_list)
            for vehicle_id,vehicle in enumerate(cur_solution):
                route = vehicle.travel_path
                for i in range(len(route)):
                    if route[i] != 0:
                        for j in range(i + 1, len(route)):
                            if route[j] != 0:
                                new_route = route[:]
                                new_route[i], new_route[j] = new_route[j], new_route[i]
                                if Solution.is_travel_path_feasible(self.graph, new_route):
                                    new_solution = copy.deepcopy(cur_solution)
                                    new_solution[vehicle_id].travel_path = new_route
                                    new_cost = Solution.cal_cost_of_all_vehicle(self.graph, new_solution)
                                    if new_cost < best_cost:
                                        best_solution.vehicle_list = new_solution
                                        best_cost = new_cost
                                        improved = True
                
                for i in range(len(route)):
                    if self.graph.nodes[route[i]].demand > 0:
                        did = self.graph.nodes[route[i]].did
                        new_route = route[:i] + route[i+1:]
                        delivery_index = new_route.index(did) if did in new_route else -1
                        if delivery_index != -1:
                            new_route = new_route[:delivery_index] + new_route[delivery_index+1:]
                        for other_vehicle_id, other_vehicle in enumerate(cur_solution):
                            if other_vehicle_id != vehicle_id:
                                oroute = other_vehicle.travel_path[:]
                                insert_pos = random.randint(1, len(oroute)-1)
                                oroute.insert(insert_pos, route[i])
                                insert_pos2 = random.randint(insert_pos + 1 , len(oroute)-1)
                                oroute.insert(insert_pos2, did)
                                new_solution = copy.deepcopy(cur_solution)
                                new_solution[vehicle_id].travel_path = new_route
                                new_solution[other_vehicle_id].travel_path = oroute
                                if Solution.is_sol_feasible(self.graph, new_solution):
                                    new_cost =  Solution.cal_cost_of_all_vehicle(self.graph, new_solution)
                                    if new_cost < best_cost:
                                        best_solution.vehicle_list = new_solution
                                        best_cost = new_cost
                                        improved = True
            print(best_cost)
            '''for vehicle in best_solution.vehicle_list:
                print(vehicle.travel_path)'''

        return best_solution, best_cost
       
    def run(self):
        #sol = self.init_solution()
        return self.imp_local_search(self.sol)