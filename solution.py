from graph import Graph
from vehicle import Vehicle

class Solution(object):
    def __init__(self, graph: Graph):
        self.graph = graph
        self.vehicle_list = [Vehicle(graph) for i in range(graph.vehicle_num)]
        self.index_to_visit = list(range(graph.num_node))
        self.index_to_visit.remove(0)
    
    def index_to_visit_empty(self):
        return len(self.index_to_visit) == 0
            
    @staticmethod
    def cal_cost_of_all_vehicle(graph: Graph, vehicle_list):
        cost = 0
        for vehicle in vehicle_list:
            cost += Vehicle.cal_total_all_cost(graph, vehicle.travel_path)[0]
        return cost
    
    @staticmethod
    def is_sol_feasible(graph: Graph, vehicle_list):
        for vehicle in vehicle_list:
            if Solution.is_travel_path_feasible(graph, vehicle.travel_path) == False:
                return False
        return True
    
    @staticmethod
    def is_travel_path_feasible(graph: Graph, travel_path):
        capacity = 0
        current_delivery = []
        for ind in travel_path:
            if ind != 0:
                if graph.nodes[ind].demand > 0:
                    current_delivery.append(ind)
                else:
                    if graph.nodes[ind].pid not in current_delivery:
                        return False
                    current_delivery.remove(graph.nodes[ind].pid)
            capacity += graph.nodes[ind].demand
            if capacity > graph.vehicle_capacity:
                return False
        return True