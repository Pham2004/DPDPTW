from graph import Graph

class Vehicle(object):
    def __init__(self, graph: Graph):
        super()
        self.current_index = 0
        self.vehicle_load = 0
        self.vehicle_time_travel = 0
        self.travel_path = [0]
        self.graph = graph
        self.current_delivery = []
    
    def check_condition(self, next_index) -> bool:
        if self.graph.nodes[next_index].demand < 0 and self.graph.nodes[next_index].pid not in self.current_delivery:
            return False
        if self.vehicle_load + self.graph.nodes[next_index].demand > self.graph.vehicle_capacity:
            return False
        return True
    
    def move_to_next_index(self, next_index):
        self.graph.count[self.current_index][next_index] += 1
        self.travel_path.append(next_index)
        dist = self.graph.dist[self.current_index][next_index]
        self.vehicle_load += self.graph.nodes[next_index].demand
        self.vehicle_time_travel += dist + max(self.graph.nodes[next_index].ready_time - self.vehicle_time_travel - dist, 0) + self.graph.nodes[next_index].service_time
        if next_index != 0:
            if self.graph.nodes[next_index].demand > 0:
                self.current_delivery.append(self.graph.nodes[next_index].id)
            else:
                self.current_delivery.remove(self.graph.nodes[next_index].pid)
        self.current_index = next_index
        
    def cal_next_index_meet_constrains(self, index_to_visit):
        next_index_meet_constrains = []
        for next_ind in index_to_visit:
            if self.check_condition(next_ind):
                next_index_meet_constrains.append(next_ind)
        return next_index_meet_constrains
            
    @staticmethod
    def cal_total_engine_energy_consumption(graph: Graph, travel_path):
        engine_energy_consumption = 0
        vehicle_load = 0
        current_ind = travel_path[0]
        for next_ind in travel_path[1:]:
            T = 1/2 * graph.cd * graph.p * graph.A + (graph.mk + vehicle_load) * graph.g * graph.cr
            G = graph.xi / (graph.kappa * graph.psi) * (graph.pi * graph.R + T / graph.eta)
            engine_energy_consumption += G * graph.dist[current_ind][next_ind]
            vehicle_load += graph.nodes[next_ind].demand
            current_ind = next_ind
        return graph.p1 * engine_energy_consumption
         
    @staticmethod   
    def cal_total_electricity_consumption(graph: Graph, travel_path):
        electricity_consumption = 0
        vehicle_time_travel = 0
        current_ind = travel_path[0]
        for next_ind in travel_path[1:]:
            wait_time = max(graph.nodes[next_ind].ready_time - vehicle_time_travel - graph.dist[current_ind][next_ind], 0) + graph.nodes[next_ind].service_time
            electricity_consumption += graph.b1 * (graph.dist[current_ind][next_ind] + wait_time) + graph.b2 * graph.nodes[next_ind].service_time
            vehicle_time_travel += graph.dist[current_ind][next_ind] + wait_time + graph.nodes[next_ind].service_time
            current_ind = next_ind
        return graph.p2 * electricity_consumption
    
    @staticmethod
    def cal_total_penalty(graph: Graph, travel_path):
        penalty = 0
        vehicle_time_travel = 0
        current_ind = travel_path[0]
        for next_ind in travel_path[1:]:
            if vehicle_time_travel > graph.nodes[next_ind].due_time:
                penalty += vehicle_time_travel - graph.nodes[next_ind].due_time
            wait_time = max(graph.nodes[next_ind].ready_time - vehicle_time_travel - graph.dist[current_ind][next_ind], 0) + graph.nodes[next_ind].service_time
            vehicle_time_travel += graph.dist[current_ind][next_ind] + wait_time + graph.nodes[next_ind].service_time
            current_ind = next_ind
        return graph.p3 * penalty
    
    @staticmethod
    def cal_total_all_cost(graph: Graph, travel_path):
        a = Vehicle.cal_total_engine_energy_consumption(graph, travel_path)
        b = Vehicle.cal_total_electricity_consumption(graph, travel_path)
        c = Vehicle.cal_total_penalty(graph, travel_path)
        return a + b + c , a , b , c
        