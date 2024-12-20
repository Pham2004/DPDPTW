import numpy as np
from graph import Graph
from solution import Solution
from vehicle import Vehicle
import random

class ACO(object):
    def __init__(self, ant_nums: int, max_iter: int, graph: Graph, r0: float, alpha: float, beta: float, gamma: float, theta: float, omega: float, gd: float, gt: float):
        self.graph = graph
        self.max_load = graph.vehicle_capacity
        self.ant_nums = ant_nums
        self.max_iter = max_iter
        self.best_fitness = None
        self.best_sol = None
        self.r0 = r0
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.theta = theta
        self.omega = omega
        self.gt = gt
        self.gd = gd
                    
    def run(self):
        for iter in range(self.max_iter):
            ants = list(Solution(self.graph) for _ in range(self.ant_nums))
            for ant in ants:
                vehicle_index = 0
                while not ant.index_to_visit_empty():
                    travel_path_length = len(ant.vehicle_list[vehicle_index].travel_path)
                    result = random.choices([1,0], weights=[travel_path_length, 3], k=1)[0]
                    if result == 1:
                        vehicle_index += 1
                        vehicle_index %= self.graph.vehicle_num
                    probably_next = ant.vehicle_list[vehicle_index].cal_next_index_meet_constrains(ant.index_to_visit)
                    if len(probably_next) > 0:
                        next_index = self.select_next_index(ant.vehicle_list[vehicle_index], probably_next)
                        ant.vehicle_list[vehicle_index].move_to_next_index(next_index)
                        ant.index_to_visit.remove(next_index)
                    else:                        
                        vehicle_index += 1
                        vehicle_index %= self.graph.vehicle_num
                for vehicle in ant.vehicle_list:
                    vehicle.move_to_next_index(0) 
            
            all_current_fitness = np.array([Solution.cal_cost_of_all_vehicle(self.graph, ant.vehicle_list) for ant in ants])
            
            best_index = np.argmin(all_current_fitness)
            if self.best_sol is None or all_current_fitness[best_index] < self.best_fitness:
                self.best_sol = ants[int(best_index)]
                self.best_fitness = all_current_fitness[best_index]
            
            self.graph.global_update_pheromone(self.best_sol, self.best_fitness)
            
            print(f'* Iter {iter}: best fitness {self.best_fitness}')
        
        return self.best_sol
                    
    def fix(lst):
        result = []
        for i in range(len(lst)):
            if i == 0 or lst[i] != 0 or (i > 0 and lst[i-1] != 0):
                result.append(lst[i])
        return result
            
    def select_next_index(self, vehicle: Vehicle, index_to_visit):
        current_index = vehicle.current_index
        r = np.random.uniform(0 , 1)
        next_index = -1
        val = -1
        if r <= self.r0:
            for index in index_to_visit:
                wait = max(self.graph.nodes[index].ready_time - vehicle.vehicle_time_travel - self.graph.dist[current_index][index], 0)
                if wait == 0:
                    wait = 1
                cur = self.graph.pheromone_mat[current_index][index] ** self.alpha \
                    * self.graph.heuristic_info_mat[current_index][index] ** self.beta \
                    * (1 / (self.graph.nodes[index].due_time - self.graph.nodes[index].ready_time)) ** self.gamma \
                    * (1 / wait) ** self.theta
                if next_index == -1 or cur > val:
                    next_index = index
                    val = cur
        else:
            transition_prob = np.zeros(len(index_to_visit))
            for i , index in enumerate(index_to_visit):
                wait = max(self.graph.nodes[index].ready_time - vehicle.vehicle_time_travel - self.graph.dist[current_index][index], 0)
                if wait == 0:
                    wait = 1
                transition_prob[i] = self.graph.pheromone_mat[current_index][index] ** self.alpha \
                                    * self.graph.heuristic_info_mat[current_index][index] ** self.beta \
                                    * (1 / (self.graph.nodes[index].due_time - self.graph.nodes[index].ready_time)) ** self.gamma \
                                    * (1 / wait) ** self.theta
            
            sum_tran_prob = np.sum(transition_prob)
            #assert sum_tran_prob != 0, print(self.graph.pheromone_mat[current_index][index_to_visit])
            next_index = ACO.stochastic_accept(index_to_visit, transition_prob)

        return next_index
    
    @staticmethod
    def stochastic_accept(index_to_visit, transition_prob):
        N = len(index_to_visit)

        sum_tran_prob = np.sum(transition_prob)
        if sum_tran_prob != 0:
            norm_transition_prob = transition_prob / sum_tran_prob
        else:
            norm_transition_prob = transition_prob

        while True:
            ind = int(N * np.random.random())
            if sum_tran_prob == 0:
                return index_to_visit[ind]
            if np.random.random() <= norm_transition_prob[ind]:
                return index_to_visit[ind]
            