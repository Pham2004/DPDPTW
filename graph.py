import numpy as np
from node import Node

class Graph:
    def __init__(self, file_path, cd: float, xi: float, kappa: float, p: float, A: float, mk: float, g: float, cr: float, b1: float, b2: float, p1: float, p2: float, p3: float, psi: float, pi: float, R: float, eta: float, rho, Q):
        super()
        self.num_node, self.nodes, self.dist, self.vehicle_num, self.vehicle_capacity\
            = self.creat_from_file(file_path)
        self.cd = cd
        self.p = p
        self.A = A 
        self.mk = mk
        self.g = g
        self.cr = cr
        self.b1 = b1
        self.b2 = b2
        self.p2 = p2
        self.p1 = p1
        self.p3 = p3
        self.xi = xi
        self.kappa = kappa
        self.psi = psi 
        self.pi = pi
        self.R = R 
        self.eta = eta
        self.rho = rho
        self.Q = Q
        self.count = np.zeros((self.num_node, self.num_node))
        self.pheromone_mat = np.ones((self.num_node, self.num_node))
        self.heuristic_info_mat = 1 / self.dist
            
    def creat_from_file(self, file_path):
        node_list = []
        with open(file_path, 'rt') as f:
            count = 1
            for line in f:
                if count == 1:
                    vehicle_num, vehicle_capacity, vehicle_speed = line.split()
                    vehicle_num = int(vehicle_num)
                    vehicle_capacity = int(vehicle_capacity)
                else:
                    node_list.append(line.split())
                count += 1
        num_nodes = len(node_list)
        nodes = list(Node(int(item[0]), float(item[1]), float(item[2]), float(item[3]), float(item[4]), float(item[5]), float(item[6]), int(item[7]), int(item[8])) for item in node_list)

        dist = np.zeros((num_nodes, num_nodes))
        for i in range(num_nodes):
            node_a = nodes[i]
            dist[i][i] = 1e-8
            for j in range(i+1, num_nodes):
                node_b = nodes[j]
                dist[i][j] = Graph.calculate_dist(node_a, node_b)
                if (dist[i][j] < 1e-8):
                    dist[i][j] = float('inf')
                dist[j][i] = dist[i][j]
                
        return num_nodes, nodes, dist, vehicle_num, vehicle_capacity

                
    @staticmethod
    def calculate_dist(node_a, node_b):
        return np.linalg.norm((node_a.x - node_b.x, node_a.y - node_b.y))
    
    def global_update_pheromone(self, best_sol, best_path_distance):
        self.pheromone_mat = (1-self.rho) * self.pheromone_mat
        for vehicle in best_sol.vehicle_list:
            current_ind = vehicle.travel_path[0]
            for next_ind in vehicle.travel_path[1:]:
                self.pheromone_mat[current_ind][next_ind] += (self.Q / best_path_distance) * self.count[current_ind][next_ind]
                current_ind = next_ind
        self.count = np.zeros((self.num_node, self.num_node))
