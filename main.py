from graph import Graph
from local_search import LCS
from vehicle import Vehicle
import time
from aco import ACO

def solve(testcase, al, bt, gm, tt):
    file_path = 'D:/code/simulator/pdp_100/' + testcase + '.txt'
    g = Graph(file_path, cd=0.7, xi=1, kappa=44, p=1.2, A=3.192, mk=3.2, g=9.81, cr=0.01, b1=2, b2=2.5, p1=300, p2=8.15, p3=50, psi=737, pi=0.2, R=165, eta=0.36, rho=0.85, Q=1000)
    start_time = time.time()
    algo = ACO(ant_nums= 20, max_iter= 200, graph= g, r0= 0.5, alpha= al, beta= bt, gamma= gm, theta= tt, omega= 0.6, gd= 60, gt= 5)
    sol = algo.run()
    lcs = LCS(g,sol)
    sol , cost = lcs.run()
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Thời gian chạy: {elapsed_time:.5f} giây")
    a = 0
    b = 0 
    c = 0
    for vehicle in sol.vehicle_list:
        print(vehicle.travel_path)
        a += Vehicle.cal_total_engine_energy_consumption(g, vehicle.travel_path)
        b += Vehicle.cal_total_electricity_consumption(g, vehicle.travel_path)
        c += Vehicle.cal_total_penalty(g, vehicle.travel_path)
        
    print(a,b,c)


for i in range(1,2):
    for al in range(1,4):
        for bt in range(1,4):
            for gm in range(1,4):
                for tt in range(1,4):
                    solve('lc10' + str(i), al, bt, gm, tt)