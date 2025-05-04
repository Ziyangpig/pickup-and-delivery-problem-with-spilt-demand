#%%
from data_generate import data_generate
from pdp_parallel import VehicleRoutingProblemParallel
from time import time
import pandas as pd

import itertools


def main(time_limit=1200):
    seed = 2
    ns = [15] # 订单数量列表 
    cargo_sizes = [[0.3,0.8],[0.5,0.1]] #订单货物大小列表
    K,number_vehicle = [90, 120, 150],10 #车辆容量，车辆数量
    Ws, Ts = [60,120],[900]#[300,600,900] # 时间窗宽度，plan horizon
    L = 80 # 运输区域边长

    # 使用 itertools.product 生成所有组合
    combinations = list(itertools.product(cargo_sizes, Ws))
    cats = ['A','B','C','D']
    test_results = pd.DataFrame(columns=['instance_id','solve_time','gap','OBJ','迭代次数','结果未改善次数','iter_time','master_time','check_gap_time','routes','chartering'])

    #%%
    for n in ns:
        for ii,comb in enumerate(combinations):
            for T in Ts:
                G, N, vehicle, orders= data_generate(n,comb[0],comb[1],T,seed,L,K,number_vehicle)
                # init_routes
                dummy_route = [{'route': [(0, 0),],
                    'q': [[0 for i in range(n)],],
                    'cvr': 0,
                    'time': [0],
                    'time_cost': 0,
                    'load': [[],],
                    'split': [[],]},
                                ]
                init_routes={}
                for i in vehicle.keys():
                    init_routes[i]=dummy_route


                a = time()

                VRP=VehicleRoutingProblemParallel(G,N,vehicle,orders)
                VRP.solve(init_routes,time_limit,'label',1000,True,False)
                
                b = time()
                
                selected_routes=[]
                for i, item in VRP.masterproblem.x.items():
                    for j, c in item.items():
                        if c.X == 1:
                            selected_routes.append({str((i,j)):VRP.routes[str(i)][j]})      
                
                cat = cats[ii]
                instance_id = f'{cat}-{T}-{n}' 

                print(VRP.routes)
                print(VRP.masterproblem.print_solution())
                print('车辆信息',vehicle, '\n', '订单信息：',orders, '\n', N)
                
                print(instance_id)
                print('solve time:',b-a)
                print('迭代次数：',VRP._iteration,'结果未改善次数：',VRP._no_improvement)
                print('iter_time',VRP.iter_time)
                print('master time',VRP.master_iter_time)
                print('check gap time',VRP.check_gap_time)
                print('GAP',VRP.masterproblem.prob.MIPGap)
                print('OBJ',VRP.masterproblem.prob.ObjVal)
                
                
                
                r_i = {'instance_id':instance_id,'solve_time':b-a,'gap':VRP.masterproblem.prob.MIPGap,'OBJ':VRP.masterproblem.prob.ObjVal,'迭代次数':VRP._iteration,'结果未改善次数':VRP._no_improvement,
                    'iter_time':str(VRP.iter_time),'master_time':str(VRP.master_iter_time),'check_gap_time':str(VRP.check_gap_time),
                    'routes':str(selected_routes),'chartering':str([i.X for i in VRP.masterproblem.y.values()]),}
                test_results = pd.concat([test_results,pd.DataFrame(r_i,index = [instance_id])])
                
                with pd.ExcelWriter(f".\output\CG_test_results_paraller.xlsx", engine='openpyxl', mode='a',if_sheet_exists='new') as writer:
                    test_results.to_excel(writer, sheet_name=f'order{n}')
            
            
if __name__ == "__main__":
    main(time_limit=1200)



