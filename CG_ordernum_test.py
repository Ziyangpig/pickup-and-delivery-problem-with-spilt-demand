from data_process import data_process
from pdp import VehicleRoutingProblem
import time
import pandas as pd


order_list = [6]
ship_num = [0,1,2,] #0,1,2,3,7,8
sample_freq = 6


for order_num in order_list:
    test_results = pd.DataFrame(columns=['instance_id', 'solve_time', 'iter_time', 'routes', 'chartering', 'gap'])
    for f in range(sample_freq):
        time.sleep(1)
        # 数据准备
        a = time.time()
        G, N, vehicle, orders, init_routes = data_process(order_num,ship_num)

        VRP = VehicleRoutingProblem(G, N, vehicle, orders)
        VRP.solve(init_routes, 32400)
        b = time.time()
        print('求解时间：', b - a)
        selected_routes=[]
        for i, item in VRP.masterproblem.x.items():
            for j, c in item.items():
                if c.X == 1:
                    selected_routes.append({str((i,j)):VRP.routes[str(i)][j]})
                    print(i, j)
                    print(VRP.routes[str(i)][j])
        print('iter_time',VRP.iter_time)
        print('GAP',VRP.masterproblem.prob.MIPGap)
        print(VRP.routes)
        r_i = {'instance_id':f,'solve_time':b-a,'iter_time':str(VRP.iter_time),'routes':str(selected_routes),
               'chartering':str([i.X for i in VRP.masterproblem.y.values()]),'gap':VRP.masterproblem.prob.MIPGap}
        test_results = pd.concat([test_results,pd.DataFrame(r_i,index=[f])])


# test_results.to_excel("CG_test_results_ship6.xlsx",sheet_name='order3')
    with pd.ExcelWriter("CG_test_results_ship3.xlsx", engine='openpyxl', mode='a',if_sheet_exists='new') as writer:
        test_results.to_excel(writer, sheet_name=f'order{order_num}')
