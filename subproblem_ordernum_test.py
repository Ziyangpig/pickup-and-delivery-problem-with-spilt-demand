from data_process import data_process
from subproblem import _SubProblemLP
import time
import pandas as pd

order_num = 5
ship_num = 9
sample_freq = 20

solver = 'gurobi'

test_results = pd.DataFrame(columns=['ship_id','solve_time','travel_time','route','q'])
# vehicle[ship_id]['avail_t'] = 0
for v in range(ship_num):
    ship_id = str(v)
    for f in range(sample_freq):
        time.sleep(1)
        # 数据准备
        G, N, vehicle, orders, init_routes = data_process(order_num)
        duals = {"route_selection": -2, }
        duals["demand_const"] = [-100 for i in range(len(orders))]
        # 开始求解
        a = time.time()
        m = _SubProblemLP(duals, G[ship_id], N, orders, vehicle[ship_id], solver)
        m.solve(600)
        b = time.time()
        # 记录结果
        r =[]
        q=[]
        for i, item in m.x.items():
            if item.X == 1:
                print(i)
                r.append(i)
        for i, item in m.q.items():
            if item.X > 0.1:
                q.append([i,item.X])
        r_i={'ship_id':ship_id,'solve_time':b-a,'travel_time':m.ta[m.portnum - 1].X-m.vehicle['avail_t'],'route':str(r),'q':str(q)}
        test_results=pd.concat([test_results, pd.DataFrame(r_i, index=[f])])
        print(f,'共求解时长：', b - a)
        print(m.q)
        print('travel time:',m.ta[m.portnum - 1].X)

test_results.to_excel("test_results_5.xlsx")

