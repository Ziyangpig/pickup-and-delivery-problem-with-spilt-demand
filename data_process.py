import pandas as pd
import numpy as np
from datetime import datetime,timedelta


def data_process(order_num=None,ship_num=None,is_mass_cargo=True):

    # order = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-货盘.csv", encoding='utf-8')
    # vehicle = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-运力.csv", encoding='gbk')
    # port_info = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口信息表.csv", encoding='utf-8')
    # port_restr = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口限制表.csv", encoding='utf-8')
    # port_draft = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口吃水限制.csv", encoding='utf-8')
    # travel = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港间航行表.csv", encoding='utf-8')
    #
    # travel = travel.drop_duplicates(subset=['开始港口', '结束港口'], keep='first')
    #
    # order.to_json('order.json', orient='records')
    # vehicle.to_json('vehicle.json', orient='records')
    # port_info.to_json('port_info.json', orient='records')
    # port_restr.to_json('port_restr.json', orient='records')
    # port_draft.to_json('port_draft.json', orient='records')
    # travel.to_json('travel.json', orient='records')

    order = pd.read_json("order.json", orient="records")
    vehicle = pd.read_json("vehicle.json", orient="records")
    travel = pd.read_json("travel.json", orient="records")

    # 订单抽样   8800 0.28为界分组，最大抽样数20
    if isinstance(order_num, int):  # 28,38,
        # 创建一个要排除的索引列表
        exclude_indices = [0,27,28,38,39,41] # 35，40，17，19
        # 进行抽样，指定不抽取的部分
        order = order.drop(exclude_indices)
        # 订单货量分组
        if is_mass_cargo:
            order = order[order['纸浆装货量（吨）'] >= 9000]
        else:
            order = order[order['纸浆装货量（吨）'] < 9000]

        order = order.sample(n=order_num, replace=False)

        print(order.index)
        order = order.reset_index(drop=True)

    elif isinstance(order_num, list):
        order = order.iloc[order_num].reset_index(drop=True)
        order = order.reset_index(drop=True)
    else:
        print('order应指定订单抽样数量或者index的索引列表')
        print('默认使用全部订单')
    n = len(order)
    # 创建一个要排除的船索引列表
    exclude_vehicle = [3] # 5,7
    # ship 抽样
    if isinstance(ship_num, int):
        vehicle = vehicle.drop(exclude_vehicle)
        vehicle = vehicle.sample(n=ship_num, replace=False)
        print(vehicle.index)
        vehicle = vehicle.reset_index(drop=False)
    elif isinstance(ship_num, list):
        vehicle = vehicle.iloc[ship_num].reset_index(drop=True)
        vehicle = vehicle.reset_index(drop=True)
    else:
        vehicle = vehicle.drop(exclude_vehicle)
        vehicle = vehicle.reset_index(drop=False)
        print('ship应指定船只抽样数量或者index的索引列表')
        print('默认使用除禁用外的全部船只')

    # N
    data_format = '%Y-%m-%d %H:%M:%S'

    ltw_p = [datetime.strptime(t, data_format) for t in order['受载开始']]
    utw_p = [datetime.strptime(t, data_format) for t in order['受载截止']]

    ## utw_d
    utw_d = []
    for i in range(n):
        date = utw_p[i]
        if order.iloc[i]['允许跨月'] == '不允许':
            # 下个月第一天0点，即不允许跨月
            if date.month == 12:
                next_month = date.replace(day=1, year=date.year + 1, month=1, hour=0)
            else:
                next_month = date.replace(day=1, month=date.month + 1, hour=0)
            # 检测数据错误:不允许跨月由于航行时长必然不能被满足的订单，若有，则允许跨一个月
            bool_index = [a and b for a, b in zip(travel['开始港口'] == order.iloc[i]['装货港口代码'],
                                                  travel['结束港口'] == order.iloc[i]['卸货港口代码'])]
            time_direct = ltw_p[i] + timedelta(hours=travel[bool_index]['航行时间（小时）'].iloc[0])
            if time_direct >= next_month:
                if time_direct.month == 12:
                    next_month = time_direct.replace(year=time_direct.year + 1, month=1,day=1,hour=0,minute=0,second=0)
                else:
                    next_month = time_direct.replace(month=time_direct.month + 1,day=1,hour=0,minute=0,second=0)
                print(ltw_p[i],travel[bool_index]['航行时间（小时）'].iloc[0],time_direct,next_month)

            utw_d.append(next_month)
        else:
            next_year = date.replace(day=1, year=date.year + 1,  hour=0)
            utw_d.append(next_year)
    ## 规划视窗的最早时间及最晚时间
    v_data_format = '%Y/%m/%d %H:%M'
    v_t = [datetime.strptime(t, v_data_format) for t in vehicle['运力释放时间']]
    min_date = min(ltw_p + v_t)
    max_date = max(utw_d)
    min_d = min_date.replace(day=1,hour=0)


    ltw_d = ltw_p
    ltw = [min_d] + ltw_p + ltw_d + [min_d]
    utw = [max_date] + utw_p + utw_d + [max_date]
    ltw_h = [(dt - min_d).total_seconds() / 3600 for dt in ltw]
    utw_h = [(dt - min_d).total_seconds() / 3600 for dt in utw]

    N = {}
    N['ltw'] = ltw_h
    N['utw'] = utw_h
    #
    N['s'] = [0 for i in range(2*n+2)]

    # vehicle
    vt_h = [(dt - min_d).total_seconds() / 3600 for dt in v_t]
    vehicle['avail_t'] = vt_h
    vehicle = vehicle.rename(columns={'载重吨': 'K', })
    vehicle.index = vehicle.index.astype(str)
    vehicle_dict = vehicle[['mmsi', '运力释放港口', 'avail_t', 'K']].to_dict(orient='index')

    # travel_time
    # 多个订单的始发地一样，应该要变成0；始发地和终到地一样；终到地一样
    travel_time = np.full((2 * n, 2 * n), 1.1*N['utw'][0])
    for i in range(2 * n):
        port_s = order.iloc[i]['装货港口代码'] if i < n else order.iloc[i - n]['卸货港口代码']
        for j in range(2 * n):
            port_e = order.iloc[j]['装货港口代码'] if j < n else order.iloc[j - n]['卸货港口代码']
            if port_s==port_e:
                travel_time[i, j] = 0
            if port_e in list(travel[travel['开始港口'] == port_s]['结束港口']):
                bool_index = [a and b for a, b in zip(travel['开始港口'] == port_s, travel['结束港口'] == port_e)]
                travel_time[i, j] = travel[bool_index]['航行时间（小时）'].iloc[0]
    # G

    G_all={}
    for i in vehicle.index:
        G_v = np.full((2 * n + 2, 2 * n + 2), 1.1*N['utw'][0])
        G_v[:, 2 * n + 1] = 0
        avail_port = vehicle_dict[str(i)]['运力释放港口']
        for j in range(n+1):
            if j == 0:
                G_v[0, j] = 0
            # 车辆出发地和pickup点一样，应该要变成0；
            elif avail_port==order.iloc[j-1]['装货港口代码']: #G里的port j 实际上是order j-1 的pickup点
                G_v[0, j] = 0
            else:
                port_e = order.iloc[j-1]['装货港口代码']
                if port_e in list(travel[travel['开始港口'] == avail_port]['结束港口']):
                    bool_index = [a and b for a, b in
                                  zip(travel['开始港口'] == avail_port, travel['结束港口'] == port_e)]
                    G_v[0, j] = travel[bool_index]['航行时间（小时）'].iloc[0]
        G_v[1:2*n+1, 1: 2*n+1] = travel_time
        G_all[str(i)] = {'d':G_v.tolist(),'t':G_v.tolist()}

    #order
    orders = []
    for i in range(n):
        o = {}
        o['id'] = i
        o['Q'] = order.iloc[i]['纸浆装货量（吨）']
        o['R'] = 0
        o['C'] = 999999
        orders.append(o)

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
    for i in vehicle_dict.keys():
        init_routes[i]=dummy_route

    return G_all,N,vehicle_dict,orders,init_routes



if __name__ =="__main__":

    G_all,N,vehicle_dict,orders,init_routes=data_process([6,11,13,26,36],[0,3])
    print(vehicle_dict)
    G0 = G_all['0']['t']
    tap = []
    tad = []
    n=len(orders)
    for i in range(1, n+1):
        tap.append(vehicle_dict['0']['avail_t'] + G0[0][i])
        tad.append(tap[i-1]+2+G0[i][n+i])
    print(all([a > b for a, b in zip(tap, N['utw'][1:n])]))
    print(all([a > b for a, b in zip(tad, N['utw'][n+1:2*n+1])]))
    print(tap)
    print(N['utw'][1:n])
    print(tad)
    print(N['utw'][n+1:2*n+1])

