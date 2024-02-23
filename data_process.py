import pandas as pd
import numpy as np
from datetime import datetime

def data_process():

    order = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-货盘.csv", encoding='utf-8')
    vehicle = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-运力.csv", encoding='gbk')
    port_info = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口信息表.csv", encoding='utf-8')
    port_restr = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口限制表.csv", encoding='utf-8')
    port_draft = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口吃水限制.csv", encoding='utf-8')
    travel = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港间航行表.csv", encoding='utf-8')

    travel = travel.drop_duplicates(subset=['开始港口', '结束港口'], keep='first')
    n = len(order)

    # N
    data_format = '%Y-%m-%d %H:%M:%S'

    ltw_p = [datetime.strptime(t, data_format) for t in order['受载开始']]
    utw_p = [datetime.strptime(t, data_format) for t in order['受载截止']]

    ## utw_d
    utw_d = []
    for i in range(n):
        date = utw_p[i]
        if order.iloc[i]['允许跨月'] == '不允许':
            # 下个月第一天
            if date.month == 12:
                next_month = date.replace(day=1, year=date.year + 1, month=1,hour=0)
            else:
                next_month = date.replace(day=1, month=date.month + 1,hour=0)
            utw_d.append(next_month)
        else:
            # if date.month == 12:
            #     next_month = date.replace(day=1, year=date.year + 1, month=2,hour=0)
            # elif date.month == 11:
            #     next_month = date.replace(day=1, year=date.year + 1, month=1, hour=0)
            # else:
            #     next_month = date.replace(day=1, month=date.month + 2, hour=0)
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
    N['s'] = [2 for i in range(2*n+2)]

    # vehicle
    vt_h = [(dt - min_d).total_seconds() / 3600 for dt in v_t]
    vehicle['avail_t'] = vt_h
    vehicle = vehicle.rename(columns={'载重吨': 'K', })
    vehicle.index = vehicle.index.astype(str)
    vehicle_dict = vehicle[['mmsi', '运力释放港口', 'avail_t', 'K']].to_dict(orient='index')

    # travel_time
    travel_time = np.full((2 * n, 2 * n), 1.1*N['utw'][0])
    for i in range(2 * n):
        port_s = order.iloc[i]['装货港口代码'] if i < n else order.iloc[i - n]['卸货港口代码']
        for j in range(2 * n):
            port_e = order.iloc[j]['装货港口代码'] if j < n else order.iloc[j - n]['卸货港口代码']
            if port_e in list(travel[travel['开始港口'] == port_s]['结束港口']):
                bool_index = [a and b for a, b in zip(travel['开始港口'] == port_s, travel['结束港口'] == port_e)]
                travel_time[i, j] = travel[bool_index]['航行时间（小时）'].iloc[0]
    # G
    G_all={}
    for i in vehicle.index:
        G_v = np.full((2 * n + 2, 2 * n + 2), 1.1*N['utw'][0])
        G_v[:, 2 * n + 1] = 0
        avail_port = vehicle_dict['0']['运力释放港口']
        for j in range(n):
            if j == 0:
                G_v[0, j] = 0
            else:
                port_e = order.iloc[j]['装货港口代码']
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

    dummy_route = [{'route': [(0, 0), ],
                    'q': [0 for i in range(n)],
                    'cvr': 0, },
                   ]
    init_routes={}
    for i in vehicle_dict.keys():
        init_routes[i]=dummy_route

    return G_all,N,vehicle_dict,orders,init_routes



if __name__ =="__main__":

    G_all,N,vehicle_dict,orders,init_routes=data_process()
    G0 = G_all['0']['t']
    tap = []
    tad = []
    for i in range(1, 42):
        tap.append(vehicle_dict['0']['avail_t'] + G0[0][i])
        tad.append(tap[i-1]+2+G0[i][42+i])
    print(all([a > b for a, b in zip(tap, N['utw'][1:43])]))
    print(all([a > b for a, b in zip(tad, N['utw'][43:86])]))
    print(tad)
    print(N['utw'][43:86])

