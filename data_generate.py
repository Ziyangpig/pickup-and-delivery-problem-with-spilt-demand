import numpy as np
from scipy.spatial.distance import pdist, squareform

import networkx as nx
import matplotlib.pyplot as plt


# 假设您有一组点的坐标
#
def data_generate_old():
    points = np.array([
        [0, 0],
        [0, 1],
        [1, 1],
        [2, 0],
        [2, 1],
        [3, 1],
    ])

    # 使用pdist函数计算所有点对之间的距离
    distances = pdist(points, metric='euclidean')  # 使用欧几里得距离

    # 使用squareform函数将距离向量转换为距离矩阵
    distance_matrix = squareform(distances)
    G = {'d': distance_matrix, 't': distance_matrix, }
    N = {'ltw': [0, 2, 3, 6, 9, 0], 'utw': [50, 4, 6, 9, 12, 50], 's': [0, 1, 2, 2, 1.5, 0], 'draft': []}
    orders = [{'id': 0, 'Q': 10, 'R': 10, 'C': 50}, {'id': 1, 'Q': 5, 'R': 10, 'C': 100}]
    vehicle = {
        '0': {'K': 7,
              'avail_t': 0},
        '1': {'K': 10,
              'avail_t': 0},
        '2': {'K': 8,
              'avail_t': 0}
    }

    init_routes = {'0': [{'route': [],
                          'q': [],
                          'cvr': 0,
                          'time':[0],
                          'time_cost':0,
                          'load':[],
                          'split':[]},
                         ],
                   '1': [{'route': [],
                          'q': [],
                          'cvr': 0,
                          'time':[0],
                          'time_cost':0,
                          'load':[],
                          'split':[],},

                         ],
                   '2': [{'route': [],
                          'q': [],
                          'cvr': 0,
                          'time':[0],
                          'time_cost':0,
                          'load':[],
                          'split':[],},

                         ]
                   }

    return G,N,vehicle,orders,init_routes

def data_generate(n,cargo_size,W,T=600,seed=2,L=50,K=[90,120,150],number_vehicle = 10):
    np.random.seed(seed)
    # G
    distance_mat=generate_distance_matrix(n, seed, ld=0.4*L, ud=1.4*L)
    G = {'d': distance_mat, 't': distance_mat, }
    G_all = {}
    for i in range(number_vehicle):
        G_all[str(i)] = {'d': distance_mat, 't': distance_mat,}

    # 车辆数据生成
    avail_t=np.random.randint(0,0.1*T,number_vehicle)
    K_arr=np.random.choice(K, size=number_vehicle, replace=True)
    vehicle = {}
    for v in range(number_vehicle):
        vinfo={}
        vinfo['K'] = K_arr[v]
        vinfo['avail_t'] = avail_t[v]
        vehicle[str(v)] = vinfo
    # orders [{'id': 0, 'Q': 10, 'R': 10, 'C': 50}, {}]
    orders = []
    load = create_load(n, seed, round(min(K) * cargo_size[0]), round(min(K) * cargo_size[1]))
    for i in range(n):
        temp = {'R': 100, 'C': 200}
        temp['id']=i
        temp['Q']=load[i]
        orders.append(temp)
    # N = {'ltw': [0, 2, 3, 6, 9, 0], 'utw': [50, 4, 6, 9, 12, 50], 's': [0, 1, 2, 2, 1.5, 0],}
    ltw, utw = create_time_window(n,W,T,G,seed)
    N = {'ltw': ltw, 'utw': utw, 's':list(np.full(2*n+2,2))}

    return G_all, N, vehicle, orders

def create_time_window(N,W,T,G,seed=2):
    Tp_l,Td_l=[],[]
    np.random.seed(seed)
    for i in range(N):
        t=np.random.randint(0, 0.5*T - G['t'][i][i + N])
        Tp_l.append(t)
        Td_l.append(t+G['t'][i][i+N])
        seed += 1
        np.random.seed(seed)
    ltw = [0]+ Tp_l + Td_l + [0]
    utw = [lt+W for lt in ltw]
    utw[0],utw[-1]=T,T
    return ltw,utw



def create_pd_DiGragh(n):
    # 创建一个空的有向图
    G = nx.DiGraph()

    # 定义节点数量和起点、终点编号
    # n pickup 和 delivery 点的数量
    start_node = 0
    end_node = 2 * n + 1

    # 添加节点
    G.add_nodes_from(range(end_node + 1))

    # 添加完全图中所有边
    for i in range(start_node, end_node + 1):
        for j in range(start_node, end_node + 1):
            if i != j:
                G.add_edge(i, j)

    # 删除起点到 delivery 点及终点的边
    for i in range(n + 1, end_node + 1):
        G.remove_edge(start_node, i)

    # 删除所有点到起点的边
    for i in range(start_node, end_node + 1):
        if i != start_node:
            G.remove_edge(i, start_node)

    # 删除 delivery 点到对应的 pick 点的边
    for i in range(1, n + 1):
        G.remove_edge(i + n, i)

    # 删除 pick 点到终点的边
    for i in range(1, n + 1):
        G.remove_edge(i, end_node)

    # 删除终点到除起点外所有点的边（前面删了一遍所有点到起点的边）
    for i in range(start_node, end_node):
        if i != 0:
            G.remove_edge(end_node, i)

    return G
    #
    # # 绘制有向图
    # pos = nx.spring_layout(G)
    # nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, arrows=True)
    # plt.title("Pickup and Delivery Problem")
    # plt.show()


def generate_distance_matrix(n,seed=2,ld=1,ud=10):
    np.random.seed(seed)
    distance_matrix = np.zeros((2*n+2, 2*n+2))
    # 生成对称距离矩阵
    for i in range(2*n+2):
        for j in range(i+1, 2*n+2):
            if i == j:
                distance_matrix[i, j] = 0
            else:
                distance_matrix[i, j] = np.random.randint(ld, ud)  # 随机生成距离值
                distance_matrix[j, i] = distance_matrix[i, j]  # 对称性，距离矩阵是对称的

    # 所有点到终点的距离为0
    for i in range(2*n+2):
        distance_matrix[i,2*n+1] = 0

    # 确保满足三角不等式
    for i in range(2*n+2):
        for j in range(2*n+2):
            for k in range(2*n+2):
                if i != j and j != k and i != k:
                    if distance_matrix[i, j] + distance_matrix[j, k] < distance_matrix[i, k]:
                        distance_matrix[i, k] = distance_matrix[i, j] + distance_matrix[j, k]

    return distance_matrix


def create_load(n,minimum_load,maximum_load, seed=2):
    np.random.seed(seed)
    load = []
    for i in range(1,n+1):
        load.append(np.random.randint(minimum_load, maximum_load))
    return load

if __name__ == "__main__":
    seed = 2
    N, cargo_size = 15,[0.3,0.8]
    K = [90, 120, 150]
    W, T = 60,600
    L = 100 # 运输区域边长
    number_vehicle = 10
    G, N, vehicle, orders= data_generate(N,cargo_size,W,T,seed,L,K,number_vehicle)
    print(vehicle, '\n', orders, '\n', N,)
