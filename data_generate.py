import numpy as np
from scipy.spatial.distance import pdist, squareform

# 假设您有一组点的坐标
def data_generate():
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
    orders = [{'id': 1, 'Q': 10, 'R': 4, 'C': 50}, {'id': 2, 'Q': 5, 'R': 8, 'C': 100}]
    vehicle = {'0': {
        'K': 7,
        'avail_t': 2},
        '1': {'K': 10,
              'avail_t': 2},
        '2': {'K': 8,
              'avail_t': 0}
    }
    init_routes = {'0': [{'route': [(0, 2), (2, 4), (4, 5)],
                          'q': [0, 3],
                          'cvr': 8.9, },
                         ],
                   '1': [{'route': [(0, 1), (1, 3), (3, 5)],
                          'q': [2, 0],
                          'cvr': 9.65, },
                         ],
                   '2': [{'route': [(0, 1), (1, 3), (3, 5)],
                          'q': [2, 0],
                          'cvr': 7.65, },
                         ],
                   }
    return G,N,vehicle,orders,init_routes