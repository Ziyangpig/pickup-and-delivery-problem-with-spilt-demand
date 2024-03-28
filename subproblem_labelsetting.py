class Label_split():
    '''
    This class is used to construct the Labels for the Labelling algorithm used to solve the constrained shortest path problem
    '''

    def __init__(self, number_of_node, parent_node, arrival_time, is_split, has_split, s_alpha, V_parent, O_parent,
                 load, max_split, cost, d, n, ):
        self.id = number_of_node
        self.p = parent_node  # previous node in the path
        self.t = arrival_time  # arrival time in Label L
        self.S = is_split
        self.P = has_split
        self.pai = s_alpha
        self.V = set(V_parent)  # V is the set of requests served by this path that may have already been served
        self.construct_V()
        self.O = set(O_parent)  # O is the set of open requests. Requests that have started but not delivered yet
        self.construct_O(n)
        self.l = load  # cummulative load at this node using the path
        self.delta = max_split
        self.c = cost  # the cost on this solution
        self.d = d
        # self.sigma = id_voyage # 从1开始

    def construct_V(self):
        self.V = self.V.union({self.id})

    def construct_O(self, n):
        # print("id",self.id)
        # print("O",self.O)
        # print(self.id in self.O)
        Pnode = set(range(1, n + 1))
        D = set(range(n + 1, 2 * n + 1))

        if (self.id in D) and ((self.id - n) in self.O):
            self.O = self.O.difference({self.id - n})
        elif self.id in Pnode:
            self.O = self.O.union({self.id})

    def check_cardinality_O(self):
        if self.O.size <= 2:
            return True
        else:
            return False


class SP1:
    def __init__(self, g, data_dict, num_arcs_to_consider, paths_per_itter, is_heuristic=False):
        self.distance_matrix = data_dict["distances"]
        self.n = data_dict["num_of_requests"]
        self.load_per_req = data_dict["load"]
        self.alpha = data_dict["dual_alpha"]
        self.miu = data_dict["route_selection"]
        self.time_window = data_dict["time_windows"]
        self.speed = data_dict["vehicle_speed"]
        self.K = data_dict["vehicle_capacity"]
        self.U = []
        self.g = g
        self.num_arcs_to_consider = num_arcs_to_consider
        self.paths_per_itter = paths_per_itter
        self.heuristic = is_heuristic

    def update_dominance(self, L, new_path):
        '''
        To use this function triangle inequality should be satisfied both on distances and times
        U为目前所有路径，L为新标签
        '''

        def RC_compare(L1, L2):
            f1 = L1.c >= L2.c
            f2 = L1.c - L1.pai * L1.delta >= L2.c - L2.pai * L2.delta
            x = min(L1.delta, L2.delta)
            f3 = L1.c - L1.pai * x >= L2.c - L2.pai * x
            return f1 and f2 and f3

        def all_compare(L_i, L):
            flag = False
            if L_i.id == L.id:  # for the labels paths ending on the same node
                if (L_i.t <= L.t and L_i.S <= L.S and L_i.P <= L.P and L_i.V.issubset(L.V) and L_i.O.issubset(L.O)
                        and L_i.l <= L.l):
                    flag = RC_compare(L_i, L)
            return flag

        def heuristic_compare(L_i, L):
            flag = False
            if L_i.id == L.id:  # for the labels paths ending on the same node
                if L_i.t <= L.t and L_i.S <= L.S and L_i.P <= L.P and L_i.l <= L.l:
                    flag = RC_compare(L_i, L)
            return flag

        if self.U:
            for paths in self.U:
                L_i = paths[-1]
                if self.heuristic:
                    if heuristic_compare(L, L_i):
                        if L != L_i:
                            self.U.remove(paths)
                    if heuristic_compare(L_i, L):
                        break
                else:
                    if all_compare(L, L_i):
                        if L != L_i:
                            # 这里删除的时候，0-p-d-...这种类型路线有两种一样的，如果p点的Qi比车容量小，那p被选择split点的时候，还是全装了，
                            # 由于被指为1的后加进去，指为0的路径就会被删除，但不影响结果
                            # print("删除")
                            # print(vars(L),'\n',vars(L_i))
                            self.U.remove(paths)
                    if all_compare(L_i, L):
                        break
            self.U.append(new_path + [L])
            # print("增加",vars(L))
        else:
            self.U.append(new_path + [L])

    def travel_time(self, from_node, to_node):
        """Gets the travel times between two locations."""
        travel_time = (self.distance_matrix[from_node][to_node] / self.speed) * 60
        return travel_time

    def get_paths(self, result):
        path_info = []
        t = []
        q = []
        for path in result:
            temp_n = []
            temp_t = []
            temp_l = []
            temp_q = []
            temp_s = []
            split_load = 0  # 遍历路径时，记录当前split的量
            deliv_id = -1  # 记录当前split对应的delivery点
            for node in path:
                temp_n.append(node.id)
                temp_t.append(node.t)
                # 当出现split点时，索引到切分量、deli的id
                if node.S == 1:
                    deliv_id = node.id + self.n

                    deliv_seq = path.index(next(node2 for node2 in path if node2.id == deliv_id))
                    temp_s.append([node.id, path[deliv_seq - 1].delta])
                    split_load = path[deliv_seq - 1].delta
                    # # 如果有split为Qi的点，直接删除该标签即可，有完全等效标签
                    # if split_load == self.load_per_req[node.id]:
                    #     break
                # 当遍历到deliv_id时，结束当前split累加，等待出现新的split，split_load便被重新赋值
                if node.id == deliv_id:
                    split_load = 0
                temp_l.append(node.l + split_load)
            # if split_load == self.load_per_req[node.id]:
            #     continue
            # 根据递推完的load和split推q
            for i in range(1, self.n + 1):
                try:
                    p_index = temp_n.index(i)
                except ValueError:
                    p_index = -1
                if p_index == -1:
                    temp_q.append(0)
                else:
                    temp_q.append(temp_l[p_index] - temp_l[p_index - 1])
            # 删除split为Qi的点
            temp_s = [[i, delta] for i, delta in temp_s if delta != self.load_per_req[i]]
            temp = {'route': temp_n, 'cvr': path[-1].t, 'time_cost': path[-1].t, 'time': temp_t,
                    'q': temp_q, 'load': temp_l, 'split': temp_s}
            path_info.append(temp)
        return path_info

    # 返回一组路线或者空list，每个路线也是一个list，由一组标签组成
    def calculate_H1(self, discovered_paths, dij=None):
        dij = self.distance_matrix
        Pnode = set(range(1, self.n + 1))
        D = set(range(self.n + 1, 2 * self.n + 1))
        # initialize variables
        result = []
        r_num = 0
        # if self.heuristic:
        # 初始化标签
        s = 0
        V_parent = []
        O_parent = []
        parent_node = -1
        source_time = self.time_window[s][0]
        is_split, has_split, s_alpha, load, max_split, cost, dij_cost = 0, -1, 0, self.load_per_req[s], 0, -self.miu, 0
        #  number_of_node,parent_node,arrival_time,is_split,has_split,s_alpha,V_parent, O_parent, load,max_split, cost, d,P, D,  n,):
        L = Label_split(s, parent_node, source_time, is_split, has_split, s_alpha, V_parent, O_parent, load, max_split,
                        cost, dij_cost, self.n)
        self.U.append([L])  # U stores all the paths
        iter_t = 0
        while self.U:
            iter_t += 1
            # path是一个list，依次存储了路线上各点标签

            path = self.U.pop(0)  # U is a queue first in first out!
            L = path[-1]  # get the last node of the path
            i = L.id

            # storing the paths reaching the final node 7
            temp = []
            for node in path:
                temp.append(node.id)

            # ******************设置停止条件***************
            # 若当前标签已经到达终点i == 2*self.n+1，并且rc<0,并且该标签不在discovered_paths，则直接返回该标签，终止循环即可
            # 还可以设置paths_per_itter，目前为1，只要找到了大于paths_per_itter数量的路径，就可以直接终止循环
            if i == 2 * self.n + 1 and L.c > 0:
                path_info = self.get_paths([path])  # path是一组标签，current_path是[[一组点标号],]
                flag = 0
                for r in discovered_paths:
                    if (path_info[0]['route'] == r['route'] and path_info[0]['q'] == r['q']
                            and path_info[0]['time_cost'] == r['time_cost']):
                        flag = 1
                        break
                if flag == 0:
                    result.extend(path_info)
                    discovered_paths.extend(path_info)
                    r_num += 1
                    if r_num >= self.paths_per_itter:
                        print("共处理了", iter_t, '条路径')
                        return result, True

            # ***************按照距离先选扩展点*********
            # expand path on every incident edge
            if self.g.successors(i):
                pickup_inc = []
                delivery_inc = []
                end_inc = []
                incident_nodes = list(self.g.successors(i))  # 包含PD先行约束、起点、终点与PD点间的一些约束
                d_sorted = [dij[i][incident_node] for incident_node in incident_nodes]
                sorted_y_idx_list = sorted(range(len(d_sorted)), key=lambda x: d_sorted[x])
                incident_nodes_sorted = tuple(incident_nodes[i] for i in sorted_y_idx_list)
                # 按与i点距离远近被加到pick node和delivery node中
                for iterator1 in range(len(incident_nodes_sorted)):
                    if incident_nodes_sorted[iterator1] in Pnode and len(pickup_inc) < self.num_arcs_to_consider and \
                            incident_nodes_sorted[iterator1] not in temp:
                        pickup_inc.append(incident_nodes_sorted[iterator1])
                    elif incident_nodes_sorted[iterator1] in D and len(delivery_inc) < self.num_arcs_to_consider and \
                            incident_nodes_sorted[iterator1] not in temp:
                        delivery_inc.append(incident_nodes_sorted[iterator1])
                    elif incident_nodes_sorted[iterator1] == 2 * self.n + 1:
                        end_inc.append(incident_nodes_sorted[iterator1])
                    # 要么for遍历完所有邻接点，也没达到self.num_arcs_to_consider，
                    # 要么pn和dn均提前达到self.num_arcs_to_consider，直接结束循环
                    if len(delivery_inc) == self.num_arcs_to_consider and len(pickup_inc) == self.num_arcs_to_consider:
                        break

                inc_nodes = pickup_inc + delivery_inc + end_inc

                # *******************开始逐个扩展*******************
                for j in range(len(inc_nodes)):  # get all incident nodes from i
                    # REF
                    current_node = inc_nodes[j]
                    dij_cost = self.distance_matrix[i][current_node] + L.d
                    t_j = self.distance_matrix[i][current_node] + L.t

                    if t_j > self.time_window[current_node][1]:
                        continue
                    elif t_j < self.time_window[current_node][0]:
                        t_j = self.time_window[current_node][0]

                    cummulative_load = L.l + self.load_per_req[current_node]
                    if 0 < current_node <= self.n and (current_node not in L.V):
                        # S赋值
                        if L.l < self.K:
                            if L.P == -1:
                                if cummulative_load <= self.K:
                                    S = 2  # 表示可以增加两个标签
                                else:
                                    S = 1
                            else:
                                if cummulative_load <= self.K:
                                    S = 0
                                else:
                                    continue  # 如果已经有了split，则不能再去装不全的pickup点
                        else:
                            continue  # 0 pickup点就不用扩展了

                        def update_pick_label(S):
                            if S == 0:
                                pai = L.pai
                                P = L.P
                                l = cummulative_load
                                if P == -1:
                                    delta = 0
                                else:
                                    delta = min(L.delta, self.K - l)
                                c = L.c - self.distance_matrix[i][current_node] - self.alpha[current_node - 1] * \
                                    self.load_per_req[current_node]
                            else:  # S == 1:
                                pai = self.alpha[current_node - 1]
                                P = current_node
                                l = L.l
                                delta = min(self.load_per_req[current_node], self.K - l)
                                c = L.c - self.distance_matrix[i][current_node]
                            return pai, P, l, delta, c

                        if S == 2:
                            state_S2 = []
                            for ii in range(2):
                                state_S2.append(update_pick_label(ii))
                        else:
                            pai, P, l, delta, c = update_pick_label(S)

                    elif self.n < current_node <= 2 * self.n + 1 and ((current_node - self.n) in L.O):
                        S = 0

                        def update_delivery_label():
                            # delivery 情况 S默认为0即可
                            pai = 0 if L.P == current_node - self.n else L.pai
                            if current_node - self.n == L.P:
                                P = -1
                                l = L.l
                                c = L.c - self.distance_matrix[i][current_node] - L.delta * L.pai

                            else:
                                P = L.P
                                l = cummulative_load
                                c = L.c - self.distance_matrix[i][current_node]
                            delta = 0 if P == -1 else L.delta
                            return pai, P, l, delta, c

                        pai, P, l, delta, c = update_delivery_label()

                    elif current_node == 2 * self.n + 1 and not L.O:
                        S = 0
                        P = 0
                        pai = 0
                        l = L.l
                        delta = 0
                        c = L.c - self.distance_matrix[i][current_node]
                    else:
                        continue
                    # *********添加新标签*************
                    ################################

                    if S == 2:
                        for jj, state in enumerate(state_S2):
                            pai, P, l, delta, c = state
                            L_new = Label_split(current_node, i, t_j, jj, P, pai, L.V, L.O, l, delta, c,
                                                dij_cost, self.n)
                            self.update_dominance(L_new, path)
                    else:
                        L_new = Label_split(current_node, i, t_j, S, P, pai, L.V, L.O, l, delta, c,
                                            dij_cost, self.n)
                        self.update_dominance(L_new, path)
        print("共处理了", iter_t, '条路径')
        return result, r_num > 0
