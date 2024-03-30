# 一条边最多出现一次
import numpy as np
import gurobipy as gp
# 1.根据self.routes 得到Aijvr 索引和theta保持一致
# 2.遍历pick组合 i,i' 得到对应的违反值
# 3. 排序，筛选出部分pick组合
# 添加cuts
# dij转换 对偶值

def update_A(A,routes,v,n):
    for r in routes:
        temp_a = np.zeros((2*n+2,2*n+2))
        for i,id in enumerate(r['route'][:-1]):
            id_2 = r['route'][i+1]
            temp_a[id,id_2] = 1
        A[str(v)].append(temp_a)
    return A

def separation_customer_cut(A,n, theta, customer_cut_set):
    new_cut_set = []
    for node in range(1,n+1):
        for second_node in range(node+1,n+1):
            cut = {node, second_node}
            if cut not in customer_cut_set:
                customer_cut_set.append(cut)
                new_cut_set.append([cut, violation_value(cut, A, theta)])
    new_cut_set = sorted(new_cut_set, key=lambda x: x[1], reverse=True)
    # print(f"Separation took : {time.time()-start} sec")
    num_cut=min(len(new_cut_set),15)
    return [a[0] for a in new_cut_set[:num_cut] if a[1] > 0],customer_cut_set

def violation_value(cut, A, theta):
    value = 0
    cut = list(cut)
    for v, rs in A.items():
        for r in range(len(rs)):
            value += np.sum(rs[r][cut])*theta[v][r]
    return value - 2

def update_master_customer_cuts(RMP,A, cuts):
    for cut in cuts:
        cut = list(cut)
        temp_expr=gp.LinExpr()
        for v, rs in A.items():
            for r in range(len(rs)):
                temp_expr.add(RMP.theta[v][r].sum(),np.sum(rs[r][cut]))
        RMP.prob.addConstr(temp_expr<=2, name="customer_cut")
    return RMP

