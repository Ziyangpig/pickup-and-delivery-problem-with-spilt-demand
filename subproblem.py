import logging
import gurobipy as gp

logger = logging.getLogger(__name__)


class _SubProblemLP:
    """
    Solves the sub problem for the column generation procedure ; attemps
    to find routes with negative reduced cost.

    Inherits problem parameters from `SubproblemBase`
    """

    def __init__(self, duals,G,N,orders, vehicle, solver):
        # Input attributes
        self.G = G
        self.N = N
        self.orders = orders
        Q = [o['Q'] for o in self.orders]
        self.Q = Q
        self.sub_G = self.G
        self.duals = duals
        self.vehicle = vehicle
        self.solver = solver
        self.routes = []
        #
        r=len(self.G['d'])
        self.ordernum = int((r-2)/2)
        self.portnum = r
        self.M_t =2*self.N['utw'][self.portnum-1]

        # create problem
        self.prob = gp.Model("SubProblem")
        # flow variables

        self.x = self.prob.addVars(r,r,ub=1,lb=0,vtype=gp.GRB.BINARY,name='x')
        self.q = self.prob.addVars(range(self.ordernum),ub=self.Q,vtype=gp.GRB.CONTINUOUS,name='q')
        self.l = self.prob.addVars(r,ub=self.vehicle['K'],vtype=gp.GRB.CONTINUOUS,name='l')
        self.tl = self.prob.addVars(r,vtype=gp.GRB.CONTINUOUS,name='tl')
        self.ta = self.prob.addVars(r,vtype=gp.GRB.CONTINUOUS,name='ta')
        self.u = self.prob.addVars(r, vtype=gp.GRB.CONTINUOUS,name='u')

    def solve(self, time_limit=None):
        if time_limit and time_limit <= 0:
            return self.routes, False

        self._formulate()

        self._solve(time_limit)

        logger.debug("Solving subproblem using MIP")
        logger.debug("Status: %s" % self.prob.getAttr(gp.GRB.Attr.Status))
        more_routes = False
        if self.prob.getAttr(gp.GRB.Attr.Status) not in [3,5]: #["INFEASIBLE","UNBOUNDED"]:
            if self.prob.getAttr(gp.GRB.Attr.ObjVal) is not None:
                logger.debug("Objective: %s" % self.prob.getAttr(gp.GRB.Attr.ObjVal))
                if self.prob.getAttr(gp.GRB.Attr.ObjVal) > (10 ** -5):
                    more_routes = True
                    self._add_new_route()

        return self.routes, more_routes


    def _solve(self, time_limit):
        if self.solver == "gurobi":
            # Only specify time limit if given
            if time_limit is not None:
                self.prob.setParam("TimeLimit", time_limit)  # 设置时间限制

            self.prob.optimize()

    def _formulate(self):
        # flow balance
        for ii in range(self.portnum):
            if ii == 0:
                self.prob.addConstr(self.x.sum(ii,'*')==1)
            elif ii == self.portnum-1:
                self.prob.addConstr(self.x.sum('*',ii ) == 1)
            else:
                self.prob.addConstr(self.x.sum(ii,'*') == self.x.sum('*',ii ))
        # 每个点最多只能访问一次
        self.prob.addConstrs((self.x.sum(ii, '*') <= 1 for ii in range(self.portnum-1)),)
        # 不能单点自循环
        self.prob.addConstrs((self.x.sum(ii, ii) == 0 for ii in range(self.portnum)),)
        # order finish
        for i in range(self.ordernum):
            self.prob.addConstr(self.x.sum(i+1, '*') == self.x.sum(i+self.ordernum, '*'))
        # load
        for j in range(self.ordernum+1):
            if j == 0:
                self.prob.addConstr(self.l[j] == 0)
            else:
                self.prob.addConstrs(self.l[i]+self.q[j-1] <= self.l[j] + self.vehicle['K']*(1-self.x[(i,j)]) for i in range(self.portnum))
        for j in range(self.ordernum+1):
            if j == 0:
                self.prob.addConstr(self.l[self.portnum-1] == 0)
            else:
                self.prob.addConstrs((self.l[i] - self.q[j-1] <= self.l[j+self.ordernum] + self.vehicle['K'] * (1 - self.x[(i, j+self.ordernum)]) for i in range(self.portnum)))
        # capacity const
        for j in range(self.ordernum):
            self.prob.addConstr(self.l[j + 1 + self.ordernum]+self.q[j] <= self.vehicle['K']*self.x.sum(j+1,'*'))
        # time
        for j in range(self.portnum):
            if j == 0:
                # avail_time
                self.prob.addConstr(self.tl[j] >= self.vehicle['avail_t'])

            elif j == self.portnum - 1:
                pass
            else:
                # port load/unload time
                self.prob.addConstr((self.tl[j] + self.M_t*(1 - self.x.sum('*', j)) >= self.ta[j] + self.N['s'][j]),name="load_unload_time")
                # tw
                self.prob.addConstr(self.ta[j] + self.M_t*(1 - self.x.sum('*', j)) >= self.N['ltw'][j],name='ltw')
                self.prob.addConstr(self.ta[j] - self.M_t * (1 - self.x.sum('*', j)) <= self.N['utw'][j],name='utw')
        # time recursion between different ports
        self.prob.addConstrs((self.tl[i]+self.G['t'][i][j]-self.ta[j] <= self.M_t*(1-self.x[(i,j)])
                              for i in range(self.portnum) for j in range(self.portnum) if i!=self.portnum-1 and j!=0),name='travel_time')
        # precedence const
        self.prob.addConstrs((self.tl[i+1] + self.G['t'][i+1][i+1+self.ordernum] - self.ta[i+1+self.ordernum] <= self.M_t * (1 - self.x.sum(i+1,'*')) for i in range(self.ordernum)))
        # subtour_elimination
        self.prob.addConstrs((self.u[i] - self.u[j] + self.portnum * self.x[i, j] <= self.portnum - 1 for i in range(self.portnum) for j in range(self.portnum) if i != j),)

        # minimize reduced cost
        obj = gp.LinExpr()
        dual_demand = [-i for i in self.duals['demand_const']]
        # obj.add(-self.ta[self.portnum-1])
        obj_c = {(i, j): -value for i, row in enumerate(self.G['d']) for j, value in enumerate(row)}
        obj.add(self.x.prod(obj_c))
        obj.addTerms(dual_demand, list(self.q.values()))
        print(self.duals['route_selection'])
        obj.addConstant(-self.duals['route_selection'])
        self.prob.setObjective(obj,sense=gp.GRB.MAXIMIZE)
        # Forbid route Source-Sink
        # self.prob += (
        #     pulp.lpSum([self.x[(i, j)] for (i, j) in self.sub_G.edges()]) >= 2,
        #     "at_least_1_stop",
        # )
        self.prob.write('submodel.lp')

    def _add_new_route(self):
        new_route = {}
        # TODO 改存储形式
        new_route["route"] = [i for i, item in self.x.items() if item.X == 1]
        new_route["q"] =[value.X for value in self.q.values()]
        new_route["cvr"] =self.ta[self.portnum-1].X
        self.routes.append(new_route)

        logger.debug("new route reduced cost %s" % self.prob.getAttr(gp.GRB.Attr.ObjVal))
        logger.debug("new route cost = %s" % new_route["cvr"])


if __name__ == "__main__":
    # import numpy as np
    # from scipy.spatial.distance import pdist, squareform
    #
    # # 假设您有一组点的坐标
    # points = np.array([
    #     [0, 0],
    #     [0, 1],
    #     [1, 1],
    #     [2, 0],
    #     [2, 1],
    #     [3, 1],
    # ])
    #
    # # 使用pdist函数计算所有点对之间的距离
    # distances = pdist(points, metric='euclidean')  # 使用欧几里得距离
    #
    # # 使用squareform函数将距离向量转换为距离矩阵
    # distance_matrix = squareform(distances)
    # print(distance_matrix)
    # G={'d':distance_matrix,'t':distance_matrix,}
    # N={'ltw':[0,2,3,6,9,0],'utw':[50,4,6,9,12,50],'s':[0,1,2,2,1.5,0],'draft':[]}
    # orders=[{'id':1,'Q':10,'R':4,'C':5},{'id':2,'Q':5,'R':8,'C':4}]
    # vehicle={'0':{
    #               'K':7,
    #               'avail_t':2}
    #          }
    from data_process import data_process
    G, N, vehicle, orders, init_routes = data_process()
    duals={"route_selection":2,}
    duals["demand_const"]= [-100 for i in range(len(orders)) ]
    solver='gurobi'
    m=_SubProblemLP(duals,G['0'],N,orders, vehicle['0'], solver)
    m.solve(None)
    for i,item in m.x.items():
        if item.X == 1:
            print(i)
    print(m.q)
    print(m.ta)
    print(m.tl)
    print(m.l)
    print("初始",m.routes)
    m._add_new_route()
    print("after",m.routes)