import gurobipy as gp
import sys
import logging
import numpy as np

logger = logging.getLogger(__name__)

logging.basicConfig(level=logging.DEBUG)

class _MasterSolve():
    """
    Solves the master problem for the column generation procedure.

    """
    def __init__(self, vehicle:dict=None, routes:dict=None, orders:list=None, relax:bool= None, solver='gurobi', A=None,
                 cuts=None):
        if cuts is None:
            cuts = []
        if A is None:
            A = {}
            for v in vehicle.keys():
                A[v]=[]
        self.vehicle = vehicle
        self.routes = routes
        self.orders = orders
        self.solver = solver
        # create problem
        self.prob = gp.Model("MasterProblem")
        # objective
        self.objective = gp.LinExpr()
        # variables
        self.y = {}  # chartering variable
        self.x = {}  # route selection variable vr
        self.theta = {}

        self.cuts = cuts
        self.A = A

        # Parameter when minimizing global span
        self._n_columns = 1000

        self._formulate(relax)

    def _formulate(self,relax):
        """
        Set covering formulation.
        Variables are continuous when relaxed, otherwise binary.
        """
        # Add variables #
        # Route selection variables
        if relax:
            for i,_ in self.routes.items():
                self.x[i] = self.prob.addVar(lb=0,ub=1,vtype = gp.GRB.CONTINUOUS,name=f'x0_{i}')
            # chartering variable
            for i in range(len(self.orders)):
                self.y[i] = self.prob.addVar(0,1,vtype = gp.GRB.CONTINUOUS,name=f'y{i}')
        else:
            for i,r in self.routes.items():
                self.x[i] = self.prob.addVars(len(r),lb=0,ub=1,vtype = gp.GRB.BINARY,name=f'x{i}')
            # chartering variable
            for i in range(len(self.orders)):
                self.y[i] = self.prob.addVar(0,1,vtype = gp.GRB.BINARY,name=f'y{i}')
        # ratio variables of delivery pattern
        for i, v in self.routes.items():
            self.theta[i] = []
            for r in range(len(v)):
                # {'0':[route1 的一组变量,route 2 的一组变量,],}
                self.theta[i].append(self.prob.addVars(len(v[r]['q']),lb=0,ub=1,vtype = gp.GRB.CONTINUOUS,name=f'theta{i}'))

        # add cuts
        if self.cuts is not []:
            for cut in self.cuts:
                cut = list(cut)
                temp_expr = gp.LinExpr()
                for v, rs in self.A.items():
                    for r in range(len(rs)):
                        temp_expr.add(self.theta[v][r].sum(), np.sum(rs[r][cut]))
                self.prob.addConstr(temp_expr <= 2, name="customer_cut")



        # add links between x and theta
        if relax:
            for i, v in self.routes.items():
                temp_LinExpr = gp.LinExpr()
                for r in range(1, len(v)):
                    temp_LinExpr.add(self.theta[i][r].sum())
                self.prob.addConstr((temp_LinExpr + self.x[i] == 1),name="route_selection")
        else:
            # Add route-selection const
            for i in self.routes.keys():
                self.prob.addConstr((self.x[i].sum() == 1),name="route_selection")
            # add links between x and theta
            for i, v in self.routes.items():
                for r in range(1, len(v)):
                    self.prob.addConstr((self.theta[i][r].sum() == self.x[i][r]), name="links_between_xandthera")


        # Add demand const
        for item in self.orders:
            temp_LinExpr = gp.LinExpr()
            for i,v in self.routes.items():
                for r,route in enumerate(v):
                    q = dict([[w,pattern[item['id']]] for w,pattern in enumerate(route['q'])])
                    temp_LinExpr.add(self.theta[i][r].prod(q))
            self.prob.addConstr((temp_LinExpr >= item['Q']*(1-self.y[item['id']])),name="demand_const")
#


        # Set objective function
        costs = [-order['C'] for order in self.orders]
        revenue = [order['R'] for order in self.orders]
        self.objective.addTerms(costs,list(self.y.values()))
        self.objective.addConstant(sum(revenue))
        for i,v in self.routes.items():
            for r, route in enumerate(v):
                self.objective.add(self.theta[i][r].sum(),-route['cvr'])
        self.prob.setObjective(self.objective,gp.GRB.MAXIMIZE)

        self.prob.write('model.lp')

    def solve(self, relax, time_limit):
        self._solve(relax, time_limit)
        logger.debug("master problem relax %s" % relax)
        logger.debug("Status: %s" % self.prob.getAttr(gp.GRB.Attr.Status))
        logger.debug("Objective: %s" % self.prob.getAttr( gp.GRB.Attr.ObjVal))

        if self.prob.getAttr(gp.GRB.Attr.Status) != gp.GRB.Status.OPTIMAL:
            raise Exception("problem " + str(self.prob.getAttr(gp.GRB.Attr.Status)))
        if relax:
            duals = self.get_duals()
            logger.debug("duals : %s" % duals)
            return duals, self.prob.getAttr(gp.GRB.Attr.ObjVal)

    def _solve(self, relax: bool, time_limit=None):
        # Set variable types
        if relax:
            self.prob.setParam("MIPFocus", 2) # 重点求LP松弛
        else:
            self.prob.setParam("MIPFocus", 3) #重点求整数解
        # Solve with appropriate solver

        # 设置求解器参数
        self.prob.setParam("Method", 2)  # 设置为内点法（Barrier）
        self.prob.setParam("Crossover", 0)  # 设置为不使用割平面

        # Only specify time limit if given
        if time_limit is not None:
            self.prob.setParam("TimeLimit", time_limit)  # 设置时间限制

        self.prob.optimize()


    # def update(self, new_route):
    #     """Add new column."""
    #     self._add_route_selection_variable(new_route)

    def get_duals(self):
        """Gets the dual values of each constraint of the master problem.

        Returns:
            dict: Duals with constraint names as keys and dual variables as values
        """
        duals = {"route_selection":[],"demand_const":[],"customer_cut":[]}
        for constr in self.prob.getConstrs():
            if constr.ConstrName == "route_selection":
                duals["route_selection"].append(constr.Pi)
            elif constr.ConstrName == "demand_const":
                duals["demand_const"].append(constr.Pi)
            elif constr.ConstrName == "customer_cut":
                duals["customer_cut"].append(constr.Pi)
        return duals

    def print_solution(self):
        for v,routes in self.routes.items():
            if self.x[v][0].X == 1:
                print(f"船{v}未使用")
            else:
                for r, route in enumerate(routes):
                    if r != 0 and self.x[v][r].X == 1:
                        # dict 形式，存放车v路线r的各pattern的比例值
                        print(f'船{v}使用，其路线：', route)
                        print(self.theta[v][r])
                        theta_vr = self.prob.getAttr("X", self.theta[v][r])
                        print('配送模式比例',theta_vr)
                        q_sum = [0 for i in range(len(self.orders))]
                        for w,ratio in theta_vr.items():
                            q_sum = [qs + ratio * qi for qs,qi in zip(q_sum,route['q'][w])]
                        print(f'船{v}装载情况：',q_sum)
        print('订单外包情况：', self.y)
    def get_theta(self):
        theta_vr={}
        for v,routes in self.routes.items():
            theta_vr[v]=[]
            for r, route in enumerate(routes):
                if r == 0:
                    theta_vr[v].append(self.x[v].X)
                else:
                    theta_vr[v].append(sum(self.prob.getAttr("X", self.theta[v][r]).values()))
        return theta_vr





if __name__ == "__main__":

    orders=[{'id':0,'Q':10,'R':4,'C':5},{'id':1,'Q':4,'R':8,'C':4}]
    vehicle={'0':{'K':8},
             '1': {'K': 7},
             }
    routes={'0':[{'route':[],
                 'q':[],
                 'cvr':0,},
                {'route':[],
                 'q':[[4,4],[4,1]],
                 'cvr':2,},],
            '1':[{'route':[],
                 'q':[],
                 'cvr':0,},
                {'route':[],
                 'q':[[6,1],],
                 'cvr':2,},],
            }
    m = _MasterSolve(vehicle,routes,orders,False)
    m.solve(False,10)
    # for i,item in m.x.items():
    #     for j,c in item.items():
    #         if c.X == 1:
    #             print(i,j)
    print(m.y)
    print(m.theta)
    # print(m.prob.getConstrs())
