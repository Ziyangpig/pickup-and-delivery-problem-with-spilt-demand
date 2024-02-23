import gurobipy as gp
import sys
import logging

logger = logging.getLogger(__name__)

logging.basicConfig(level=logging.INFO, stream=sys.stdout)


class _MasterSolve():
    """
    Solves the master problem for the column generation procedure.

    """

    def __init__(self,  vehicle:dict=None, routes:dict=None, orders:list=None, relax:bool= None,solver='gurobi'):
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
            for i,v in self.routes.items():
                self.x[i] = self.prob.addVars(len(v), lb=0,ub=1,vtype = gp.GRB.CONTINUOUS)
            # chartering variable
            for item in self.orders:
                self.y[item['id']] = self.prob.addVar(0,1,vtype = gp.GRB.CONTINUOUS)
        else:
            for i,v in self.routes.items():
                self.x[i] = self.prob.addVars(len(v), lb=0,ub=1,vtype = gp.GRB.BINARY)
            # chartering variable
            for item in self.orders:
                self.y[item['id']] = self.prob.addVar(0,1,vtype = gp.GRB.BINARY)
        # Add route-selection const
        for i in self.routes.keys():
            self.prob.addConstr((self.x[i].sum() <= 1),name="route_selection")
        # Add demand const
        for item in self.orders:
            temp_LinExpr = gp.LinExpr()
            for i,v in self.routes.items():
                q=dict([[j, r['q'][item['id']-1]] for j,r in enumerate(v)])
                temp_LinExpr.add(self.x[i].prod(q))
            self.prob.addConstr((temp_LinExpr == item['Q']*(1-self.y[item['id']])),name="demand_const")

        # Set objective function
        costs = [-order['C'] for order in self.orders]
        revenue = [order['R'] for order in self.orders]
        self.objective.addTerms(costs,list(self.y.values()))
        self.objective.addConstant(sum(revenue))
        for i,v in self.routes.items():
            cvr = dict([[j, r['cvr']] for j,r in enumerate(v)])
            self.objective.add(self.x[i].prod(cvr),-1)
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
        if self.solver == "gurobi":
            # 设置求解器参数
            self.prob.setParam("Method", 2)  # 设置为内点法（Barrier）
            self.prob.setParam("Crossover", 0)  # 设置为不使用割平面

            # Only specify time limit if given
            if time_limit is not None:
                self.prob.setParam("TimeLimit", time_limit)  # 设置时间限制

            self.prob.optimize()
        else:
            pass

    # def update(self, new_route):
    #     """Add new column."""
    #     self._add_route_selection_variable(new_route)

    def get_duals(self):
        """Gets the dual values of each constraint of the master problem.

        Returns:
            dict: Duals with constraint names as keys and dual variables as values
        """
        duals = {"route_selection":[],"demand_const":[],}
        for constr in self.prob.getConstrs():
            if constr.ConstrName == "route_selection":
                duals["route_selection"].append(constr.Pi)
            elif constr.ConstrName == "demand_const":
                duals["demand_const"].append(constr.Pi)
        return duals


if __name__ == "__main__":

    orders=[{'id':1,'Q':10,'R':4,'C':5},{'id':2,'Q':4,'R':8,'C':4}]
    vehicle={'0':{'K':7},
             }
    routes={'0':[{'route':[],
                 'q':[4,3],
                 'cvr':2,},
                {'route':[],
                 'q':[5,2],
                 'cvr':2,},],
            '1':[{'route':[],
                 'q':[3,4],
                 'cvr':2,},
                {'route':[],
                 'q':[6,1],
                 'cvr':2,},],
            }
    m = _MasterSolve(vehicle,routes,orders,True)
    m.solve(True,10)
    for i,item in m.x.items():
        for j,c in item.items():
            if c.X == 1:
                print(i,j)
    print(m.y)
    print(m.x)
    # print(m.prob.getConstrs())
