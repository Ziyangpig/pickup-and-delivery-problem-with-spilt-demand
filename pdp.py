import logging
import sys
from time import time
from MasterProblem import _MasterSolve
from subproblem import _SubProblemLP
from data_process import data_process
from subproblem_labelsetting import SP1
from data_generate import create_pd_DiGragh
from data_generate import data_generate
import json
from copy import deepcopy
from math import ceil

logger = logging.getLogger(__name__)

logging.basicConfig(level=logging.DEBUG)

# duals,G,N,orders, vehicle, solver
class VehicleRoutingProblem:
    def __init__(
            self,
            G,
            N,
            vehicle : dict, #[{'capcity': 'weight' 'o' 'd' 'st' }
            orders : list, #[{P,D,TW,C,R,Q}]

    ):
        self.G = G
        self.N = N
        self.vehicle = vehicle
        self.orders = orders

        self._initial_routes = {}
        #
        self.masterproblem: _MasterSolve = None
        self.routes=  {}
        self.comp_time = None

        # Input solving parameters
        self._solver: str = None
        self._time_limit: int = None
        self._max_iter: int = None
        self._run_exact = None

        # parameters for column generation stopping criteria
        self._start_time = None
        self._more_routes = None
        self._all_more_routes  = None
        self._iteration = 0  # current iteration
        self._no_improvement = 0  # iterations after with no change in obj func
        self._lower_bound = []

        # Parameters for final solution
        self._best_value = None
        self._best_routes = []

        self.iter_time = []

        # parameters for Subproblem efficiency gurobi
        self.sub_initial_gap = 2.5
        self.k = 0.8
        # label
        self.num_arcs_to_consider = round(len(self.orders)/2,0)
        self.paths_per_itter = 4
        self.sub_heuristic = False


    def solve(
            self,
            initial_routes=None,
            time_limit=None,
            solver="gurobi",  # gurobi , label
            max_iter=None,
            sub_heuristic = False

    ):
        """Iteratively generates columns with negative reduced cost and solves as MIP.

        Returns:
            float: Optimal solution of MIP based on generated columns
        """
        # set solving attributes
        self._more_routes = True
        self._all_more_routes = True
        self._solver = solver
        self._time_limit = time_limit
        self._max_iter = max_iter
        self._start_time = time()
        self.sub_heuristic = sub_heuristic

        if initial_routes:
            self._initial_routes = initial_routes

        # If only one type of vehicle, some formatting is done

        # Pre-processing
        # self._pre_solve()

        # Initialization
        self._initialize()
        # Column generation procedure
        self._solve()

    def _initialize(self):
        """Initialization with feasible solution."""
        # if self._initial_routes:
        #     # Initial solution is given as input
        #     check_initial_routes(initial_routes=self._initial_routes, G=self.G)
        # else:
        #     # Initial solution is computed with Clarke & Wright (or round trips)
        #     self._get_initial_solution()

        # Init master problem
        self.masterproblem = _MasterSolve(
            self.vehicle,
            self._initial_routes,
            self.orders,
            True,
            self._solver,
        )
        self.routes = self._initial_routes

    def _solve(self):
        self._column_generation()
        # Solve as MIP
        self.masterproblem = _MasterSolve(
            self.vehicle,
            self.routes,
            self.orders,
            False,
            self._solver,
        )
        self.masterproblem.solve(relax=False, time_limit=self._get_time_remaining(mip=True))
        #(self._best_value,self._best_routes_as_graphs,) = self.masterproblem.get_total_cost_and_routes(relax=False)
        print(self.masterproblem.get_solution())

        # self._post_process(solver)

    def _column_generation(self):
        while self._all_more_routes:
            # Generate good columns
            a = time()
            self._find_columns() # 执行一次RMP和所有的子问题
            b = time()
            self.iter_time.append(b-a)
            print('一次列生成迭代时间:',b-a)
            if self._solver == 'gurobi':
                self.sub_initial_gap *= self.k
            elif self._solver == 'label':
                self.paths_per_itter += ceil(self._iteration/5)
            # Stop if time limit is passed
            if (
                isinstance(self._get_time_remaining(), float)
                and self._get_time_remaining() == 0.0
            ):
                logger.info("time up !")
                break
            # Stop if no improvement limit is passed or max iter exceeded
            if self._no_improvement > 100 or (
                self._max_iter and self._iteration >= self._max_iter
            ):
                break

    def _find_columns(self):
        # "Solves masterproblem and pricing problem."
        # Solve restricted relaxed master problem
        self.masterproblem.prob.setParam('OutputFlag', 0)
        print("RMP求解")
        duals, relaxed_cost = self.masterproblem.solve(relax=True, time_limit=self._get_time_remaining())
        print("iteration %s,obj %s" % (self._iteration, relaxed_cost))
        if self._solver == 'gurobi':
            print("子问题gap设置： %s" % self.sub_initial_gap)
        elif self._solver == 'label':
            pass

        # TODO: parallel
        # One subproblem per vehicle type
        iters = 0
        while True: # 不断增加考虑边数，以及启发转精确
            route_statas = []
            for v in range(len(self.vehicle)):
                print("子问题 船id： %s" % v)
                dual={}
                dual["route_selection"]=duals["route_selection"][v]
                dual["demand_const"]=duals["demand_const"]
                a = time()
                if self._solver == 'gurobi':
                    print(dual)
                    subproblem = self._def_subproblem(dual, v, greedy=False)
                    temp_routes, self._more_routes = subproblem.solve()
                elif self._solver == 'label':
                    subproblem = self._def_subproblem(dual, v, greedy=True)
                    # 把self.routes[str(v)]里路线相同配送模式不同的路线拆开，适应子问题输入
                    paths = []
                    keys = ['q','load','split']
                    for r in self.routes[str(v)]:
                        for q,l,s in zip(*list(map(lambda x:r[x],keys))):
                            r_copy = deepcopy(r)
                            r_copy['q'] = q
                            r_copy['load'] = l
                            r_copy['split'] = s
                            paths.append(r_copy)
                    temp_routes, self._more_routes = subproblem.calculate_H1(paths)

                if self._more_routes:
                    self._add_routes(temp_routes, v)
                route_statas.append(self._more_routes)

                b = time()
                print('子问题求解时间：',b-a)

            iters =+ 1
            # Keep track of convergence rate and update stopping criteria parameters
            self._all_more_routes = True
            if all(stata is False for stata in route_statas):
                self._all_more_routes = False

            if not self._all_more_routes:
                if self.num_arcs_to_consider < len(self.orders):
                    print("未找到路线，增加扩展边数，正在重新调用label算法", iters)
                    print("当前考虑边数",self.num_arcs_to_consider)
                    self.num_arcs_to_consider += round(self.num_arcs_to_consider / 2, 0)
                    self.num_arcs_to_consider = min(self.num_arcs_to_consider, len(self.orders))
                    print('当前拓展考虑边数：', self.num_arcs_to_consider)
                elif self.sub_heuristic:
                    print('扩展边数已达最大，开始使用精确标签算法计算')
                    self.sub_heuristic = False
                else:
                    print("没有满足要求的路线，列生成迭代完成")
                    break
            else:
                print(f"当前路线搜寻方法是否为启发式：{self.sub_heuristic},当前考虑边数{self.num_arcs_to_consider}",)
                print("找到新路线，一次列生成迭代完成")
                break



        self.masterproblem = _MasterSolve(
            self.vehicle,
            self.routes,
            self.orders,
            True,
            self._solver,
        )

        self._iteration += 1
        if self._iteration > 1 and relaxed_cost == self._lower_bound[-1]:
            self._no_improvement += 1
        else:
            self._no_improvement = 0
        self._lower_bound.append(relaxed_cost)

    def _add_routes(self,temp_routes,v):
        print('车辆标号：', v)
        print('当前迭代返回路线数量参数为：',self.paths_per_itter)
        print('子问题实际返回的路线数量：',len(temp_routes))
        for r in temp_routes:
            # 判断新路线及配送是否已存在并找到对应位置
            existing_route_index = -1
            existing_pattern_index = -1
            for i, item in enumerate(self.routes[str(v)]):
                if item['route'] == r['route']:
                    existing_route_index = i
                    if r['q'] in item['q']:
                        existing_pattern_index = item['q'].index(r['q'])
                        break
            # 如果存在，则保留cvr对应的值更小的那个；如果不存在则直接append
            if existing_route_index != -1:
                if existing_pattern_index != -1:
                    print('重复路线生成：')
                    print(r)
                    print(self.routes[str(v)][existing_route_index])
                    if r['cvr'] < self.routes[str(v)][existing_route_index]['cvr']:
                        self.routes[str(v)][existing_route_index]['cvr'] = r['cvr']
                        print('更新cvr', r)
                        if self._solver=='label':
                            self.routes[str(v)][existing_route_index]['time'] = r['time']
                            self.routes[str(v)][existing_route_index]['time_cost'] = r['time_cost']

                else:
                    self.routes[str(v)][existing_route_index]['q'] += [r['q']]
                    print('添加pattern', r['q'])
                    if self._solver == 'label':
                        self.routes[str(v)][existing_route_index]['load']=self.routes[str(v)][existing_route_index]['load']+[r['load']]
                        self.routes[str(v)][existing_route_index]['split']+=[r['split']]

            else:
                r['q'] = [r['q'],]
                if self._solver == 'label':
                    r['load'] = [r['load'], ]
                    r['split'] = [r['split'], ]
                self.routes[str(v)] =self.routes[str(v)]+[r]
                print('添加新路线：',r)




    def _def_subproblem(
        self,
        duals,
        v,
        greedy=False,
    ):
        """Instanciates the subproblem."""

        if greedy:
            # 数据转化 根据 self.G,self.N, self.orders, self.vehicle 转到datadict
            data_dict={}
            data_dict["distances"] = self.G[str(v)]['d']
            # dij = calculate_cost_matrix(data_dict, duals)
            data_dict["num_of_requests"] = len(self.orders)
            load=[0]+[x['Q'] for x in self.orders]
            for i in range(len(self.orders) + 1, 2 * len(self.orders) + 1):
                load.append(-load[i - len(self.orders)])
            load.append(0)
            data_dict["load"] = load
            data_dict["dual_alpha"] = duals["demand_const"]
            data_dict["route_selection"] = duals["route_selection"]
            data_dict["time_windows"] = [[l,u] for l,u in zip([self.vehicle[str(v)]['avail_t']]+self.N['ltw'][1:],self.N['utw'])]
            data_dict["vehicle_speed"] = 1
            data_dict["vehicle_capacity"] = self.vehicle[str(v)]['K']
            g = create_pd_DiGragh(data_dict["num_of_requests"])
            subproblem = SP1(g, data_dict,self.num_arcs_to_consider,self.paths_per_itter,self.sub_heuristic)

        else:
            # As LP duals,G,N,orders, vehicle, solver
            # 判断G的第一层key是t/d还是车辆标号，前者说明单车库，后者说明每辆车出发地不一样
            if 't' in self.G.keys():
                subproblem = _SubProblemLP(duals,self.G,self.N, self.orders, self.vehicle[str(v)], self._solver,self.sub_initial_gap)
            else:
                subproblem = _SubProblemLP(duals, self.G[str(v)], self.N, self.orders, self.vehicle[str(v)], self._solver,self.sub_initial_gap)
        return subproblem

    def _get_time_remaining(self, mip: bool = False):
        """
        Modified to avoid over time in subproblems.

        """
        if self._time_limit:
            remaining_time = self._time_limit - (time() - self._start_time)
            if mip:
                return max(500, remaining_time)
            if remaining_time > 0:
                return remaining_time
            return 0.0
        return None

if __name__ == "__main__":
    seed = 2
    n, cargo_size = 10,[0.3,0.8]
    K,number_vehicle = [90, 120, 150],10
    W, T = 60,600
    L = 80 # 运输区域边长
    G, N, vehicle, orders= data_generate(n,cargo_size,W,T,seed,L,K,number_vehicle)
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
    for i in vehicle.keys():
        init_routes[i]=dummy_route
    a = time()
    # G, N, vehicle, orders, init_routes = data_process(14,is_mass_cargo=True) #

    # N['utw']=[N['utw'][0] for i in range(len(N['utw']))]
    VRP=VehicleRoutingProblem(G,N,vehicle,orders)
    VRP.solve(init_routes,None,'label',1000,True)
    b=time()
    # for i,r in VRP.routes.items():
    #     print(i,r)
    print(vehicle, '\n', orders, '\n', N)
    print('求解时间：', b - a)
    print(VRP.iter_time)
    # print(VRP.masterproblem.prob.MIPGap)
    print('迭代次数：',VRP._iteration,'结果未改善次数：',VRP._no_improvement)

