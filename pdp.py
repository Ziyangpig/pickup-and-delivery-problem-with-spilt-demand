import logging
import sys
from time import time
from MasterProblem import _MasterSolve
from subproblem import _SubProblemLP
from data_process import data_process
from data_generate import data_generate



logger = logging.getLogger(__name__)

logging.basicConfig(level=logging.INFO, stream=sys.stdout)

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

    def solve(
            self,
            initial_routes=None,
            time_limit=None,
            solver="gurobi",
            max_iter=None,

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
        print(self.masterproblem.y)
        print(self.routes)
        # _,v_y=self.masterproblem.y.items()
        # print(v_y)
        #
        for i, item in self.masterproblem.x.items():
            for j, c in item.items():
                if c.X == 1:
                    print(i, j)
                    print(self.routes[str(i)][j])

        # self._post_process(solver)

    def _column_generation(self):
        while self._all_more_routes:
            # Generate good columns
            self._find_columns()
            # Stop if time limit is passed
            if (
                isinstance(self._get_time_remaining(), float)
                and self._get_time_remaining() == 0.0
            ):
                logger.info("time up !")
                break
            # Stop if no improvement limit is passed or max iter exceeded
            if self._no_improvement > 1000 or (
                self._max_iter and self._iteration >= self._max_iter
            ):
                break

    def _find_columns(self):
        # "Solves masterproblem and pricing problem."
        # Solve restricted relaxed master problem
        duals, relaxed_cost = self.masterproblem.solve(relax=True, time_limit=self._get_time_remaining())
        logger.info("iteration %s, %.6s" % (self._iteration, relaxed_cost))

        # TODO: parallel
        # One subproblem per vehicle type
        route_statas = []
        for v in range(len(self.vehicle)):
            # Solve pricing problem with randomised greedy algorithm
            dual={}
            dual["route_selection"]=duals["route_selection"][v]
            dual["demand_const"]=duals["demand_const"]
            print(dual)
            subproblem = self._def_subproblem(dual, v, greedy=False)
            temp_routes, self._more_routes = subproblem.solve()

            route_statas.append(self._more_routes)
            # Add initial_routes
            if self._more_routes:
                for r in temp_routes:
                    if r not in self.routes[str(v)]:
                        self.routes[str(v)].append(r)
            self.masterproblem = _MasterSolve(
                self.vehicle,
                self.routes,
                self.orders,
                True,
                self._solver,
            )

        # Keep track of convergence rate and update stopping criteria parameters
        if all(stata is False for stata in route_statas):
            self._all_more_routes = False

        self._iteration += 1
        if self._iteration > 1 and relaxed_cost == self._lower_bound[-1]:
            self._no_improvement += 1
        else:
            self._no_improvement = 0
        self._lower_bound.append(relaxed_cost)

    def _def_subproblem(
        self,
        duals,
        v,
        greedy=False,
    ):
        """Instanciates the subproblem."""

        if greedy:
            pass
        else:
            # As LP duals,G,N,orders, vehicle, solver
            if 't' in self.G.keys():
                subproblem = _SubProblemLP(duals,self.G,self.N, self.orders, self.vehicle[str(v)], self._solver)
            else:
                subproblem = _SubProblemLP(duals, self.G[str(v)], self.N, self.orders, self.vehicle[str(v)], self._solver)
        return subproblem

    def _get_time_remaining(self, mip: bool = False):
        """
        Modified to avoid over time in subproblems.

        Returns:
            - None if no time limit set.
            - time remaining (in seconds) if time remaining > 0 and mip = False
            - 5 if time remaining < 5 and mip = True
            - 0 if time remaining < 0
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
    # G, N, vehicle, orders, init_routes = data_generate()
    G, N, vehicle, orders, init_routes = data_process()
    N['utw']=[N['utw'][0] for i in range(len(N['utw']))]
    VRP=VehicleRoutingProblem(G,N,vehicle,orders)
    VRP.solve(init_routes)
    # TODO 初始路线得到
