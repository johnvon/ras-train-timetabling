#include <data/data.h>
#include <params/params.h>

#if USE_CPLEX
    #include <solver/solver.h>
    #include <solver/sequential_solver.h>
#else
	#include <solver/solver_heuristic.h>
	#include <solver/sequential_solver_heuristic.h>
#endif

#include <iostream>
#include <fstream>

int main(int argc, char* argv[]) {
    auto p = params(argv[2]);
    auto d = data(argv[1], p);

    #if USE_CPLEX
        auto s = sequential_solver(d);
        s.solve_sequentially();
    #else
        sequential_solver_heuristic s = sequential_solver_heuristic(d);
        s.solve_sequentially();
	#endif

    return 0;
}
