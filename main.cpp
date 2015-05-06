#include <data/data.h>
#include <params/params.h>

#ifdef USE_CPLEX
    #include <solver/solver.h>
    #include <solver/sequential_solver.h>
#endif

#include <iostream>

int main(int argc, char* argv[]) {
    auto p = params(argv[2]);
    auto d = data(argv[1], p);

    #ifdef USE_CPLEX
        auto s = sequential_solver(d);
        s.solve_sequentially();
    #endif

    return 0;
}