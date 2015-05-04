#include <data/data.h>
#include <params/params.h>
#include <solver/solver.h>
#include <solver/sequential_solver.h>

#include <iostream>

int main(int argc, char* argv[]) {
    auto p = params(argv[2]);
    auto d = data(argv[1], p);
    // auto s = solver(d);
    auto s = sequential_solver(d);

    // s.solve();
    s.solve_sequentially();

    return 0;
}