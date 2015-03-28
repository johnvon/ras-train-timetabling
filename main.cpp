#include <data/data.h>
#include <params/params.h>
#include <solver/solver.h>

int main(int argc, char* argv[]) {
    auto p = params(argv[2]);
    auto d = data(argv[1], p);
    auto s = solver(d);

    s.solve();

    return 0;
}