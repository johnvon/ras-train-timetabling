#include <data.h>
#include <solver.h>

int main(int argc, char* argv[]) {
    data        d(argv[1]);
    params      p(argv[2]);
    solver      s(d, p);
    
    s.solve();
    
    return 0;
}