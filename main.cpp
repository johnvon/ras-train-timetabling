#include <data.h>
#include <solver.h>

int main(int argc, char* argv[]) {
    data        d(argv[1]);
    solver      s(d);
    
    s.solve();
    
    return 0;
}