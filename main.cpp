#include <data.h>
#include <solver.h>

int main(int argc, char* argv[]) {
    data        d(argv[1]);
    solver      s(d);
    
    auto use_max_travel_time = (strcmp(argv[2], "max_travel_time=true") == 0);
    auto use_alt_min_travel_time = (strcmp(argv[3], "alt_min_travel_time=true") == 0);
    
    s.solve(use_max_travel_time, use_alt_min_travel_time);
    
    return 0;
}