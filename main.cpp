#include <data/data.h>
#include <params/params.h>

int main(int argc, char* argv[]) {
    auto p = params(argv[2]);
    auto d = data(argv[1], p);
    
    return 0;
}