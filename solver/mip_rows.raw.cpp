int row_num {0};

#include <solver/rows/exit_sigma.raw.cpp>
std::cout << "\texit_sigma: created (" << row_num << ")" << std::endl;
#include <solver/rows/enter_tau.raw.cpp>
std::cout << "\tenter_tau: created (" << row_num << ")" << std::endl;
#include <solver/rows/set_y.raw.cpp>
std::cout << "\tset_y: created (" << row_num << ")" << std::endl;
#include <solver/rows/flow.raw.cpp>
std::cout << "\tflow: created (" << row_num << ")" << std::endl;
#include <solver/rows/z_sa.raw.cpp>
std::cout << "\tz_sa: created (" << row_num << ")" << std::endl;
#include <solver/rows/set_z.raw.cpp>
std::cout << "\tset_z: created (" << row_num << ")" << std::endl;
#include <solver/rows/set_theta.raw.cpp>
std::cout << "\tset_theta: created (" << row_num << ")" << std::endl;
#include <solver/rows/wt_1.raw.cpp>
std::cout << "\twt_1: created (" << row_num << ")" << std::endl;
#include <solver/rows/wt_2.raw.cpp>
std::cout << "\twt_2: created (" << row_num << ")" << std::endl;
#include <solver/rows/sa_delay.raw.cpp>
std::cout << "\tsa_delay: created (" << row_num << ")" << std::endl;
#include <solver/rows/min_time.raw.cpp>
std::cout << "\tmin_time: created (" << row_num << ")" << std::endl;
#include <solver/rows/headway.raw.cpp>
std::cout << "\theadway: created (" << row_num << ")" << std::endl;
#include <solver/rows/can_take_siding.raw.cpp>
std::cout << "\tcan_take_siding: created (" << row_num << ")" << std::endl;
#include <solver/rows/heavy_siding.raw.cpp>
std::cout << "\theavy_siding: created (" << row_num << ")" << std::endl;