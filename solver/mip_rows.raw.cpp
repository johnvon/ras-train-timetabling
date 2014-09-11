int row_num {0};

#include <solver/rows/exit_sigma.raw.cpp>
std::cout << "\texit_sigma: created (" << row_num << ")" << std::endl;
#include <solver/rows/enter_tau.raw.cpp>
std::cout << "\tenter_tau: created (" << row_num << ")" << std::endl;
#include <solver/rows/one_train.raw.cpp>
std::cout << "\tone_train: created (" << row_num << ")" << std::endl;
#include <solver/rows/flow.raw.cpp>
std::cout << "\tflow: created (" << row_num << ")" << std::endl;
#include <solver/rows/ensure_sa_visit.raw.cpp>
std::cout << "\tensure_sa_visit: created (" << row_num << ")" << std::endl;
#include <solver/rows/wt_1.raw.cpp>
std::cout << "\twt_1: created (" << row_num << ")" << std::endl;
#include <solver/rows/wt_2.raw.cpp>
std::cout << "\twt_2: created (" << row_num << ")" << std::endl;
#include <solver/rows/sa_delay.raw.cpp>
std::cout << "\tsa_delay: created (" << row_num << ")" << std::endl;
#include <solver/rows/min_time.raw.cpp>
std::cout << "\tmin_time: created (" << row_num << ")" << std::endl;

// If max running time of all switches and xovers is 1
// this constraint can be enforced in the graph!
// If this condition stopped holding, we woud need
// to reintroduce this constraint.
// #include <solver/rows/exact_time.raw.cpp>
// std::cout << "\texact_time: created (" << row_num << ")" << std::endl;

#include <solver/rows/headway1.raw.cpp>
std::cout << "\theadway1: created (" << row_num << ")" << std::endl;
#include <solver/rows/headway2.raw.cpp>
std::cout << "\theadway2: created (" << row_num << ")" << std::endl;
#include <solver/rows/headway3.raw.cpp>
std::cout << "\theadway3: created (" << row_num << ")" << std::endl;
#include <solver/rows/headway4.raw.cpp>
std::cout << "\theadway4: created (" << row_num << ")" << std::endl;
#include <solver/rows/can_take_siding.raw.cpp>
std::cout << "\tcan_take_siding: created (" << row_num << ")" << std::endl;
#include <solver/rows/heavy_siding.raw.cpp>
std::cout << "\theavy_siding: created (" << row_num << ")" << std::endl;

// This ensures the price is always positive!
#include <solver/rows/positive_obj.raw.cpp>
std::cout << "\tpositive_obj: created (" << row_num << ")" << std::endl;