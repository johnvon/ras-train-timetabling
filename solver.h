#ifndef SOLVER_H
#define SOLVER_H

#include <cstring>

#include <ilcplex/ilocplex.h>

#include <data.h>

using var_vector = IloNumVarArray;
using var_matrix_2d = IloArray<var_vector>;
using var_matrix_3d = IloArray<var_matrix_2d>;
using var_matrix_4d = IloArray<var_matrix_3d>;
using var_matrix_5d = IloArray<var_matrix_4d>;

using cst_vector = IloRangeArray;
using cst_matrix_2d = IloArray<cst_vector>;
using cst_matrix_3d = IloArray<cst_matrix_2d>;

class solver {
    data& d;
    
    auto real_node(auto s, auto t) const;
public:
    solver(data& d) : d{d} {};
    
    void solve(bool use_max_travel_time, bool use_alt_min_travel_time) const;
};

#endif