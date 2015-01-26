#ifndef SOLVER_H
#define SOLVER_H

#include <cstring>

#include <ilcplex/ilocplex.h>

#include <data.h>
#include <params.h>

class solver {
    using var_vector = IloNumVarArray;
    using var_matrix_2d = IloArray<var_vector>;
    using var_matrix_3d = IloArray<var_matrix_2d>;
    using var_matrix_4d = IloArray<var_matrix_3d>;

    using cst_vector = IloRangeArray;
    using cst_matrix_2d = IloArray<cst_vector>;
    using cst_matrix_3d = IloArray<cst_matrix_2d>;
    
    data& d;
    params& p;

public:
    solver(data& d, params& p) : d{d}, p{p} {};
    
    void solve() const;
};

#endif