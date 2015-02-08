#ifndef SOLVER_H
#define SOLVER_H

#include <cstring>

#include <ilcplex/ilocplex.h>

#include <graph/data.h>
#include <params/params.h>

class solver {
    using var_vector = IloNumVarArray;
    using var_matrix_2d = IloArray<var_vector>;
    using var_matrix_3d = IloArray<var_matrix_2d>;
    using var_matrix_4d = IloArray<var_matrix_3d>;

    using cst_vector = IloRangeArray;
    using cst_matrix_2d = IloArray<cst_vector>;
    using cst_matrix_3d = IloArray<cst_matrix_2d>;
    
    struct times {
        double variable_creation;
        double constraints_creation;
        double objf_creation;
        double cplex_at_root;
        double cplex_total;
        
        times() : variable_creation{0}, constraints_creation{0}, objf_creation{0}, cplex_at_root{0}, cplex_total{0} {}
    };
    
    data&   d;
    params& p;
    times   t;
    
    void create_model(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_vector& var_d, var_matrix_2d& e, var_matrix_2d& var_travel_time);
    void print_results(double obj_value);

public:
    solver(data& d, params& p) : d{d}, p{p}, t{times()} {};
    
    void solve();
};

#endif