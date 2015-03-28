#ifndef SOLVER_H
#define SOLVER_H

#include <data/array.h>
#include <data/data.h>

/*! This class models a MIP solver for the dispatching problem */
struct solver {
    /*! This struct packs data related to timing the operations performed by the solver */
    struct times {
        /*! Time used creating the CPLEX variable objects */
        double variable_creation;
        
        /*! Time used creating CPLEX constraints and setting the appropriate coefficients */
        double constraints_creation;
        
        /*! Time uesd creating the CPLEX obj function and setting the appropriate coefficients */
        double objf_creation;
        
        /*! Time spent by CPLEX solving the root node */
        double cplex_at_root;
        
        /*! Total time spent by CPLEX solving the model */
        double cplex_total;
        
        /*! Empty constructor */
        times() : variable_creation{0}, constraints_creation{0}, objf_creation{0}, cplex_at_root{0}, cplex_total{0} {}
    };
    
    /*! Reference to the data object */
    data& d;
    
    /*! Timing data */
    times t;

    /*! Basic constructor */
    solver(data& d) : d{d}, t{times()} {};
    
    /*! Solve the model! */
    auto solve() -> void;
    
private:
    
    auto create_model(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_travel_time) -> void;
    auto create_variables(IloEnv& env, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void;
    auto create_objective_function(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void;

    auto create_constraints_exit_sigma(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_enter_tau(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_flow(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_max_one_train(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_set_excess_travel_time(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void;
    auto create_constraints_min_travel_time(IloEnv& env, IloModel& model, var_matrix_2d& var_excess_travel_time) -> void;
    auto create_constraints_headway_1(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_headway_2(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_headway_3(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_siding(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_heavy(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void;
    auto create_constraints_cant_stop(IloEnv& env, IloModel& model, var_matrix_2d& var_excess_travel_time) -> void;
    
    auto print_results(double obj_value) -> void;
    auto print_summary(IloEnv& env, IloCplex& cplex, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void;
};

#endif