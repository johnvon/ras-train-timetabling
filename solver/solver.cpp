#include <solver/solver.h>

#if USE_GRAPHER
    #include <grapher/grapher.h>
#endif

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <limits>

auto solver::solve() -> boost::optional<bv<path>> {
    using namespace chrono;
    
    auto t_start = high_resolution_clock::time_point();
    auto t_end = high_resolution_clock::time_point();
    auto time_span = duration<double>();
    auto lb_at_root = std::numeric_limits<double>::max();
    auto lb_at_end = std::numeric_limits<double>::max();
    auto ub_at_root = std::numeric_limits<double>::max();
    auto ub_at_end = std::numeric_limits<double>::max();
    
    IloEnv env;
    IloModel model(env);
    
    var_matrix_4d var_x(env, d.nt);
    var_matrix_2d var_excess_travel_time(env, d.nt);
    
    create_model(env, model, var_x, var_excess_travel_time);

    IloCplex cplex(model);

    cplex.exportModel("model.lp");
    cplex.setParam(IloCplex::TiLim, d.p.cplex.time_limit);
    cplex.setParam(IloCplex::Threads, d.p.cplex.threads);
    cplex.setParam(IloCplex::NodeLim, 0);
    cplex.setOut(env.getNullStream());

    t_start = high_resolution_clock::now();
    
    auto success_at_root_node = cplex.solve();
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.cplex_at_root = time_span.count();

    // Check that CPLEX gives a negative status at root, but not just because it couldn't find a feasible solution.
    // In fact, we want to stop only if the problem is proven infeasible.
    // Not having found a feasible solution is ok, we can still find it later.
    if(!success_at_root_node && cplex.getCplexStatus() != IloCplex::NodeLimInfeas) {
        std::cerr << "bc_solver.cpp::solve() \t CPLEX problem encountered at root node" << std::endl;
        std::cerr << "bc_solver.cpp::solve() \t CPLEX status: " << cplex.getStatus() << std::endl;
        std::cerr << "bc_solver.cpp::solve() \t CPLEX ext status: " << cplex.getCplexStatus() << std::endl;
        
        cplex.exportModel("model_err.lp");
        return boost::none;
    }
    
    lb_at_root = cplex.getBestObjValue();
    
    if(cplex.isPrimalFeasible()) {
        ub_at_root = cplex.getObjValue();
    } else {
        ub_at_root = std::numeric_limits<double>::max();
    }
    
    cplex.setParam(IloCplex::NodeLim, 2100000000);
    
    t_start = high_resolution_clock::now();
    
    auto success_at_later_node = cplex.solve();
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.cplex_total = t.cplex_at_root + time_span.count();
    
    if(!success_at_later_node) {
        std::cerr << "Cplex infeasible at later node: " << cplex.getStatus() << " - " << cplex.getCplexStatus() << std::endl;
        return boost::none;
    } else {
        lb_at_end = cplex.getBestObjValue();
        
        if(cplex.isPrimalFeasible()) {
            ub_at_end = cplex.getObjValue();
        } else {
            ub_at_end = std::numeric_limits<double>::max();
        }
        
        std::cerr << "Cplex UB value: " << ub_at_end << std::endl;
    }
    
    auto paths = make_paths(env, cplex, var_x, var_excess_travel_time);
    
    print_results(ub_at_root, ub_at_end, lb_at_root, lb_at_end);
    print_summary(paths);
    print_graph(paths);
    
    env.end();
    
    return paths;
}

auto solver::make_paths(IloEnv& env, IloCplex& cplex, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> bv<path> {
    auto x = uint_matrix_4d(d.nt, uint_matrix_3d(d.ns + 2, uint_matrix_2d(d.ni + 2, uint_vector(d.ns + 2, 0u))));
    auto paths = bv<path>();

    for(auto i = 0u; i < d.nt; i++) {
        auto cost = 0.0;
        
        for(auto s1 = 0u; s1 <= d.ns + 1; s1++) {
            cost += d.pri.delay[d.trn.type[i]] * cplex.getValue(var_excess_travel_time[i][s1]);
            
            for(auto t = 0u; t <= d.ni + 1; t++) {
                for(auto s2 = 0u; s2 <= d.ns + 1; s2++) {
                    if(d.gr.adj[i][s1][t][s2]) {
                        if(cplex.getValue(var_x[i][s1][t][s2]) > 0.0) {
                            x[i][s1][t][s2] = 1u;
                            cost += d.gr.costs[i][s1][t][s2] * cplex.getValue(var_x[i][s1][t][s2]);
                        }
                    } else {
                        x[i][s1][t][s2] = 0u;
                    }
                }
            }
        }
        
        paths.push_back(path(d, i, x.at(i), cost));
    }
    
    return paths;
}

auto solver::print_summary(const bv<path>& paths) const -> void {
    for(const auto& p : paths) {
        p.print_summary(std::cerr);
    }
}

auto solver::print_graph(const bv<path>& paths) const -> void {
    #if USE_GRAPHER
        auto ger = grapher(d, paths);
        ger.write_graph();
    #endif
}

auto solver::print_results(double ub_at_root, double ub_at_end, double lb_at_root, double lb_at_end) const -> void {
    std::ofstream results_file;
    results_file.open(d.p.results_file, std::ios::out | std::ios::app);
    
    results_file << d.ins.file_name << "\t";
    results_file << t.variable_creation << "\t";
    results_file << t.constraints_creation << "\t";
    results_file << t.objf_creation << "\t";
    results_file << t.cplex_at_root << "\t";
    results_file << t.cplex_total << "\t";
    results_file << ub_at_root << "\t";
    results_file << ub_at_end << "\t";
    results_file << lb_at_root << "\t";
    results_file << lb_at_end << std::endl;
    
    results_file.close();
}

auto solver::create_variables(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void {
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {        
        var_x[i] = var_matrix_3d(env, d.ns + 2);
        var_excess_travel_time[i] = var_vector(env, d.ns + 2);
        
        for(auto s1 = 0u; s1 <= d.ns + 1; s1++) {
            var_x[i][s1] = var_matrix_2d(env, d.ni + 2);
            
            name.str(""); name << "var_excess_travel_time_" << i << "_" << s1;
            var_excess_travel_time[i][s1] = IloNumVar(env, 0, d.ni + 2, IloNumVar::Int, name.str().c_str());
            model.add(var_excess_travel_time[i][s1]);
            
            for(auto t = 0u; t <= d.ni; t++) {
                var_x[i][s1][t] = var_vector(env, d.ns + 2);
                
                for(auto s2 = 0u; s2 <= d.ns + 1; s2++) {
                    if(d.gr.adj[i][s1][t][s2]) {
                        name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t << "_" << s2;
                        var_x[i][s1][t][s2] = IloNumVar(env, 0, 1, IloNumVar::Bool, name.str().c_str());
                    }
                }
            }
        }
    }
}

auto solver::create_constraints_exit_sigma(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_vector cst_exit_sigma(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        IloExpr expr(env);
        
        for(auto s : d.trn.orig_segs[i]) {
            for(auto t = 0u; t <= d.ni - d.net.min_travel_time[i][s]; t++) {
                if(d.gr.adj[i][0][t][s]) {
                    expr += var_x[i][0][t][s];
                }
            }
        }
        
        if(d.gr.adj[i][0][0][d.ns + 1]) {
            // Dummy path!
            expr += var_x[i][0][0][d.ns + 1];
        }
        
        cst_exit_sigma[i] = IloRange(env, 1, expr, 1, name.str().c_str());
        expr.end();
    }
    
   model.add(cst_exit_sigma);
}

auto solver::create_constraints_enter_tau(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_vector cst_enter_tau(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        name.str(""); name << "cst_enter_tau_" << i;
        IloExpr expr(env);
        
        for(auto s : d.trn.dest_segs[i]) {
            for(auto t = d.net.min_time_to_arrive[i][s] + d.net.min_travel_time[i][s] - 1; t < d.ni; t++) {
                if(d.gr.adj[i][s][t][d.ns+1]) {
                    expr += var_x[i][s][t][d.ns+1];
                }
            }
        }
        
        for(auto s = 1u; s <= d.ns; s++) {
            if(d.gr.adj[i][s][d.ni][d.ns+1]) {
                expr += var_x[i][s][d.ni][d.ns+1];
            }
        }
        
        if(d.gr.adj[i][0][0][d.ns + 1]) {
            // Dummy path!
            expr += var_x[i][0][0][d.ns + 1];
        }
        
        cst_enter_tau[i] = IloRange(env, 1, expr, 1, name.str().c_str());
        expr.end();
    }
    
    model.add(cst_enter_tau);
}

auto solver::create_constraints_flow(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_3d cst_flow(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s = 1u; s <= d.ns; s++) {
            cst_flow[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = d.net.min_time_to_arrive[i][s]; t <= d.ni; t++) {
                if(d.gr.v[i][s][t]) {
                    name.str(""); name << "cst_flow_" << i << "_" << s << "_" << t;
                    IloExpr expr(env);
                    
                    for(auto ss : d.gr.inverse_delta[i][s]) {
                        if(d.gr.adj[i][ss][t-1][s]) {
                            expr += var_x[i][ss][t-1][s];
                        }
                    }
                    
                    for(auto ss : d.gr.delta[i][s]) {
                        if(d.gr.adj[i][s][t][ss]) {
                            expr -= var_x[i][s][t][ss];
                        }
                    }
                    
                    // Escape arcs
                    if( t == d.ni && // Time must be last time interval
                        d.gr.adj[i][s][d.ni][d.ns+1] && // They should be connected with tau
                        std::find(d.gr.delta[i][s].begin(), d.gr.delta[i][s].end(), d.ns+1) == d.gr.delta[i][s].end() // And tau must not be a "real" connection
                    ) {
                        expr -= var_x[i][s][d.ni][d.ns+1];
                    }
                    
                    cst_flow[i][s][t] = IloRange(env, 0, expr, 0, name.str().c_str());
                    expr.end();
                }
            }
            
            model.add(cst_flow[i][s]);
        }
    }
}

auto solver::create_constraints_max_one_train(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_2d cst_max_one_train(env, d.ns + 2);
    std::stringstream name;
    
    for(auto s = 1u; s <= d.ns; s++) {
        cst_max_one_train[s] = cst_vector(env, d.ni + 2);
        
        for(auto t = 1u; t <= d.ni; t++) {
            name.str(""); name << "cst_max_one_train_" << s << "_" << t;
            IloExpr expr(env);
            
            for(auto i = 0u; i < d.nt; i++) {
                for(auto ss : d.gr.inverse_delta[i][s]) {
                    if(d.gr.adj[i][ss][t-1][s]) {
                        expr += var_x[i][ss][t-1][s];
                    }
                }
            }
            
            cst_max_one_train[s][t] = IloRange(env, -IloInfinity, expr, 1, name.str().c_str());
            expr.end();
        }
        
        model.add(cst_max_one_train[s]);
    }
}

auto solver::create_constraints_set_excess_travel_time(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void {
    cst_matrix_2d cst_set_excess_travel_time(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        cst_set_excess_travel_time[i] = cst_vector(env, d.ns + 2);
        
        for(auto s = 1u; s <= d.ns; s++) {
            name.str(""); name << "cst_set_excess_travel_time_" << i << "_" << s;
            IloExpr expr(env);
            
            expr -= var_excess_travel_time[i][s];
            
            for(auto t = 1u; t <= d.ni; t++) {
                if(d.gr.v[i][s][t]) {
                    for(auto ss : d.gr.bar_delta[i][s]) {
                        if(d.gr.adj[i][s][t][ss]) {
                            expr += static_cast<long>(t) * var_x[i][s][t][ss];
                        }
                    }
                    
                    for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                        if(d.gr.adj[i][ss][t-1][s]) {
                            expr -= static_cast<long>(t + d.net.min_travel_time[i][s] - 1) * var_x[i][ss][t-1][s];
                        }
                    }
                }
            }
            
            // Escape arc
            if( d.gr.adj[i][s][d.ni][d.ns+1] && // Must be connected
                std::find(d.gr.bar_delta[i][s].begin(), d.gr.bar_delta[i][s].end(), d.ns+1) == d.gr.bar_delta[i][s].end() // And tau must not be a "real" connection
            ) {
                expr += static_cast<long>(d.ni) * var_x[i][s][d.ni][d.ns+1];
            }
            
            cst_set_excess_travel_time[i][s] = IloRange(env, 0, expr, 0, name.str().c_str());
            expr.end();
        }
        
        model.add(cst_set_excess_travel_time[i]);
    }
}

auto solver::create_constraints_min_travel_time(IloEnv& env, IloModel& model, var_matrix_2d& var_excess_travel_time) -> void {
    // var_excess_travel_time >= 0 already implies min travel time constraints are respected
}

auto solver::create_constraints_headway_1(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_2d cst_headway_1(env, d.ns + 2);
    std::stringstream name;
    
    for(auto s = 1u; s <= d.ns; s++) {
        cst_headway_1[s] = cst_vector(env, d.ni + 2);
        
        for(auto t = 1u; t <= d.ni; t++) {
            name.str(""); name << "cst_headway1_" << s << "_" << t;
            IloExpr expr(env);
                        
            auto min_time = static_cast<unsigned int>(std::max(1, static_cast<int>(t - d.headway)));
            
            for(auto i = 0u; i < d.nt; i++) {
                for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                    for(auto tt = min_time; tt <= t; tt++) {
                        if(d.gr.adj[i][ss][tt-1][s]) {
                            expr += var_x[i][ss][tt-1][s];
                        }
                    }
                }
            }
                        
            cst_headway_1[s][t] = IloRange(env, -IloInfinity, expr, 1, name.str().c_str());
            expr.end();
        }
        
        model.add(cst_headway_1[s]);
    }
}

auto solver::create_constraints_headway_2(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_2d cst_headway_2(env, d.ns + 2);
    std::stringstream name;
        
    for(auto s = 1u; s <= d.ns; s++) {
        cst_headway_2[s] = cst_vector(env, d.ni + 2);
        
        for(auto t = 1u; t <= d.ni; t++) {
            name.str(""); name << "cst_headway2_" << s << "_" << t;
            IloExpr expr(env);
            
            auto min_time = static_cast<unsigned int>(std::max(0, static_cast<int>(t - d.headway)));
            
            for(auto i = 0u; i < d.nt; i++) {
                for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                    if(d.gr.adj[i][ss][t-1][s]) {
                        expr += var_x[i][ss][t-1][s];
                    }
                }
                                
                for(auto ss : d.gr.bar_delta[i][s]) {
                    for(auto tt = min_time; tt < t; tt++) {
                        if(d.gr.adj[i][s][tt][ss]) {
                            expr += var_x[i][s][tt][ss];
                        }
                    }
                }
            }
            
            cst_headway_2[s][t] = IloRange(env, -IloInfinity, expr, 1, name.str().c_str());
            expr.end();
        }
        
        model.add(cst_headway_2[s]);
    }
}

auto solver::create_constraints_headway_3(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_2d cst_headway_3(env, d.ns + 2);
    std::stringstream name;

    for(auto s = 1u; s <= d.ns; s++) {
        cst_headway_3[s] = cst_vector(env, d.ni + 2);

        for(auto t = 1u; t <= d.ni; t++) {
            name.str(""); name << "cst_headway3_" << s << "_" << t;
            IloExpr expr(env);
            
            auto max_time = std::min(d.ni + 1, t + d.headway);

            for(auto i = 0u; i < d.nt; i++) {
                for(auto ss : d.gr.bar_delta[i][s]) {
                    if(d.gr.adj[i][s][t][ss]) {
                        expr += var_x[i][s][t][ss];
                    }
                }

                // Escape arcs
                if( t == d.ni && // If it's the last time interval
                    d.gr.adj[i][s][d.ni][d.ns+1] && // And s is connected to tau
                    std::find(d.gr.bar_delta[i][s].begin(), d.gr.bar_delta[i][s].end(), d.ns+1) == d.gr.bar_delta[i][s].end() // And such a connection is not a "real" one
                ) {
                    expr += var_x[i][s][d.ni][d.ns+1];
                }

                for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                    for(auto tt = t + 1; tt <= max_time; tt++) {
                        if(d.gr.adj[i][ss][tt-1][s]) {
                            expr += var_x[i][ss][tt-1][s];
                        }
                    }
                }
            }
            
            cst_headway_3[s][t] = IloRange(env, -IloInfinity, expr, 1, name.str().c_str());
            expr.end();
        }

        model.add(cst_headway_3[s]);
    }
}

auto solver::create_constraints_siding(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_3d cst_siding(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        cst_siding[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s : d.net.sidings) {
            cst_siding[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = 1u; t <= d.ni; t++) {
                name.str(""); name << "cst_siding_" << i << "_" << s << "_" << t;
                IloExpr expr(env);
                
                auto min_time = static_cast<unsigned int>(std::max(1, static_cast<int>(t - d.headway)));
                auto max_time = std::min(d.ni + 1, t + d.headway);
                
                for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                    if(d.gr.adj[i][ss][t-1][s]) {
                        expr += var_x[i][ss][t-1][s];
                    }
                }
                
                for(auto j = 0u; j < d.nt; j++) {
                    if(j != i) {
                        for(auto tt = min_time; tt <= max_time; tt++) {
                            for(auto mm : d.net.main_tracks[s]) {
                                for(auto ss : d.gr.inverse_delta[j][mm]) {
                                    if(d.gr.adj[j][ss][tt-1][mm]) {
                                        expr -= var_x[j][ss][tt-1][mm];
                                    }
                                }
                            }
                        }
                    }
                }
                
                cst_siding[i][s][t] = IloRange(env, -IloInfinity, expr, 0, name.str().c_str());
                expr.end();
            }
            
            model.add(cst_siding[i][s]);
        }
    }
}

auto solver::create_constraints_heavy(IloEnv& env, IloModel& model, var_matrix_4d& var_x) -> void {
    cst_matrix_3d cst_heavy(env, d.nt);
    std::stringstream name;
    
    for(auto i = 0u; i < d.nt; i++) {
        if(d.trn.is_heavy[i]) {
            cst_heavy[i] = cst_matrix_2d(env, d.ns + 2);
        
            for(auto s : d.net.sidings) {
                cst_heavy[i][s] = cst_vector(env, d.ni + 2);
            
                for(auto t = 1u; t <= d.ni; t++) {
                    name.str(""); name << "cst_heavy_" << i << "_" << s << "_" << t;
                    IloExpr expr(env);
                    
                    auto min_time = static_cast<unsigned int>(std::max(1, static_cast<int>(t - d.headway)));
                    auto max_time = std::min(d.ni + 1, t + d.headway);
                
                    for(auto ss : d.gr.bar_inverse_delta[i][s]) {
                        if(d.gr.adj[i][ss][t-1][s]) {
                            expr += var_x[i][ss][t-1][s];
                        }
                    }
                
                    for(auto j = 0u; j < d.nt; j++) {
                        if(!d.trn.is_sa[j] && j != i) {
                            for(auto tt = min_time; tt <= max_time; tt++) {
                                for(auto mm : d.net.main_tracks[s]) {
                                    for(auto ss : d.gr.inverse_delta[j][mm]) {
                                        if(d.gr.adj[j][ss][tt-1][mm]) {
                                            expr += var_x[j][ss][tt-1][mm];
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    cst_heavy[i][s][t] = IloRange(env, -IloInfinity, expr, 1, name.str().c_str());
                    expr.end();
                }
            
                model.add(cst_heavy[i][s]);
            }
        }
    }
}

auto solver::create_constraints_cant_stop(IloEnv& env, IloModel& model, var_matrix_2d& var_excess_travel_time) -> void {
    // This constraint is equivalent to setting var_excess_travel_time = 0 on x-overs
    for(auto i = 0u; i < d.nt; i++) {
        for(auto s : d.net.xovers) {
            var_excess_travel_time[i][s].setUB(0);
        }
    }
}

auto solver::create_objective_function(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void {
    IloExpr expr(env);

    for(auto i = 0u; i < d.nt; i++) {
        for(auto s = 1u; s <= d.ns; s++) {
            expr += d.pri.delay[d.trn.type[i]] * var_excess_travel_time[i][s];
        }
        
        for(auto s1 = 0u; s1 <= d.ns + 1; s1++) {
            for(auto t = 0u; t <= d.ni + 1; t++) {
                for(auto s2 = 0u; s2 < d.ns + 1; s2++) {
                    if(d.gr.adj[i][s1][t][s2] && d.gr.costs[i][s1][t][s2] > 0) {
                        expr += d.gr.costs[i][s1][t][s2] * var_x[i][s1][t][s2];
                    }
                }
            }
        }
    }
    
    IloObjective obj = IloMinimize(env, expr);
    IloRange cst_positive_obj(env, 0, expr, IloInfinity, "cst_positive_obj");
    
    expr.end();

    model.add(obj);
    model.add(cst_positive_obj);
}

auto solver::create_model(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_matrix_2d& var_excess_travel_time) -> void {
    using namespace std::chrono;
    
    auto t_start = high_resolution_clock::time_point();
    auto t_end = high_resolution_clock::time_point();
    auto time_span = duration<double>();
    
    std::cerr << "Creating variables" << std::endl;
    
    t_start = high_resolution_clock::now();
    create_variables(env, model, var_x, var_excess_travel_time);
    t_end = high_resolution_clock::now();

    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.variable_creation = time_span.count();
    
    std::cerr << "Creating constraints" << std::endl;
    
    t_start = high_resolution_clock::now();
    std::cerr << "\tExit sigma" << std::endl;
    create_constraints_exit_sigma(env, model, var_x);
    std::cerr << "\tEnter tau" << std::endl;
    create_constraints_enter_tau(env, model, var_x);
    std::cerr << "\tFlow" << std::endl;
    create_constraints_flow(env, model, var_x);
    std::cerr << "\tMax one train" << std::endl;
    create_constraints_max_one_train(env, model, var_x);
    std::cerr << "\tSet excess travel time" << std::endl;
    create_constraints_set_excess_travel_time(env, model, var_x, var_excess_travel_time);
    std::cerr << "\tMin travel time" << std::endl;
    create_constraints_min_travel_time(env, model, var_excess_travel_time);
    std::cerr << "\tHeadway 1" << std::endl;
    create_constraints_headway_1(env, model, var_x);
    std::cerr << "\tHeadway 2" << std::endl;
    create_constraints_headway_2(env, model, var_x);
    std::cerr << "\tHeadway 3" << std::endl;
    create_constraints_headway_3(env, model, var_x);
    std::cerr << "\tSiding" << std::endl;
    create_constraints_siding(env, model, var_x);
    std::cerr << "\tHeavy" << std::endl;
    create_constraints_heavy(env, model, var_x);
    std::cerr << "\tCan't stop" << std::endl;
    create_constraints_cant_stop(env, model, var_excess_travel_time);
    t_end = high_resolution_clock::now();
    
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.constraints_creation = time_span.count();
    
    std::cerr << "Creating objective function" << std::endl;
    
    t_start = high_resolution_clock::now();
    create_objective_function(env, model, var_x, var_excess_travel_time);
    t_end = high_resolution_clock::now();
    
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.objf_creation = time_span.count();
}
