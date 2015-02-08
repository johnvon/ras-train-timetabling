#include <solver/solver.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

void solver::solve() {
    using namespace std::chrono;
    
    auto t_start = high_resolution_clock::time_point();
    auto t_end = high_resolution_clock::time_point();
    auto time_span = duration<double>();
    auto obj_value = 0.0;
    
    IloEnv env;
    IloModel model(env);
    
    var_matrix_4d var_x(env, d.nt);
    var_vector var_d(env, d.nt);
    var_matrix_2d var_e(env, d.nt);
    var_matrix_2d var_travel_time(env, d.nt);
    
    create_model(env, model, var_x, var_d, var_e, var_travel_time);

    IloCplex cplex(model);

    cplex.exportModel("model.lp");    
    cplex.setParam(IloCplex::TiLim, p.cplex.time_limit);
    cplex.setParam(IloCplex::Threads, p.cplex.threads);
    cplex.setParam(IloCplex::NodeLim, 0);

    t_start = high_resolution_clock::now();
    
    auto success_at_root_node = cplex.solve();
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.cplex_at_root = time_span.count();

    // Check that CPLEX gives a negative status at root, but not just because it couldn't find a feasible solution.
    // In fact, we want to stop only if the problem is proven infeasible.
    // Not having found a feasible solution is ok, we can still find it later.
    if(!success_at_root_node && cplex.getCplexStatus() != IloCplex::NodeLimInfeas) {
        std::cout << "Cplex infeasible at root node: " << cplex.getStatus() << " - " << cplex.getCplexStatus() << std::endl;
        obj_value = -1; // Failure at root node
    } else {
        cplex.setParam(IloCplex::NodeLim, 2100000000);
        
        t_start = high_resolution_clock::now();
        
        auto success_at_later_node = cplex.solve();
        
        t_end = high_resolution_clock::now();
        time_span = duration_cast<duration<double>>(t_end - t_start);
        t.cplex_total = t.cplex_at_root + time_span.count();
        
        if(!success_at_later_node) {
            std::cout << "Cplex infeasible at later node: " << cplex.getStatus() << " - " << cplex.getCplexStatus() << std::endl;
            obj_value = -2; // Failure at a later node
        } else {
            obj_value = cplex.getObjValue();
        }
    }
    
    if(obj_value >= 0) {
        std::cout << "Cplex ojective value: " << obj_value << std::endl;
    }
    
    print_results(obj_value);
    
    env.end();
}

void solver::print_results(double obj_value) {
    std::ofstream results_file;
    results_file.open(p.results_file, std::ios::out | std::ios::app);
    
    results_file << d.file_name << "\t";
    results_file << std::boolalpha << p.heuristics.simplified_objective_function << "\t";
    results_file << std::boolalpha << p.heuristics.corridor.active << "\t";
    results_file << std::boolalpha << p.heuristics.sparsification.active << "\t";
    results_file << t.variable_creation << "\t";
    results_file << t.constraints_creation << "\t";
    results_file << t.objf_creation << "\t";
    results_file << t.cplex_at_root << "\t";
    results_file << t.cplex_total << "\t";
    results_file << obj_value << std::endl;
}

void solver::create_model(IloEnv& env, IloModel& model, var_matrix_4d& var_x, var_vector& var_d, var_matrix_2d& var_e, var_matrix_2d& var_travel_time) {
    using namespace std::chrono;
    
    auto t_start = high_resolution_clock::time_point();
    auto t_end = high_resolution_clock::time_point();
    auto time_span = duration<double>();
    
    std::stringstream name;
    
    t_start = high_resolution_clock::now();
    
    for(auto i = 0; i < d.nt; i++) {        
        var_x[i] = var_matrix_3d(env, d.ns + 2);
        
        name.str(""); name << "var_d_" << i; auto ub_d = std::max(d.tr_wt[i], d.ni - d.tr_wt[i]);
        var_d[i] = IloNumVar(env, 0.0, ub_d, IloNumVar::Int, name.str().c_str());
        
        var_travel_time[i] = IloNumVarArray(env, d.ns + 2);
        
        if(d.tr_sa[i]) {
            var_e[i] = IloNumVarArray(env, d.ns + 2);
            
            for(auto n = 0; n < d.sa_num[i]; n++) {
                name.str(""); name << "var_e_" << i << "_" << n;
                auto ub_e = std::max(d.sa_times[i][n], d.ni - d.sa_times[i][n]);
                var_e[i][n] = IloNumVar(env, 0.0, ub_e, IloNumVar::Int, name.str().c_str());
            }
        }
        
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            var_x[i][s1] = var_matrix_2d(env, d.ni + 2);
            
            name.str(""); name << "var_travel_time_" << i << "_" << s1;
            var_travel_time[i][s1] = IloNumVar(env, 0, d.max_travel_time[i][s1], IloNumVar::Int, name.str().c_str());
            
            for(auto t = 0; t < d.ni + 1; t++) {
                var_x[i][s1][t] = IloNumVarArray(env, d.ns + 2);
                
                for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                    if(d.adj[i][s1][t][s2]) {
                        name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t << "_" << s2;
                        var_x[i][s1][t][s2] = IloNumVar(env, 0, 1, IloNumVar::Bool, name.str().c_str());
                    }
                }
            }
        }
    }
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.variable_creation = time_span.count();
    
    cst_vector cst_exit_sigma(env, d.nt);
    cst_vector cst_enter_tau(env, d.nt);
    cst_matrix_2d cst_max_one_train(env, d.ns + 2);
    cst_matrix_3d cst_flow(env, d.nt);
    cst_matrix_2d cst_visit_sa(env, d.nt);
    cst_vector cst_wt_delay_1(env, d.nt);
    cst_vector cst_wt_delay_2(env, d.nt);
    cst_matrix_2d cst_sa_delay(env, d.nt);
    cst_matrix_3d cst_min_travel_time(env, d.nt); // This is also used to respect the known UB on the solution value
    cst_matrix_2d cst_alt_min_travel_time(env, d.nt); // Alternative formulation
    cst_matrix_3d cst_headway_1(env, d.nt);
    cst_matrix_3d cst_headway_2(env, d.nt);
    cst_matrix_3d cst_headway_3(env, d.nt);
    cst_matrix_3d cst_headway_4(env, d.nt);
    cst_matrix_3d cst_siding(env, d.nt);
    cst_matrix_3d cst_cant_stop(env, d.nt);
    cst_matrix_3d cst_heavy(env, d.nt);
    cst_matrix_2d cst_set_travel_time(env, d.nt);

    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s : d.tr_orig_seg[i]) {
            for(auto t = 0; t <= d.ni - d.min_travel_time[i][s] - 1; t++) {
                if(d.adj[i][0][t][s]) {
                    cst_exit_sigma[i].setLinearCoef(var_x[i][0][t][s], 1);
                }
            }
        }
        
        name.str(""); name << "cst_enter_tau_" << i;
        cst_enter_tau[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s = 1; s < d.ns + 1; s++) {
            for(auto t = d.min_time_to_arrive_at[i][s] + d.min_travel_time[i][s] - 1; t < d.ni + 1; t++) {
                if(d.adj[i][s][t][d.ns+1]) {
                    cst_enter_tau[i].setLinearCoef(var_x[i][s][t][d.ns+1], 1);
                }
            }
        }
        
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        
        if(p.model.alternative_min_travel_time_cst) {
            cst_alt_min_travel_time[i] = cst_vector(env, d.ns + 2);
        } else {
            cst_min_travel_time[i] = cst_matrix_2d(env, d.ns + 2);
        }
        
        for(auto s = 1; s < d.ns + 1; s++) {
            cst_flow[i][s] = cst_vector(env, d.ni + 2);
            
            if(p.model.alternative_min_travel_time_cst) {
                name.str(""); name << "cst_alt_min_travel_time_" << i << "_" << s;
                cst_alt_min_travel_time[i][s] = IloRange(env, -IloInfinity, 0, name.str().c_str());
            } else {
                cst_min_travel_time[i][s] = cst_vector(env, d.ni + 2);
            }
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni + 1; t++) {
                if(d.v[i][s][t]) {
                    name.str(""); name << "cst_flow_" << i << "_" << s << "_" << t;
                    cst_flow[i][s][t] = IloRange(env, 0, 0, name.str().c_str());
                    
                    for(auto ss : d.inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_flow[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                    
                    for(auto ss : d.tnetwork[i][s]) {
                        if(d.adj[i][s][t][ss]) {
                            cst_flow[i][s][t].setLinearCoef(var_x[i][s][t][ss], -1);
                        }
                    }
                    
                    if(t == d.ni && d.adj[i][s][d.ni][d.ns+1]) { // Escape arc
                        cst_flow[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], -1);
                    }
                }
            }
            
            if(p.model.alternative_min_travel_time_cst) {
                for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni; t++) {
                    if(d.adj[i][s][t][s]) {
                        cst_alt_min_travel_time[i][s].setLinearCoef(var_x[i][s][t][s], -1);
                    }
                }
                
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.ni + 1; t++) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_alt_min_travel_time[i][s].setLinearCoef(var_x[i][ss][t-1][s], d.min_travel_time[i][s] - 1);
                        }
                    }
                }
                
                model.add(cst_alt_min_travel_time[i]);
            } else {
                for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.ni - d.min_travel_time[i][s]; t++) {
                    if(d.v[i][s][t]) {
                        name.str(""); name << "cst_min_travel_time_" << i << "_" << s << "_" << t;
                        cst_min_travel_time[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                    
                        for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                            if(d.adj[i][ss][t-1][s]) {
                                cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                            }
                        }
                    
                        for(auto ss : d.bar_tnetwork[i][s]) {
                            for(auto tt = t; tt < std::min(d.ni, t + d.min_travel_time[i][s] + 1); tt++) {
                                if(d.adj[i][s][tt][ss]) {
                                    cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                                }
                            }
                            
                            if(d.ni <= t + d.min_travel_time[i][s] + 1 && d.adj[i][s][d.ni][d.ns+1]) { // Escape arc
                                cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], 1);
                            }
                        }
                        
                        if(p.model.max_travel_time_cst) {
                            for(auto ss : d.bar_tnetwork[i][s]) {
                                for(auto tt = t + d.max_travel_time[i][s] + 1; tt < d.ni + 1; tt++) {
                                    if(d.adj[i][s][tt][ss]) {
                                        cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                                    }
                                }
                                
                                if(d.adj[i][ss][d.ni][d.ns+1]) { // Escape arc
                                    cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], 1);
                                }
                            }
                        }
                    }
                }
                
                model.add(cst_min_travel_time[i][s]);
            }
            
            model.add(cst_flow[i][s]);
        }

        name.str(""); name << "cst_wt_delay_1_" << i;
        auto W1 = d.tr_wt[i] - d.wt_tw_left;
        cst_wt_delay_1[i] = IloRange(env, W1, IloInfinity, name.str().c_str());
        
        name.str(""); name << "cst_wt_delay_2_" << i;
        auto W2 = d.tr_wt[i] + d.wt_tw_right;
        cst_wt_delay_2[i] = IloRange(env, -IloInfinity, W2, name.str().c_str());

        cst_wt_delay_1[i].setLinearCoef(var_d[i], 1.0);
        cst_wt_delay_2[i].setLinearCoef(var_d[i], -1.0);
        
        for(auto s : d.tr_dest_seg[i]) {
            for(auto t = d.min_time_to_arrive_at[i][s] + d.min_travel_time[i][s] - 1; t <= d.ni; t++) {
                if(d.adj[i][s][t][d.ns+1]) {
                    cst_wt_delay_1[i].setLinearCoef(var_x[i][s][t][d.ns+1], t);
                    cst_wt_delay_2[i].setLinearCoef(var_x[i][s][t][d.ns+1], t);
                }
            }
        }
        
        if(d.tr_sa[i]) {
            cst_visit_sa[i] = cst_vector(env, d.sa_num[i]);
            cst_sa_delay[i] = cst_vector(env, d.sa_num[i]);
            
            for(auto n = 0; n < d.sa_num[i]; n++) {
                name.str(""); name << "cst_visit_sa_" << i << "_" << n;
                cst_visit_sa[i][n] = IloRange(env, 1, IloInfinity, name.str().c_str());
                
                name.str(""); name << "cst_sa_delay_" << i << "_" << n;
                auto ais = d.sa_times[i][n] + d.sa_tw_right - 1;
                cst_sa_delay[i][n] = IloRange(env, -IloInfinity, ais, name.str().c_str());
                
                cst_sa_delay[i][n].setLinearCoef(var_e[i][n], -1.0);
                                
                for(auto s : d.segments_for_sa[i][n]) {
                    for(auto ss : d.bar_tnetwork[i][s]) {
                        for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni; t++) {
                            if(d.adj[i][s][t][ss]) {
                                cst_visit_sa[i][n].setLinearCoef(var_x[i][s][t][ss], 1);
                                cst_sa_delay[i][n].setLinearCoef(var_x[i][s][t][ss], t);
                            }
                        }
                    }
                }
            }
            
            // model.add(cst_visit_sa[i]); ### WE DON'T ADD THIS CONSTRAINT SINCE WHEN WE HAVE ESCAPE ARCS
            //                                 IN ITS STEAD, WE COULD ADD THE PENALTY CONSTRAINT (SEE MY NOTES)
            //                                 IN PRACTICE, SINCE VISITING SA IS COMPULSORY FOR TRAINS THAT GO
            //                                 UP TO THEIR END TERMINAL, WE DON'T NEED THIS NEW CONSTRAINT
            model.add(cst_sa_delay[i]);
        }
        
        cst_headway_1[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_2[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_3[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_4[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s = 1; s < d.ns + 1; s++) {
            cst_headway_1[i][s] = cst_vector(env, d.ni + 2);
            cst_headway_2[i][s] = cst_vector(env, d.ni + 2);
            cst_headway_3[i][s] = cst_vector(env, d.ni + 2);
            cst_headway_4[i][s] = cst_vector(env, d.ni + 2);
                        
            for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni + 1; t++) {
                if(d.v[i][s][t]) {
                    if(!d.bar_inverse_tnetwork[i][s].empty()) {
                        auto any_arc = false;
                        for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                            if(d.adj[i][ss][t-1][s]) {
                                any_arc = true;
                                break;
                            }
                        }
                        
                        if(any_arc) {
                            name.str(""); name << "cst_headway_1_" << i << "_" << s << "_" << t;
                            cst_headway_1[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                            name.str(""); name << "cst_headway_2_" << i << "_" << s << "_" << t;
                            cst_headway_2[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                        }
                    }
                    
                    if(!d.bar_tnetwork[i][s].empty()) {
                        auto any_arc = false;
                        for(auto ss : d.bar_tnetwork[i][s]) {
                            if(d.adj[i][s][t][ss]) {
                                any_arc = true;
                                break;
                            }
                        }
                        if(t == d.ni && d.adj[i][s][d.ni][d.ns+1]) { // Consider escape arcs too
                            any_arc = true;
                        }
                        
                        if(any_arc) {
                            name.str(""); name << "cst_headway_3_" << i << "_" << s << "_" << t;
                            cst_headway_3[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                            name.str(""); name << "cst_headway_4_" << i << "_" << s << "_" << t;
                            cst_headway_4[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                        }
                    }
                }
                                
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    if(d.adj[i][ss][t-1][s]) {
                        cst_headway_1[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        cst_headway_2[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                    }
                }
                
                for(auto ss : d.bar_tnetwork[i][s]) {
                    if(d.adj[i][s][t][ss]) {
                        cst_headway_3[i][s][t].setLinearCoef(var_x[i][s][t][ss], 1);
                        cst_headway_4[i][s][t].setLinearCoef(var_x[i][s][t][ss], 1);
                    }
                }
                
                if(t == d.ni && d.adj[i][s][d.ni][d.ns+1]) { // Escape arc
                    cst_headway_3[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], 1);
                    cst_headway_4[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], 1);
                }
                
                for(auto j = 0; j < d.nt; j++) {
                    if(j != i) {
                        for(auto ss : d.bar_inverse_tnetwork[j][s]) {
                            for(auto tt = std::max(1, t - d.headway); tt <= t - 1; tt++) {
                                if(d.adj[j][ss][tt-1][s] && d.adj[i][ss][t-1][s]) {
                                    cst_headway_1[i][s][t].setLinearCoef(var_x[j][ss][tt-1][s], 1);
                                }
                            }
                            
                            for(auto tt = t + 1; tt <= std::min(d.ni, t + d.headway); tt++) {
                                if(d.adj[j][ss][tt-1][s] && d.adj[i][s][t][ss]) {
                                    cst_headway_3[i][s][t].setLinearCoef(var_x[j][ss][tt-1][s], 1);
                                }
                            }
                        }
                        
                        for(auto ss : d.bar_tnetwork[j][s]) {
                            for(auto tt = std::max(0, t - d.headway); tt <= t - 1; tt++) {
                                if(d.adj[j][s][tt][ss] && d.adj[i][ss][t-1][s]) {
                                    cst_headway_2[i][s][t].setLinearCoef(var_x[j][s][tt][ss], 1);
                                }
                            }
                            
                            for(auto tt = t + 1; tt <= std::min(d.ni, t + d.headway); tt++) {
                                if(d.adj[j][s][tt][ss] && d.adj[i][s][t][ss]) {
                                    cst_headway_4[i][s][t].setLinearCoef(var_x[j][s][tt][ss], 1);
                                }
                            }
                        }
                        
                        if(d.ni <= t + d.headway && d.adj[j][s][d.ni][d.ns+1]) { // Escape arc
                            cst_headway_4[i][s][t].setLinearCoef(var_x[j][s][d.ni][d.ns+1], 1);
                        }
                    }
                }
            }
            
            model.add(cst_headway_1[i][s]);
            model.add(cst_headway_2[i][s]);
            model.add(cst_headway_3[i][s]);
            model.add(cst_headway_4[i][s]);
        }
        
        cst_siding[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s : d.sidings) {
            cst_siding[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni; t++) {
                if(d.v[i][s][t]) {
                    name.str(""); name << "cst_siding_" << i << "_" << s << "_" << t;
                    cst_siding[i][s][t] = IloRange(env, 0, IloInfinity, name.str().c_str());
                
                    for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_siding[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], -1);
                        }
                    }
                
                    for(auto j = 0; j < d.nt; j++) {
                        if(j != i) {
                            for(auto tt = std::max(1, t - d.headway); tt <= std::min(d.ni, t + d.headway); tt++) {
                                for(auto ss1 : d.main_tracks[s]) {
                                    for(auto ss2 : d.bar_inverse_tnetwork[j][ss1]) {
                                        if(d.adj[j][ss2][tt-1][ss1]) {
                                            cst_siding[i][s][t].setLinearCoef(var_x[j][ss2][tt-1][ss1], 1);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            model.add(cst_siding[i][s]);
        }
        
        cst_cant_stop[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s : d.xovers) {
            cst_cant_stop[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.ni - d.min_travel_time[i][s]; t++) {
                if(d.v[i][s][t]) {
                    name.str(""); name << "cst_cant_stop_" << i << "_" << s << "_" << t;
                    cst_cant_stop[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                
                    for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_cant_stop[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                
                    for(auto ss : d.bar_tnetwork[i][s]) {
                        for(auto tt = t + d.min_travel_time[i][s]; tt < d.ni + 1; tt++) {
                            if(d.adj[i][s][tt][ss]) {
                                cst_cant_stop[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                            }
                        }
                    }
                    
                    if(t == d.ni && d.adj[i][s][d.ni][d.ns+1]) { // Escape arc
                        cst_cant_stop[i][s][t].setLinearCoef(var_x[i][s][d.ni][d.ns+1], 1);
                    }
                }
            }
            
            model.add(cst_cant_stop[i][s]);
        }
        
        if(d.tr_heavy[i]) {
            cst_heavy[i] = cst_matrix_2d(env, d.ns + 2);
            
            for(auto s : d.sidings) {
                cst_heavy[i][s] = cst_vector(env, d.ni + 2);
                
                for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni + 1; t++) {
                    name.str(""); name << "cst_heavy_" << i << "_" << s << "_" << t;
                    cst_heavy[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                    
                    for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_heavy[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                    
                    for(auto j = 0; j < d.nt; j++) {
                        if(j != i && !d.tr_sa[j]) {
                            for(auto ss1 : d.main_tracks[s]) {
                                for(auto ss2 : d.bar_inverse_tnetwork[j][ss1]) {
                                    if(d.adj[j][ss2][t-1][ss1]) {
                                        cst_heavy[i][s][t].setLinearCoef(var_x[j][ss2][t-1][ss1], 1);
                                    }
                                }
                            }
                        }
                    }
                }
                
                model.add(cst_heavy[i][s]);
            }
        }
        
        cst_set_travel_time[i] = cst_vector(env, d.ns + 2);
        
        for(auto s = 1; s < d.ns + 1; s++) {
            name.str(""); name << "cst_set_travel_time_" << i << "_" << s;
            cst_set_travel_time[i][s] = IloRange(env, 0, 0, name.str().c_str());
            
            cst_set_travel_time[i][s].setLinearCoef(var_travel_time[i][s], 1);
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni + 1; t++) {
                for(auto ss : d.bar_tnetwork[i][s]) {
                    if(d.adj[i][s][t][ss]) {
                        cst_set_travel_time[i][s].setLinearCoef(var_x[i][s][t][ss], -t);
                    }
                }
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    if(d.adj[i][ss][t-1][s]) {
                        cst_set_travel_time[i][s].setLinearCoef(var_x[i][ss][t-1][s], t + d.min_travel_time[i][s] - 1);
                    }
                }
            }
            
            if(d.adj[i][s][d.ni][d.ns+1]) { // Escape arc
                cst_set_travel_time[i][s].setLinearCoef(var_x[i][s][d.ni][d.ns+1], -d.ni);
            }
        }
        
        model.add(cst_set_travel_time[i]);
    }
    
    for(auto s = 1; s < d.ns + 1; s++) {
        auto first_time = (*std::min_element(d.min_time_to_arrive_at.begin(), d.min_time_to_arrive_at.end(), [s] (const auto& r1, const auto& r2) { return r1[s] < r2[s]; }))[s];
        // auto last_time = (*std::max_element(d.max_time_to_leave_from.begin(), d.max_time_to_leave_from.end(), [s] (const auto& r1, const auto& r2) { return r1[s] < r2[s]; }))[s]; ### (unused because of trains that might not reach their terminal)
        
        cst_max_one_train[s] = cst_vector(env, d.ni + 2);
        
        for(auto t = first_time; t < d.ni + 2; t++) {
            if(d.v_for_someone[s][t]) {
                name.str(""); name << "cst_max_one_train_" << s << "_" << t;
                cst_max_one_train[s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
            
                for(auto i : d.trains_for[s][t]) {
                    for(auto ss : d.inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_max_one_train[s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                }
            }
        }
        
        model.add(cst_max_one_train[s]);
    }
    
    model.add(cst_exit_sigma);
    model.add(cst_enter_tau);
    model.add(cst_wt_delay_1);
    model.add(cst_wt_delay_2);

    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.constraints_creation = time_span.count();
    
    t_start = high_resolution_clock::now();

    IloObjective obj = IloMinimize(env);
    IloRange cst_positive_obj(env, 0, IloInfinity, "cst_positive_obj");

    for(auto i = 0; i < d.nt; i++) {
        obj.setLinearCoef(var_d[i], d.wt_price);
        cst_positive_obj.setLinearCoef(var_d[i], d.wt_price);
        
        if(!p.heuristics.simplified_objective_function) {
            for(auto s = 1; s < d.ns + 1; s++) {
                obj.setLinearCoef(var_travel_time[i][s], d.delay_price[d.tr_class[i]]);
                cst_positive_obj.setLinearCoef(var_travel_time[i][s], d.delay_price[d.tr_class[i]]);
            }
        
            if(d.tr_sa[i]) {
                for(auto n = 0; n < d.sa_num[i]; n++) {
                    obj.setLinearCoef(var_e[i][n], d.sa_price);
                    cst_positive_obj.setLinearCoef(var_e[i][n], d.sa_price);
                }
            }
        
            for(auto s : d.unpreferred_segments[i]) {
                for(auto t = d.min_time_to_arrive_at[i][s]; t < d.ni + 1; t++) {
                    for(auto ss : d.inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s]) {
                            obj.setLinearCoef(var_x[i][ss][t-1][s], d.unpreferred_price);
                            cst_positive_obj.setLinearCoef(var_x[i][ss][t-1][s], d.unpreferred_price);
                        }
                    }
                }
            }
        }
    }

    model.add(obj);
    model.add(cst_positive_obj);

    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    t.objf_creation = time_span.count();
}