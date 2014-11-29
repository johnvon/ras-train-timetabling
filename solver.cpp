#include <solver.h>

#include <algorithm>
#include <chrono>
#include <sstream>

void solver::solve(bool use_max_travel_time, bool use_alt_min_travel_time) const {
    using namespace std::chrono;
    high_resolution_clock::time_point t_start, t_end;
    duration<double> time_span;
    
    double eps = 1e-6;
    
    IloEnv env;
    IloModel model(env);
    
    var_matrix_4d var_x(env, d.nt);
    var_vector var_d(env, d.nt);
    var_matrix_2d var_e(env, d.nt);
    
    std::stringstream name;
        
    std::cout << "Creating variables..." << std::endl;
    t_start = high_resolution_clock::now();
    
    for(auto i = 0; i < d.nt; i++) {
        std::cout << "\tTrain #" << i << std::endl;
        
        var_x[i] = var_matrix_3d(env, d.ns + 2);
        
        name.str(""); name << "var_d_" << i; auto ub_d = std::max(d.tr_wt[i], d.ni - d.tr_wt[i]);
        var_d[i] = IloNumVar(env, 0.0, ub_d, IloNumVar::Int, name.str().c_str());
        
        var_e[i] = IloNumVarArray(env, d.ns + 2);
        
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            var_x[i][s1] = var_matrix_2d(env, d.ni + 2);
                        
            if(d.sa[i][s1]) {
                name.str(""); name << "var_e_" << i << "_" << s1;
                auto ub_e = std::max(d.sa_times[i][s1], d.ni - d.sa_times[i][s1]);
                var_e[i][s1] = IloNumVar(env, 0.0, ub_e, IloNumVar::Int, name.str().c_str());
            }
            
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
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    IloObjective obj = IloMinimize(env);
    
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
    IloRange cst_positive_obj(env, 0, IloInfinity, "cst_positive_obj");

    std::cout << "Adding constraints..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s : d.tr_orig_seg[i]) {
            for(auto t = 0; t <= d.max_time_to_leave_from[i][s] - d.min_travel_time[i][s] - 1; t++) {
                if(d.adj[i][0][t][s]) {
                    cst_exit_sigma[i].setLinearCoef(var_x[i][0][t][s], 1);
                }
            }
        }
        
        name.str(""); name << "cst_enter_tau_" << i;
        cst_enter_tau[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s : d.tr_dest_seg[i]) {
            for(auto t = d.min_time_to_arrive_at[i][s] + d.min_travel_time[i][s] - 1; t <= d.ni; t++) {
                if(d.adj[i][s][t][d.ns+1]) {
                    cst_enter_tau[i].setLinearCoef(var_x[i][s][t][d.ns+1], 1);
                }
            }
        }
        
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        
        if(use_alt_min_travel_time) {
            cst_alt_min_travel_time[i] = cst_vector(env, d.ns + 2);
        } else {
            cst_min_travel_time[i] = cst_matrix_2d(env, d.ns + 2);
        }
        
        for(auto s = 1; s < d.ns + 1; s++) {
            cst_flow[i][s] = cst_vector(env, d.ni + 2);
            
            if(use_alt_min_travel_time) {
                name.str(""); name << "cst_alt_min_travel_time_" << i << "_" << s;
                cst_alt_min_travel_time[i][s] = IloRange(env, -IloInfinity, 0, name.str().c_str());
            } else {
                cst_min_travel_time[i][s] = cst_vector(env, d.ni + 2);
            }
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
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
                }
            }
            
            if(use_alt_min_travel_time) {
                for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s] - 1; t++) {
                    if(d.adj[i][s][t][s]) {
                        cst_alt_min_travel_time[i][s].setLinearCoef(var_x[i][s][t][s], -1);
                    }
                }
                
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_alt_min_travel_time[i][s].setLinearCoef(var_x[i][ss][t-1][s], d.min_travel_time[i][s] - 1);
                        }
                    }
                }
            } else {
                for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s] - d.min_travel_time[i][s]; t++) {
                    if(d.v[i][s][t]) {
                        name.str(""); name << "cst_min_travel_time_" << i << "_" << s << "_" << t;
                        cst_min_travel_time[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                    
                        for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                            if(d.adj[i][ss][t-1][s]) {
                                cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                            }
                        }
                    
                        for(auto tt = t; tt < t + d.min_travel_time[i][s] + 1; tt++) {
                            for(auto ss : d.bar_tnetwork[i][s]) {
                                if(d.adj[i][s][tt][ss]) {
                                    cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                                }
                            }
                        }
                        
                        if(use_max_travel_time) {
                            for(auto tt = t + d.max_travel_time[i][s] + 1; tt <= d.max_time_to_leave_from[i][s]; tt++) {
                                for(auto ss : d.bar_tnetwork[i][s]) {
                                    if(d.adj[i][s][tt][ss]) {
                                        cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                                    }
                                }
                            }
                        }
                    }
                }
                
                model.add(cst_min_travel_time[i][s]);
            }
            
            if(use_alt_min_travel_time) {
                model.add(cst_alt_min_travel_time[i]);
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
            cst_visit_sa[i] = cst_vector(env, d.ns + 2);
            cst_sa_delay[i] = cst_vector(env, d.ns + 2);
            
            for(auto s : d.sa_seg[i]) {
                name.str(""); name << "cst_visit_sa_" << i << "_" << s;
                cst_visit_sa[i][s] = IloRange(env, 1, IloInfinity, name.str().c_str());
                
                name.str(""); name << "cst_sa_delay_" << i << "_" << s;
                auto ais = d.sa_times[i][s] + d.sa_tw_right - 1;
                cst_sa_delay[i][s] = IloRange(env, -IloInfinity, ais, name.str().c_str());
                
                cst_sa_delay[i][s].setLinearCoef(var_e[i][s], -1.0);
                
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
                        if(d.adj[i][ss][t-1][s]) {
                            cst_visit_sa[i][s].setLinearCoef(var_x[i][ss][t-1][s], 1);
                            cst_sa_delay[i][s].setLinearCoef(var_x[i][ss][t-1][s], t);
                        }
                    }
                }
            }
            
            model.add(cst_visit_sa[i]);
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
                        
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
                name.str(""); name << "cst_headway_1_" << i << "_" << s << "_" << t;
                cst_headway_1[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                name.str(""); name << "cst_headway_2_" << i << "_" << s << "_" << t;
                cst_headway_2[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                name.str(""); name << "cst_headway_3_" << i << "_" << s << "_" << t;
                cst_headway_3[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                name.str(""); name << "cst_headway_4_" << i << "_" << s << "_" << t;
                cst_headway_4[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                                
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
                
                for(auto j = 0; j < d.nt; j++) {
                    if(j != i) {
                        for(auto ss : d.bar_inverse_tnetwork[j][s]) {
                            for(auto tt = std::max(1, t - d.headway); tt <= t - 1; tt++) {
                                if(d.adj[j][ss][tt-1][s]) {
                                    cst_headway_1[i][s][t].setLinearCoef(var_x[j][ss][tt-1][s], 1);
                                }
                            }
                            
                            for(auto tt = t + 1; tt <= std::min(d.ni, t + d.headway); tt++) {
                                if(d.adj[j][ss][tt-1][s]) {
                                    cst_headway_3[i][s][t].setLinearCoef(var_x[j][ss][tt-1][s], 1);
                                }
                            }
                        }
                        
                        for(auto ss : d.bar_tnetwork[j][s]) {
                            for(auto tt = std::max(0, t - d.headway); tt <= t - 1; tt++) {
                                if(d.adj[j][s][tt][ss]) {
                                    cst_headway_2[i][s][t].setLinearCoef(var_x[j][s][tt][ss], 1);
                                }
                            }
                            
                            for(auto tt = t + 1; tt <= std::min(d.ni, t + d.headway); tt++) {
                                if(d.adj[j][s][tt][ss]) {
                                    cst_headway_4[i][s][t].setLinearCoef(var_x[j][s][tt][ss], 1);
                                }
                            }
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
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
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
            
            model.add(cst_siding[i][s]);
        }
        
        cst_cant_stop[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s : d.xovers) {
            cst_cant_stop[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s] - d.min_travel_time[i][s]; t++) {
                name.str(""); name << "cst_cant_stop_" << i << "_" << s << "_" << t;
                cst_cant_stop[i][s][t] = IloRange(env, -IloInfinity, 1, name.str().c_str());
                
                for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                    if(d.adj[i][ss][t-1][s]) {
                        cst_cant_stop[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                    }
                }
                
                for(auto ss : d.bar_tnetwork[i][s]) {
                    for(auto tt = t + d.min_travel_time[i][s]; tt <= d.max_time_to_leave_from[i][s]; tt++) {
                        if(d.adj[i][s][tt][ss]) {
                            cst_cant_stop[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                        }
                    }
                }
            }
            
            model.add(cst_cant_stop[i][s]);
        }
        
        if(d.tr_heavy[i]) {
            cst_heavy[i] = cst_matrix_2d(env, d.ns + 2);
            
            for(auto s : d.sidings) {
                cst_heavy[i][s] = cst_vector(env, d.ni + 2);
                
                for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
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
    }
    
    for(auto s = 1; s < d.ns + 1; s++) {
        auto first_time = (*std::min_element(d.min_time_to_arrive_at.begin(), d.min_time_to_arrive_at.end(), [s] (const auto& r1, const auto& r2) { return r1[s] < r2[s]; }))[s];
        auto last_time = (*std::max_element(d.max_time_to_leave_from.begin(), d.max_time_to_leave_from.end(), [s] (const auto& r1, const auto& r2) { return r1[s] < r2[s]; }))[s];
        
        cst_max_one_train[s] = cst_vector(env, d.ni + 2);
        
        for(auto t = first_time; t <= last_time; t++) {
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
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Setting objective function coefficients..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        obj.setLinearCoef(var_d[i], d.wt_price);
        cst_positive_obj.setLinearCoef(var_d[i], d.wt_price);
                
        if(d.tr_sa[i]) {
            for(auto s : d.sa_seg[i]) {
                obj.setLinearCoef(var_e[i][s], d.sa_price);
                cst_positive_obj.setLinearCoef(var_e[i][s], d.sa_price);
            }
        }
        
        for(auto s1 = 0; s1 < d.ns + 1; s1++) {
            for(auto t = d.min_time_to_arrive_at[i][s1]; t <= d.max_time_to_leave_from[i][s1]; t++) {
                for(auto s2 : d.tnetwork[i][s1]) {
                    if(d.adj[i][s1][t][s2]) {
                        obj.setLinearCoef(var_x[i][s1][t][s2], d.arc_cost[i][s1][t][s2]);
                        cst_positive_obj.setLinearCoef(var_x[i][s1][t][s2], d.arc_cost[i][s1][t][s2]);
                    }
                }
            }
        }
    }

    model.add(obj);
    model.add(cst_positive_obj);

    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);

    std::cout << "\t" << time_span.count() << " seconds" << std::endl;

    IloCplex cplex(model);

    cplex.exportModel("model.lp");

    if(!cplex.solve()) {
        std::cout << "Cplex status: " << cplex.getStatus() << " " << cplex.getCplexStatus() << std::endl;
        return;
    }

    std::cout << "Cplex optimal value: " << cplex.getObjValue() << std::endl;
    
    auto x = int_matrix_4d(d.nt, int_matrix_3d(d.ns + 2, int_matrix(d.ni + 2, ivec(d.ns + 2, 0))));
    
    for(auto i = 0; i < d.nt; i++) {
        std::cout << "Train " << i << std::endl;
        auto dv = cplex.getValue(var_d[i]);
        if(dv > eps) {
            std::cout << "\tArrived at its terminal " << dv << " time units outside its time window" << std::endl;
        } else {
            std::cout << "\tArrived at its terminal within the time window" << std::endl;
        }
        
        for(auto s = 0; s < d.ns + 2; s++) {
            if(d.sa[i][s]) {
                auto ev = cplex.getValue(var_e[i][s]);
                if(ev > eps) {
                    std::cout << "\tArrived at its SA segment " << s << ", " << ev << " time units after its time window" << std::endl;
                } else {
                    std::cout << "\tArrived at its SA segment " << s << " within its time window" << std::endl;
                }
            }
        }
        
        for(auto s = 1; s < d.ns + 1; s++) {            
            for(auto t = 1; t < d.ni + 2; t++) {
                for(auto ss = 0; ss < d.ns + 2; ss++) {
                    if(d.adj[i][ss][t-1][s]) {
                        auto xv = cplex.getValue(var_x[i][ss][t-1][s]);
                        if(xv > eps) {
                            std::cout << "\t\t" << (s == ss ? "[" : "") << "Came from " << (ss == 0 ? "sigma" : std::to_string(ss)) << " at time " << t-1 << " and arrived in " << s << " at time " << t << (s == ss ? "]" : "") << std::endl;
                        }
                    }
                    if(d.adj[i][s][t][ss]) {
                        auto xv = cplex.getValue(var_x[i][s][t][ss]);
                        if(xv > eps) {
                            std::cout << "\t\t" << (s == ss ? "[" : "") << "Left " << s << " at time " << t << " and arrived in " << (ss == d.ns + 1 ? "tau" : std::to_string(ss)) << " at time " << t+1 << (s == ss ? "]" : "") << std::endl;
                        }
                    }
                }
            }
        }
    }
    
    env.end();
}
