#include <solver.h>

#include <algorithm>
#include <chrono>
#include <sstream>

auto solver::real_node(auto s, auto t) const {
    return (s > 0 && s <= d.ns && t > 0 && t <= d.ni);
}


void solver::solve(bool use_max_travel_time, bool use_alt_min_travel_time) const {
    using namespace std::chrono;
    high_resolution_clock::time_point t_start, t_end;
    duration<double> time_span;
    
    // double eps = 1e-6;
    
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
                    if(d.adj[i][s1][t][s2][t+1]) {
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
    IloRange positive_obj(env, 0, IloInfinity, "positive_obj");

    std::cout << "Adding constraints..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s : d.tr_orig_seg[i]) {
            for(auto t = 0; t <= d.max_time_to_leave_from[i][s] - d.min_travel_time[i][s] - 1; t++) {
                if(d.adj[i][0][t][s][t+1]) {
                    cst_exit_sigma[i].setLinearCoef(var_x[i][0][t][s], 1);
                }
            }
        }
        
        name.str(""); name << "cst_enter_tau_" << i;
        cst_enter_tau[i] = IloRange(env, 1, 1, name.str().c_str());
        
        for(auto s : d.tr_dest_seg[i]) {
            for(auto t = d.min_time_to_arrive_at[i][s] + d.min_travel_time[i][s] - 1; t <= d.ni; t++) {
                if(d.adj[i][s][t][d.ns+1][t+1]) {
                    cst_enter_tau[i].setLinearCoef(var_x[i][s][t][d.ns+1], 1);
                }
            }
        }
        
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        cst_min_travel_time[i] = cst_matrix_2d(env, d.ns + 2);
        
        for(auto s = 1; s < d.ns + 1; s++) {
            cst_flow[i][s] = cst_vector(env, d.ni + 2);
            cst_min_travel_time[i][s] = cst_vector(env, d.ni + 2);
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s]; t++) {
                if(d.v[i][s][t]) {
                    name.str(""); name << "cst_flow_" << i << "_" << s << "_" << t;
                    cst_flow[i][s][t] = IloRange(env, 0, 0, name.str().c_str());
                    
                    for(auto ss : d.inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s][t]) {
                            cst_flow[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                    
                    for(auto ss : d.tnetwork[i][s]) {
                        if(d.adj[i][s][t][ss][t+1]) {
                            cst_flow[i][s][t].setLinearCoef(var_x[i][s][t][ss], -1);
                        }
                    }
                }
            }
            
            for(auto t = d.min_time_to_arrive_at[i][s]; t <= d.max_time_to_leave_from[i][s] - d.min_travel_time[i][s]; t++) {
                if(d.v[i][s][t]) {
                    name.str(""); name << "cst_min_travel_time_" << i << "_" << s << "_" << t;
                    cst_min_travel_time[i][s][t] = IloRange(env, -IloInfinite, 1, name.str().c_str());
                    
                    for(auto ss : d.bar_inverse_tnetwork[i][s]) {
                        if(d.adj[i][ss][t-1][s][t]) {
                            cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][ss][t-1][s], 1);
                        }
                    }
                    
                    auto top_time = std::min(t + d.min_travel_time[i][s] + 1, d.ni + 1);
                    for(auto tt = t; tt < top_time; tt++) {
                        for(auto ss : d.bar_tnetwork[i][s]) {
                            if(d.adj[i][s][tt][ss][tt+1]) {
                                cst_min_travel_time[i][s][t].setLinearCoef(var_x[i][s][tt][ss], 1);
                            }
                        }
                    }
                }
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
                if(d.adj[i][s][t][d.ns+1][t+1]) {
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
                        if(d.adj[i][ss][t-1][s][t]) {
                            cst_visit_sa[i][s].setLinearCoef(var_x[i][ss][t-1][s], 1);
                            cst_sa_delay[i][s].setLinearCoef(var_x[i][ss][t-1][s], t);
                        }
                    }
                }
            }
            
            model.add(cst_visit_sa[i]);
            model.add(cst_sa_delay[i]);
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
                        if(d.adj[i][ss][t-1][s][t]) {
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
    model.add(cst_wt_delay_1);

    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Setting objective function coefficients..." << std::endl;
    t_start = high_resolution_clock::now();

    // ...

    model.add(obj);
    model.add(positive_obj);

    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);

    std::cout << "\t" << time_span.count() << " seconds" << std::endl;

    IloCplex cplex(model);

    cplex.exportModel("model.lp");

    cplex.solve();

//     if(!cplex.solve()) {
//         std::cout << "Cplex status: " << cplex.getStatus() << " " << cplex.getCplexStatus() << std::endl;
//         return;
//     }
//
//     std::cout << "Cplex optimal value: " << cplex.getObjValue() << std::endl;
//
//     auto x = int_matrix_5d(d.nt, int_matrix_4d(d.ns + 2, int_matrix_3d(d.ni + 2, int_matrix(d.ns + 2, ivec(d.ni + 2, 0)))));
//     for(auto i = 0; i < d.nt; i++) {
//         auto dv = cplex.getValue(var_d[i]);
//         if(dv > eps) {
//             std::cout << "d[" << i << "]: " << dv << std::endl;
//         }
//         for(auto s1 = 0; s1 < d.ns + 2; s1++) {
//             if(d.accessible[i][s1]) {
//                 if(d.sa[i][s1]) {
//                     auto ev = cplex.getValue(var_e[i][s1]);
//                     if(ev > eps) {
//                         std::cout << "e[" << i << "][" << s1 << "]: " << ev << std::endl;
//                     }
//                 }
//                 for(auto t1 = 0; t1 < d.ni + 2; t1++) {
//                     if(d.v[i][s1][t1]) {
//                         for(const auto& s2 : d.tnetwork[i][s1]) {
//                             if(d.accessible[i][s2]) {
//                                 for(auto t2 = 0; t2 < d.ni + 2; t2++) {
//                                     if(d.v[i][s2][t2] && d.adj[i][s1][t1][s2][t2]) {
//                                         auto xv = cplex.getValue(var_x[i][s1][t1][s2][t2]);
//                                         if(xv > eps) {
//                                             x[i][s1][t1][s2][t2] = 1;
//                                         }
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }
//
//     for(auto i = 0; i < d.nt; i++) {
//         std::cout << std::endl << "Train #" << i << std::endl;
//         auto current_seg = 0, current_time = 0;
//
//         restart:
//         while(current_seg != d.ns + 1) {
//             for(auto s = 0; s < d.ns + 2; s++) {
//                 for(auto t = 0; t < d.ni + 2; t++) {
//                     if(x[i][current_seg][current_time][s][t] == 1) {
//                         if(s != current_seg) {
//                             if(s != d.ns + 1) {
//                                 std::cout << "\tReaches segment (" << d.seg_w_ext[s] << ", " << d.seg_e_ext[s] << ")[" << d.seg_type[s] << "] at time " << t << std::endl;
//                             }
//                             current_seg = s;
//                         }
//                         current_time = t;
//                         goto restart;
//                     }
//                 }
//             }
//         }
//     }
    
    env.end();
}
