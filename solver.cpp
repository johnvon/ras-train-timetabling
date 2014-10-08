#include <solver.h>

#include <algorithm>
#include <chrono>
#include <sstream>

auto solver::real_node(auto s, auto t) const {
    return (s > 0 && s <= d.ns && t > 0 && t <= d.ni);
}


void solver::solve() const {
    using namespace std::chrono;
    high_resolution_clock::time_point t_start, t_end;
    duration<double> time_span;
    
    IloEnv env;
    IloModel model(env);
    
    var_matrix_5d var_x(env, d.nt);
    var_vector var_d(env, d.nt);
    var_matrix_2d var_e(env, d.nt);
    
    std::stringstream name;
        
    std::cout << "Creating variables..." << std::endl;
    t_start = high_resolution_clock::now();
    
    for(auto i = 0; i < d.nt; i++) {
        var_x[i] = var_matrix_4d(env, d.ns + 2);
        
        name.str(""); name << "var_d_" << i; auto ub_d = std::max(d.tr_wt[i], d.ni - d.tr_wt[i]);
        var_d[i] = IloNumVar(env, 0.0, ub_d, IloNumVar::Int, name.str().c_str());
        
        var_e[i] = IloNumVarArray(env, d.ns + 2);
        
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1]) {
                if(d.sa[i][s1]) {
                    name.str(""); name << "var_e_" << i << "_" << s1; auto ub_e = std::max(d.sa_times[i][s1], d.ni - d.sa_times[i][s1]);
                    var_e[i][s1] = IloNumVar(env, 0.0, ub_e, IloNumVar::Int, name.str().c_str());
                }
                
                var_x[i][s1] = var_matrix_3d(env, d.ni + 2);
                
                for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                    if(d.v[i][s1][t1]) {
                        var_x[i][s1][t1] = var_matrix_2d(env, d.ns + 2);
                        
                        for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                            if(d.accessible[i][s2]) {
                                var_x[i][s1][t1][s2] = IloNumVarArray(env, d.ni + 2);
                                
                                for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                    if(d.v[i][s2][t2] && d.adj[i][s1][t1][s2][t2]) {
                                        name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t1 << "_" << s2 << "_" << t2;
                                        var_x[i][s1][t1][s2][t2] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, name.str().c_str());
                                    }
                                }
                            }
                        }
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
    cst_matrix_3d cst_min_travel_time(env, d.nt);
    cst_matrix_3d cst_headway_1(env, d.nt);
    cst_matrix_3d cst_headway_2(env, d.nt);
    cst_matrix_3d cst_headway_3(env, d.nt);
    
    std::cout << "Adding constraints..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1.0, 1.0, name.str().c_str());
        
        name.str(""); name << "cst_enter_tau_" << i;
        cst_enter_tau[i] = IloRange(env, 1.0, 1.0, name.str().c_str());
        
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        
        if(d.tr_sa[i]) {
            cst_visit_sa[i] = cst_vector(env, d.ns + 2);
            cst_sa_delay[i] = cst_vector(env, d.ns + 2);
        }
        
        cst_min_travel_time[i] = cst_matrix_2d(env, d.ns + 2);
        
        cst_headway_1[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_2[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_3[i] = cst_matrix_2d(env, d.ns + 2);
        
        name.str(""); name << "cst_wt_delay_1_" << i;
        auto W1 = d.tr_wt[i] - d.wt_tw_left;
        cst_wt_delay_1[i] = IloRange(env, W1, IloInfinity, name.str().c_str());
        
        name.str(""); name << "cst_wt_delay_2_" << i;
        auto W2 = d.tr_wt[i] + d.wt_tw_right;
        cst_wt_delay_2[i] = IloRange(env, -IloInfinity, W2, name.str().c_str());

        cst_wt_delay_1[i].setLinearCoef(var_d[i], 1.0);
        
        cst_wt_delay_2[i].setLinearCoef(var_d[i], -1.0);
        
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {            
            if(s1 > 0 && s1 <= d.ns) {
                if(i == 0) {
                    // Here we initialise constraints that don't have "for all i"
                    if(d.accessible_by_someone[s1]) {
                        cst_max_one_train[s1] = cst_vector(env, d.ni + 2);
                    }
                }
                
                cst_flow[i][s1] = cst_vector(env, d.ni + 2);
                
                cst_min_travel_time[i][s1] = cst_vector(env, d.ni + 2);
                
                cst_headway_1[i][s1] = cst_vector(env, d.ni + 2);
                cst_headway_2[i][s1] = cst_vector(env, d.ni + 2);
                cst_headway_3[i][s1] = cst_vector(env, d.ni + 2);
                
                if(d.sa[i][s1]) {
                    name.str(""); name << "cst_visit_sa_" << i << "_" << s1;
                    cst_visit_sa[i][s1] = IloRange(env, 1.0, IloInfinity, name.str().c_str());
                    
                    auto ais = d.sa_times[i][s1] + d.sa_tw_right - 1;
                    name.str(""); name << "cst_sa_delay_" << i << "_" << s1;
                    cst_sa_delay[i][s1] = IloRange(env, -IloInfinity, ais, name.str().c_str());
                    
                    cst_sa_delay[i][s1].setLinearCoef(var_e[i][s1], -1.0);
                }
                
                for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                    if(real_node(s1, t1)) {
                        if(i == 0) {
                            // Here we initialise constraints that don't have "for all i"
                            if(d.v_for_someone[s1][t1]) {
                                name.str(""); name << "cst_max_one_train_" << s1 << "_" << t1;
                                cst_max_one_train[s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                            }
                        }
                    
                        if(d.accessible[i][s1] && d.v[i][s1][t1]) {
                            name.str(""); name << "cst_flow_" << i << "_" << s1 << "_" << t1;
                            cst_flow[i][s1][t1] = IloRange(env, 0.0, 0.0, name.str().c_str());
                            
                            name.str(""); name << "cst_min_travel_time_" << i << "_" << s1 << "_" << t1;
                            cst_min_travel_time[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                            
                            name.str(""); name << "cst_headway_1_" << i << "_" << s1 << "_" << t1;
                            cst_headway_1[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                            
                            name.str(""); name << "cst_headway_2_" << i << "_" << s1 << "_" << t1;
                            cst_headway_1[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                            
                            name.str(""); name << "cst_headway_3_" << i << "_" << s1 << "_" << t1;
                            cst_headway_3[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        
                            if(d.adj[i][0][0][s1][t1]) {
                                cst_exit_sigma[i].setLinearCoef(var_x[i][0][0][s1][t1], 1.0);
                            } 
                
                            if(d.adj[i][s1][t1][d.ns + 1][d.ni + 1]) {
                                cst_enter_tau[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], 1.0);
                                cst_wt_delay_1[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], t1);
                                cst_wt_delay_2[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], t1);
                            }
                        
                            for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                                if(d.accessible[i][s2]) {
                                    for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                        if(d.v[i][s2][t2]) {
                                            if(d.adj[i][s1][t1][s2][t2]) {
                                                cst_flow[i][s1][t1].setLinearCoef(var_x[i][s1][t1][s2][t2], -1.0);
                                                
                                                if(s2 != s1) {
                                                    cst_headway_3[i][s1][t1].setLinearCoef(var_x[i][s1][t1][s2][t2], 1.0);
                                                }
                                            }
                                            if(d.adj[i][s2][t2][s1][t1]) {
                                                cst_max_one_train[s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                cst_flow[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                
                                                if(s2 != s1) {
                                                    cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                    cst_headway_1[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                    cst_headway_2[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                }
                                                
                                                if(d.tr_sa[i] && d.sa[i][s1]) {
                                                    cst_visit_sa[i][s1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                    
                                                    if(s2 != s1) {
                                                        cst_sa_delay[i][s1].setLinearCoef(var_x[i][s2][t2][s1][t1], t1);
                                                    }
                                                }
                                            }
                                        } // If v(i, s2, t2)
                                    } // For t2
                                } // If accessible(i, s2)
                            } // For s2
                        } // If accessible(i, s1) and v(i, s1, t1)
                    } // If real_node(s1, t1)
                    
                    for(auto tt = t1; tt < t1 + d.min_travel_time[i][s1] - 1; tt++) {
                        if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                            if(real_node(s1, tt) && d.v[i][s1][tt]) {
                                for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                                    if(s2 != s1 && d.accessible[i][s2]) {
                                        for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                            if(d.v[i][s2][t2] && d.adj[i][s1][tt][s2][t2]) {
                                                cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s1][tt][s2][t2], 1.0);
                                            }
                                        }
                                    }
                                } // For s2
                            } // If real_node(s1, tt) and v(i, s1, tt)
                        } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                    } // For tt
                    
                    for(auto j = 0; j < d.nt; j++) {
                        if(j != i) {
                            for(auto tt = t1 - d.headway; tt < t1; tt++) {
                                if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                    if(real_node(s1, tt) && d.accessible[j][s1] && d.v[j][s1][tt]) {
                                        for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                                            if(s2 != s1) {
                                                for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                                    if(d.v[j][s2][t2] && d.adj[j][s2][t2][s1][tt]) {
                                                        cst_headway_1[i][s1][t1].setLinearCoef(var_x[j][s2][t2][s1][tt], 1.0);
                                                    }
                                                    if(d.v[j][s2][t2] && d.adj[j][s1][tt][s2][t2]) {
                                                        cst_headway_2[i][s1][t1].setLinearCoef(var_x[j][s1][tt][s2][t2], 1.0);
                                                    }
                                                }
                                            }
                                        } // For s2
                                    } // If real_node(s1, tt) and accessible(j, s1) and v(j, s1, tt)
                                } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                            } // For tt
                            
                            for(auto tt = t1 + 1; tt <= t1 + d.headway; tt++) {
                                if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                    if(real_node(s1, tt) && d.accessible[j][s1] && d.v[j][s1][tt]) {
                                        for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                                            if(s2 != s1) {
                                                for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                                    if(d.v[j][s2][t2] && d.adj[j][s2][t2][s1][tt]) {
                                                        cst_headway_3[i][s1][t1].setLinearCoef(var_x[j][s2][t2][s1][tt], 1.0);
                                                    }
                                                }
                                            }
                                        } // For s2
                                    } // If real_node(s1, tt) and accessible(j, s1) and v(j, s1, tt)
                                } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                            } // For tt
                        } // If j != i
                    } // For j
                    
                } // For t1
                
                if(i == d.nt - 1) {
                    // Here we add constraints that don't have "for all i"
                    model.add(cst_max_one_train[s1]);
                }
                
                if(d.tr_sa[i]) {
                    model.add(cst_visit_sa[i]);
                    model.add(cst_sa_delay[i]);
                }
                
                model.add(cst_flow[i][s1]);
                model.add(cst_min_travel_time[i][s1]);
                model.add(cst_headway_1[i][s1]);
                model.add(cst_headway_2[i][s1]);
                model.add(cst_headway_3[i][s1]);
            } // If s1 >= 1 and s1 <= ns
        } // For s1
    } // For i
    
    model.add(cst_exit_sigma);
    model.add(cst_enter_tau);
    model.add(cst_wt_delay_1);
    model.add(cst_wt_delay_2);
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    IloCplex cplex(model);
    
    cplex.exportModel("model.lp");
}