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
    
    double eps = 1e-6;
    
    IloEnv env;
    IloModel model(env);
    
    var_matrix_5d var_x(env, d.nt);
    var_vector var_d(env, d.nt);
    var_matrix_2d var_e(env, d.nt);
    
    std::stringstream name;
        
    std::cout << "Creating variables..." << std::endl;
    t_start = high_resolution_clock::now();
    
    for(auto i = 0; i < d.nt; i++) {
        std::cout << "\tTrain #" << i << std::endl;
        
        var_x[i] = var_matrix_4d(env, d.ns + 2);
        
        name.str(""); name << "var_d_" << i; auto ub_d = std::max(d.tr_wt[i], d.ni - d.tr_wt[i]);
        var_d[i] = IloNumVar(env, 0.0, ub_d, IloNumVar::Int, name.str().c_str());
        
        var_e[i] = IloNumVarArray(env, d.ns + 2);

        var_x[i][0] = var_matrix_3d(env, 1);
        var_x[i][0][0] = var_matrix_2d(env, d.ns + 2);
        // Sigma-to-node arcs
        for(const auto& s2 : d.tr_orig_seg[i]) {
            if(d.accessible[i][s2]) {
                var_x[i][0][0][s2] = IloNumVarArray(env, d.ni + 2);
                for(auto t2 = 1; t2 < d.ni + 2; t2++) {
                    if(d.v[i][s2][t2] && d.adj[i][0][0][s2][t2]) {
                        name.str(""); name << "var_x_" << i << "_sigma_" << s2 << "_" << t2;
                        var_x[i][0][0][s2][t2] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, name.str().c_str());
                    }
                }
            }
        }
        
        // Node-to-node arcs
        for(auto s1 = 1; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1]) {
                if(d.sa[i][s1]) {
                    name.str(""); name << "var_e_" << i << "_" << s1; auto ub_e = std::max(d.sa_times[i][s1], d.ni - d.sa_times[i][s1]);
                    var_e[i][s1] = IloNumVar(env, 0.0, ub_e, IloNumVar::Int, name.str().c_str());
                }
                
                var_x[i][s1] = var_matrix_3d(env, d.ni + 2);
                
                for(auto t1 = 1; t1 < d.ni + 1; t1++) {
                    if(d.v[i][s1][t1]) {
                        var_x[i][s1][t1] = var_matrix_2d(env, d.ns + 2);
                        
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2]) {
                                var_x[i][s1][t1][s2] = IloNumVarArray(env, d.ni + 2);
                                
                                if(d.v[i][s2][t1+1] && d.adj[i][s1][t1][s2][t1+1]) {
                                    name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t1 << "_" << s2 << "_" << (t1+1);
                                    var_x[i][s1][t1][s2][t1+1] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, name.str().c_str());
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Node-to-tau arcs
        for(const auto& s1 : d.tr_dest_seg[i]) {
            if(d.accessible[i][s1]) {
                for(auto t1 = 0; t1 < d.ni + 1; t1++) {
                    if(d.v[i][s1][t1] && d.adj[i][s1][t1][d.ns + 1][d.ni + 1]) {
                        var_x[i][s1][t1][d.ns + 1] = IloNumVarArray(env, d.ni + 2);
                        name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t1 << "_tau";
                        var_x[i][s1][t1][d.ns + 1][d.ni + 1] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, name.str().c_str());
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

    std::cout << "Adding constraints..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        std::cout << "\tTrain #" << i << std::endl;
        
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1.0, 1.0, name.str().c_str());
        
        name.str(""); name << "cst_enter_tau_" << i;
        cst_enter_tau[i] = IloRange(env, 1.0, 1.0, name.str().c_str());
        
        cst_flow[i] = cst_matrix_2d(env, d.ns + 2);
        
        if(d.tr_sa[i]) {
            cst_visit_sa[i] = cst_vector(env, d.ns + 2);
            cst_sa_delay[i] = cst_vector(env, d.ns + 2);
        }
        
        if(use_alt_min_travel_time) {
            cst_alt_min_travel_time[i] = cst_vector(env, d.ns + 2);
        } else {
            cst_min_travel_time[i] = cst_matrix_2d(env, d.ns + 2);
        }
        
        cst_headway_1[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_2[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_3[i] = cst_matrix_2d(env, d.ns + 2);
        cst_headway_4[i] = cst_matrix_2d(env, d.ns + 2);
        
        cst_cant_stop[i] = cst_matrix_2d(env, d.ns + 2);
        
        cst_siding[i] = cst_matrix_2d(env, d.ns + 2);
        
        if(d.tr_heavy[i]) {
            cst_heavy[i] = cst_matrix_2d(env, d.ns + 2);
        }
        
        name.str(""); name << "cst_wt_delay_1_" << i;
        auto W1 = d.tr_wt[i] - d.wt_tw_left;
        cst_wt_delay_1[i] = IloRange(env, W1, IloInfinity, name.str().c_str());
        
        name.str(""); name << "cst_wt_delay_2_" << i;
        auto W2 = d.tr_wt[i] + d.wt_tw_right;
        cst_wt_delay_2[i] = IloRange(env, -IloInfinity, W2, name.str().c_str());

        cst_wt_delay_1[i].setLinearCoef(var_d[i], 1.0);
        
        cst_wt_delay_2[i].setLinearCoef(var_d[i], -1.0);
        
        for(auto s1 = 1; s1 < d.ns + 1; s1++) {
            std::cout << "\t\tSegment #" << s1 << std::endl;
            
            if(i == 0) {
                // Here we initialise constraints that don't have "for all i"
                if(d.accessible_by_someone[s1]) {
                    cst_max_one_train[s1] = cst_vector(env, d.ni + 2);
                }
            }
            
            cst_flow[i][s1] = cst_vector(env, d.ni + 2);
            
            if(use_alt_min_travel_time) {
                name.str(""); name << "cst_alt_min_travel_time_" << i << "_" << s1;
                cst_alt_min_travel_time[i][s1] = IloRange(env, -IloInfinity, 0, name.str().c_str());
            } else {
                cst_min_travel_time[i][s1] = cst_vector(env, d.ni + 2);
            }
            
            cst_headway_1[i][s1] = cst_vector(env, d.ni + 2);
            cst_headway_2[i][s1] = cst_vector(env, d.ni + 2);
            cst_headway_3[i][s1] = cst_vector(env, d.ni + 2);
            cst_headway_4[i][s1] = cst_vector(env, d.ni + 2);
            
            if(d.seg_type[s1] == 'X') {
                cst_cant_stop[i][s1] = cst_vector(env, d.ni + 2);
            }
            
            if(d.sa[i][s1]) {
                name.str(""); name << "cst_visit_sa_" << i << "_" << s1;
                cst_visit_sa[i][s1] = IloRange(env, 1.0, IloInfinity, name.str().c_str());
                
                auto ais = d.sa_times[i][s1] + d.sa_tw_right - 1;
                name.str(""); name << "cst_sa_delay_" << i << "_" << s1;
                cst_sa_delay[i][s1] = IloRange(env, -IloInfinity, ais, name.str().c_str());
                
                cst_sa_delay[i][s1].setLinearCoef(var_e[i][s1], -1.0);
            }
            
            if(d.seg_type[s1] == 'S') {
                cst_siding[i][s1] = cst_vector(env, d.ni + 2);
                if(d.tr_heavy[i]) {
                    cst_heavy[i][s1] = cst_vector(env, d.ni + 2);
                }
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
                        
                        if(!use_alt_min_travel_time) {
                            name.str(""); name << "cst_min_travel_time_" << i << "_" << s1 << "_" << t1;
                            cst_min_travel_time[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        }
                        
                        name.str(""); name << "cst_headway_1_" << i << "_" << s1 << "_" << t1;
                        cst_headway_1[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        
                        name.str(""); name << "cst_headway_2_" << i << "_" << s1 << "_" << t1;
                        cst_headway_2[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        
                        name.str(""); name << "cst_headway_3_" << i << "_" << s1 << "_" << t1;
                        cst_headway_3[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        
                        name.str(""); name << "cst_headway_4_" << i << "_" << s1 << "_" << t1;
                        cst_headway_4[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                    
                        if(d.adj[i][0][0][s1][t1]) {
                            cst_exit_sigma[i].setLinearCoef(var_x[i][0][0][s1][t1], 1.0);
                        }
            
                        if(d.adj[i][s1][t1][d.ns + 1][d.ni + 1]) {
                            cst_enter_tau[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], 1.0);
                            cst_wt_delay_1[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], t1);
                            cst_wt_delay_2[i].setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], t1);
                        }
                        
                        if(d.seg_type[s1] == 'S') {
                            name.str(""); name << "cst_siding_" << i << "_" << s1 << "_" << t1;
                            cst_siding[i][s1][t1] = IloRange(env, 0.0, IloInfinity, name.str().c_str());
                            
                            if(d.tr_heavy[i]) {
                                name.str(""); name << "cst_heavy_" << i << "_" << s1 << "_" << t1;
                                cst_heavy[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                            }
                        }
                        
                        if(d.seg_type[s1] == 'X') {
                            name.str(""); name << "cst_cant_stop_" << i << "_" << s1 << "_" << t1;
                            cst_cant_stop[i][s1][t1] = IloRange(env, -IloInfinity, 1.0, name.str().c_str());
                        }
                        
                        if(use_alt_min_travel_time) {
                            if(d.v[i][s1][t1+1]) {
                                cst_alt_min_travel_time[i][s1].setLinearCoef(var_x[i][s1][t1][s1][t1+1], -1.0);
                            }
                        }
                        
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2]) {
                                auto times = std::vector<int>({0, t1 - 1, t1 + 1, d.ni + 1});
                                times.erase(std::unique(times.begin(), times.end()), times.end());
                                for(auto t2 : times) {
                                    if(d.v[i][s2][t2]) {
                                        if(d.adj[i][s1][t1][s2][t2]) {
                                            cst_flow[i][s1][t1].setLinearCoef(var_x[i][s1][t1][s2][t2], -1.0);
                                            
                                            if(s2 != s1) {
                                                cst_headway_3[i][s1][t1].setLinearCoef(var_x[i][s1][t1][s2][t2], 1.0);
                                                cst_headway_4[i][s1][t1].setLinearCoef(var_x[i][s1][t1][s2][t2], 1.0);
                                            }
                                        }
                                        if(d.adj[i][s2][t2][s1][t1]) {
                                            cst_max_one_train[s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                            cst_flow[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                            
                                            if(s2 != s1) {
                                                if(use_alt_min_travel_time) {
                                                    cst_alt_min_travel_time[i][s1].setLinearCoef(var_x[i][s2][t2][s1][t1], d.min_travel_time[i][s1] - 1);
                                                } else {
                                                    cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                }
                                                
                                                cst_headway_1[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                cst_headway_2[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                
                                                if(d.seg_type[s1] == 'S') {
                                                    cst_siding[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], -1.0);
                                                    
                                                    if(d.tr_heavy[i]) {
                                                        cst_heavy[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                    }
                                                }
                                                
                                                if(d.seg_type[s1] == 'X') {
                                                    cst_cant_stop[i][s1][t1].setLinearCoef(var_x[i][s2][t2][s1][t1], 1.0);
                                                }
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
                
                if(!use_alt_min_travel_time) {
                    if(use_max_travel_time) {
                        for(auto tt = t1 + d.max_travel_time[i][s1] + 1; tt < d.ni + 2; tt++) {
                            if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                if(real_node(s1, tt) && d.v[i][s1][tt]) {
                                    for(const auto& s2 : d.tnetwork[i][s1]) {
                                        if(s2 != s1 && d.accessible[i][s2]) {
                                            if(d.v[i][s2][tt+1] && d.adj[i][s1][tt][s2][tt+1]) {
                                                cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s1][tt][s2][tt+1], 1.0);
                                            }
                                        }
                                        if(tt < d.ni && d.adj[i][s1][tt][d.ns + 1][d.ni + 1]) {
                                            cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s1][tt][d.ns + 1][d.ni + 1], 1.0);
                                        }
                                    } // For s2
                                } // If real_node(s1, tt) and v(i, s1, tt)
                            } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                        } // For tt
                    }
                    
                    for(auto tt = t1; tt < t1 + d.min_travel_time[i][s1] - 1; tt++) {
                        if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                            if(real_node(s1, tt) && d.v[i][s1][tt]) {
                                for(const auto& s2 : d.tnetwork[i][s1]) {
                                    if(s2 != s1 && d.accessible[i][s2]) {
                                        if(d.v[i][s2][tt+1] && d.adj[i][s1][tt][s2][tt+1]) {
                                            cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s1][tt][s2][tt+1], 1.0);
                                        }
                                    }
                                } // For s2
                                if(tt < d.ni && d.adj[i][s1][tt][d.ns + 1][d.ni + 1]) {
                                    cst_min_travel_time[i][s1][t1].setLinearCoef(var_x[i][s1][tt][d.ns + 1][d.ni + 1], 1.0);
                                }
                            } // If real_node(s1, tt) and v(i, s1, tt)
                        } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                    } // For tt
                }
                
                if(d.seg_type[s1] == 'X') {
                    for(auto tt = t1 + d.min_travel_time[i][s1]; tt < d.ni + 2; tt++) {
                        if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                            if(real_node(s1, tt) && d.v[i][s1][tt]) {
                                for(const auto& s2 : d.tnetwork[i][s1]) {
                                    if(s2 != s1 && d.accessible[i][s2]) {
                                        if(d.v[i][s2][tt+1] && d.adj[i][s1][tt][s2][tt+1]) {
                                            cst_cant_stop[i][s1][t1].setLinearCoef(var_x[i][s1][tt][s2][tt+1], 1.0);
                                        }
                                    }
                                } // For s2
                                if(tt < d.ni && d.adj[i][s1][tt][d.ns + 1][d.ni + 1]) {
                                    cst_cant_stop[i][s1][t1].setLinearCoef(var_x[i][s1][tt][d.ns + 1][d.ni + 1], 1.0);
                                }
                            } // If real_node(s1, tt) and v(i, s1, tt)
                        } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                    } // For tt
                } // If seg_type(s1) is X or T
                
                for(auto j = 0; j < d.nt; j++) {
                    if(j != i) {
                        for(auto tt = t1 - d.headway; tt < t1; tt++) {
                            if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                if(real_node(s1, tt) && d.accessible[j][s1] && d.v[j][s1][tt]) {
                                    for(const auto& s2 : d.tnetwork[i][s1]) {
                                        if(s2 != s1) {
                                            if(d.v[j][s2][tt-1] && d.adj[j][s2][tt-1][s1][tt]) {
                                                cst_headway_1[i][s1][t1].setLinearCoef(var_x[j][s2][tt-1][s1][tt], 1.0);
                                            }
                                            if(d.v[j][s2][tt+1] && d.adj[j][s1][tt][s2][tt+1]) {
                                                cst_headway_2[i][s1][t1].setLinearCoef(var_x[j][s1][tt][s2][tt+1], 1.0);
                                            }
                                        }
                                    } // For s2
                                    if(tt > 1 && d.adj[j][0][0][s1][tt]) {
                                        cst_headway_1[i][s1][t1].setLinearCoef(var_x[j][0][0][s1][tt], 1.0);
                                    }
                                    if(tt < d.ni && d.adj[j][s1][tt][d.ns + 1][d.ni + 1]) {
                                        cst_headway_2[i][s1][t1].setLinearCoef(var_x[j][s1][tt][d.ns + 1][d.ni + 1], 1.0);
                                    }
                                 } // If real_node(s1, tt) and accessible(j, s1) and v(j, s1, tt)
                            } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                        } // For tt
                        
                        for(auto tt = t1 + 1; tt <= t1 + d.headway; tt++) {
                            if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                if(real_node(s1, tt) && d.accessible[j][s1] && d.v[j][s1][tt]) {
                                    for(const auto& s2 : d.tnetwork[i][s1]) {
                                        if(s2 != s1) {
                                            if(d.v[j][s2][tt-1] && d.adj[j][s2][tt-1][s1][tt]) {
                                                cst_headway_3[i][s1][t1].setLinearCoef(var_x[j][s2][tt-1][s1][tt], 1.0);
                                            }
                                            if(d.v[j][s2][tt+1] && d.adj[j][s1][tt][s2][tt+1]) {
                                                cst_headway_4[i][s1][t1].setLinearCoef(var_x[j][s1][tt][s2][tt+1], 1.0);
                                            }
                                        }
                                    } // For s2
                                    if(tt > 1 && d.adj[j][0][0][s1][tt]) {
                                        cst_headway_3[i][s1][t1].setLinearCoef(var_x[j][0][0][s1][tt], 1.0);
                                    }
                                    if(tt < d.ni && d.adj[j][s1][tt][d.ns + 1][d.ni + 1]) {
                                        cst_headway_4[i][s1][t1].setLinearCoef(var_x[j][s1][tt][d.ns + 1][d.ni + 1], 1.0);
                                    }
                                } // If real_node(s1, tt) and accessible(j, s1) and v(j, s1, tt)
                            } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                        } // For tt
                        
                        for(auto tt = t1 - d.headway; tt <= t1 + d.headway; tt++) {
                            if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1] && d.seg_type[s1] == 'S') {
                                for(auto ss = 0; ss < d.ns + 2; ss++) {
                                    if(d.is_main[s1][ss] && real_node(ss, tt) && d.accessible[j][ss] && d.v[j][ss][tt]) {
                                        for(const auto& s2 : d.tnetwork[i][ss]) {
                                            if(s2 != ss) {
                                                if(d.v[j][s2][tt-1] && d.adj[j][s2][tt-1][ss][tt]) {
                                                    cst_siding[i][s1][t1].setLinearCoef(var_x[j][s2][tt-1][ss][tt], 1.0);
                                                }
                                            }
                                        } // For s2
                                        if(tt > 1 && d.adj[j][0][0][ss][tt]) {
                                            cst_siding[i][s1][t1].setLinearCoef(var_x[j][0][0][ss][tt], 1.0);
                                        }
                                    } // If is_main(s1, ss) and real_node(ss, tt) and accessible(j, ss) and v(j, ss, tt)
                                } // For ss
                            } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1) and seg_type(s1) == 'S'
                        } // For tt
                        
                        if(!d.tr_sa[j] && d.seg_type[s1] == 'S' && d.tr_heavy[i]) {
                            if(real_node(s1, t1) && d.accessible[i][s1] && d.v[i][s1][t1]) {
                                for(auto ss = 0; ss < d.ns + 2; ss++) {
                                    if(d.is_main[s1][ss] && real_node(ss, t1) && d.accessible[j][ss] && d.v[j][ss][t1]) {
                                        for(const auto& s2 : d.tnetwork[i][ss]) {
                                            if(s2 != ss) {
                                                if(d.v[j][s2][t1-1] && d.adj[j][s2][t1-1][ss][t1]) {
                                                    cst_heavy[i][s1][t1].setLinearCoef(var_x[j][s2][t1-1][ss][t1], 1.0);
                                                }
                                            }
                                        } // For s2
                                        if(t1 > 1 && d.adj[j][0][0][ss][t1]) {
                                            cst_heavy[i][s1][t1].setLinearCoef(var_x[j][0][0][ss][t1], 1.0);
                                        }
                                    } // If is_main(s1, ss) and real_node(ss, t1) and accessible(j, ss) and v(j, ss, t1)
                                } // For ss
                            } // If real_node(s1, t1) and accessible(i, s1) and v(i, s1, t1)
                        } // If !tr_sa(j) and seg_type(s1) == 'S' and tr_heavy(i)
                    } // If j != i
                } // For j
                
            } // For t1
            
            if(i == d.nt - 1) {
                // Here we add constraints that don't have "for all i"
                model.add(cst_max_one_train[s1]);
            }
            
            model.add(cst_flow[i][s1]);
            
            if(!use_alt_min_travel_time) {
                model.add(cst_min_travel_time[i][s1]);
            }
            
            model.add(cst_headway_1[i][s1]);
            model.add(cst_headway_2[i][s1]);
            model.add(cst_headway_3[i][s1]);
            model.add(cst_headway_4[i][s1]);
            
            if(d.seg_type[s1] == 'S') {
                model.add(cst_siding[i][s1]);
                if(d.tr_heavy[i]) {
                    model.add(cst_heavy[i][s1]);
                }
            }
            
            if(d.seg_type[s1] == 'X') {
                model.add(cst_cant_stop[i][s1]);
            }
        } // For s1
        
        if(d.tr_sa[i]) {
            model.add(cst_visit_sa[i]);
            model.add(cst_sa_delay[i]);
        }
        
        if(use_alt_min_travel_time) {
            model.add(cst_alt_min_travel_time[i]);
        }
    } // For i
    
    model.add(cst_exit_sigma);
    model.add(cst_enter_tau);
    model.add(cst_wt_delay_1);
    model.add(cst_wt_delay_2);
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Calculating objective function coefficients..." << std::endl;
    t_start = high_resolution_clock::now();
    
    double_matrix_5d coeff(d.nt, double_matrix_4d(d.ns + 2, double_matrix_3d(d.ni + 2, double_matrix(d.ns + 2, dvec(d.ni + 2, 0.0)))));
    IloRange positive_obj(env, 0, IloInfinity, "positive_obj");
    
    for(auto i = 0; i < d.nt; i++) {
        obj.setLinearCoef(var_d[i], d.wt_price);
        positive_obj.setLinearCoef(var_d[i], d.wt_price);
        
        for(auto s1 = 1; s1 < d.ns + 1; s1++) {
            if(d.accessible[i][s1]) {
                if(d.tr_sa[i] && d.sa[i][s1]) {
                    obj.setLinearCoef(var_e[i][s1], d.sa_price);
                    positive_obj.setLinearCoef(var_e[i][s1], d.sa_price);
                }
                
                for(auto t1 = 1; t1 < d.ni + 1; t1++) {
                    if(d.v[i][s1][t1]) {
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2]) {
                                if(d.v[i][s2][t1+1] && s2 != s1 && d.adj[i][s1][t1][s2][t1+1]) {
                                    coeff[i][s1][t1][s2][t1+1] += d.delay_price[d.tr_class[i]] * t1;
                                }
                                if(d.v[i][s2][t1-1] && s2 != s1 && d.adj[i][s2][t1-1][s1][t1]) {
                                    coeff[i][s2][t1-1][s1][t1] -= d.delay_price[d.tr_class[i]] * (t1 + d.min_travel_time[i][s1] - 1);
                                }
                            }
                        }
                        if(t1 < d.ni && d.adj[i][s1][t1][d.ns + 1][d.ni + 1]) {
                            coeff[i][s1][t1][d.ns + 1][d.ni + 1] += d.delay_price[d.tr_class[i]] * t1;
                        }
                        if(t1 > 1 && d.adj[i][0][0][s1][t1]) {
                            coeff[i][0][0][s1][t1] -= d.delay_price[d.tr_class[i]] * (t1 + d.min_travel_time[i][s1] - 1);
                        }
                    }
                }
            }
        }
        
        for(auto s1 = 1; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1]) {
                for(auto t1 = 1; t1 < d.ni + 2; t1++) {
                    if(d.v[i][s1][t1]) {
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2] && d.v[i][s2][t1-1] && d.adj[i][s2][t1-1][s1][t1]) {
                                auto eta = ((d.tr_westbound[i] && !d.seg_westbound[s1]) || (d.tr_eastbound[i] && !d.seg_eastbound[s1]));
                                if(eta) {
                                    coeff[i][s2][t1-1][s1][t1] += d.unpreferred_price;
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
    
    std::cout << "Setting objective function coefficients..." << std::endl;
    t_start = high_resolution_clock::now();
        
    for(auto i = 0; i < d.nt; i++) {
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1]) {
                for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                    if(d.v[i][s1][t1]) {
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2]) {
                                if(d.v[i][s2][t1+1] && d.adj[i][s1][t1][s2][t1+1] && std::abs(coeff[i][s1][t1][s2][t1+1]) > eps) {
                                    obj.setLinearCoef(var_x[i][s1][t1][s2][t1+1], coeff[i][s1][t1][s2][t1+1]);
                                    positive_obj.setLinearCoef(var_x[i][s1][t1][s2][t1+1], coeff[i][s1][t1][s2][t1+1]);
                                }
                                if(d.v[i][s2][t1-1] && d.adj[i][s2][t1-1][s1][t1] && std::abs(coeff[i][s2][t1-1][s1][t1]) > eps) {
                                    obj.setLinearCoef(var_x[i][s2][t1-1][s1][t1], coeff[i][s2][t1-1][s1][t1]);
                                    positive_obj.setLinearCoef(var_x[i][s2][t1-1][s1][t1], coeff[i][s2][t1-1][s1][t1]);
                                }
                            }
                        }
                        if(t1 < d.ni && d.adj[i][s1][t1][d.ns + 1][d.ni + 1] && std::abs(coeff[i][s1][t1][d.ns + 1][d.ni + 1]) > eps) {
                            obj.setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], coeff[i][s1][t1][d.ns + 1][d.ni + 1]);
                            positive_obj.setLinearCoef(var_x[i][s1][t1][d.ns + 1][d.ni + 1], coeff[i][s1][t1][d.ns + 1][d.ni + 1]);
                        }
                        if(t1 > 1 && d.adj[i][0][0][s1][t1] && std::abs(coeff[i][0][0][s1][t1]) > eps) {
                            obj.setLinearCoef(var_x[i][0][0][s1][t1], coeff[i][0][0][s1][t1]);
                            positive_obj.setLinearCoef(var_x[i][0][0][s1][t1], coeff[i][0][0][s1][t1]);
                        }
                    }
                }
            }
        }
    }
    
    model.add(obj);
    model.add(positive_obj);
    
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
    
    auto x = int_matrix_5d(d.nt, int_matrix_4d(d.ns + 2, int_matrix_3d(d.ni + 2, int_matrix(d.ns + 2, ivec(d.ni + 2, 0)))));
    for(auto i = 0; i < d.nt; i++) {
        auto dv = cplex.getValue(var_d[i]);
        if(dv > eps) {
            std::cout << "d[" << i << "]: " << dv << std::endl;
        }
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1]) {
                if(d.sa[i][s1]) {
                    auto ev = cplex.getValue(var_e[i][s1]);
                    if(ev > eps) {
                        std::cout << "e[" << i << "][" << s1 << "]: " << ev << std::endl;
                    }
                }
                for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                    if(d.v[i][s1][t1]) {
                        for(const auto& s2 : d.tnetwork[i][s1]) {
                            if(d.accessible[i][s2]) {
                                for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                    if(d.v[i][s2][t2] && d.adj[i][s1][t1][s2][t2]) {
                                        auto xv = cplex.getValue(var_x[i][s1][t1][s2][t2]);
                                        if(xv > eps) {
                                            x[i][s1][t1][s2][t2] = 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    for(auto i = 0; i < d.nt; i++) {
        std::cout << std::endl << "Train #" << i << std::endl;
        auto current_seg = 0, current_time = 0;

        restart:
        while(current_seg != d.ns + 1) {
            for(auto s = 0; s < d.ns + 2; s++) {
                for(auto t = 0; t < d.ni + 2; t++) {
                    if(x[i][current_seg][current_time][s][t] == 1) {
                        if(s != current_seg) {
                            if(s != d.ns + 1) {
                                std::cout << "\tReaches segment (" << d.seg_w_ext[s] << ", " << d.seg_e_ext[s] << ")[" << d.seg_type[s] << "] at time " << t << std::endl;
                            }
                            current_seg = s;
                        }
                        current_time = t;
                        goto restart;
                    }
                }
            }
        }
    }
    
    env.end();
}
