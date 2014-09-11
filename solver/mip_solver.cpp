#include <solver/mip_solver.h>

#include <ilcplex/ilocplex.h>

#include <chrono>

void MipSolver::solve() const {
    using namespace std::chrono;
    high_resolution_clock::time_point t_start, t_end;
    duration<double> time_span;
    
    IloEnv env;
    IloModel model(env);
    
    IloNumVarArray var_x(env);
    IloNumVarArray var_d(env);
    IloNumVarArray var_e(env);
    
    IloRangeArray eq_exit_sigma(env);
    IloRangeArray eq_enter_tau(env);
    IloRangeArray eq_one_train(env);
    IloRangeArray eq_flow(env);
    IloRangeArray eq_ensure_sa_visit(env);
    IloRangeArray eq_wt_1(env);
    IloRangeArray eq_wt_2(env);
    IloRangeArray eq_sa_delay(env);
    IloRangeArray eq_min_time(env);
    // IloRangeArray eq_exact_time(env);
    IloRangeArray eq_headway1(env);
    IloRangeArray eq_headway2(env);
    IloRangeArray eq_headway3(env);
    IloRangeArray eq_headway4(env);
    IloRangeArray eq_can_take_siding(env);
    IloRangeArray eq_heavy_siding(env);
    IloRangeArray eq_positive_obj(env);
    
    IloObjective obj = IloMinimize(env);
    
    int nt {d->trains_number};
    int ns {d->segments_number};
    int ti {d->time_intervals};
    
    std::cout << "Creating model rows..." << std::endl;
    t_start = high_resolution_clock::now();
    #include <solver/mip_rows.raw.cpp>
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << "Created model rows... " << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Creating model columns..." << std::endl;
    t_start = high_resolution_clock::now();
    #include <solver/mip_columns.raw.cpp>
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << "Created model columns... " << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Adding constraints to model... ";
    t_start = high_resolution_clock::now();
    model.add(obj);
    model.add(eq_exit_sigma);
    model.add(eq_enter_tau);
    model.add(eq_one_train);
    model.add(eq_flow);
    model.add(eq_ensure_sa_visit);
    model.add(eq_wt_1);
    model.add(eq_wt_2);
    model.add(eq_sa_delay);
    model.add(eq_min_time);
    // model.add(eq_exact_time);
    model.add(eq_headway1);
    model.add(eq_headway2);
    model.add(eq_headway3);
    model.add(eq_headway4);
    model.add(eq_can_take_siding);
    model.add(eq_heavy_siding);
    model.add(eq_positive_obj);
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << time_span.count() << " seconds" << std::endl;
    
    std::cout << "Adding columns to model... ";
    t_start = high_resolution_clock::now();
    model.add(var_x);
    model.add(var_d);
    model.add(var_e);
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << time_span.count() << " seconds" << std::endl;
    
    IloCplex cplex(model);
    cplex.extract(model);
    
    std::cout << "Writing model to file... ";
    t_start = high_resolution_clock::now();
    cplex.exportModel("model.lp");
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << time_span.count() << " seconds" << std::endl;
    
    cplex.setParam(IloCplex::Threads, 4);
    
    std::cout << "Solving model..." << std::endl;
    if(cplex.solve()) {
        std::cout << "Model solved. Extracting solution values." << std::endl;
        
        IloNumArray nx(env); cplex.getValues(nx, var_x);
        IloNumArray nd(env); cplex.getValues(nd, var_d);
        IloNumArray ne(env); cplex.getValues(ne, var_e);
        
        std::cout << "The solution is the following:" << std::endl;
        
        auto solx = std::vector<std::vector<std::vector<int>>>(nt, std::vector<std::vector<int>>(ns, std::vector<int>(ti + 1, 0))); // ti + 1 because time starts from 1, not from 0
        int num_col {0};
        
        // Print x
        for(int i = 0; i < nt; i++) {
            for(std::tie(vi1, vi1_end) = vertices(graphs[i]->g); vi1 != vi1_end; ++vi1) {
                for(std::tie(vi2, vi2_end) = vertices(graphs[i]->g); vi2 != vi2_end; ++vi2) {
                    if(edge(*vi1, *vi2, graphs[i]->g).second) {
                        if(nx[num_col] > 0) {
                            const Node& n1 {graphs[i]->g[*vi1]};
                            const Node& n2 {graphs[i]->g[*vi2]};
                            
                            if(n1.s != nullptr && n2.s != nullptr) {
                                solx[i][n1.s->id][n1.t] = 1;
                                solx[i][n2.s->id][n2.t] = 1;
                            }
                            
                            std::cout << "x[" << i << "][" << n1.str() << "][" << n2.str() << "] = " << nx[num_col] << std::endl;
                        }
                        num_col++;
                    }
                }
            }
        }
        
        // Print d
        num_col = 0;
        for(int i = 0; i < nt; i++) {
            if(nd[num_col] > 0) {
                std::cout << "d[" << i << "] = " << nd[num_col] << std::endl;
            }
            num_col++;
        }
        
        // Print e
        num_col = 0;
        for(int i = 0; i < nt; i++) {
            if(d->trains[i].schedule_adherence) {
                std::vector<int> segs;
                std::vector<int> times;
                for(auto kv : d->trains[i].schedule) {
                    segs.push_back(kv.first);
                    times.push_back(kv.second);
                }
        
                for(int s = 0; s < d->trains[i].schedule.size(); s++) {
                    if(ne[num_col] > 0) {
                        std::cout << "e[" << i << "][" << segs[s] << "] = " << ne[num_col] << std::endl;
                    }
                    num_col++;
                }
            }
        }
        
        nx.end(); nd.end(); ne.end();
        
        std::vector<int> starting_seg(nt, -1);
        std::vector<int> ending_seg(nt, -1);
        std::vector<int> starting_time(nt, -1);
        std::vector<int> ending_time(nt, -1);
        
        for(int i = 0; i < nt; i++) {        
            for(int t = 1; t <= ti; t++) {
                bool first_segment {false};
            
                for(int s = 0; s < ns; s++) {
                    if(solx[i][s][t] > 0) {
                        first_segment = true;
                        starting_seg[i] = s;
                        break;
                    }
                }
            
                if(first_segment) {
                    starting_time[i] = t;
                    break;
                }
            }
        
            for(int t = ti; t > 1; t--) {
                bool last_segment {false};
            
                for(int s = 0; s < ns; s++) {
                    if(solx[i][s][t] > 0) {
                        last_segment = true;
                        ending_seg[i] = s;
                        break;
                    }
                }
            
                if(last_segment) {
                    ending_time[i] = t;
                    break;
                }
            }
            
            std::cout << "Train " << i << std::endl;
            std::cout << "\tStarted at segment " << starting_seg[i] << " at time " << starting_time[i] << std::endl;
            std::cout << "\tEnded at segment " << ending_seg[i] << " at time " << ending_time[i] << std::endl;
            std::cout << "\t\tEnd terminal TW: [" << (d->trains[i].terminal_wt - d->want_time_tw_start) << ", " << (d->trains[i].terminal_wt + d->want_time_tw_end) << "]" << std::endl;
            
            for(auto kv : d->trains[i].schedule) {
                int visit_time {-1};
                
                for(int t = ti; t > 1; t--) {
                    if(solx[i][kv.first][t] > 0) {
                        visit_time = t;
                        break;
                    }
                }
                
                std::cout << "\tVisited SA point through segment " << kv.first << " at time " << visit_time << std::endl;
                std::cout << "\t\tSA point optimal time: " << kv.second << std::endl;
                std::cout << "\t\tSA point max visit time: " << (kv.second + d->schedule_tw_end) << std::endl;
            }
            
            for(int s = 0; s < ns; s++) {
                int entry_time {-1};
                int exit_time {-1};
                
                for(int t = 1; t <= ti; t++) {
                    if(solx[i][s][i] > 0) {
                        entry_time = t;
                        break;
                    }
                }
                
                for(int t = ti; t > 1; t--) {
                    if(solx[i][s][t] > 0) {
                        exit_time = t;
                        break;
                    }
                }
                
                if(entry_time > 0 && exit_time > 0) {
                    double speed {0};
                    if(d->segments[s]->type == '0') {
                        if(d->trains[i].westbound) {
                            speed = d->speed_ew;
                        } else {
                            speed = d->speed_we;
                        }
                    } else if(d->segments[s]->type == '1') {
                        speed = d->speed_ew;
                    } else if(d->segments[s]->type == '2') {
                        speed = d->speed_we;
                    } else if(d->segments[s]->type == 'S') {
                        speed = d->speed_siding;
                    } else if(d->segments[s]->type == 'T') {
                        speed = d->speed_switch;
                    } else if(d->segments[s]->type == 'X') {
                        speed = d->speed_xover;
                    } else {
                        throw std::runtime_error("Unrecognised segment type!");
                    }
                    
                    speed *= d->trains[i].speed_multi;        
                    double m_s_i {ceil(d->segments[s]->length / speed)};
                    
                    if(exit_time - entry_time + 1 > m_s_i) {
                        std::cout << "\tStayed on segment " << s << " for " << (exit_time - entry_time + 1) << " minutes" << std::endl;
                        std::cout << "\t\tEntered at the beginning of minute " << entry_time << std::endl;
                        std::cout << "\t\tWent out at the end of minute " << exit_time << std::endl;
                        std::cout << "\t\tOptimal travel time is of " << m_s_i << " minutes" << std::endl;
                    }
                }
            }
        }
    }
    
    env.end();    
}