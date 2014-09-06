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
    IloRangeArray eq_headway1(env);
    IloRangeArray eq_headway2(env);
    IloRangeArray eq_headway3(env);
    IloRangeArray eq_headway4(env);
    IloRangeArray eq_can_take_siding(env);
    IloRangeArray eq_heavy_siding(env);
    
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
    model.add(eq_headway1);
    model.add(eq_headway2);
    model.add(eq_headway3);
    model.add(eq_headway4);
    model.add(eq_can_take_siding);
    model.add(eq_heavy_siding);
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
        
        int num_col {0};
        
        // Print x
        for(int i = 0; i < nt; i++) {
            for(std::tie(vi1, vi1_end) = vertices(graphs[i]->g); vi1 != vi1_end; ++vi1) {
                for(std::tie(vi2, vi2_end) = vertices(graphs[i]->g); vi2 != vi2_end; ++vi2) {
                    if(edge(*vi1, *vi2, graphs[i]->g).second) {
                        if(nx[num_col] > 0) {
                            const Node& n1 {graphs[i]->g[*vi1]};
                            const Node& n2 {graphs[i]->g[*vi2]};
                            
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
                        std::cout << "e[" << i << "][" << segs[i] << "] = " << ne[num_col] << std::endl;
                    }
                    num_col++;
                }
            }
        }
        
        nx.end(); nd.end(); ne.end();
    }
    
    env.end();    
}