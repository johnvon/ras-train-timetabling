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
    IloNumVarArray var_y(env);
    IloNumVarArray var_z(env);
    IloNumVarArray var_theta(env);
    IloNumVarArray var_d(env);
    IloNumVarArray var_e(env);
    
    IloRangeArray eq_exit_sigma(env);
    IloRangeArray eq_enter_tau(env);
    IloRangeArray eq_set_y(env);
    IloRangeArray eq_flow(env);
    IloRangeArray eq_z_sa(env);
    IloRangeArray eq_set_z(env);
    IloRangeArray eq_set_theta(env);
    IloRangeArray eq_wt_1(env);
    IloRangeArray eq_wt_2(env);
    IloRangeArray eq_sa_delay(env);
    IloRangeArray eq_min_time(env);
    IloRangeArray eq_headway(env);
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
    model.add(eq_set_y);
    model.add(eq_flow);
    model.add(eq_z_sa);
    model.add(eq_set_z);
    model.add(eq_set_theta);
    model.add(eq_wt_1);
    model.add(eq_wt_2);
    model.add(eq_sa_delay);
    model.add(eq_min_time);
    model.add(eq_headway);
    model.add(eq_can_take_siding);
    model.add(eq_heavy_siding);
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << time_span.count() << " seconds" << std::endl;
    
    IloCplex cplex(model);
    
    std::cout << "Writing model to file... ";
    t_start = high_resolution_clock::now();
    cplex.exportModel("model.lp");
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    std::cout << time_span.count() << " seconds" << std::endl;
    
    cplex.setParam(IloCplex::Threads, 4);
    
    std::cout << "Solving model..." << std::endl;
    cplex.solve();
    
    env.end();    
}