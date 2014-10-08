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
    
    var_vector var_x(env);
    var_vector var_d(env, d.nt);
    var_vector var_e(env);
    
    std::stringstream name;
    int_matrix_5d idx(d.nt, int_matrix_4d(d.ns + 2, int_matrix_3d(d.ni + 2, int_matrix(d.ns + 2, bv<int>(d.ni + 2, -1)))));
    int_matrix ide(d.nt, bv<int>(d.ns + 2, -1));
    
    auto col_x = 0, col_e = 0;
    
    std::cout << "Creating variables..." << std::endl;
    t_start = high_resolution_clock::now();
    
    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "var_d_" << i; auto ub_d = std::max(d.tr_wt[i], d.ni - d.tr_wt[i]);
        var_d[i] = IloNumVar(env, 0.0, ub_d, IloNumVar::Int, name.str().c_str());
        
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            if(d.accessible[i][s1] && d.sa[i][s1]) {
                name.str(""); name << "var_e_" << i << "_" << s1; auto ub_e = std::max(d.sa_times[i][s1], d.ni - d.sa_times[i][s1]);
                var_e.add(IloNumVar(env, 0.0, ub_e, IloNumVar::Int, name.str().c_str()));
                ide[i][s1] = col_e++;
            }
            for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                if(d.v[i][s1][t1]) {
                    for(auto s2 = 0; s2 < d.ns + 2; s2++) {
                        if(d.accessible[i][s2]) {
                            for(auto t2 = 0; t2 < d.ni + 2; t2++) {
                                if(d.v[i][s2][t2] && d.adj[i][s1][t1][s2][t2]) {
                                    name.str(""); name << "var_x_" << i << "_" << s1 << "_" << t1 << "_" << s2 << "_" << t2;
                                    var_x.add(IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, name.str().c_str()));
                                    idx[i][s1][t1][s2][t2] = col_x++;
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
    
    std::cout << "Adding constraints..." << std::endl;
    t_start = high_resolution_clock::now();

    for(auto i = 0; i < d.nt; i++) {
        name.str(""); name << "cst_exit_sigma_" << i;
        cst_exit_sigma[i] = IloRange(env, 1.0, 1.0, name.str().c_str());
        for(auto s1 = 0; s1 < d.ns + 2; s1++) {
            for(auto t1 = 0; t1 < d.ni + 2; t1++) {
                if(idx[i][0][0][s1][t1] != -1) {
                    cst_exit_sigma[i].setLinearCoef(var_x[idx[i][0][0][s1][t1]], 1.0);
                }
            }
        }
    }
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
    
    IloCplex cplex(model);
    
    model.add(cst_exit_sigma);
    
    cplex.exportModel("model.lp");
}