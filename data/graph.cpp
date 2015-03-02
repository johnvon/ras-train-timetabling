#include <data/graph.h>

#include <algorithm>
#include <iostream>

graph::graph(unsigned int nt, unsigned int ns, unsigned int ni, const params& p, const trains& trn, const mows& mnt, const segments& seg, const network& net, const time_windows& tiw, const prices& pri) {
    n_nodes = 0u;
    n_arcs = 0u;
    v_for_someone = bool_matrix_2d(ns + 2, bool_vector(ns + 2, false));
    delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    inverse_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    bar_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    bar_inverse_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    trains_for = uint_matrix_3d(ns + 2, uint_matrix_2d(ni + 2, uint_vector()));
    v = bool_matrix_3d(nt, bool_matrix_2d(ns + 2, bool_vector(ni + 2, false)));
    adj = bool_matrix_4d(nt, bool_matrix_3d(ns + 2, bool_matrix_2d(ni + 2, bool_vector(ns + 2, false))));
    n_out = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector(ni + 2, 0u)));
    n_in = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector(ni + 2, 0u)));
    costs = double_matrix_4d(nt, double_matrix_3d(ns + 2, double_matrix_2d(ni + 2, double_vector(ns + 2, 0.0))));
    first_time_we_need_tau = uint_vector(nt, 0u);
        
    calculate_deltas(nt, ns, trn, seg);
    calculate_vertices(nt, ns, ni, trn, mnt, seg, net);
    calculate_starting_arcs(nt, ni, p, trn, seg, net);
    calculate_ending_arcs(nt, ns, ni, p, trn, net);
    calculate_escape_arcs(nt, ns, ni);
    calculate_stop_arcs(nt, ns, ni, net);
    calculate_movement_arcs(nt, ns, ni, net);
    cleanup(nt, ns, ni);
    calculate_costs(nt, ns, ni, trn, net, tiw, pri);
    
    for(auto i = 0u; i < nt; i++) {
        for(auto s1 = 0u; s1 <= ns + 1; s1++) {
            for(auto s2 = s1 + 1; s2 <= ns + 1; s2++) {
                assert(
                    (std::find(delta.at(i).at(s1).begin(), delta.at(i).at(s1).end(), s2) != delta.at(i).at(s1).end()) ==
                    (std::find(inverse_delta.at(i).at(s2).begin(), inverse_delta.at(i).at(s2).end(), s1) != inverse_delta.at(i).at(s2).end())
                );
            }
        }
    }
}

auto graph::calculate_deltas(unsigned int nt, unsigned int ns, const trains& trn, const segments& seg) -> void {
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {        
        for(auto i = 0u; i < nt; i++) {
            delta[i][s1].push_back(s1);
            inverse_delta[i][s1].push_back(s1);
            
            if(std::find(trn.orig_segs.at(i).begin(), trn.orig_segs.at(i).end(), s1) != trn.orig_segs.at(i).end()) {
                delta[i][0].push_back(s1);
                inverse_delta[i][s1].push_back(0);
                
                bar_delta[i][0].push_back(s1);
                bar_inverse_delta[i][s1].push_back(0);
            }
            
            if(std::find(trn.dest_segs.at(i).begin(), trn.dest_segs.at(i).end(), s1) != trn.dest_segs.at(i).end()) {
                delta[i][s1].push_back(ns + 1);
                inverse_delta[i][ns + 1].push_back(s1);
                
                bar_delta[i][s1].push_back(ns + 1);
                bar_inverse_delta[i][ns + 1].push_back(s1);
            }
        }
        
        for(auto s2 = 0u; s2 <= ns + 1; s2++) {
            if(seg.e_ext.at(s1) == seg.w_ext.at(s2) || seg.w_ext.at(s1) == seg.e_ext.at(s2)) {                
                for(auto i = 0u; i < nt; i++) {
                    if( (seg.e_ext.at(s1) == seg.w_ext.at(s2) && trn.is_eastbound.at(i)) ||
                        (seg.w_ext.at(s1) == seg.e_ext.at(s2) && trn.is_westbound.at(i))
                    ) {
                        assert(s1 != s2);
                        
                        delta[i][s1].push_back(s2);
                        bar_delta[i][s1].push_back(s2);
                    }
                    
                    if( (seg.e_ext.at(s1) == seg.w_ext.at(s2) && trn.is_westbound.at(i)) ||
                        (seg.w_ext.at(s1) == seg.e_ext.at(s2) && trn.is_eastbound.at(i))
                    ) {
                        assert(s1 != s2);
                        
                        inverse_delta[i][s1].push_back(s2);
                        bar_inverse_delta[i][s1].push_back(s2);
                    }
                }
            }
        }
    }
}

auto graph::calculate_vertices(unsigned int nt, unsigned int ns, unsigned int ni, const trains& trn, const mows& mnt, const segments& seg, const network& net) -> void {
    for(auto i = 0u; i < nt; i++) {                
        for(auto s = 1u; s <= ns; s++) {
            if(trn.is_hazmat.at(i) && seg.type.at(s) == 'S') {
                continue;
            }
            
            if(seg.type.at(s) == 'S' && trn.length.at(i) > seg.original_length.at(s)) {
                continue;
            }
            
            if(std::find(trn.dest_segs.at(i).begin(), trn.dest_segs.at(i).end(), s) != trn.dest_segs.at(i).end()) {
                auto potential_tau_time = net.min_time_to_arrive.at(i).at(s) + net.min_travel_time.at(i).at(s) - 1;
                
                if(potential_tau_time < first_time_we_need_tau.at(i)) {
                    first_time_we_need_tau[i] = potential_tau_time;
                }
            }
            
            for(auto t = net.min_time_to_arrive.at(i).at(s); t <= ni; t++) {
                if(mnt.is_mow.at(s).at(t)) {
                    continue;
                }
                
                v[i][s][t] = true;
                n_nodes++;
                v_for_someone[s][t] = true;
                trains_for[s][t].push_back(i);
            }
        }
        
        for(auto t = 0u; t <= ni; t++) {
            v[i][0][t] = true;
            n_nodes++;
            trains_for[0][t].push_back(i);
        }
        
        for(auto t = first_time_we_need_tau.at(i); t <= ni + 1; t++) {
            v[i][ns + 1][t] = true;
            n_nodes++;
            trains_for[ns + 1][t].push_back(i);
        }
    }
}

auto graph::calculate_starting_arcs(unsigned int nt, unsigned int ni, const params& p, const trains& trn, const segments& seg, const network& net) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s : trn.orig_segs.at(i)) {
            for(auto t = 1u; t <= ni - net.min_travel_time.at(i).at(s); t++) {                
                if(p.heuristics.constructive.active) {
                    if(p.heuristics.constructive.only_start_at_main && seg.type.at(s) == 'S') {
                        continue;
                    }
                    if(p.heuristics.constructive.fix_start) {
                        if(t != trn.entry_time.at(i)) {
                            continue;
                        }
                    }
                }

                if(v.at(i).at(0).at(t - 1) && v.at(i).at(s).at(t)) {
                    adj[i][0][t - 1][s] = true;
                    n_out[i][0][t - 1]++;
                    n_in[i][s][t]++;
                    n_arcs++;
                }
            }
        }
    }
}

auto graph::calculate_ending_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const params& p, const trains& trn, const network& net) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s : trn.dest_segs.at(i)) {
            for(auto t = net.min_time_to_arrive.at(i).at(s) + net.min_travel_time.at(i).at(s) - 1; t <= ni; t++) {
                if(v.at(i).at(s).at(t)) {
                    if(p.heuristics.constructive.active && p.heuristics.constructive.fix_end) {
                        if(t != trn.want_time.at(i)) {
                            continue;
                        }
                    }
                    
                    if(v.at(i).at(ns + 1).at(t + 1) && v.at(i).at(s).at(t)) {
                        adj[i][s][t][ns + 1] = true;
                        n_out[i][s][t]++;
                        n_in[i][ns + 1][t + 1]++;
                        n_arcs++;
                    }
                }
            }
        }
    }
}

auto graph::calculate_escape_arcs(unsigned int nt, unsigned int ns, unsigned int ni) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {
            if(v.at(i).at(s).at(ni) && v.at(i).at(ns + 1).at(ni + 1)) {
                adj[i][s][ni][ns + 1] = true;
                n_out[i][s][ni]++;
                n_in[i][ns + 1][ni + 1]++;
                n_arcs++;
            }
        }
    }
}

auto graph::calculate_stop_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const network& net) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {            
            for(auto t = net.min_time_to_arrive.at(i).at(s); t < ni; t++) {
                if(v.at(i).at(s).at(t) && v.at(i).at(s).at(t + 1)) {
                    adj[i][s][t][s] = true;
                    n_out[i][s][t]++;
                    n_in[i][s][t + 1]++;
                    n_arcs++;
                }
            }
        }
    }
}

auto graph::calculate_movement_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const network& net) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s1 = 1u; s1 <= ns; s1++) {            
            for(auto s2 = 1u; s2 <= ns; s2++) {
                if(std::find(bar_delta.at(i).at(s1).begin(), bar_delta.at(i).at(s1).end(), s2) != bar_delta.at(i).at(s1).end()) {
                    for(auto t = net.min_time_to_arrive.at(i).at(s1) + net.min_travel_time.at(i).at(s1) - 1; t <= ni - net.min_travel_time.at(i).at(s2); t++) {                        
                        if(v.at(i).at(s1).at(t) && v.at(i).at(s2).at(t + 1)) {
                            adj[i][s1][t][s2] = true;
                            n_out[i][s1][t]++;
                            n_in[i][s2][t + 1]++;
                            n_arcs++;
                        }
                    }
                }
            }
        }
    }
}

auto graph::cleanup(unsigned int nt, unsigned int ns, unsigned int ni) -> void {
    for(auto i = 0u; i < nt; i++) {
        auto clean = false;
            
        while(!clean) {
            clean = true;
            
            for(auto s1 = 0u; s1 <= ns + 1; s1++) {
                for(auto t1 = 0u; t1 <= ni + 1; t1++) {
                    auto _in = n_in.at(i).at(s1).at(t1);
                    auto _out = n_out.at(i).at(s1).at(t1);
                    
                    if(!v.at(i).at(s1).at(t1)) {
                        assert(_in == 0);
                        assert(_out == 0);
                    } else {
                        if( (s1 != 0 && s1 != ns + 1 && (_in == 0 || _out == 0 || _in + _out <= 1)) ||
                            ((s1 == 0 || s1 == ns + 1) && (_in + _out == 0))
                        ) {
                            for(auto s2 = 0u; s2 <= ns + 1; s2++) {
                                if(t1 <= ni && adj.at(i).at(s1).at(t1).at(s2)) {
                                    adj[i][s1][t1][s2] = false;
                                    n_in[i][s2][t1+1]--;
                                    n_arcs--;
                                }
                                if(t1 > 0u && adj.at(i).at(s2).at(t1 - 1).at(s1)) {
                                    adj[i][s2][t1 - 1][s1] = false;
                                    n_out[i][s2][t1 - 1]--;
                                    n_arcs--;
                                }
                            }
                            
                            n_in[i][s1][t1] = 0;
                            n_out[i][s1][t1] = 0;
                            v[i][s1][t1] = false;
                            n_nodes--;
                            
                            auto still_valid_vertex = false;
                            for(auto i = 0u; i < nt; i++) {
                                if(v.at(i).at(s1).at(t1)) {
                                    still_valid_vertex = true;
                                    break;
                                }
                            }
                            v_for_someone[s1][t1] = still_valid_vertex;
                            clean = false;
                        }
                    }
                }
            }
        }
    }
}

auto graph::calculate_costs(unsigned int nt, unsigned int ns, unsigned int ni, const trains& trn, const network& net, const time_windows& tiw, const prices& pri) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s : trn.dest_segs.at(i)) {
            for(auto t = net.min_time_to_arrive.at(i).at(s) + net.min_travel_time.at(i).at(s) - 1; t <= ni; t++) {
                if(adj.at(i).at(s).at(t).at(ns + 1)) {
                    if(t < trn.want_time.at(i) - tiw.wt_left) {
                        auto advance = trn.want_time.at(i) - tiw.wt_left - t;
                        costs[i][s][t][ns + 1] = pri.wt * advance;
                    }
                    
                    if(t > trn.want_time.at(i) + tiw.wt_right + 1) {
                        auto delay = t - trn.want_time.at(i) - tiw.wt_right - 1;
                        costs[i][s][t][ns + 1] = pri.wt * delay;
                    }
                }
            }
        }
        
        for(auto n = 0u; n < trn.sa_num.at(i); n++) {
            for(auto t = trn.sa_times.at(i).at(n) + tiw.sa_right + 1; t <= ni; t++) {
                for(auto s1 : trn.sa_segs.at(i).at(n)) {
                    for(auto s2 : bar_delta.at(i).at(s1)) {
                        if(adj.at(i).at(s1).at(t).at(s2)) {
                            auto delay = t - trn.sa_times.at(i).at(n) - tiw.sa_right - 1;
                            costs[i][s1][t][s2] = pri.sa * delay;
                        }
                    }
                }
            }
        }
        
        for(auto s : trn.unpreferred_segs.at(i)) {
            for(auto t = net.min_time_to_arrive.at(i).at(s); t < ni; t++) {
                if(adj.at(i).at(s).at(t).at(s)) {
                    costs[i][s][t][s] = pri.unpreferred;
                }
            }
        }
    }
}