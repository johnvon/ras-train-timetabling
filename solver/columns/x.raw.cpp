for(int i = 0; i < nt; i++) {
    for(std::tie(vi1, vi1_end) = vertices(graphs[i]->g); vi1 != vi1_end; ++vi1) {
        for(std::tie(vi2, vi2_end) = vertices(graphs[i]->g); vi2 != vi2_end; ++vi2) {
            if(edge(*vi1, *vi2, graphs[i]->g).second) {
                // Variable x^i_{vi1,vi2}
            
                const Node& n1 {graphs[i]->g[*vi1]};
                const Node& n2 {graphs[i]->g[*vi2]};
                
                // std::cout << "=> x^" << i << "_{" << n1.str() << "," << n2.str() << "}" << std::endl;
            
                // ********** Obj Function **********
                int obj_coeff {0}; // x is not in the objective function
                IloNumColumn col = obj(obj_coeff);
                                        
                // ********** eq_exit_sigma **********
                for(int ii = 0; ii < nt; ii++) {
                    int coeff {0};
                
                    if(i == ii) {
                        if(n1.source) { // If vi1 == sigma
                            bool vi2_in_delta_plus_vi1 {false};
                                        
                            oei_t oei, oei_end;
                            for(std::tie(oei, oei_end) = out_edges(*vi1, graphs[ii]->g); oei != oei_end; ++oei) {
                                const Node& ntarget = graphs[ii]->g[target(*oei, graphs[ii]->g)];
                                                        
                                if(ntarget == n2) {
                                    vi2_in_delta_plus_vi1 = true;
                                    break;
                                }
                            }
                    
                            if(vi2_in_delta_plus_vi1) {
                                coeff = 1;
                            }
                        }
                    }
                
                    col += eq_exit_sigma[ii](coeff);
                }
                            
                // ********** eq_enter_tau **********
                for(int ii = 0; ii < nt; ii++) {
                    int coeff {0};
                
                    if(i == ii) {
                        if(n2.sink) { // If vi2 == tau
                            bool vi1_in_delta_minus_vi2 {false};
                    
                            iei_t iei, iei_end;
                            for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[ii]->g); iei != iei_end; ++iei) {
                                const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                if(nsource == n1) {
                                    vi1_in_delta_minus_vi2 = true;
                                    break;
                                }
                            }
                    
                            if(vi1_in_delta_minus_vi2) {
                                coeff = 1;
                            }
                        }
                    }
                
                    col += eq_enter_tau[ii](coeff);
                }
                                            
                // ********** eq_set_y **********
                int num_row {0};
                for(int ii = 0; ii < nt; ii++) {
                    vi_t vi, vi_end;
                    for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
                        const Node& n {graphs[ii]->g[*vi]};
            
                        if(n.s != nullptr) {
                            int coeff {0};
                            if(i == ii) {
                                if(n2 == n) {
                                    bool vi1_in_delta_minus_vi {false};
                                    iei_t iei, iei_end;
                                    for(std::tie(iei, iei_end) = in_edges(*vi, graphs[ii]->g); iei != iei_end; ++iei) {
                                        const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                        if(nsource == n1) {
                                            vi1_in_delta_minus_vi = true;
                                            break;
                                        }
                                    }
                                    if(vi1_in_delta_minus_vi) {
                                        coeff = 1;
                                    }
                                }
                            }
                            
                            col += eq_set_y[num_row++](coeff);
                        }
                    }
                }
                            
                // ********** eq_flow **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    vi_t vi, vi_end;
                    for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
                        const Node& n {graphs[ii]->g[*vi]};
                        if(n.s != nullptr) {
                            int coeff {0};
                        
                            if(i == ii) {
                                if(n2 == n) {
                                    bool vi1_in_delta_minus_vi {false};

                                    iei_t iei, iei_end;
                                    for(std::tie(iei, iei_end) = in_edges(*vi, graphs[ii]->g); iei != iei_end; ++iei) {
                                        const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                        if(nsource == n1) {
                                            vi1_in_delta_minus_vi = true;
                                            break;
                                        }
                                    }

                                    if(vi1_in_delta_minus_vi) {
                                        coeff = 1;
                                    }
                                }

                                if(n1 == n) {
                                    bool vi2_in_delta_plus_vi {false};

                                    oei_t oei, oei_end;
                                    for(std::tie(oei, oei_end) = out_edges(*vi, graphs[ii]->g); oei != oei_end; ++oei) {
                                        const Node& ntarget = graphs[ii]->g[target(*oei, graphs[ii]->g)];
                                        if(ntarget == n2) {
                                            vi2_in_delta_plus_vi = true;
                                            break;
                                        }
                                    }

                                    if(vi2_in_delta_plus_vi) {
                                        coeff = -1;
                                    }
                                }
                            }

                            col += eq_flow[num_row++](coeff);
                        }
                    }
                }
                                
                // ********** eq_z_sa **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    if(d->trains[ii].schedule_adherence) {
                        for(int chk = 0; chk < d->trains[ii].schedule.size(); chk++) {
                            int coeff {0};
                            col += eq_z_sa[num_row++](coeff);
                        }
                    }
                }
                                
                // ********** eq_set_z **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    for(int ss = 0; ss < ns; ss++) {
                        int coeff {0};
                        col += eq_set_z[num_row++](coeff);
                    }
                }
                                
                // ********** eq_set_theta **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    for(int ss = 0; ss < ns; ss++) {
                        int coeff {0};

                        if(i == ii) {
                            if(n1.s != nullptr && n1.s->id == ss) {
                                bool vi2_in_delta_plus_bar_vi1 {false};

                                oei_t oei, oei_end;
                                for(std::tie(oei, oei_end) = out_edges(*vi1, graphs[ii]->g); oei != oei_end; ++oei) {
                                    const Node& ntarget = graphs[ii]->g[target(*oei, graphs[ii]->g)];
                                    if(ntarget == n2 && !(ntarget.s != nullptr && ntarget.s->id == n1.s->id)) {
                                        vi2_in_delta_plus_bar_vi1 = true;
                                        break;
                                    }
                                }

                                if(vi2_in_delta_plus_bar_vi1) {
                                    coeff = -n1.t;
                                }
                            }

                            if(n2.s != nullptr && n2.s->id == ss) {
                                bool vi1_in_delta_minus_bar_vi2 {false};

                                iei_t iei, iei_end;
                                for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[ii]->g); iei != iei_end; ++iei) {
                                    const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                    if(nsource == n1 && !(nsource.s != nullptr && nsource.s->id == n2.s->id)) {
                                        vi1_in_delta_minus_bar_vi2 = true;
                                        break;
                                    }
                                }

                                if(vi1_in_delta_minus_bar_vi2) {
                                    coeff = n2.t;
                                }
                            }
                        }

                        col += eq_set_theta[num_row++](coeff);
                    }
                }
                                
                // ********** eq_wt_1 **********
                for(int ii = 0; ii < nt; ii++) {
                    int coeff {0};
                    
                    if(i == ii) {
                        if(n2.sink) {
                            bool vi1_in_delta_minus_vi2 {false};

                            iei_t iei, iei_end;
                            for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[ii]->g); iei != iei_end; ++iei) {
                                const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                if(nsource == n1) {
                                    vi1_in_delta_minus_vi2 = true;
                                    break;
                                }
                            }

                            if(vi1_in_delta_minus_vi2) {
                                coeff = -n1.t;
                            }
                        }
                    }
                    
                    col += eq_wt_1[ii](coeff);
                }
                                
                // ********** eq_wt_2 **********
                for(int ii = 0; ii < nt; ii++) {
                    int coeff {0};
                    
                    if(i == ii) {
                        if(n2.sink) {
                            bool vi1_in_delta_minus_vi2 {false};

                            iei_t iei, iei_end;
                            for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[ii]->g); iei != iei_end; ++iei) {
                                const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                if(nsource == n1) {
                                    vi1_in_delta_minus_vi2 = true;
                                    break;
                                }
                            }

                            if(vi1_in_delta_minus_vi2) {
                                coeff = n1.t;
                            }
                        }
                    }
                    
                    col += eq_wt_2[ii](coeff);
                }
                                
                // ********** eq_sa_delay **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    if(d->trains[ii].schedule_adherence) {
                        for(auto kv : d->trains[ii].schedule) {
                            int segment {kv.first};
                            int schedule_time {kv.second};
                            
                            int coeff {0};
                            
                            if(i == ii && n2.s != nullptr && n2.s->id == segment) {
                                bool vi1_in_delta_minus_bar_vi2 {false};

                                iei_t iei, iei_end;
                                for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[ii]->g); iei != iei_end; ++iei) {
                                    const Node& nsource = graphs[ii]->g[source(*iei, graphs[ii]->g)];
                                    if(nsource == n1 && !(nsource.s != nullptr && nsource.s->id == n2.s->id)) {
                                        vi1_in_delta_minus_bar_vi2 = true;
                                        break;
                                    }
                                }

                                if(vi1_in_delta_minus_bar_vi2) {
                                    coeff = n2.t;
                                }
                            }
                                                        
                            col += eq_sa_delay[num_row++](coeff);
                        }
                    }
                }
                                
                // ********** eq_min_time **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    for(int ss = 0; ss < ns; ss++) {
                        int coeff {0};
                        col += eq_min_time[num_row++](coeff);
                    }
                }
                                
                // ********** eq_headway **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    for(int ss = 0; ss < ns; ss++) {
                        for(int tt = 1; tt <= ti; tt++) {
                            if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                                int coeff {0};
                                col += eq_headway[num_row++](coeff);
                            }
                        }
                    }
                }
                                
                // ********** eq_can_take_siding **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    for(int ss = 0; ss < ns; ss++) {
                        if(d->segments[ss]->type == 'S') {
                            for(int tt = 1; tt <= ti; tt++) {
                                if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                                    int coeff {0};
                                    col += eq_can_take_siding[num_row++](coeff);
                                }
                            }
                        }
                    }
                }
                
                // ********** eq_heavy_siding **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    if(d->trains[ii].heavy) {
                        for(int ss = 0; ss < ns; ss++) {
                            if(d->segments[ss]->type == 'S') {
                                for(int tt = 1; tt <= ti; tt++) {
                                    if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                                        int coeff {0};
                                        col += eq_heavy_siding[num_row++](coeff);
                                    }
                                }
                            }
                        }
                    }
                }
                            
                IloNumVar v(col, 0.0, 1.0, IloNumVar::Bool, (
                    "x_train_" + std::to_string(i) + "_from_" + n1.str() + "_to_" + n2.str()
                ).c_str());
                var_x.add(v);
            }
        }
    }
}