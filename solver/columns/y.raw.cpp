for(int i = 0; i < nt; i++) {
    for(std::tie(vi1, vi1_end) = vertices(graphs[i]->g); vi1 != vi1_end; ++vi1) {
        // Variable y^i_vi1
            
        const Node& n1 {graphs[i]->g[*vi1]};
        
        int eta_i_n {1};
        if( n1.s == nullptr ||
            n1.s->eastbound && d->trains[i].eastbound ||
            n1.s->westbound && d->trains[i].westbound) {
            eta_i_n = 0;    
        }
        
        // std::cout << "=> y^" << i << "_{" << n1.str() << "}" << std::endl;
        
        // ********** Obj Function **********
        double obj_coeff {0};
        obj_coeff = eta_i_n * d->unpreferred_price;
        
        IloNumColumn col = obj(obj_coeff);
        
        // ********** eq_exit_sigma **********
        for(int ii = 0; ii < nt; ii++) {
            int coeff {0};
            col += eq_exit_sigma[ii](coeff);
        }
        
        // ********** eq_enter_tau **********
        for(int ii = 0; ii < nt; ii++) {
            int coeff {0};
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
                        if(n1 == n) {
                            coeff = -1;
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
                
                if(i == ii && n1.s != nullptr && n1.s->id == ss) {
                    coeff = 1;
                }
                
                col += eq_set_z[num_row++](coeff);
            }
        }
        
        // ********** eq_set_theta **********
        num_row = 0;
        for(int ii = 0; ii < nt; ii++) {
            for(int ss = 0; ss < ns; ss++) {
                int coeff {0};
                col += eq_set_theta[num_row++](coeff);
            }
        }
        
        // ********** eq_wt_1 **********
        for(int ii = 0; ii < nt; ii++) {
            int coeff {0};
            col += eq_wt_1[ii](coeff);
        }
        
        // ********** eq_wt_2 **********
        for(int ii = 0; ii < nt; ii++) {
            int coeff {0};
            col += eq_wt_2[ii](coeff);
        }
        
        // ********** eq_sa_delay **********
        num_row = 0;
        for(int ii = 0; ii < nt; ii++) {
            if(d->trains[ii].schedule_adherence) {
                for(auto kv : d->trains[ii].schedule) {
                    int coeff {0};
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
                for(int tt = 0; tt < ti; tt++) {
                    if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                        int coeff {0};
                        int h {d->headway};
                    
                        if(i == ii) {
                            if(n1.s != nullptr && n1.t == tt && n1.s->id == ss) {
                                coeff = 2 * h + 1;
                            }
                        } else {
                            if(n1.s != nullptr && n1.s->id == ss && n1.t >= tt - h && n1.t <= tt + h) {
                                coeff = 1;
                            }
                        }
                    
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
                    for(int tt = 0; tt < ti; tt++) {
                        if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                            int coeff {0};
                            int h {d->headway};
                        
                            if(n1.s != nullptr && n1.s->id == ss && i == ii) {
                                coeff = 1;
                            }
                        
                            if(i != ii && n1.s != nullptr && n1.t >= tt - h && n1.t <= tt + h && d->is_main(n1.s, d->segments[ss])) {
                                coeff = -1;
                            }
                        
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
                        for(int tt = 0; tt < ti; tt++) {
                            if(graphs[ii]->vertex_for(d->segments[ss], tt).first) {
                                int coeff {0};
                                
                                if(i == ii && n1.s != nullptr && n1.s->id == ss && n1.t == tt) {
                                    coeff = 1;
                                }
                                
                                if(i != ii && !d->trains[i].schedule_adherence && n1.s != nullptr && d->is_main(n1.s, d->segments[ss])) {
                                    coeff = -1;
                                }
                                
                                col += eq_heavy_siding[num_row++](coeff);
                            }
                        }
                    }
                }
            }
        }
        
        IloNumVar v(col, 0.0, 1.0, IloNumVar::Bool, (
            "y_train_" + std::to_string(i) + "_node_" + n1.str()
        ).c_str());
        var_y.add(v);
    }
}