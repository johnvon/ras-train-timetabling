for(int i = 0; i < nt; i++) {
    // Variable d^i_s
    
    double c_w_i {d->terminal_delay_price};
    
    // ********** Obj Function **********
    double obj_coeff {0};
    obj_coeff = c_w_i;
    
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
    
    // ********** eq_one_train **********
    int num_row {0};
    for(int ss = 0; ss < ns; ss++) {
        for(int tt = 1; tt <= ti; tt++) {
            int coeff {0};
            col += eq_one_train[num_row++](coeff);
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
    
    // ********** eq_ensure_sa_visit **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        if(d->trains[ii].schedule_adherence) {
            for(int chk = 0; chk < d->trains[ii].schedule.size(); chk++) {
                int coeff {0};
                col += eq_ensure_sa_visit[num_row++](coeff);
            }
        }
    }

    // ********** eq_wt_1 **********
    for(int ii = 0; ii < nt; ii++) {
        int coeff {0};
        
        if(i == ii) {
            coeff = -1;
        }
        
        col += eq_wt_1[ii](coeff);
    }
    
    // ********** eq_wt_2 **********
    for(int ii = 0; ii < nt; ii++) {
        int coeff {0};
        
        if(i == ii) {
            coeff = -1;
        }
        
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
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr) {
                int coeff {0};
                col += eq_min_time[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_headway1 **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr) {
                int coeff {0};
                col += eq_headway1[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_headway2 **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr) {
                int coeff {0};
                col += eq_headway2[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_headway3 **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr) {
                int coeff {0};
                col += eq_headway3[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_headway4 **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr) {
                int coeff {0};
                col += eq_headway4[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_can_take_siding **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[ii]->g[*vi];
            if(n.s != nullptr && n.s->type == 'S') {
                int coeff {0};
                col += eq_can_take_siding[num_row++](coeff);
            }
        }
    }
    
    // ********** eq_heavy_siding **********
    num_row = 0;
    for(int ii = 0; ii < nt; ii++) {
        if(d->trains[ii].heavy) {
            vi_t vi, vi_end;
            for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
                const Node& n = graphs[ii]->g[*vi];
                if(n.s != nullptr && n.s->type == 'S') {
                    int coeff {0};
                    col += eq_heavy_siding[num_row++](coeff);
                }
            }
        }
    }
    
    // We strengthen the bound on d by calculating the maximum possible delay
    int max_delay {std::max(d->trains[i].terminal_wt, ti - d->trains[i].terminal_wt)};
    
    IloNumVar v(col, 0.0, max_delay, IloNumVar::Int, (
        "d_train_" + std::to_string(i)
    ).c_str());
    var_d.add(v);
}