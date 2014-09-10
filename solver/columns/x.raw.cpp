vi_t vi1, vi1_end, vi2, vi2_end;

for(int i = 0; i < nt; i++) {
    for(std::tie(vi1, vi1_end) = vertices(graphs[i]->g); vi1 != vi1_end; ++vi1) {
        for(std::tie(vi2, vi2_end) = vertices(graphs[i]->g); vi2 != vi2_end; ++vi2) {
            if(edge(*vi1, *vi2, graphs[i]->g).second) {
                // Variable x^i_{vi1,vi2}
            
                const Node& n1 {graphs[i]->g[*vi1]};
                const Node& n2 {graphs[i]->g[*vi2]};
                
                // std::cout << "=> x^" << i << "_{" << n1.str() << "," << n2.str() << "}" << std::endl;
                            
                double c_d_i {d->general_delay_price.at(d->trains[i].cl)};
                
                double speed2 {0};
                if(n2.s != nullptr) {
                    if(n2.s->type == '0') {
                        if(d->trains[i].westbound) {
                            speed2 = d->speed_ew;
                        } else {
                            speed2 = d->speed_we;
                        }
                    } else if(n2.s->type == '1') {
                        speed2 = d->speed_ew;
                    } else if(n2.s->type == '2') {
                        speed2 = d->speed_we;
                    } else if(n2.s->type == 'S') {
                        speed2 = d->speed_siding;
                    } else if(n2.s->type == 'T') {
                        speed2 = d->speed_switch;
                    } else if(n2.s->type == 'X') {
                        speed2 = d->speed_xover;
                    } else {
                        throw std::runtime_error("Unrecognised segment type!");
                    }
                }
                speed2 *= d->trains[i].speed_multi;        
                double m_s_i2 {0};
                
                if(n2.s != nullptr) {
                    m_s_i2 = ceil(n2.s->length / speed2);
                }
                
                double speed1 {0};
                if(n1.s != nullptr) {
                    if(n1.s->type == '0') {
                        if(d->trains[i].westbound) {
                            speed1 = d->speed_ew;
                        } else {
                            speed1 = d->speed_we;
                        }
                    } else if(n1.s->type == '1') {
                        speed1 = d->speed_ew;
                    } else if(n1.s->type == '2') {
                        speed1 = d->speed_we;
                    } else if(n1.s->type == 'S') {
                        speed1 = d->speed_siding;
                    } else if(n1.s->type == 'T') {
                        speed1 = d->speed_switch;
                    } else if(n1.s->type == 'X') {
                        speed1 = d->speed_xover;
                    } else {
                        throw std::runtime_error("Unrecognised segment type!");
                    }
                }
                speed1 *= d->trains[i].speed_multi;        
                double m_s_i1 {0};
                
                if(n1.s != nullptr) {
                    m_s_i1 = ceil(n1.s->length / speed1);
                }
                
                int eta_i_n2 {1};
                if( n2.s == nullptr ||
                    (n2.s->eastbound && d->trains[i].eastbound) ||
                    (n2.s->westbound && d->trains[i].westbound)) {
                    eta_i_n2 = 0;
                }
                                            
                // ********** Obj Function **********
                int obj_coeff {0};
                
                if(n1.s != nullptr) {
                    bool vi2_in_delta_plus_bar_vi1 {false};
                                        
                    oei_t oei, oei_end;
                    for(std::tie(oei, oei_end) = out_edges(*vi1, graphs[i]->g); oei != oei_end; ++oei) {
                        const Node& ntarget = graphs[i]->g[target(*oei, graphs[i]->g)];
                                                            
                        if(ntarget == n2 && !(ntarget.s != nullptr && ntarget.s->id == n1.s->id)) {
                            vi2_in_delta_plus_bar_vi1 = true;
                            break;
                        }
                    }
    
                    if(vi2_in_delta_plus_bar_vi1) {
                        obj_coeff += c_d_i * n1.t;
                    }
                }

                if(n2.s != nullptr) {
                    bool vi1_in_delta_minus_bar_vi2 {false};

                    iei_t iei, iei_end;
                    for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[i]->g); iei != iei_end; ++iei) {
                        const Node& nsource = graphs[i]->g[source(*iei, graphs[i]->g)];
                        if(nsource == n1 && !(nsource.s != nullptr && nsource.s->id == n2.s->id)) {
                            vi1_in_delta_minus_bar_vi2 = true;
                            break;
                        }
                    }

                    if(vi1_in_delta_minus_bar_vi2) {
                        obj_coeff -= c_d_i * (n2.t + m_s_i2 - 1);
                    }
                }
                    
                if(n1.s != nullptr && n2.s != nullptr) {          
                    bool vi1_in_delta_minus_vi2 {false};
        
                    for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[i]->g); iei != iei_end; ++iei) {
                        const Node& nsource = graphs[i]->g[source(*iei, graphs[i]->g)];
                        if(nsource == n1) {
                            vi1_in_delta_minus_vi2 = true;
                            break;
                        }
                    }
        
                    if(vi1_in_delta_minus_vi2) {
                        obj_coeff += eta_i_n2 * d->unpreferred_price;
                    }
                }
                
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
                                                            
                // ********** eq_one_train **********
                int num_row {0};
                for(int ss = 0; ss < ns; ss++) {
                    for(int tt = 1; tt <= ti; tt++) {
                        int coeff {0};
                        
                        if(graphs[i]->vertex_for(d->segments[ss], tt).first) {
                            if(n2.s != nullptr && n2.s->id == ss && n2.t == tt) {
                                bool vi1_in_delta_minus_vi2 {false};
                                
                                iei_t iei, iei_end;
                                for(std::tie(iei, iei_end) = in_edges(*vi2, graphs[i]->g); iei != iei_end; ++iei) {
                                    const Node& nsource = graphs[i]->g[source(*iei, graphs[i]->g)];
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
                                                
                // ********** ensure_sa_visit **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    if(d->trains[ii].schedule_adherence) {
                        for(const auto& kv : d->trains[ii].schedule) {
                            int coeff {0};
                            int segment {kv.first};
                            
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
                                    coeff = 1;
                                }
                            }
                            
                            col += eq_ensure_sa_visit[num_row++](coeff);
                        }
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
                    vi_t vi, vi_end;
                    for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
                        const Node& n = graphs[ii]->g[*vi];
                                                
                        if(n.s != nullptr) {
                            int coeff {0};
                            
                            if(i == ii) {                                
                                if(n == n2) { // First sum
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
                                        coeff = 1;
                                    }
                                } else {
                                    if( n1.s != nullptr &&
                                        n.s->id == n1.s->id &&
                                        n1.t >= n.t &&
                                        n1.t < n.t + m_s_i1 - 1
                                    ) { // Second sum
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
                                            coeff = 1;
                                        }
                                    }
                                }
                            }
                            col += eq_min_time[num_row++](coeff);
                        }
                    }
                }
                
                // ********** eq_exact_time **********
                num_row = 0;
                for(int ii = 0; ii < nt; ii++) {
                    vi_t vi, vi_end;
                    for(std::tie(vi, vi_end) = vertices(graphs[ii]->g); vi != vi_end; ++vi) {
                        const Node& n = graphs[ii]->g[*vi];
                                                
                        if(n.s != nullptr && (n.s->type == 'T' || n.s->type == 'X')) {
                            int coeff {0};
                            
                            if(i == ii) {                                
                                if(n == n2) { // First sum
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
                                        coeff = 1;
                                    }
                                } else {
                                    if( n1.s != nullptr &&
                                        n.s->id == n1.s->id &&
                                        n1.t != n.t + m_s_i1 - 1
                                    ) { // Second sum
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
                                            coeff = 1;
                                        }
                                    }
                                }
                            }
                            col += eq_exact_time[num_row++](coeff);
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
                            
                            if(i == ii) {
                                if(n2 == n) { // First sum
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
                                        coeff = 1;
                                    }
                                }
                            } else { // i != ii
                                if( n2.s != nullptr &&
                                    n2.s->id == n.s->id &&
                                    n2.t >= n.t - d->headway &&
                                    n2.t <= n.t - 1
                                ) { // Second sum
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
                                        coeff = 1;
                                    }
                                }
                            }
                            
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
                            
                            if(i == ii) {
                                if(n2 == n) { // First sum
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
                                        coeff = 1;
                                    }
                                }
                            } else {
                                if( n1.s != nullptr &&
                                    n1.s->id == n.s->id &&
                                    n1.t >= n.t - d->headway &&
                                    n1.t <= n.t - 1
                                ) { // Second sum
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
                                        coeff = 1;
                                    }
                                }
                            }
                            
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
                            
                            if(i == ii) {
                                if(n1 == n) { // First sum
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
                                        coeff = 1;
                                    }
                                }
                            } else {
                                if( n2.s != nullptr &&
                                    n2.s->id == n.s->id &&
                                    n2.t >= n.t + 1 &&
                                    n2.t <= n.t + d->headway
                                ) { // Second sum
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
                                        coeff = 1;
                                    }
                                }
                            }
                            
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
                            
                            if(i == ii) {
                                if(n1 == n) { // First sum
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
                                        coeff = 1;
                                    }
                                }
                            } else {
                                if( n1.s != nullptr &&
                                    n1.s->id == n.s->id &&
                                    n1.t >= n.t + 1 &&
                                    n1.t <= n.t + d->headway
                                ) { // Second sum
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
                                        coeff = 1;
                                    }
                                }
                            }
                            
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
                            
                            if(i == ii) {
                                if(n2 == n) { // First sum
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
                                        coeff = 1;
                                    }
                                }
                            } else { // (i != ii)
                                if( n2.s != nullptr &&
                                    n2.t >= n.t - d->headway &&
                                    n2.t <= n.t + d->headway &&
                                    d->is_main(n2.s, n.s)
                                ) { // Second sum
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
                                        coeff = -1;
                                    }
                                }
                            }
                            
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
                                
                                if(i == ii) {
                                    if(n == n2) { // First sum
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
                                            coeff = 1;
                                        }
                                    }
                                } else { // (i != ii)
                                    if(!d->trains[i].schedule_adherence) {
                                        if( n2.s != nullptr &&
                                            n2.t == n.t &&
                                            d->is_main(n2.s, n.s)
                                        ) { // Second sum
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
                                                coeff = 1;
                                            }
                                        }
                                    }
                                }
                                
                                col += eq_heavy_siding[num_row++](coeff);
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