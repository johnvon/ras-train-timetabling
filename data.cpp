#include <data.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/range/algorithm/count.hpp>

using namespace boost::property_tree;
using namespace boost;

auto data::read_speeds(const ptree& pt) -> void {
    speed_ew = pt.get<double>("speed_ew");
    speed_we = pt.get<double>("speed_we");
    speed_siding = pt.get<double>("speed_siding");
    speed_switch = pt.get<double>("speed_switch");
    speed_xover = pt.get<double>("speed_xover");
}

auto data::read_relevant_times(const ptree& pt) -> void {
    wt_tw_left = pt.get<int>("want_time_tw_start");
    wt_tw_right = pt.get<int>("want_time_tw_end");
    sa_tw_right = pt.get<int>("schedule_tw_end");
    headway = pt.get<int>("headway");
}

auto data::read_prices(const ptree& pt) -> void {
    for(char cl = 'A'; cl <= 'F'; cl++) {
        auto price = pt.get_child("general_delay_price").get<double>(std::string(1,cl));
        delay_price.emplace(cl, price);
    }
    
    wt_price = pt.get<double>("terminal_delay_price");
    sa_price = pt.get<double>("schedule_delay_price");
    unpreferred_price = pt.get<double>("unpreferred_price");
}

auto data::read_segments(const ptree& pt) -> void {
    // Sigma:
    seg_w_ext.push_back(-1002);
    seg_e_ext.push_back(-1001);
    seg_w_min_dist.push_back(-1000);
    seg_e_min_dist.push_back(-1000);
    seg_type.push_back('D');
    seg_length.push_back(-1);
    seg_siding_length.push_back(-1);
    seg_eastbound.push_back(false);
    seg_westbound.push_back(false);
    
    BOOST_FOREACH(const ptree::value_type& segment_child, pt.get_child("segments")) {
        seg_w_ext.push_back(segment_child.second.get<int>("extreme_1"));
        seg_e_ext.push_back(segment_child.second.get<int>("extreme_2"));
        
        seg_w_min_dist.push_back(segment_child.second.get<double>("min_distance_from_w"));
        seg_e_min_dist.push_back(segment_child.second.get<double>("min_distance_from_e"));
        
        seg_type.push_back(segment_child.second.get<char>("type"));
        seg_length.push_back(segment_child.second.get<double>("length"));
        if(seg_type.back() == 'S') {
            seg_siding_length.push_back(segment_child.second.get<double>("siding_length"));
        } else {
            seg_siding_length.push_back(-1);
        }
        
        seg_eastbound.push_back(segment_child.second.get<bool>("eastbound"));
        seg_westbound.push_back(segment_child.second.get<bool>("westbound"));
    }
    
    // Tau:
    seg_w_ext.push_back(-999);
    seg_e_ext.push_back(-998);
    seg_w_min_dist.push_back(-1000);
    seg_e_min_dist.push_back(-1000);
    seg_type.push_back('D');
    seg_length.push_back(-1);
    seg_siding_length.push_back(-1);
    seg_eastbound.push_back(false);
    seg_westbound.push_back(false);
    
    assert((int)seg_w_ext.size() == ns + 2);
    assert((int)seg_e_ext.size() == ns + 2);
    assert((int)seg_w_min_dist.size() == ns + 2);
    assert((int)seg_e_min_dist.size() == ns + 2);
    assert((int)seg_type.size() == ns + 2);
    assert((int)seg_length.size() == ns + 2);
    assert((int)seg_siding_length.size() == ns + 2);
    assert((int)seg_eastbound.size() == ns + 2);
    assert((int)seg_westbound.size() == ns + 2);
}

auto data::read_trains(const ptree& pt) -> void {
    BOOST_FOREACH(const ptree::value_type& train_child, pt.get_child("trains")) {
        tr_class.push_back(train_child.second.get<char>("class"));
        tr_sa.push_back(train_child.second.get<bool>("schedule_adherence"));
        tr_entry_time.push_back(train_child.second.get<int>("entry_time"));
        tr_orig_ext.push_back(train_child.second.get<int>("origin_node"));
        tr_dest_ext.push_back(train_child.second.get<int>("destination_node"));
        tr_eastbound.push_back(train_child.second.get<bool>("eastbound"));
        tr_westbound.push_back(train_child.second.get<bool>("westbound"));
        tr_speed_mult.push_back(train_child.second.get<double>("speed_multi"));
        tr_length.push_back(train_child.second.get<double>("length"));
        tr_tob.push_back(train_child.second.get<int>("tob"));
        tr_heavy.push_back(tr_tob.back() > 100);
        tr_hazmat.push_back(train_child.second.get<bool>("hazmat"));
        tr_wt.push_back(train_child.second.get<int>("terminal_wt"));
        
        sa_ext.push_back(ivec());
        sa_times.push_back(ivec());
        BOOST_FOREACH(const ptree::value_type& schedule_child, train_child.second.get_child("schedule")) {
            auto extreme = schedule_child.second.get<int>("node");
            auto ti = schedule_child.second.get<int>("time");
            
            if(ti <= ni) {
                sa_ext.back().push_back(extreme);
                sa_times.back().push_back(ti);
            }
        }
    }
    
    assert((int)tr_class.size() == nt);
    assert((int)tr_sa.size() == nt);
    assert((int)tr_entry_time.size() == nt);
    assert((int)tr_orig_ext.size() == nt);
    assert((int)tr_dest_ext.size() == nt);
    assert((int)tr_eastbound.size() == nt);
    assert((int)tr_westbound.size() == nt);
    assert((int)tr_speed_mult.size() == nt);
    assert((int)tr_length.size() == nt);
    assert((int)tr_tob.size() == nt);
    assert((int)tr_hazmat.size() == nt);
    assert((int)tr_wt.size() == nt);
    assert((int)sa_ext.size() == nt);
    assert((int)sa_times.size() == nt);
}

auto data::read_mows(const ptree& pt) -> void {
    BOOST_FOREACH(const ptree::value_type& mow_child, pt.get_child("mow")) {
        mow_ext_w.push_back(mow_child.second.get<int>("extreme_1"));
        mow_ext_e.push_back(mow_child.second.get<int>("extreme_2"));
        mow_start_times.push_back(mow_child.second.get<int>("start_time"));
        mow_end_times.push_back(mow_child.second.get<int>("end_time"));
    }
}

auto data::calculate_max_speeds() -> void {
    tr_max_speed = bv<double>(nt, 0);
    
    for(auto i = 0; i < nt; i++) {
        tr_max_speed[i] = tr_speed_mult[i] * (tr_westbound[i] ? speed_ew : speed_we);
    }
}

auto data::calculate_train_orig_dest_segments() -> void {
    tr_orig_seg = int_matrix(nt, ivec());
    tr_dest_seg = int_matrix(nt, ivec());
        
    for(auto i = 0; i < nt; i++) {
        for(auto s = 0; s < ns + 2; s++) {
            if(tr_eastbound[i] && seg_w_ext[s] == tr_orig_ext[i]) {
                tr_orig_seg[i].push_back(s);
            }
            if(tr_westbound[i] && seg_e_ext[s] == tr_orig_ext[i]) {
                tr_orig_seg[i].push_back(s);
            }
            if(tr_eastbound[i] && seg_e_ext[s] == tr_dest_ext[i]) {
                tr_dest_seg[i].push_back(s);
            }
            if(tr_westbound[i] && seg_w_ext[s] == tr_dest_ext[i]) {
                tr_dest_seg[i].push_back(s);
            }
        }
        
        assert(tr_orig_seg[i].size() > 0);
        assert(tr_dest_seg[i].size() > 0);
    }
}

auto data::calculate_mows() -> void {
    mow = indicator_matrix(ns + 2, bvec(ni + 2, false));
    
    assert(mow_ext_e.size() == mow_ext_w.size());
    assert(mow_start_times.size() == mow_end_times.size());
    assert(mow_ext_e.size() == mow_start_times.size());
        
    for(auto m = 0u; m < mow_ext_e.size(); m++) {
        for(auto s = 0; s < ns + 2; s++) {
            if(seg_e_ext[s] == mow_ext_e[m] && seg_w_ext[s] == mow_ext_w[m]) {
                for(auto t = mow_start_times[m]; t <= mow_end_times[m]; t++) {
                    mow[s][t] = true;
                }
            }
        }
    }
}

auto data::calculate_schedules() -> void {
    assert((int)sa_ext.size() == nt);
    assert((int)sa_times.size() == nt);
    
    sa_num = ivec(nt, 0);
    segments_for_sa = int_matrix_3d(nt);
    
    for(auto i = 0; i < nt; i++) {
        if(!tr_sa[i]) {
            assert(sa_ext[i].size() == 0);
            assert(sa_times[i].size() == 0);
        } else {
            assert(sa_ext[i].size() == sa_times[i].size());
            sa_num[i] = sa_ext[i].size();
            segments_for_sa[i] = int_matrix(sa_ext[i].size());
            
            for(auto n = 0u; n < sa_ext[i].size(); n++) {
                for(auto s = 0; s < ns + 2; s++) {
                    if( (tr_westbound[i] && seg_w_ext[s] == sa_ext[i][n]) ||
                        (tr_eastbound[i] && seg_e_ext[s] == sa_ext[i][n])
                    ) {
                        segments_for_sa[i][n].push_back(s);
                    }
                }
            }
        }
    }
}

auto data::calculate_network() -> void {
    network = indicator_matrix(ns + 2, bvec(ns + 2, false));
    tnetwork = int_matrix_3d(nt, int_matrix(ns + 2));
    inverse_tnetwork = int_matrix_3d(nt, int_matrix(ns + 2));
    bar_tnetwork = int_matrix_3d(nt, int_matrix(ns + 2));
    bar_inverse_tnetwork = int_matrix_3d(nt, int_matrix(ns + 2));
    
    for(auto s1 = 0; s1 < ns + 2; s1++) {
        if(seg_type[s1] == 'S') {
            sidings.push_back(s1);
        }
        if(seg_type[s1] == 'X') {
            xovers.push_back(s1);
        }
        
        for(auto i = 0; i < nt; i++) {
            tnetwork[i][s1].push_back(s1);
            inverse_tnetwork[i][s1].push_back(s1);
            
            if(std::find(tr_orig_seg[i].begin(), tr_orig_seg[i].end(), s1) != tr_orig_seg[i].end()) {
                tnetwork[i][0].push_back(s1);
                inverse_tnetwork[i][s1].push_back(0);
                bar_tnetwork[i][0].push_back(s1);
                bar_inverse_tnetwork[i][s1].push_back(0);
            }
            
            if(std::find(tr_dest_seg[i].begin(), tr_dest_seg[i].end(), s1) != tr_dest_seg[i].end()) {
                tnetwork[i][s1].push_back(ns + 1);
                inverse_tnetwork[i][ns + 1].push_back(s1);
                bar_tnetwork[i][s1].push_back(ns + 1);
                bar_inverse_tnetwork[i][ns + 1].push_back(s1);
            }
        }
        
        for(auto s2 = 0; s2 < ns + 2; s2++) {
            if(seg_e_ext[s1] == seg_w_ext[s2] || seg_w_ext[s1] == seg_e_ext[s2]) {
                network[s1][s2] = true;
                for(auto i = 0; i < nt; i++) {
                    if((seg_e_ext[s1] == seg_w_ext[s2] && tr_eastbound[i]) || (seg_w_ext[s1] == seg_e_ext[s2] && tr_westbound[i])) {
                        tnetwork[i][s1].push_back(s2);
                        bar_tnetwork[i][s1].push_back(s2);
                    }
                    if((seg_e_ext[s1] == seg_w_ext[s2] && tr_westbound[i]) || (seg_w_ext[s1] == seg_e_ext[s2] && tr_eastbound[i])) {
                        inverse_tnetwork[i][s1].push_back(s2);
                        bar_inverse_tnetwork[i][s1].push_back(s2);
                    }
                }
            }
        }
    }
    
    for(auto s1 = 0; s1 < ns + 2; s1++) {
        assert(network[s1][s1] == false);
        for(auto s2 = s1 + 1; s2 < ns + 2; s2++) {
            assert(network[s1][s2] == network[s2][s1]);
            for(auto i = 0; i < nt; i++) {
                assert(std::find(tnetwork[i][s1].begin(), tnetwork[i][s1].end(), s1) != tnetwork[i][s1].end());
                assert(
                    (std::find(tnetwork[i][s1].begin(), tnetwork[i][s1].end(), s2) != tnetwork[i][s1].end()) ==
                    (std::find(inverse_tnetwork[i][s2].begin(), inverse_tnetwork[i][s2].end(), s1) != inverse_tnetwork[i][s2].end())
                );
            }
        }
    }
}

auto data::calculate_auxiliary_data() -> void {
    min_time_to_arrive_at = int_matrix(nt, ivec(ns + 2, 0));
    max_time_to_leave_from = int_matrix(nt, ivec(ns + 2, ni));
    min_travel_time = int_matrix(nt, ivec(ns + 2, -1));
    max_travel_time = int_matrix(nt, ivec(ns + 2, -1));
    unpreferred = indicator_matrix(nt, bvec(ns + 2, false));
    unpreferred_segments = int_matrix(nt);
        
    for(auto i = 0; i < nt; i++) {
        for(auto s = 1; s < ns + 1; s++) {
            auto time_from_w = (int) std::ceil(seg_w_min_dist[s] / tr_max_speed[i]);
            auto time_from_e = (int) std::ceil(seg_e_min_dist[s] / tr_max_speed[i]);
            
            min_time_to_arrive_at[i][s] = (tr_westbound[i] ? time_from_e : time_from_w) + tr_entry_time[i];
            max_time_to_leave_from[i][s] = (tr_westbound[i] ? (ni - time_from_w) : (ni - time_from_e));
            
            if((tr_westbound[i] && !seg_westbound[s]) || (tr_eastbound[i] && !seg_eastbound[s])) {
                unpreferred_segments[i].push_back(s);
                unpreferred[i][s] = true;
            }
            
            auto speed = 0.0;
            auto speed_aux = 0.0;
            if(seg_type[s] == '0' || seg_type[s] == '1' || seg_type[s] == '2') {
                if(tr_westbound[i]) {
                    speed = speed_ew;
                } else {
                    speed = speed_we;
                }
            } else if(seg_type[s] == 'S') {
                speed = speed_siding;
                speed_aux = speed_switch;
            } else if(seg_type[s] == 'X') {
                speed = speed_xover;
            }
            
            speed *= tr_speed_mult[i];
            speed_aux *= tr_speed_mult[i];
            
            if(seg_type[s] != 'S') {
                min_travel_time[i][s] = std::ceil(seg_length[s] / speed);
            } else {
                min_travel_time[i][s] = std::ceil(seg_siding_length[s] / speed) +
                                        std::ceil((seg_length[s] - seg_siding_length[s]) / speed_aux);
            }
            max_travel_time[i][s] = min_travel_time[i][s] + std::ceil(total_cost_ub / delay_price[tr_class[i]]);
        }
    }
    
    main_tracks = int_matrix(ns + 2);
    for(auto s = 1; s < ns + 1; s++) {
        for(auto m = 1; m < ns + 1; m++) {
            auto is_main = (
                seg_type[s] == 'S' &&
                (seg_type[m] == '0' || seg_type[m] == '1' || seg_type[m] == '2') &&
                (seg_e_ext[s] == seg_e_ext[m] || seg_w_ext[s] == seg_w_ext[m])
            );
            if(is_main) {
                main_tracks[s].push_back(m);
            }
        }
    }
}

auto data::calculate_vertices() -> void {
    v = vertices_map(nt, indicator_matrix(ns + 2, bvec(ni + 2, false)));
    v_for_someone = indicator_matrix(ns + 2, bvec(ni + 2, false));
    trains_for = int_matrix_3d(ns + 2, int_matrix(ni + 2));
    last_time_we_need_sigma = ivec(nt, 0);
    first_time_we_need_tau = ivec(nt, ni + 1);
    
    for(auto i = 0; i < nt; i++) {                
        for(auto s = 1; s < ns + 1; s++) {
            if(tr_hazmat[i] && seg_type[s] == 'S') {
                continue;
            }
            
            if(std::find(tr_orig_seg[i].begin(), tr_orig_seg[i].end(), s) != tr_orig_seg[i].end()) {
                if(max_time_to_leave_from[i][s] - min_travel_time[i][s] - 1 > last_time_we_need_sigma[i]) {
                    last_time_we_need_sigma[i] = max_time_to_leave_from[i][s] - min_travel_time[i][s] - 1;
                }
            }
            
            if(std::find(tr_dest_seg[i].begin(), tr_dest_seg[i].end(), s) != tr_dest_seg[i].end()) {
                if(min_time_to_arrive_at[i][s] + min_travel_time[i][s] - 1 < first_time_we_need_tau[i]) {
                    first_time_we_need_tau[i] = min_time_to_arrive_at[i][s] + min_travel_time[i][s] - 1;
                }
            }
            
            for(auto t = min_time_to_arrive_at[i][s]; t <= max_time_to_leave_from[i][s]; t++) {
                if(mow[s][t]) {
                    continue;
                }
                
                if(p.heuristics.corridor) {
                    if(t > p.heuristics.corridor_pct_around_ideal * (min_time_to_arrive_at[i][s] + min_travel_time[i][s]) + p.heuristics.corridor_minutes_around_ideal) {
                        continue;
                    }
                }
                
                v[i][s][t] = true;
                v_for_someone[s][t] = true;
                trains_for[s][t].push_back(i);
            }
        }
        
        for(auto t = 0; t <= last_time_we_need_sigma[i]; t++) {
            v[i][0][t] = true;
            trains_for[0][t].push_back(i);
        }
        
        for(auto t = first_time_we_need_tau[i]; t <= ni + 1; t++) {
            v[i][ns + 1][t] = true;
            trains_for[ns + 1][t].push_back(i);
        }
    }
}

auto data::generate_sigma_s_arcs() -> void {
    for(auto i = 0; i < nt; i++) {
        for(auto s : tr_orig_seg[i]) {
            for(auto t = 1; t <= max_time_to_leave_from[i][s] - min_travel_time[i][s]; t++) {
                if(
                    (t + nt - max_time_to_leave_from[i][s] > tr_wt[i] + wt_tw_right) &&
                    (wt_price * (t + (nt - max_time_to_leave_from[i][s]) - (tr_wt[i] + wt_tw_right)) > total_cost_ub)
                ) {
                    continue;
                }
                
                if(v[i][0][t-1] && v[i][s][t]) {
                    adj[i][0][t-1][s] = true;
                    n_out[i][0][t-1]++;
                    n_in[i][s][t]++;
                }
            }
        }
    }
}

auto data::generate_s_tau_arcs() -> void {
    for(auto i = 0; i < nt; i++) {
        for(auto s : tr_dest_seg[i]) {
            for(auto t = min_time_to_arrive_at[i][s] + min_travel_time[i][s] - 1; t <= ni; t++) {
                if(v[i][s][t]) {
                    if(
                        (t < tr_wt[i] - wt_tw_left) &&
                        (wt_price * (tr_wt[i] - wt_tw_left - t) > total_cost_ub)
                    ) {
                        continue;
                    }
                    
                    if(
                        (t > tr_wt[i] + wt_tw_right) &&
                        (wt_price * (t - tr_wt[i] - wt_tw_right) > total_cost_ub)
                    ) {
                        continue;
                    }
                    
                    if(v[i][ns+1][t+1] && v[i][s][t]) {
                        adj[i][s][t][ns+1] = true;
                        n_out[i][s][t]++;
                        n_in[i][ns+1][t+1]++;
                    }
                }
            }
        }
    }
}

auto data::generate_stop_arcs() -> void {
    for(auto i = 0; i < nt; i++) {
        for(auto s = 1; s < ns + 1; s++) {
            auto is_sa = false;
            auto sa_number = -1;
            
            if(tr_sa[i]) {
                for(auto n = 0; n < sa_num[i]; n++) {
                    if(std::find(segments_for_sa[i][n].begin(), segments_for_sa[i][n].end(), s) != segments_for_sa[i][n].end()) {
                        is_sa = true;
                        sa_number = n;
                    }
                }
            }
            
            for(auto t = min_time_to_arrive_at[i][s]; t <= max_time_to_leave_from[i][s] - 1; t++) {
                if(t + 1 <= ni && v[i][s][t] && v[i][s][t + 1]) {
                    if(is_sa && t+1 > sa_times[i][sa_number] + sa_tw_right && sa_price * (t+1 - (sa_times[i][sa_number] + sa_tw_right)) > total_cost_ub) {
                        continue;
                    }
                    
                    if(
                        (t+1 + nt - max_time_to_leave_from[i][s] > tr_wt[i] + wt_tw_right) &&
                        (wt_price * (t+1 + (nt - max_time_to_leave_from[i][s]) - (tr_wt[i] + wt_tw_right)) > total_cost_ub)
                    ) {
                        continue;
                    }
                    
                    if(v[i][s][t] && v[i][s][t+1]) {
                        adj[i][s][t][s] = true;
                        n_out[i][s][t]++;
                        n_in[i][s][t+1]++;
                    }
                }
            }
        }
    }
}

auto data::generate_movement_arcs() -> void {
    for(auto i = 0; i < nt; i++) {
        for(auto s1 = 1; s1 < ns + 1; s1++) {
            auto is_sa_1 = false;
            auto sa_number_1 = -1;
            
            if(tr_sa[i]) {
                for(auto n = 0; n < sa_num[i]; n++) {
                    if(std::find(segments_for_sa[i][n].begin(), segments_for_sa[i][n].end(), s1) != segments_for_sa[i][n].end()) {
                        is_sa_1 = true;
                        sa_number_1 = n;
                    }
                }
            }
            
            for(auto s2 = 1; s2 < ns + 1; s2++) {
                if(network[s1][s2] && (
                    (tr_eastbound[i] && seg_e_ext[s1] == seg_w_ext[s2]) ||
                    (tr_westbound[i] && seg_w_ext[s1] == seg_e_ext[s2])
                )) {
                    auto is_sa_2 = false;
                    auto sa_number_2 = -1;
            
                    if(tr_sa[i]) {
                        for(auto n = 0; n < sa_num[i]; n++) {
                            if(std::find(segments_for_sa[i][n].begin(), segments_for_sa[i][n].end(), s2) != segments_for_sa[i][n].end()) {
                                is_sa_2 = true;
                                sa_number_2 = n;
                            }
                        }
                    }
                    
                    for(auto t = min_time_to_arrive_at[i][s1] + min_travel_time[i][s1] - 1; t <= max_time_to_leave_from[i][s2] - min_travel_time[i][s2]; t++) {
                        if(is_sa_1 && t > sa_times[i][sa_number_1] + sa_tw_right && sa_price * (t - (sa_times[i][sa_number_1] + sa_tw_right)) > total_cost_ub) {
                            continue;
                        }
                        
                        if(
                            (t + nt - max_time_to_leave_from[i][s1] > tr_wt[i] + wt_tw_right) &&
                            (wt_price * (t + (nt - max_time_to_leave_from[i][s1]) - (tr_wt[i] + wt_tw_right)) > total_cost_ub)
                        ) {
                            continue;
                        }
                        
                        if(
                            (t+1 + nt - max_time_to_leave_from[i][s2] > tr_wt[i] + wt_tw_right) &&
                            (wt_price * (t + (nt - max_time_to_leave_from[i][s2]) - (tr_wt[i] + wt_tw_right)) > total_cost_ub)
                        ) {
                            continue;
                        }

                        if(is_sa_2 && t+1 > sa_times[i][sa_number_2] + sa_tw_right && sa_price * (t+1 - (sa_times[i][sa_number_2] + sa_tw_right)) > total_cost_ub) {
                            continue;
                        }
                        
                        if(v[i][s1][t] && v[i][s2][t+1]) {
                            adj[i][s1][t][s2] = true;
                            n_out[i][s1][t]++;
                            n_in[i][s2][t+1]++;
                        }
                    }
                }
            }
        }
    }
}

auto data::cleanup_adjacency() -> void {     
    for(auto i = 0; i < nt; i++) {
        auto clean = false;
            
        while(!clean) {
            clean = true;
            
            for(auto s1 = 0; s1 < ns + 2; s1++) {
                for(auto t1 = 0; t1 < ni + 2; t1++) {
                    auto _in = n_in[i][s1][t1];
                    auto _out = n_out[i][s1][t1];
                    
                    if(!v[i][s1][t1]) {
                        assert(_in == 0);
                        assert(_out == 0);
                    } else {
                        if( (s1 != 0 && s1 != ns + 1 && (_in == 0 || _out == 0 || _in + _out <= 1)) ||
                            ((s1 == 0 || s1 == ns + 1) && (_in + _out == 0))
                        ) {
                            for(auto s2 = 0; s2 < ns + 2; s2++) {
                                if(t1 < ni + 1 && adj[i][s1][t1][s2]) {
                                    adj[i][s1][t1][s2] = false;
                                    n_in[i][s2][t1+1]--;
                                }
                                if(t1 > 0 && adj[i][s2][t1-1][s1]) {
                                    adj[i][s2][t1-1][s1] = false;
                                    n_out[i][s2][t1-1]--;
                                }
                            }
                            
                            n_in[i][s1][t1] = 0;
                            n_out[i][s1][t1] = 0;
                            v[i][s1][t1] = false;
                            
                            auto still_valid_vertex = false;
                            for(auto i = 0; i < nt; i++) {
                                if(v[i][s1][t1]) {
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

auto data::calculate_adjacency() -> void {
    std::cout << "\t\t\tAllocating memory" << std::endl;
    adj = graph_adjacency_map(nt, indicator_matrix_3d(ns + 2, indicator_matrix(ni + 2, bvec(ns + 2, false))));
    n_in = vertex_count_matrix(nt, int_matrix(ns + 2, ivec(ni + 2, 0)));
    n_out = vertex_count_matrix(nt, int_matrix(ns + 2, ivec(ni + 2, 0)));
    
    std::cout << "\t\t\tSigma - s arcs" << std::endl;
    generate_sigma_s_arcs();
    std::cout << "\t\t\tS - tau arcs" << std::endl;
    generate_s_tau_arcs();
    std::cout << "\t\t\tStop arcs" << std::endl;
    generate_stop_arcs();
    std::cout << "\t\t\tMovement arcs" << std::endl;
    generate_movement_arcs();
    std::cout << "\t\t\tClean-up" << std::endl;
    cleanup_adjacency();
}

auto data::print_adjacency() const -> void {
    for(auto i = 0; i < nt; i++) {
        std::cout << "== Train " << i << std::endl;
        for(auto s1 = 0; s1 < ns + 2; s1++) {
            for(auto t1 = 0; t1 < ni + 2; t1++) {
                for(auto s2 = 0; s2 < ns + 2; s2++) {
                    if(adj[i][s1][t1][s2]) {
                        std::cout << "(" << s1 << "," << t1 << ") => (" << s2 << "," << t1+1 << ")" << std::endl;
                    }
                }
            }
        }
    }
}

data::data(const std::string& file_name, const params& p) : file_name{file_name}, p{p} {
    using namespace std::chrono;
    high_resolution_clock::time_point t_start, t_end;
    duration<double> time_span;
    
    ptree pt;
    read_json(file_name, pt);
        
    instance_name = pt.get<std::string>("name");
    
    nt = pt.get<int>("trains_number");
    ns = pt.get<int>("segments_number");
    ni = pt.get<int>("time_intervals");
    total_cost_ub = pt.get<double>("total_cost_ub");
    
    read_speeds(pt);
    read_relevant_times(pt);
    read_prices(pt);
    read_segments(pt);
    read_trains(pt);
    read_mows(pt);
    
    std::cout << "Creating graphs..." << std::endl;
    
    t_start = high_resolution_clock::now();
    
    std::cout << "\t\tCalculating origin and destination segments..." << std::endl;
    calculate_max_speeds();
    calculate_train_orig_dest_segments();
    std::cout << "\t\tCalculating MOWs..." << std::endl;
    calculate_mows();
    std::cout << "\t\tCalculating schedules for SA trains..." << std::endl;
    calculate_schedules();
    std::cout << "\t\tCalculating network..." << std::endl;
    calculate_network();
    std::cout << "\t\tCalculating auxiliary data..." << std::endl;
    calculate_auxiliary_data();
    std::cout << "\t\tCalculating vertices..." << std::endl;
    calculate_vertices();
    std::cout << "\t\tCalculating adjacency matrix..." << std::endl;
    calculate_adjacency();
    
    t_end = high_resolution_clock::now();
    time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "\t" << time_span.count() << " seconds" << std::endl;
        
    // print_adjacency();
}