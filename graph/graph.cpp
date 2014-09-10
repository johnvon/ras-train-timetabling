#include <graph/graph.h>

#include <algorithm>
#include <iostream>
#include <limits>

Graph::Graph(const std::shared_ptr<const Data> d, const Train tr) : d{d}, tr{tr} {
    Node sigma {nullptr, -1, true, false};
    Node tau {nullptr, -1, false, true};
    
    vertex_t source_vertex {add_vertex(g)};
    vertex_t sink_vertex {add_vertex(g)};
    
    g[source_vertex] = sigma;
    g[sink_vertex] = tau;
    
    std::unordered_map<std::shared_ptr<const Segment>, std::pair<int, int>> times = calculate_times();
    
    for(auto s : d->segments) {
        if(tr.hazmat && (s->type == 'S' || s->type == 'T')) {
            continue;
        }
        
        for(int t = tr.entry_time; t <= d->time_intervals; t++) {
            if(mow(s,t)) {
                continue;
            }
                        
            if(t < times.at(s).first || t > times.at(s).second) {
                continue;
            }
                        
            Node n {s, t};
            vertex_t v {add_vertex(g)};
            g[v] = n;
        }
    }
        
    /****************** 1: sigma -> starting segment ******************/
    
    for(auto s : d->segments) {
        if(s->extreme_1 != tr.origin_node && s->extreme_2 != tr.origin_node) {
            continue;
        }
        
        for(int t = tr.entry_time; t < d->time_intervals; t++) {
            auto ev = vertex_for(s, t);
            
            if(ev.first) {
                vertex_t v {ev.second};
                add_edge(source_vertex, v, g);
            }
        }
    }
        
    /****************** 2: ending segment -> tau ******************/
    
    for(auto s : d->segments) {
        if(s->extreme_1 != tr.destination_node && s->extreme_2 != tr.destination_node) {
            continue;
        }
        
        for(int t = tr.entry_time + 1; t <= d->time_intervals; t++) {
            auto ev = vertex_for(s, t);
            
            if(ev.first) {
                vertex_t v {ev.second};
                add_edge(v, sink_vertex, g);
            }
        }
    }
        
    /****************** 3: (s, t) -> (s, t + 1) ******************/
    
    for(auto s : d->segments) {
        if(s->type != 'T' && s->type != 'X') {
            for(int t = tr.entry_time; t < d->time_intervals; t++) {
                auto ev_1 = vertex_for(s, t);
                auto ev_2 = vertex_for(s, t + 1);
            
                if(ev_1.first && ev_2.first) {
                    vertex_t v_1 {ev_1.second};
                    vertex_t v_2 {ev_2.second};
                    add_edge(v_1, v_2, g);
                }
            }
        }
    }
        
    /****************** 4: (s, t) -> (s', t + 1) ******************/
    
    for(auto s_1 : d->segments) {
        for(int t = tr.entry_time; t < d->time_intervals; t++) {
            for(auto s_2 : d->segments) {
                if((tr.eastbound && s_1->extreme_2 == s_2->extreme_1) || (tr.westbound && s_1->extreme_1 == s_2->extreme_2)) {
                    auto ev_1 = vertex_for(s_1, t);
                    auto ev_2 = vertex_for(s_2, t + 1);
                    
                    if(ev_1.first && ev_2.first) {
                        vertex_t v_1 {ev_1.second};
                        vertex_t v_2 {ev_2.second};
                        add_edge(v_1, v_2, g);
                    }
                }
            }
        }
    }
        
    /****************** 5: cleanup ******************/
    
    bool clean = false;
    while(!clean) {
        clean = true;
        vi_t vi, vi_end, vi_next;
        tie(vi, vi_end) = vertices(g);
        for(vi_next = vi; vi != vi_end; vi = vi_next) {
            ++vi_next;
            int n_out = (int)out_degree(*vi, g);
            int n_in = (int)in_degree(*vi, g);
            if((g[*vi].s != nullptr) && (n_out == 0 || n_in == 0 || n_out + n_in <= 1)) {
                clear_vertex(*vi, g);
                remove_vertex(*vi, g);
                clean = false;
            }
        }
    }
    
    std::cout << "Train: " << tr.id << " (" << tr.cl << "), vertices: " << num_vertices(g) << ", edges: " << num_edges(g) << std::endl;
}

bool Graph::mow(const std::shared_ptr<const Segment> s, const int t) const {
    for(auto m : d->mow) {
        if((s->extreme_1 == m.extreme_1 && s->extreme_2 == m.extreme_2) || (s->extreme_2 == m.extreme_1 && s->extreme_1 == m.extreme_2)) {
            if(t >= m.start_time && t <= m.end_time) {
                return true;
            }
        }
    }
    return false;
}

std::pair<bool, vertex_t> Graph::vertex_for(const std::shared_ptr<const Segment> s, const int t) const {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if(g[*vi].s != nullptr && g[*vi].s->id == s->id && g[*vi].t == t) {
            return std::make_pair(true, *vi);
        }
    }
    return std::make_pair(false, vertex_t());
}

std::pair<bool, vertex_t> Graph::sigma() const {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if(g[*vi].s == nullptr && g[*vi].source) {
            return std::make_pair(true, *vi);
        }
    }
    return std::make_pair(false, vertex_t());
}

std::pair<bool, vertex_t> Graph::tau() const {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if(g[*vi].s == nullptr && g[*vi].sink) {
            return std::make_pair(true, *vi);
        }
    }
    return std::make_pair(false, vertex_t());
}

std::unordered_map<std::shared_ptr<const Segment>, std::pair<int, int>> Graph::calculate_times() const {
    std::unordered_map<std::shared_ptr<const Segment>, std::pair<int, int>> times;
    double speed;
    
    if(tr.westbound) {
        speed = d->speed_ew;
    } else {
        speed = d->speed_we;
    }
    
    speed *= tr.speed_multi;
    
    for(auto s : d->segments) {
        int time_from_w {static_cast<int>(ceil(s->min_distance_from_w / speed))};
        int time_from_e {static_cast<int>(ceil(s->min_distance_from_e / speed))};
        
        if(tr.westbound) {
            times.emplace(s, std::make_pair(time_from_e, d->time_intervals - time_from_w));
        } else {
            times.emplace(s, std::make_pair(time_from_w, d->time_intervals - time_from_e));
        }
    }
    
    return times;
}