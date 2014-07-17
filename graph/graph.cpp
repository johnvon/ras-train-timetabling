#include <graph/graph.h>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

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

// std::unordered_map<std::shared_ptr<const Segment>, std::pair<int, int>> Graph::calculate_times() const {
//     typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_index_t, int>, property<edge_weight_t, double>> tmp_graph_t;
//     typedef graph_traits<tmp_graph_t>::vertex_descriptor tmp_vertex_t;
//     typedef graph_traits<tmp_graph_t>::edge_descriptor tmp_edge_t;
//     typedef graph_traits<tmp_graph_t>::vertex_iterator tmp_vi_t;
//
//     tmp_graph_t tg;
//     tmp_vertex_t source_vertex, sink_vertex;
//     std::vector<int> inserted_junctions;
//
//     for(auto s : d->segments) {
//         tmp_vertex_t v1, v2;
//
//         if(std::find(inserted_junctions.begin(), inserted_junctions.end(), s->extreme_1) != inserted_junctions.end()) {
//             inserted_junctions.push_back(s->extreme_1);
//             v1 = add_vertex(tg);
//             put(vertex_index, tg, v1, s->extreme_1);
//
//             if(s->extreme_1 == tr.origin_node) {
//                 source_vertex = v1;
//             }
//
//             if(s->extreme_1 == tr.destination_node) {
//                 sink_vertex = v1;
//             }
//         } else {
//             tmp_vi_t vi, vi_end;
//             for(std::tie(vi, vi_end) = vertices(tg); vi != vi_end; ++vi) {
//                 if(get(vertex_index, tg, *vi) == s->extreme_1) {
//                     v1 = *vi;
//                     break;
//                 }
//             }
//         }
//
//         if(std::find(inserted_junctions.begin(), inserted_junctions.end(), s->extreme_2) != inserted_junctions.end()) {
//             inserted_junctions.push_back(s->extreme_2);
//             v2 = add_vertex(tg);
//             put(vertex_index, tg, v2, s->extreme_2);
//
//             if(s->extreme_2 == tr.origin_node) {
//                 source_vertex = v2;
//             }
//
//             if(s->extreme_2 == tr.destination_node) {
//                 sink_vertex = v2;
//             }
//         } else {
//             tmp_vi_t vi, vi_end;
//             for(std::tie(vi, vi_end) = vertices(tg); vi != vi_end; ++vi) {
//                 if(get(vertex_index, tg, *vi) == s->extreme_2) {
//                     v2 = *vi;
//                     break;
//                 }
//             }
//         }
//
//         double speed {0};
//
//         if(s->type == '0') {
//             if(tr.westbound) {
//                 speed = d->speed_ew;
//             } else {
//                 speed = d->speed_we;
//             }
//         } else if(s->type == '1') {
//             speed = d->speed_ew;
//         } else if(s->type == '2') {
//             speed = d->speed_we;
//         } else if(s->type == 'S') {
//             speed = d->speed_siding;
//         } else if(s->type == 'T') {
//             speed = d->speed_switch;
//         } else if(s->type == 'X') {
//             speed = d->speed_xover;
//         } else {
//             throw std::runtime_error("Unrecognised segment type!");
//         }
//
//         speed *= tr.speed_multi;
//         double min_time {ceil(s->length / speed)};
//
//         tmp_edge_t e = add_edge(v1, v2, tg).first;
//         put(edge_weight, tg, e, min_time);
//     }
//
//     std::vector<tmp_vertex_t> from_source_predecessor(num_vertices(tg));
//     std::vector<tmp_vertex_t> to_sink_predecessor(num_vertices(tg));
//     std::vector<int> from_source_distance(num_vertices(tg));
//     std::vector<int> to_sink_distance(num_vertices(tg));
//
//     dijkstra_shortest_paths(tg, source_vertex,
//         predecessor_map(
//             make_iterator_property_map(
//                 from_source_predecessor.begin(),
//                 get(vertex_index, tg)
//             )
//         ).
//         distance_map(
//             make_iterator_property_map(
//                 from_source_distance.begin(),
//                 get(vertex_index,tg)
//             )
//         )
//     );
//
//     // ...
//
//     dijkstra_shortest_paths(tg, sink_vertex,
//         predecessor_map(
//             make_iterator_property_map(
//                 from_source_predecessor.begin(),
//                 get(vertex_index, tg)
//             )
//         ).
//         distance_map(
//             make_iterator_property_map(
//                 from_source_distance.begin(),
//                 get(vertex_index,tg)
//             )
//         )
//     );
//
//     // ...
//
//     return std::unordered_map<std::shared_ptr<const Segment>, std::pair<int, int>>();
// }