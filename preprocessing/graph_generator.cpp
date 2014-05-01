#ifndef GRAPH_GENERATOR_CPP
#define GRAPH_GENERATOR_CPP

#include <memory>
using std::make_shared;
#include <tuple>
#include <string>
using std::to_string;
#include <stdexcept>
using std::runtime_error;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <network/graph.h>
#include <preprocessing/data.h>
#include <preprocessing/graph_generator.h>

std::shared_ptr<Graph> GraphGenerator::create_graph(const Data& data, std::shared_ptr<Train> train) {
    std::shared_ptr<Graph> g = make_shared<Graph>(BoostGraph(), train);
    
    if(DEV) {
        cerr << "Building graph for train #" << train->id << endl;
    }
    
    /* Create nodes */
    int node_id = 0;
    for(std::shared_ptr<Junction> j : data.junctions) {
        for(int t = 1; t <= data.num_times; ++t) {
            Vertex v = add_vertex(g->graph);
            g->graph[v] = make_shared<Node>(node_id++, j, t);
        }
    }
    
    if(DEV) {
        cerr << "\tNodes created: " << num_vertices(g->graph) << endl;
    }
    
    /* Create arcs */
    int arc_id = 0;
    for(std::shared_ptr<Track> track : data.tracks) {
        if(track->direction != train->direction) {
            continue; // Since trains can't back up we only create the eastbound arc for eastbound trains and the westbound arc for westbound trains
        }
        
        if(track->type == TrackType::SIDING && train->hazmat) {
            continue; // Inhalation hazard trains can't take sidings
        }
        
        if(track->type == TrackType::SIDING && train->length < track->length) {
            continue; // The siding is too short
        }
        
        std::shared_ptr<Junction> j1 = track->extremes.first;
        std::shared_ptr<Junction> j2 = track->extremes.second;
        
        for(int t = 1; t <= data.num_times; t++) {
            float train_speed = track->max_speed * train->speed_multiplier;
            int min_arrival_time = t + ceil(track->length / train_speed);
            
            if(min_arrival_time > data.num_times) { // We are overflowing the time horizon
                break; // Don't create arcs starting at time t, t+1, ...
            }
            
            for(int s = min_arrival_time; s <= data.num_times; s++) { // Create all possible arcs
                if(is_in_maintenance(track, t, s, data.mow)) {
                    continue; // Check that there is no MOW during the travel period
                }
                
                create_arc(j1, t, j2, s, g, arc_id++, track);
                // if(DEV) {
                //     cout << "Created arc from (" << j1->id << ", " << t << ") to (" << j2->id << ", " << s << ")" << endl;
                // }
            }
        }
    }
    
    if(DEV) {
        cerr << "\tArcs created: " << num_edges(g->graph) << endl;
    }
    
    clean_graph(g);
    
    if(DEV) {
        cerr << "\tGraph cleaned. New number of nodes: " << num_vertices(g->graph) << "; new number of arcs: " << num_edges(g->graph) << endl;
    }
    
    return g;
}

void GraphGenerator::clean_graph(std::shared_ptr<Graph> g) {
    bool clean = false;
    
    if(DEV) {
        cerr << "\tCleaning ";
    }
    
    // Remove vertices without in/out arcs
    while(!clean) {
        if(DEV) {
            cerr << ".";
        }
        
        
        vit vi, vi_end, vi_next;
        tie(vi, vi_end) = vertices(g->graph);
        clean = true;
        
        for(vi_next = vi; vi != vi_end; vi = vi_next) {
            ++vi_next;
            int n_out = (int)out_degree(*vi, g->graph);
            int n_in = (int)in_degree(*vi, g->graph);
            if(!g->graph[*vi]->junction->is_terminal && (n_out == 0 || n_in == 0)) {
                // if(DEV) {
                //     cout << "Removing (" << g->graph[*vi]->junction->id << ", " << g->graph[*vi]->time_interval << "): in = " << n_in << ", out = " << n_out << endl;
                // }
                clear_vertex(*vi, g->graph);
                remove_vertex(*vi, g->graph);
                clean = false;
            }
        }
    }
    
    if(DEV) {
        cerr << endl;
    }
}

void GraphGenerator::create_arc(std::shared_ptr<Junction> j1, const int t1, std::shared_ptr<Junction> j2, const int t2, std::shared_ptr<Graph> g, const int id, std::shared_ptr<Track> track) {
    bool found1, found2;
    Vertex v1, v2;
    
    tie(found1, v1) = g->get_vertex(j1, t1);
    
    if(!found1) {
        throw runtime_error("Couldn't find vertex for junction " + to_string(j1->id) + " at time " + to_string(t1));
    }
    
    tie(found2, v2) = g->get_vertex(j2, t2);
    
    if(!found2) {
        throw runtime_error("Couldn't find vertex for junction " + to_string(j2->id) + " at time " + to_string(t2));
    }
    
    Edge e = add_edge(v1, v2, g->graph).first;
    g->graph[e] = make_shared<Arc>(id, track);
}

bool GraphGenerator::is_in_maintenance(std::shared_ptr<Track> track, const int t1, const int t2, const vector<Mow>& mow) {
    for(const Mow& m : mow) {
        std::shared_ptr<Junction> mj1 = nullptr, mj2 = nullptr;
        int mst = -1, met = -1;
        
        std::tie(mj1, mj2, mst, met) = m;
        
        if((mj1 == track->extremes.first && mj2 == track->extremes.second) || (mj2 == track->extremes.first && mj1 == track->extremes.second)) {
            if((t1 >= mst && t1 <= met) || (t2 >= mst && t2 <= met)) {
                return true;
            }
        }
    }
    return false;
}

#endif