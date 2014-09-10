row_num = 0;
for(int i = 0; i < nt; i++) {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(graphs[i]->g); vi != vi_end; ++vi) {
        const Node& n = graphs[i]->g[*vi];
        if(n.s != nullptr) {
            eq_headway3.add(IloRange(env, -IloInfinity, 1));
            eq_headway3[row_num++].setName(("headway3_train_" + std::to_string(i) + "_node_" + n.str()).c_str());
        }
    }
}