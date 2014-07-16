row_num = 0;
for(int i = 0; i < nt; i++) {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(graphs[i]->g); vi != vi_end; ++vi) {
        const Node& n = graphs[i]->g[*vi];
        if(n.s != nullptr) {
            eq_set_y.add(IloRange(env, 0.0, 0.0));
            eq_set_y[row_num++].setName(("set_y_train_" + std::to_string(i) + "_node_" + n.str()).c_str());
        }
    }
}