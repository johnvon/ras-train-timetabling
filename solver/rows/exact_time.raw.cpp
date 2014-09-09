row_num = 0;
for(int i = 0; i < nt; i++) {
    vi_t vi, vi_end;
    for(std::tie(vi, vi_end) = vertices(graphs[i]->g); vi != vi_end; ++vi) {
        const Node& n = graphs[i]->g[*vi];
        if(n.s != nullptr && (n.s->type == 'T' || n.s->type == 'X')) {
            eq_exact_time.add(IloRange(env, -IloInfinity, 1.0));
            eq_exact_time[row_num++].setName(("X_exact_time_train_" + std::to_string(i) + "_node_" + n.str()).c_str());
        }
    }
}