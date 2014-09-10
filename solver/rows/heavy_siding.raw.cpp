row_num = 0;
for(int i = 0; i < nt; i++) {
    if(d->trains[i].heavy) {
        vi_t vi, vi_end;
        for(std::tie(vi, vi_end) = vertices(graphs[i]->g); vi != vi_end; ++vi) {
            const Node& n = graphs[i]->g[*vi];
            if(n.s != nullptr && n.s->type == 'S') {
                eq_heavy_siding.add(IloRange(env, -IloInfinity, 1.0));
                eq_heavy_siding[row_num++].setName(("heavy_siding_train_" + std::to_string(i) + "_node_" + n.str()).c_str());
            }
        }
    }
}