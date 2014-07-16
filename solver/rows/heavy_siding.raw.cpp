row_num = 0;
for(int i = 0; i < nt; i++) {
    if(d->trains[i].heavy) {
        for(int j = 0; j < ns; j++) {
            if(d->segments[j]->type == 'S') {
                for(int k = 0; k < ti; k++) {
                    if(graphs[i]->vertex_for(d->segments[j], k).first) {
                        eq_heavy_siding.add(IloRange(env, -IloInfinity, 1.0));
                        eq_heavy_siding[row_num++].setName(("heavy_siding_train_" + std::to_string(i) + "_segment_" + std::to_string(j) + "_time_" + std::to_string(k)).c_str());
                    }
                }
            }
        }
    }
}