row_num = 0;
for(int i = 0; i < nt; i++) {
    for(int j = 0; j < ns; j++) {
        for(int k = 0; k < ti; k++) {
            if(graphs[i]->vertex_for(d->segments[j], k).first) {
                eq_headway.add(IloRange(env, -IloInfinity, 2 * d->headway + 1));
                eq_headway[row_num++].setName(("headway_train_" + std::to_string(i) + "_segment_" + std::to_string(j) + "_time_" + std::to_string(k)).c_str());
            }
        }
    }
}