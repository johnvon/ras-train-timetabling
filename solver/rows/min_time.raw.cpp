row_num = 0;
for(int i = 0; i < nt; i++) {
    for(int j = 0; j < ns; j++) {
        eq_min_time.add(IloRange(env, -IloInfinity, 0.0));
        eq_min_time[row_num++].setName(("min_time_train_" + std::to_string(i) + "_segment_" + std::to_string(j)).c_str());
    }
}