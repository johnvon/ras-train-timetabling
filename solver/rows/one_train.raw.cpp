row_num = 0;
for(int j = 0; j < ns; j++) {
    for(int k = 1; k <= ti; k++) {
        eq_one_train.add(IloRange(env, -IloInfinity, 1.0));
        eq_one_train[row_num++].setName(("one_train_segment_" + std::to_string(j) + "_time_" + std::to_string(k)).c_str());
    }
}