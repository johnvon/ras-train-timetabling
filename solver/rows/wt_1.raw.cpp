row_num = 0;
for(int i = 0; i < nt; i++) {
    int W1 {d->trains[i].terminal_wt - d->want_time_tw_start};
    eq_wt_1.add(IloRange(env, -IloInfinity, -W1));
    eq_wt_1[row_num++].setName(("wt_1_train_" + std::to_string(i)).c_str());
}