row_num = 0;
for(int i = 0; i < nt; i++) {
    int W2 {d->trains[i].terminal_wt + d->want_time_tw_end};
    eq_wt_2.add(IloRange(env, -IloInfinity, W2));
    eq_wt_2[row_num++].setName(("wt_2_train_" + std::to_string(i)).c_str());
}