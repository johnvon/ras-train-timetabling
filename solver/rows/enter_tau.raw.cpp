row_num = 0;
for(int i = 0; i < nt; i++) {
    eq_enter_tau.add(IloRange(env, 1.0, 1.0));
    eq_enter_tau[row_num++].setName(("_enter_tau_train_" + std::to_string(i)).c_str());
}