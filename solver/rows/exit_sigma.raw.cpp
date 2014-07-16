row_num = 0;
for(int i = 0; i < nt; i++) {
    eq_exit_sigma.add(IloRange(env, 1.0, 1.0));
    eq_exit_sigma[row_num++].setName(("_exit_sigma_train_" + std::to_string(i)).c_str());
}