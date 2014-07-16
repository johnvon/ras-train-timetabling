row_num = 0;
for(int i = 0; i < nt; i++) {
    for(int j = 0; j < ns; j++) {
        eq_set_theta.add(IloRange(env, 0.0, 0.0));
        eq_set_theta[row_num++].setName(("set_theta_train_" + std::to_string(i) + "_segment_" + std::to_string(j)).c_str());
    }
}