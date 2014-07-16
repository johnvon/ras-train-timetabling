row_num = 0;
for(int i = 0; i < nt; i++) {
    for(int j = 0; j < ns; j++) {
        eq_set_z.add(IloRange(env, -IloInfinity, 0.0));
        eq_set_z[row_num++].setName(("set_z_train_" + std::to_string(i) + "_segment_" + std::to_string(j)).c_str());
    }
}