row_num = 0;
for(int i = 0; i < nt; i++) {
    if(d->trains[i].schedule_adherence) {
        for(int j = 0; j < d->trains[i].schedule.size(); j++) {
            eq_z_sa.add(IloRange(env, 1.0, 1.0));
            eq_z_sa[row_num++].setName(("z_sa_train_" + std::to_string(i) + "_schedule_" + std::to_string(j)).c_str());
        }
    }
}