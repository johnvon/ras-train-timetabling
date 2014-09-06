row_num = 0;
for(int i = 0; i < nt; i++) {
    if(d->trains[i].schedule_adherence) {
        for(int j = 0; j < d->trains[i].schedule.size(); j++) {
            eq_ensure_sa_visit.add(IloRange(env, 1.0, IloInfinity));
            eq_ensure_sa_visit[row_num++].setName(("X_ensure_sa_visit_train_" + std::to_string(i) + "_schedule_" + std::to_string(j)).c_str());
        }
    }
}