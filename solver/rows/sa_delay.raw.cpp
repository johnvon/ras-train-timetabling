row_num = 0;
for(int i = 0; i < nt; i++) {
    if(d->trains[i].schedule_adherence) {
        for(auto kv : d->trains[i].schedule) {
            int segment {kv.first};
            int schedule_time {kv.second};
            int ais {schedule_time + d->schedule_tw_end - 1};
            
            eq_sa_delay.add(IloRange(env, -IloInfinity, ais));
            eq_sa_delay[row_num++].setName(("sa_delay_train_" + std::to_string(i) + "_schedule_for_" + std::to_string(d->segments[segment]->id)).c_str());
        }
    }
}