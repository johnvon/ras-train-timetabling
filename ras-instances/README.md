`data`
------

This folder contains the RAS competition instances:

* `raw` contains the raw data, as distributed for the competition
* `normal_tw` contains translations of the raw data in the JSON format
* `tight_tw` contains instances based upon those in `normal_tw`, but where the profile penalty for want times and schedule adherence points doesn't have a plateau: e.g., a train gets zero penalty only if he arrives exactly at the want time. This eliminates some symmetries. Furthermore, `tight_tw` contains many *sub-instances* where only a subset of trains is present.