\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj: - 6 z_train_0_segment_0 - 6 z_train_0_segment_1 - 6 z_train_0_segment_2
      - 6 z_train_0_segment_3 - 6 z_train_0_segment_4 - 6 z_train_0_segment_5
      - 12 z_train_0_segment_6 - 10 z_train_1_segment_0
      - 10 z_train_1_segment_1 - 10 z_train_1_segment_2
      - 10 z_train_1_segment_3 - 10 z_train_1_segment_4
      - 10 z_train_1_segment_5 - 20 z_train_1_segment_6
      + 6 theta_train_0_segment_0 + 6 theta_train_0_segment_1
      + 6 theta_train_0_segment_2 + 6 theta_train_0_segment_3
      + 6 theta_train_0_segment_4 + 6 theta_train_0_segment_5
      + 6 theta_train_0_segment_6 + 10 theta_train_1_segment_0
      + 10 theta_train_1_segment_1 + 10 theta_train_1_segment_2
      + 10 theta_train_1_segment_3 + 10 theta_train_1_segment_4
      + 10 theta_train_1_segment_5 + 10 theta_train_1_segment_6 + d_train_0
      + d_train_1 + 3 _e_train_0_segment_1 + x32
Subject To
 _exit_sigma_train_0:            x_train_0_from_sigma_to_0_1
                                 + x_train_0_from_sigma_to_0_2
                                 + x_train_0_from_sigma_to_0_3
                                 + x_train_0_from_sigma_to_0_4
                                 + x_train_0_from_sigma_to_0_5
                                 + x_train_0_from_sigma_to_0_6
                                 + x_train_0_from_sigma_to_0_7
                                 + x_train_0_from_sigma_to_0_8
                                 + x_train_0_from_sigma_to_0_9
                                 + x_train_0_from_sigma_to_0_10
                                 + x_train_0_from_sigma_to_0_11
                                 + x_train_0_from_sigma_to_0_12
                                 + x_train_0_from_sigma_to_0_13
                                 + x_train_0_from_sigma_to_0_14
                                 + x_train_0_from_sigma_to_0_15
                                 + x_train_0_from_sigma_to_0_16
                                 + x_train_0_from_sigma_to_0_17
                                 + x_train_0_from_sigma_to_0_18
                                 + x_train_0_from_sigma_to_0_19
                                 + x_train_0_from_sigma_to_0_20
                                 + x_train_0_from_sigma_to_0_21
                                 + x_train_0_from_sigma_to_0_22
                                 + x_train_0_from_sigma_to_0_23
                                 + x_train_0_from_sigma_to_0_24
                                 + x_train_0_from_sigma_to_0_25
                                 + x_train_0_from_sigma_to_0_26
                                 + x_train_0_from_sigma_to_0_27  = 1
 _exit_sigma_train_1:            x_train_1_from_sigma_to_6_1
                                 + x_train_1_from_sigma_to_6_2
                                 + x_train_1_from_sigma_to_6_3
                                 + x_train_1_from_sigma_to_6_4
                                 + x_train_1_from_sigma_to_6_5
                                 + x_train_1_from_sigma_to_6_6
                                 + x_train_1_from_sigma_to_6_7
                                 + x_train_1_from_sigma_to_6_8
                                 + x_train_1_from_sigma_to_6_9
                                 + x_train_1_from_sigma_to_6_10
                                 + x_train_1_from_sigma_to_6_11
                                 + x_train_1_from_sigma_to_6_12
                                 + x_train_1_from_sigma_to_6_13
                                 + x_train_1_from_sigma_to_6_14
                                 + x_train_1_from_sigma_to_6_15
                                 + x_train_1_from_sigma_to_6_16
                                 + x_train_1_from_sigma_to_6_17
                                 + x_train_1_from_sigma_to_6_18
                                 + x_train_1_from_sigma_to_6_19
                                 + x_train_1_from_sigma_to_6_20
                                 + x_train_1_from_sigma_to_6_21
                                 + x_train_1_from_sigma_to_6_22
                                 + x_train_1_from_sigma_to_6_23
                                 + x_train_1_from_sigma_to_6_24
                                 + x_train_1_from_sigma_to_6_25
                                 + x_train_1_from_sigma_to_6_26
                                 + x_train_1_from_sigma_to_6_27  = 1
 _enter_tau_train_0:             x_train_0_from_6_4_to_tau
                                 + x_train_0_from_6_5_to_tau
                                 + x_train_0_from_6_6_to_tau
                                 + x_train_0_from_6_7_to_tau
                                 + x_train_0_from_6_8_to_tau
                                 + x_train_0_from_6_9_to_tau
                                 + x_train_0_from_6_10_to_tau
                                 + x_train_0_from_6_11_to_tau
                                 + x_train_0_from_6_12_to_tau
                                 + x_train_0_from_6_13_to_tau
                                 + x_train_0_from_6_14_to_tau
                                 + x_train_0_from_6_15_to_tau
                                 + x_train_0_from_6_16_to_tau
                                 + x_train_0_from_6_17_to_tau
                                 + x_train_0_from_6_18_to_tau
                                 + x_train_0_from_6_19_to_tau
                                 + x_train_0_from_6_20_to_tau
                                 + x_train_0_from_6_21_to_tau
                                 + x_train_0_from_6_22_to_tau
                                 + x_train_0_from_6_23_to_tau
                                 + x_train_0_from_6_24_to_tau
                                 + x_train_0_from_6_25_to_tau
                                 + x_train_0_from_6_26_to_tau
                                 + x_train_0_from_6_27_to_tau
                                 + x_train_0_from_6_28_to_tau
                                 + x_train_0_from_6_29_to_tau
                                 + x_train_0_from_6_30_to_tau  = 1
 _enter_tau_train_1:             x_train_1_from_0_4_to_tau
                                 + x_train_1_from_0_5_to_tau
                                 + x_train_1_from_0_6_to_tau
                                 + x_train_1_from_0_7_to_tau
                                 + x_train_1_from_0_8_to_tau
                                 + x_train_1_from_0_9_to_tau
                                 + x_train_1_from_0_10_to_tau
                                 + x_train_1_from_0_11_to_tau
                                 + x_train_1_from_0_12_to_tau
                                 + x_train_1_from_0_13_to_tau
                                 + x_train_1_from_0_14_to_tau
                                 + x_train_1_from_0_15_to_tau
                                 + x_train_1_from_0_16_to_tau
                                 + x_train_1_from_0_17_to_tau
                                 + x_train_1_from_0_18_to_tau
                                 + x_train_1_from_0_19_to_tau
                                 + x_train_1_from_0_20_to_tau
                                 + x_train_1_from_0_21_to_tau
                                 + x_train_1_from_0_22_to_tau
                                 + x_train_1_from_0_23_to_tau
                                 + x_train_1_from_0_24_to_tau
                                 + x_train_1_from_0_25_to_tau
                                 + x_train_1_from_0_26_to_tau
                                 + x_train_1_from_0_27_to_tau
                                 + x_train_1_from_0_28_to_tau
                                 + x_train_1_from_0_29_to_tau
                                 + x_train_1_from_0_30_to_tau  = 1
 set_y_train_0_node_0_1:         x_train_0_from_sigma_to_0_1
                                 - y_train_0_node_0_1  = 0
 set_y_train_0_node_0_2:         x_train_0_from_sigma_to_0_2
                                 + x_train_0_from_0_1_to_0_2
                                 - y_train_0_node_0_2  = 0
 set_y_train_0_node_0_3:         x_train_0_from_sigma_to_0_3
                                 + x_train_0_from_0_2_to_0_3
                                 - y_train_0_node_0_3  = 0
 set_y_train_0_node_0_4:         x_train_0_from_sigma_to_0_4
                                 + x_train_0_from_0_3_to_0_4
                                 - y_train_0_node_0_4  = 0
 set_y_train_0_node_0_5:         x_train_0_from_sigma_to_0_5
                                 + x_train_0_from_0_4_to_0_5
                                 - y_train_0_node_0_5  = 0
 set_y_train_0_node_0_6:         x_train_0_from_sigma_to_0_6
                                 + x_train_0_from_0_5_to_0_6
                                 - y_train_0_node_0_6  = 0
 set_y_train_0_node_0_7:         x_train_0_from_sigma_to_0_7
                                 + x_train_0_from_0_6_to_0_7
                                 - y_train_0_node_0_7  = 0
 set_y_train_0_node_0_8:         x_train_0_from_sigma_to_0_8
                                 + x_train_0_from_0_7_to_0_8
                                 - y_train_0_node_0_8  = 0
 set_y_train_0_node_0_9:         x_train_0_from_sigma_to_0_9
                                 + x_train_0_from_0_8_to_0_9
                                 - y_train_0_node_0_9  = 0
 set_y_train_0_node_0_10:        x_train_0_from_sigma_to_0_10
                                 + x_train_0_from_0_9_to_0_10
                                 - y_train_0_node_0_10  = 0
 set_y_train_0_node_0_11:        x_train_0_from_sigma_to_0_11
                                 + x_train_0_from_0_10_to_0_11
                                 - y_train_0_node_0_11  = 0
 set_y_train_0_node_0_12:        x_train_0_from_sigma_to_0_12
                                 + x_train_0_from_0_11_to_0_12
                                 - y_train_0_node_0_12  = 0
 set_y_train_0_node_0_13:        x_train_0_from_sigma_to_0_13
                                 + x_train_0_from_0_12_to_0_13
                                 - y_train_0_node_0_13  = 0
 set_y_train_0_node_0_14:        x_train_0_from_sigma_to_0_14
                                 + x_train_0_from_0_13_to_0_14
                                 - y_train_0_node_0_14  = 0
 set_y_train_0_node_0_15:        x_train_0_from_sigma_to_0_15
                                 + x_train_0_from_0_14_to_0_15
                                 - y_train_0_node_0_15  = 0
 set_y_train_0_node_0_16:        x_train_0_from_sigma_to_0_16
                                 + x_train_0_from_0_15_to_0_16
                                 - y_train_0_node_0_16  = 0
 set_y_train_0_node_0_17:        x_train_0_from_sigma_to_0_17
                                 + x_train_0_from_0_16_to_0_17
                                 - y_train_0_node_0_17  = 0
 set_y_train_0_node_0_18:        x_train_0_from_sigma_to_0_18
                                 + x_train_0_from_0_17_to_0_18
                                 - y_train_0_node_0_18  = 0
 set_y_train_0_node_0_19:        x_train_0_from_sigma_to_0_19
                                 + x_train_0_from_0_18_to_0_19
                                 - y_train_0_node_0_19  = 0
 set_y_train_0_node_0_20:        x_train_0_from_sigma_to_0_20
                                 + x_train_0_from_0_19_to_0_20
                                 - y_train_0_node_0_20  = 0
 set_y_train_0_node_0_21:        x_train_0_from_sigma_to_0_21
                                 + x_train_0_from_0_20_to_0_21
                                 - y_train_0_node_0_21  = 0
 set_y_train_0_node_0_22:        x_train_0_from_sigma_to_0_22
                                 + x_train_0_from_0_21_to_0_22
                                 - y_train_0_node_0_22  = 0
 set_y_train_0_node_0_23:        x_train_0_from_sigma_to_0_23
                                 + x_train_0_from_0_22_to_0_23
                                 - y_train_0_node_0_23  = 0
 set_y_train_0_node_0_24:        x_train_0_from_sigma_to_0_24
                                 + x_train_0_from_0_23_to_0_24
                                 - y_train_0_node_0_24  = 0
 set_y_train_0_node_0_25:        x_train_0_from_sigma_to_0_25
                                 + x_train_0_from_0_24_to_0_25
                                 - y_train_0_node_0_25  = 0
 set_y_train_0_node_0_26:        x_train_0_from_sigma_to_0_26
                                 + x_train_0_from_0_25_to_0_26
                                 - y_train_0_node_0_26  = 0
 set_y_train_0_node_0_27:        x_train_0_from_sigma_to_0_27
                                 + x_train_0_from_0_26_to_0_27
                                 - y_train_0_node_0_27  = 0
 set_y_train_0_node_1_2:         x_train_0_from_0_1_to_1_2 - y_train_0_node_1_2
                                  = 0
 set_y_train_0_node_1_3:         x_train_0_from_0_2_to_1_3
                                 + x_train_0_from_1_2_to_1_3
                                 - y_train_0_node_1_3  = 0
 set_y_train_0_node_1_4:         x_train_0_from_0_3_to_1_4
                                 + x_train_0_from_1_3_to_1_4
                                 - y_train_0_node_1_4  = 0
 set_y_train_0_node_1_5:         x_train_0_from_0_4_to_1_5
                                 + x_train_0_from_1_4_to_1_5
                                 - y_train_0_node_1_5  = 0
 set_y_train_0_node_1_6:         x_train_0_from_0_5_to_1_6
                                 + x_train_0_from_1_5_to_1_6
                                 - y_train_0_node_1_6  = 0
 set_y_train_0_node_1_7:         x_train_0_from_0_6_to_1_7
                                 + x_train_0_from_1_6_to_1_7
                                 - y_train_0_node_1_7  = 0
 set_y_train_0_node_1_8:         x_train_0_from_0_7_to_1_8
                                 + x_train_0_from_1_7_to_1_8
                                 - y_train_0_node_1_8  = 0
 set_y_train_0_node_1_9:         x_train_0_from_0_8_to_1_9
                                 + x_train_0_from_1_8_to_1_9
                                 - y_train_0_node_1_9  = 0
 set_y_train_0_node_1_10:        x_train_0_from_0_9_to_1_10
                                 + x_train_0_from_1_9_to_1_10
                                 - y_train_0_node_1_10  = 0
 set_y_train_0_node_1_11:        x_train_0_from_0_10_to_1_11
                                 + x_train_0_from_1_10_to_1_11
                                 - y_train_0_node_1_11  = 0
 set_y_train_0_node_1_12:        x_train_0_from_0_11_to_1_12
                                 + x_train_0_from_1_11_to_1_12
                                 - y_train_0_node_1_12  = 0
 set_y_train_0_node_1_13:        x_train_0_from_0_12_to_1_13
                                 + x_train_0_from_1_12_to_1_13
                                 - y_train_0_node_1_13  = 0
 set_y_train_0_node_1_14:        x_train_0_from_0_13_to_1_14
                                 + x_train_0_from_1_13_to_1_14
                                 - y_train_0_node_1_14  = 0
 set_y_train_0_node_1_15:        x_train_0_from_0_14_to_1_15
                                 + x_train_0_from_1_14_to_1_15
                                 - y_train_0_node_1_15  = 0
 set_y_train_0_node_1_16:        x_train_0_from_0_15_to_1_16
                                 + x_train_0_from_1_15_to_1_16
                                 - y_train_0_node_1_16  = 0
 set_y_train_0_node_1_17:        x_train_0_from_0_16_to_1_17
                                 + x_train_0_from_1_16_to_1_17
                                 - y_train_0_node_1_17  = 0
 set_y_train_0_node_1_18:        x_train_0_from_0_17_to_1_18
                                 + x_train_0_from_1_17_to_1_18
                                 - y_train_0_node_1_18  = 0
 set_y_train_0_node_1_19:        x_train_0_from_0_18_to_1_19
                                 + x_train_0_from_1_18_to_1_19
                                 - y_train_0_node_1_19  = 0
 set_y_train_0_node_1_20:        x_train_0_from_0_19_to_1_20
                                 + x_train_0_from_1_19_to_1_20
                                 - y_train_0_node_1_20  = 0
 set_y_train_0_node_1_21:        x_train_0_from_0_20_to_1_21
                                 + x_train_0_from_1_20_to_1_21
                                 - y_train_0_node_1_21  = 0
 set_y_train_0_node_1_22:        x_train_0_from_0_21_to_1_22
                                 + x_train_0_from_1_21_to_1_22
                                 - y_train_0_node_1_22  = 0
 set_y_train_0_node_1_23:        x_train_0_from_0_22_to_1_23
                                 + x_train_0_from_1_22_to_1_23
                                 - y_train_0_node_1_23  = 0
 set_y_train_0_node_1_24:        x_train_0_from_0_23_to_1_24
                                 + x_train_0_from_1_23_to_1_24
                                 - y_train_0_node_1_24  = 0
 set_y_train_0_node_1_25:        x_train_0_from_0_24_to_1_25
                                 + x_train_0_from_1_24_to_1_25
                                 - y_train_0_node_1_25  = 0
 set_y_train_0_node_1_26:        x_train_0_from_0_25_to_1_26
                                 + x_train_0_from_1_25_to_1_26
                                 - y_train_0_node_1_26  = 0
 set_y_train_0_node_1_27:        x_train_0_from_0_26_to_1_27
                                 + x_train_0_from_1_26_to_1_27
                                 - y_train_0_node_1_27  = 0
 set_y_train_0_node_2_2:         x_train_0_from_0_1_to_2_2 - y_train_0_node_2_2
                                  = 0
 set_y_train_0_node_2_3:         x_train_0_from_0_2_to_2_3
                                 + x_train_0_from_2_2_to_2_3
                                 - y_train_0_node_2_3  = 0
 set_y_train_0_node_2_4:         x_train_0_from_0_3_to_2_4
                                 + x_train_0_from_2_3_to_2_4
                                 - y_train_0_node_2_4  = 0
 set_y_train_0_node_2_5:         x_train_0_from_0_4_to_2_5
                                 + x_train_0_from_2_4_to_2_5
                                 - y_train_0_node_2_5  = 0
 set_y_train_0_node_2_6:         x_train_0_from_0_5_to_2_6
                                 + x_train_0_from_2_5_to_2_6
                                 - y_train_0_node_2_6  = 0
 set_y_train_0_node_2_7:         x_train_0_from_0_6_to_2_7
                                 + x_train_0_from_2_6_to_2_7
                                 - y_train_0_node_2_7  = 0
 set_y_train_0_node_2_8:         x_train_0_from_0_7_to_2_8
                                 + x_train_0_from_2_7_to_2_8
                                 - y_train_0_node_2_8  = 0
 set_y_train_0_node_2_9:         x_train_0_from_0_8_to_2_9
                                 + x_train_0_from_2_8_to_2_9
                                 - y_train_0_node_2_9  = 0
 set_y_train_0_node_2_10:        x_train_0_from_0_9_to_2_10
                                 + x_train_0_from_2_9_to_2_10
                                 - y_train_0_node_2_10  = 0
 set_y_train_0_node_2_11:        x_train_0_from_0_10_to_2_11
                                 + x_train_0_from_2_10_to_2_11
                                 - y_train_0_node_2_11  = 0
 set_y_train_0_node_2_12:        x_train_0_from_0_11_to_2_12
                                 + x_train_0_from_2_11_to_2_12
                                 - y_train_0_node_2_12  = 0
 set_y_train_0_node_2_13:        x_train_0_from_0_12_to_2_13
                                 + x_train_0_from_2_12_to_2_13
                                 - y_train_0_node_2_13  = 0
 set_y_train_0_node_2_14:        x_train_0_from_0_13_to_2_14
                                 + x_train_0_from_2_13_to_2_14
                                 - y_train_0_node_2_14  = 0
 set_y_train_0_node_2_15:        x_train_0_from_0_14_to_2_15
                                 + x_train_0_from_2_14_to_2_15
                                 - y_train_0_node_2_15  = 0
 set_y_train_0_node_2_16:        x_train_0_from_0_15_to_2_16
                                 + x_train_0_from_2_15_to_2_16
                                 - y_train_0_node_2_16  = 0
 set_y_train_0_node_2_17:        x_train_0_from_0_16_to_2_17
                                 + x_train_0_from_2_16_to_2_17
                                 - y_train_0_node_2_17  = 0
 set_y_train_0_node_2_18:        x_train_0_from_0_17_to_2_18
                                 + x_train_0_from_2_17_to_2_18
                                 - y_train_0_node_2_18  = 0
 set_y_train_0_node_2_19:        x_train_0_from_0_18_to_2_19
                                 + x_train_0_from_2_18_to_2_19
                                 - y_train_0_node_2_19  = 0
 set_y_train_0_node_2_20:        x_train_0_from_0_19_to_2_20
                                 + x_train_0_from_2_19_to_2_20
                                 - y_train_0_node_2_20  = 0
 set_y_train_0_node_2_21:        x_train_0_from_0_20_to_2_21
                                 + x_train_0_from_2_20_to_2_21
                                 - y_train_0_node_2_21  = 0
 set_y_train_0_node_2_22:        x_train_0_from_0_21_to_2_22
                                 + x_train_0_from_2_21_to_2_22
                                 - y_train_0_node_2_22  = 0
 set_y_train_0_node_2_23:        x_train_0_from_0_22_to_2_23
                                 + x_train_0_from_2_22_to_2_23
                                 - y_train_0_node_2_23  = 0
 set_y_train_0_node_2_24:        x_train_0_from_0_23_to_2_24
                                 + x_train_0_from_2_23_to_2_24
                                 - y_train_0_node_2_24  = 0
 set_y_train_0_node_2_25:        x_train_0_from_0_24_to_2_25
                                 + x_train_0_from_2_24_to_2_25
                                 - y_train_0_node_2_25  = 0
 set_y_train_0_node_2_26:        x_train_0_from_0_25_to_2_26
                                 + x_train_0_from_2_25_to_2_26
                                 - y_train_0_node_2_26  = 0
 set_y_train_0_node_2_27:        x_train_0_from_0_26_to_2_27
                                 + x_train_0_from_2_26_to_2_27
                                 - y_train_0_node_2_27  = 0
 set_y_train_0_node_2_28:        x_train_0_from_0_27_to_2_28
                                 + x_train_0_from_2_27_to_2_28
                                 - y_train_0_node_2_28  = 0
 set_y_train_0_node_3_5:         x_train_0_from_1_4_to_3_5 - y_train_0_node_3_5
                                  = 0
 set_y_train_0_node_3_6:         x_train_0_from_1_5_to_3_6
                                 + x_train_0_from_3_5_to_3_6
                                 - y_train_0_node_3_6  = 0
 set_y_train_0_node_3_7:         x_train_0_from_1_6_to_3_7
                                 + x_train_0_from_3_6_to_3_7
                                 - y_train_0_node_3_7  = 0
 set_y_train_0_node_3_8:         x_train_0_from_1_7_to_3_8
                                 + x_train_0_from_3_7_to_3_8
                                 - y_train_0_node_3_8  = 0
 set_y_train_0_node_3_9:         x_train_0_from_1_8_to_3_9
                                 + x_train_0_from_3_8_to_3_9
                                 - y_train_0_node_3_9  = 0
 set_y_train_0_node_3_10:        x_train_0_from_1_9_to_3_10
                                 + x_train_0_from_3_9_to_3_10
                                 - y_train_0_node_3_10  = 0
 set_y_train_0_node_3_11:        x_train_0_from_1_10_to_3_11
                                 + x_train_0_from_3_10_to_3_11
                                 - y_train_0_node_3_11  = 0
 set_y_train_0_node_3_12:        x_train_0_from_1_11_to_3_12
                                 + x_train_0_from_3_11_to_3_12
                                 - y_train_0_node_3_12  = 0
 set_y_train_0_node_3_13:        x_train_0_from_1_12_to_3_13
                                 + x_train_0_from_3_12_to_3_13
                                 - y_train_0_node_3_13  = 0
 set_y_train_0_node_3_14:        x_train_0_from_1_13_to_3_14
                                 + x_train_0_from_3_13_to_3_14
                                 - y_train_0_node_3_14  = 0
 set_y_train_0_node_3_15:        x_train_0_from_1_14_to_3_15
                                 + x_train_0_from_3_14_to_3_15
                                 - y_train_0_node_3_15  = 0
 set_y_train_0_node_3_16:        x_train_0_from_1_15_to_3_16
                                 + x_train_0_from_3_15_to_3_16
                                 - y_train_0_node_3_16  = 0
 set_y_train_0_node_3_17:        x_train_0_from_1_16_to_3_17
                                 + x_train_0_from_3_16_to_3_17
                                 - y_train_0_node_3_17  = 0
 set_y_train_0_node_3_18:        x_train_0_from_1_17_to_3_18
                                 + x_train_0_from_3_17_to_3_18
                                 - y_train_0_node_3_18  = 0
 set_y_train_0_node_3_19:        x_train_0_from_1_18_to_3_19
                                 + x_train_0_from_3_18_to_3_19
                                 - y_train_0_node_3_19  = 0
 set_y_train_0_node_3_20:        x_train_0_from_1_19_to_3_20
                                 + x_train_0_from_3_19_to_3_20
                                 - y_train_0_node_3_20  = 0
 set_y_train_0_node_3_21:        x_train_0_from_1_20_to_3_21
                                 + x_train_0_from_3_20_to_3_21
                                 - y_train_0_node_3_21  = 0
 set_y_train_0_node_3_22:        x_train_0_from_1_21_to_3_22
                                 + x_train_0_from_3_21_to_3_22
                                 - y_train_0_node_3_22  = 0
 set_y_train_0_node_3_23:        x_train_0_from_1_22_to_3_23
                                 + x_train_0_from_3_22_to_3_23
                                 - y_train_0_node_3_23  = 0
 set_y_train_0_node_3_24:        x_train_0_from_1_23_to_3_24
                                 + x_train_0_from_3_23_to_3_24
                                 - y_train_0_node_3_24  = 0
 set_y_train_0_node_3_25:        x_train_0_from_1_24_to_3_25
                                 + x_train_0_from_3_24_to_3_25
                                 - y_train_0_node_3_25  = 0
 set_y_train_0_node_3_26:        x_train_0_from_1_25_to_3_26
                                 + x_train_0_from_3_25_to_3_26
                                 - y_train_0_node_3_26  = 0
 set_y_train_0_node_3_27:        x_train_0_from_1_26_to_3_27
                                 + x_train_0_from_3_26_to_3_27
                                 - y_train_0_node_3_27  = 0
 set_y_train_0_node_3_28:        x_train_0_from_1_27_to_3_28
                                 + x_train_0_from_3_27_to_3_28
                                 - y_train_0_node_3_28  = 0
 set_y_train_0_node_4_3:         x_train_0_from_2_2_to_4_3 - y_train_0_node_4_3
                                  = 0
 set_y_train_0_node_4_4:         x_train_0_from_2_3_to_4_4
                                 + x_train_0_from_4_3_to_4_4
                                 - y_train_0_node_4_4  = 0
 set_y_train_0_node_4_5:         x_train_0_from_2_4_to_4_5
                                 + x_train_0_from_4_4_to_4_5
                                 - y_train_0_node_4_5  = 0
 set_y_train_0_node_4_6:         x_train_0_from_2_5_to_4_6
                                 + x_train_0_from_4_5_to_4_6
                                 - y_train_0_node_4_6  = 0
 set_y_train_0_node_4_7:         x_train_0_from_2_6_to_4_7
                                 + x_train_0_from_4_6_to_4_7
                                 - y_train_0_node_4_7  = 0
 set_y_train_0_node_4_8:         x_train_0_from_2_7_to_4_8
                                 + x_train_0_from_4_7_to_4_8
                                 - y_train_0_node_4_8  = 0
 set_y_train_0_node_4_9:         x_train_0_from_2_8_to_4_9
                                 + x_train_0_from_4_8_to_4_9
                                 - y_train_0_node_4_9  = 0
 set_y_train_0_node_4_10:        x_train_0_from_2_9_to_4_10
                                 + x_train_0_from_4_9_to_4_10
                                 - y_train_0_node_4_10  = 0
 set_y_train_0_node_4_11:        x_train_0_from_2_10_to_4_11
                                 + x_train_0_from_4_10_to_4_11
                                 - y_train_0_node_4_11  = 0
 set_y_train_0_node_4_12:        x_train_0_from_2_11_to_4_12
                                 + x_train_0_from_4_11_to_4_12
                                 - y_train_0_node_4_12  = 0
 set_y_train_0_node_4_13:        x_train_0_from_2_12_to_4_13
                                 + x_train_0_from_4_12_to_4_13
                                 - y_train_0_node_4_13  = 0
 set_y_train_0_node_4_14:        x_train_0_from_2_13_to_4_14
                                 + x_train_0_from_4_13_to_4_14
                                 - y_train_0_node_4_14  = 0
 set_y_train_0_node_4_15:        x_train_0_from_2_14_to_4_15
                                 + x_train_0_from_4_14_to_4_15
                                 - y_train_0_node_4_15  = 0
 set_y_train_0_node_4_16:        x_train_0_from_2_15_to_4_16
                                 + x_train_0_from_4_15_to_4_16
                                 - y_train_0_node_4_16  = 0
 set_y_train_0_node_4_17:        x_train_0_from_2_16_to_4_17
                                 + x_train_0_from_4_16_to_4_17
                                 - y_train_0_node_4_17  = 0
 set_y_train_0_node_4_18:        x_train_0_from_2_17_to_4_18
                                 + x_train_0_from_4_17_to_4_18
                                 - y_train_0_node_4_18  = 0
 set_y_train_0_node_4_19:        x_train_0_from_2_18_to_4_19
                                 + x_train_0_from_4_18_to_4_19
                                 - y_train_0_node_4_19  = 0
 set_y_train_0_node_4_20:        x_train_0_from_2_19_to_4_20
                                 + x_train_0_from_4_19_to_4_20
                                 - y_train_0_node_4_20  = 0
 set_y_train_0_node_4_21:        x_train_0_from_2_20_to_4_21
                                 + x_train_0_from_4_20_to_4_21
                                 - y_train_0_node_4_21  = 0
 set_y_train_0_node_4_22:        x_train_0_from_2_21_to_4_22
                                 + x_train_0_from_4_21_to_4_22
                                 - y_train_0_node_4_22  = 0
 set_y_train_0_node_4_23:        x_train_0_from_2_22_to_4_23
                                 + x_train_0_from_4_22_to_4_23
                                 - y_train_0_node_4_23  = 0
 set_y_train_0_node_4_24:        x_train_0_from_2_23_to_4_24
                                 + x_train_0_from_4_23_to_4_24
                                 - y_train_0_node_4_24  = 0
 set_y_train_0_node_4_25:        x_train_0_from_2_24_to_4_25
                                 + x_train_0_from_4_24_to_4_25
                                 - y_train_0_node_4_25  = 0
 set_y_train_0_node_4_26:        x_train_0_from_2_25_to_4_26
                                 + x_train_0_from_4_25_to_4_26
                                 - y_train_0_node_4_26  = 0
 set_y_train_0_node_4_27:        x_train_0_from_2_26_to_4_27
                                 + x_train_0_from_4_26_to_4_27
                                 - y_train_0_node_4_27  = 0
 set_y_train_0_node_4_28:        x_train_0_from_2_27_to_4_28
                                 + x_train_0_from_4_27_to_4_28
                                 - y_train_0_node_4_28  = 0
 set_y_train_0_node_4_29:        x_train_0_from_2_28_to_4_29
                                 + x_train_0_from_4_28_to_4_29
                                 - y_train_0_node_4_29  = 0
 set_y_train_0_node_5_6:         x_train_0_from_3_5_to_5_6 - y_train_0_node_5_6
                                  = 0
 set_y_train_0_node_5_7:         x_train_0_from_3_6_to_5_7
                                 + x_train_0_from_5_6_to_5_7
                                 - y_train_0_node_5_7  = 0
 set_y_train_0_node_5_8:         x_train_0_from_3_7_to_5_8
                                 + x_train_0_from_5_7_to_5_8
                                 - y_train_0_node_5_8  = 0
 set_y_train_0_node_5_9:         x_train_0_from_3_8_to_5_9
                                 + x_train_0_from_5_8_to_5_9
                                 - y_train_0_node_5_9  = 0
 set_y_train_0_node_5_10:        x_train_0_from_3_9_to_5_10
                                 + x_train_0_from_5_9_to_5_10
                                 - y_train_0_node_5_10  = 0
 set_y_train_0_node_5_11:        x_train_0_from_3_10_to_5_11
                                 + x_train_0_from_5_10_to_5_11
                                 - y_train_0_node_5_11  = 0
 set_y_train_0_node_5_12:        x_train_0_from_3_11_to_5_12
                                 + x_train_0_from_5_11_to_5_12
                                 - y_train_0_node_5_12  = 0
 set_y_train_0_node_5_13:        x_train_0_from_3_12_to_5_13
                                 + x_train_0_from_5_12_to_5_13
                                 - y_train_0_node_5_13  = 0
 set_y_train_0_node_5_14:        x_train_0_from_3_13_to_5_14
                                 + x_train_0_from_5_13_to_5_14
                                 - y_train_0_node_5_14  = 0
 set_y_train_0_node_5_15:        x_train_0_from_3_14_to_5_15
                                 + x_train_0_from_5_14_to_5_15
                                 - y_train_0_node_5_15  = 0
 set_y_train_0_node_5_16:        x_train_0_from_3_15_to_5_16
                                 + x_train_0_from_5_15_to_5_16
                                 - y_train_0_node_5_16  = 0
 set_y_train_0_node_5_17:        x_train_0_from_3_16_to_5_17
                                 + x_train_0_from_5_16_to_5_17
                                 - y_train_0_node_5_17  = 0
 set_y_train_0_node_5_18:        x_train_0_from_3_17_to_5_18
                                 + x_train_0_from_5_17_to_5_18
                                 - y_train_0_node_5_18  = 0
 set_y_train_0_node_5_19:        x_train_0_from_3_18_to_5_19
                                 + x_train_0_from_5_18_to_5_19
                                 - y_train_0_node_5_19  = 0
 set_y_train_0_node_5_20:        x_train_0_from_3_19_to_5_20
                                 + x_train_0_from_5_19_to_5_20
                                 - y_train_0_node_5_20  = 0
 set_y_train_0_node_5_21:        x_train_0_from_3_20_to_5_21
                                 + x_train_0_from_5_20_to_5_21
                                 - y_train_0_node_5_21  = 0
 set_y_train_0_node_5_22:        x_train_0_from_3_21_to_5_22
                                 + x_train_0_from_5_21_to_5_22
                                 - y_train_0_node_5_22  = 0
 set_y_train_0_node_5_23:        x_train_0_from_3_22_to_5_23
                                 + x_train_0_from_5_22_to_5_23
                                 - y_train_0_node_5_23  = 0
 set_y_train_0_node_5_24:        x_train_0_from_3_23_to_5_24
                                 + x_train_0_from_5_23_to_5_24
                                 - y_train_0_node_5_24  = 0
 set_y_train_0_node_5_25:        x_train_0_from_3_24_to_5_25
                                 + x_train_0_from_5_24_to_5_25
                                 - y_train_0_node_5_25  = 0
 set_y_train_0_node_5_26:        x_train_0_from_3_25_to_5_26
                                 + x_train_0_from_5_25_to_5_26
                                 - y_train_0_node_5_26  = 0
 set_y_train_0_node_5_27:        x_train_0_from_3_26_to_5_27
                                 + x_train_0_from_5_26_to_5_27
                                 - y_train_0_node_5_27  = 0
 set_y_train_0_node_5_28:        x_train_0_from_3_27_to_5_28
                                 + x_train_0_from_5_27_to_5_28
                                 - y_train_0_node_5_28  = 0
 set_y_train_0_node_5_29:        x_train_0_from_3_28_to_5_29
                                 + x_train_0_from_5_28_to_5_29
                                 - y_train_0_node_5_29  = 0
 set_y_train_0_node_6_4:         x_train_0_from_4_3_to_6_4 - y_train_0_node_6_4
                                  = 0
 set_y_train_0_node_6_5:         x_train_0_from_4_4_to_6_5
                                 + x_train_0_from_6_4_to_6_5
                                 - y_train_0_node_6_5  = 0
 set_y_train_0_node_6_6:         x_train_0_from_4_5_to_6_6
                                 + x_train_0_from_6_5_to_6_6
                                 - y_train_0_node_6_6  = 0
 set_y_train_0_node_6_7:         x_train_0_from_4_6_to_6_7
                                 + x_train_0_from_5_6_to_6_7
                                 + x_train_0_from_6_6_to_6_7
                                 - y_train_0_node_6_7  = 0
 set_y_train_0_node_6_8:         x_train_0_from_4_7_to_6_8
                                 + x_train_0_from_5_7_to_6_8
                                 + x_train_0_from_6_7_to_6_8
                                 - y_train_0_node_6_8  = 0
 set_y_train_0_node_6_9:         x_train_0_from_4_8_to_6_9
                                 + x_train_0_from_5_8_to_6_9
                                 + x_train_0_from_6_8_to_6_9
                                 - y_train_0_node_6_9  = 0
 set_y_train_0_node_6_10:        x_train_0_from_4_9_to_6_10
                                 + x_train_0_from_5_9_to_6_10
                                 + x_train_0_from_6_9_to_6_10
                                 - y_train_0_node_6_10  = 0
 set_y_train_0_node_6_11:        x_train_0_from_4_10_to_6_11
                                 + x_train_0_from_5_10_to_6_11
                                 + x_train_0_from_6_10_to_6_11
                                 - y_train_0_node_6_11  = 0
 set_y_train_0_node_6_12:        x_train_0_from_4_11_to_6_12
                                 + x_train_0_from_5_11_to_6_12
                                 + x_train_0_from_6_11_to_6_12
                                 - y_train_0_node_6_12  = 0
 set_y_train_0_node_6_13:        x_train_0_from_4_12_to_6_13
                                 + x_train_0_from_5_12_to_6_13
                                 + x_train_0_from_6_12_to_6_13
                                 - y_train_0_node_6_13  = 0
 set_y_train_0_node_6_14:        x_train_0_from_4_13_to_6_14
                                 + x_train_0_from_5_13_to_6_14
                                 + x_train_0_from_6_13_to_6_14
                                 - y_train_0_node_6_14  = 0
 set_y_train_0_node_6_15:        x_train_0_from_4_14_to_6_15
                                 + x_train_0_from_5_14_to_6_15
                                 + x_train_0_from_6_14_to_6_15
                                 - y_train_0_node_6_15  = 0
 set_y_train_0_node_6_16:        x_train_0_from_4_15_to_6_16
                                 + x_train_0_from_5_15_to_6_16
                                 + x_train_0_from_6_15_to_6_16
                                 - y_train_0_node_6_16  = 0
 set_y_train_0_node_6_17:        x_train_0_from_4_16_to_6_17
                                 + x_train_0_from_5_16_to_6_17
                                 + x_train_0_from_6_16_to_6_17
                                 - y_train_0_node_6_17  = 0
 set_y_train_0_node_6_18:        x_train_0_from_4_17_to_6_18
                                 + x_train_0_from_5_17_to_6_18
                                 + x_train_0_from_6_17_to_6_18
                                 - y_train_0_node_6_18  = 0
 set_y_train_0_node_6_19:        x_train_0_from_4_18_to_6_19
                                 + x_train_0_from_5_18_to_6_19
                                 + x_train_0_from_6_18_to_6_19
                                 - y_train_0_node_6_19  = 0
 set_y_train_0_node_6_20:        x_train_0_from_4_19_to_6_20
                                 + x_train_0_from_5_19_to_6_20
                                 + x_train_0_from_6_19_to_6_20
                                 - y_train_0_node_6_20  = 0
 set_y_train_0_node_6_21:        x_train_0_from_4_20_to_6_21
                                 + x_train_0_from_5_20_to_6_21
                                 + x_train_0_from_6_20_to_6_21
                                 - y_train_0_node_6_21  = 0
 set_y_train_0_node_6_22:        x_train_0_from_4_21_to_6_22
                                 + x_train_0_from_5_21_to_6_22
                                 + x_train_0_from_6_21_to_6_22
                                 - y_train_0_node_6_22  = 0
 set_y_train_0_node_6_23:        x_train_0_from_4_22_to_6_23
                                 + x_train_0_from_5_22_to_6_23
                                 + x_train_0_from_6_22_to_6_23
                                 - y_train_0_node_6_23  = 0
 set_y_train_0_node_6_24:        x_train_0_from_4_23_to_6_24
                                 + x_train_0_from_5_23_to_6_24
                                 + x_train_0_from_6_23_to_6_24
                                 - y_train_0_node_6_24  = 0
 set_y_train_0_node_6_25:        x_train_0_from_4_24_to_6_25
                                 + x_train_0_from_5_24_to_6_25
                                 + x_train_0_from_6_24_to_6_25
                                 - y_train_0_node_6_25  = 0
 set_y_train_0_node_6_26:        x_train_0_from_4_25_to_6_26
                                 + x_train_0_from_5_25_to_6_26
                                 + x_train_0_from_6_25_to_6_26
                                 - y_train_0_node_6_26  = 0
 set_y_train_0_node_6_27:        x_train_0_from_4_26_to_6_27
                                 + x_train_0_from_5_26_to_6_27
                                 + x_train_0_from_6_26_to_6_27
                                 - y_train_0_node_6_27  = 0
 set_y_train_0_node_6_28:        x_train_0_from_4_27_to_6_28
                                 + x_train_0_from_5_27_to_6_28
                                 + x_train_0_from_6_27_to_6_28
                                 - y_train_0_node_6_28  = 0
 set_y_train_0_node_6_29:        x_train_0_from_4_28_to_6_29
                                 + x_train_0_from_5_28_to_6_29
                                 + x_train_0_from_6_28_to_6_29
                                 - y_train_0_node_6_29  = 0
 set_y_train_0_node_6_30:        x_train_0_from_4_29_to_6_30
                                 + x_train_0_from_5_29_to_6_30
                                 + x_train_0_from_6_29_to_6_30
                                 - y_train_0_node_6_30  = 0
 set_y_train_1_node_0_4:         x_train_1_from_2_3_to_0_4 - y_train_1_node_0_4
                                  = 0
 set_y_train_1_node_0_5:         x_train_1_from_0_4_to_0_5
                                 + x_train_1_from_2_4_to_0_5
                                 - y_train_1_node_0_5  = 0
 set_y_train_1_node_0_6:         x_train_1_from_0_5_to_0_6
                                 + x_train_1_from_2_5_to_0_6
                                 - y_train_1_node_0_6  = 0
 set_y_train_1_node_0_7:         x_train_1_from_0_6_to_0_7
                                 + x_train_1_from_1_6_to_0_7
                                 + x_train_1_from_2_6_to_0_7
                                 - y_train_1_node_0_7  = 0
 set_y_train_1_node_0_8:         x_train_1_from_0_7_to_0_8
                                 + x_train_1_from_1_7_to_0_8
                                 + x_train_1_from_2_7_to_0_8
                                 - y_train_1_node_0_8  = 0
 set_y_train_1_node_0_9:         x_train_1_from_0_8_to_0_9
                                 + x_train_1_from_1_8_to_0_9
                                 + x_train_1_from_2_8_to_0_9
                                 - y_train_1_node_0_9  = 0
 set_y_train_1_node_0_10:        x_train_1_from_0_9_to_0_10
                                 + x_train_1_from_1_9_to_0_10
                                 + x_train_1_from_2_9_to_0_10
                                 - y_train_1_node_0_10  = 0
 set_y_train_1_node_0_11:        x_train_1_from_0_10_to_0_11
                                 + x_train_1_from_1_10_to_0_11
                                 + x_train_1_from_2_10_to_0_11
                                 - y_train_1_node_0_11  = 0
 set_y_train_1_node_0_12:        x_train_1_from_0_11_to_0_12
                                 + x_train_1_from_1_11_to_0_12
                                 + x_train_1_from_2_11_to_0_12
                                 - y_train_1_node_0_12  = 0
 set_y_train_1_node_0_13:        x_train_1_from_0_12_to_0_13
                                 + x_train_1_from_1_12_to_0_13
                                 + x_train_1_from_2_12_to_0_13
                                 - y_train_1_node_0_13  = 0
 set_y_train_1_node_0_14:        x_train_1_from_0_13_to_0_14
                                 + x_train_1_from_1_13_to_0_14
                                 + x_train_1_from_2_13_to_0_14
                                 - y_train_1_node_0_14  = 0
 set_y_train_1_node_0_15:        x_train_1_from_0_14_to_0_15
                                 + x_train_1_from_1_14_to_0_15
                                 + x_train_1_from_2_14_to_0_15
                                 - y_train_1_node_0_15  = 0
 set_y_train_1_node_0_16:        x_train_1_from_0_15_to_0_16
                                 + x_train_1_from_1_15_to_0_16
                                 + x_train_1_from_2_15_to_0_16
                                 - y_train_1_node_0_16  = 0
 set_y_train_1_node_0_17:        x_train_1_from_0_16_to_0_17
                                 + x_train_1_from_1_16_to_0_17
                                 + x_train_1_from_2_16_to_0_17
                                 - y_train_1_node_0_17  = 0
 set_y_train_1_node_0_18:        x_train_1_from_0_17_to_0_18
                                 + x_train_1_from_1_17_to_0_18
                                 + x_train_1_from_2_17_to_0_18
                                 - y_train_1_node_0_18  = 0
 set_y_train_1_node_0_19:        x_train_1_from_0_18_to_0_19
                                 + x_train_1_from_1_18_to_0_19
                                 + x_train_1_from_2_18_to_0_19
                                 - y_train_1_node_0_19  = 0
 set_y_train_1_node_0_20:        x_train_1_from_0_19_to_0_20
                                 + x_train_1_from_1_19_to_0_20
                                 + x_train_1_from_2_19_to_0_20
                                 - y_train_1_node_0_20  = 0
 set_y_train_1_node_0_21:        x_train_1_from_0_20_to_0_21
                                 + x_train_1_from_1_20_to_0_21
                                 + x_train_1_from_2_20_to_0_21
                                 - y_train_1_node_0_21  = 0
 set_y_train_1_node_0_22:        x_train_1_from_0_21_to_0_22
                                 + x_train_1_from_1_21_to_0_22
                                 + x_train_1_from_2_21_to_0_22
                                 - y_train_1_node_0_22  = 0
 set_y_train_1_node_0_23:        x_train_1_from_0_22_to_0_23
                                 + x_train_1_from_1_22_to_0_23
                                 + x_train_1_from_2_22_to_0_23
                                 - y_train_1_node_0_23  = 0
 set_y_train_1_node_0_24:        x_train_1_from_0_23_to_0_24
                                 + x_train_1_from_1_23_to_0_24
                                 + x_train_1_from_2_23_to_0_24
                                 - y_train_1_node_0_24  = 0
 set_y_train_1_node_0_25:        x_train_1_from_0_24_to_0_25
                                 + x_train_1_from_1_24_to_0_25
                                 + x_train_1_from_2_24_to_0_25
                                 - y_train_1_node_0_25  = 0
 set_y_train_1_node_0_26:        x_train_1_from_0_25_to_0_26
                                 + x_train_1_from_1_25_to_0_26
                                 + x_train_1_from_2_25_to_0_26
                                 - y_train_1_node_0_26  = 0
 set_y_train_1_node_0_27:        x_train_1_from_0_26_to_0_27
                                 + x_train_1_from_1_26_to_0_27
                                 + x_train_1_from_2_26_to_0_27
                                 - y_train_1_node_0_27  = 0
 set_y_train_1_node_0_28:        x_train_1_from_0_27_to_0_28
                                 + x_train_1_from_1_27_to_0_28
                                 + x_train_1_from_2_27_to_0_28
                                 - y_train_1_node_0_28  = 0
 set_y_train_1_node_0_29:        x_train_1_from_0_28_to_0_29
                                 + x_train_1_from_1_28_to_0_29
                                 + x_train_1_from_2_28_to_0_29
                                 - y_train_1_node_0_29  = 0
 set_y_train_1_node_0_30:        x_train_1_from_0_29_to_0_30
                                 + x_train_1_from_1_29_to_0_30
                                 + x_train_1_from_2_29_to_0_30
                                 - y_train_1_node_0_30  = 0
 set_y_train_1_node_1_6:         x_train_1_from_3_5_to_1_6 - y_train_1_node_1_6
                                  = 0
 set_y_train_1_node_1_7:         x_train_1_from_1_6_to_1_7
                                 + x_train_1_from_3_6_to_1_7
                                 - y_train_1_node_1_7  = 0
 set_y_train_1_node_1_8:         x_train_1_from_1_7_to_1_8
                                 + x_train_1_from_3_7_to_1_8
                                 - y_train_1_node_1_8  = 0
 set_y_train_1_node_1_9:         x_train_1_from_1_8_to_1_9
                                 + x_train_1_from_3_8_to_1_9
                                 - y_train_1_node_1_9  = 0
 set_y_train_1_node_1_10:        x_train_1_from_1_9_to_1_10
                                 + x_train_1_from_3_9_to_1_10
                                 - y_train_1_node_1_10  = 0
 set_y_train_1_node_1_11:        x_train_1_from_1_10_to_1_11
                                 + x_train_1_from_3_10_to_1_11
                                 - y_train_1_node_1_11  = 0
 set_y_train_1_node_1_12:        x_train_1_from_1_11_to_1_12
                                 + x_train_1_from_3_11_to_1_12
                                 - y_train_1_node_1_12  = 0
 set_y_train_1_node_1_13:        x_train_1_from_1_12_to_1_13
                                 + x_train_1_from_3_12_to_1_13
                                 - y_train_1_node_1_13  = 0
 set_y_train_1_node_1_14:        x_train_1_from_1_13_to_1_14
                                 + x_train_1_from_3_13_to_1_14
                                 - y_train_1_node_1_14  = 0
 set_y_train_1_node_1_15:        x_train_1_from_1_14_to_1_15
                                 + x_train_1_from_3_14_to_1_15
                                 - y_train_1_node_1_15  = 0
 set_y_train_1_node_1_16:        x_train_1_from_1_15_to_1_16
                                 + x_train_1_from_3_15_to_1_16
                                 - y_train_1_node_1_16  = 0
 set_y_train_1_node_1_17:        x_train_1_from_1_16_to_1_17
                                 + x_train_1_from_3_16_to_1_17
                                 - y_train_1_node_1_17  = 0
 set_y_train_1_node_1_18:        x_train_1_from_1_17_to_1_18
                                 + x_train_1_from_3_17_to_1_18
                                 - y_train_1_node_1_18  = 0
 set_y_train_1_node_1_19:        x_train_1_from_1_18_to_1_19
                                 + x_train_1_from_3_18_to_1_19
                                 - y_train_1_node_1_19  = 0
 set_y_train_1_node_1_20:        x_train_1_from_1_19_to_1_20
                                 + x_train_1_from_3_19_to_1_20
                                 - y_train_1_node_1_20  = 0
 set_y_train_1_node_1_21:        x_train_1_from_1_20_to_1_21
                                 + x_train_1_from_3_20_to_1_21
                                 - y_train_1_node_1_21  = 0
 set_y_train_1_node_1_22:        x_train_1_from_1_21_to_1_22
                                 + x_train_1_from_3_21_to_1_22
                                 - y_train_1_node_1_22  = 0
 set_y_train_1_node_1_23:        x_train_1_from_1_22_to_1_23
                                 + x_train_1_from_3_22_to_1_23
                                 - y_train_1_node_1_23  = 0
 set_y_train_1_node_1_24:        x_train_1_from_1_23_to_1_24
                                 + x_train_1_from_3_23_to_1_24
                                 - y_train_1_node_1_24  = 0
 set_y_train_1_node_1_25:        x_train_1_from_1_24_to_1_25
                                 + x_train_1_from_3_24_to_1_25
                                 - y_train_1_node_1_25  = 0
 set_y_train_1_node_1_26:        x_train_1_from_1_25_to_1_26
                                 + x_train_1_from_3_25_to_1_26
                                 - y_train_1_node_1_26  = 0
 set_y_train_1_node_1_27:        x_train_1_from_1_26_to_1_27
                                 + x_train_1_from_3_26_to_1_27
                                 - y_train_1_node_1_27  = 0
 set_y_train_1_node_1_28:        x_train_1_from_1_27_to_1_28
                                 + x_train_1_from_3_27_to_1_28
                                 - y_train_1_node_1_28  = 0
 set_y_train_1_node_1_29:        x_train_1_from_1_28_to_1_29
                                 + x_train_1_from_3_28_to_1_29
                                 - y_train_1_node_1_29  = 0
 set_y_train_1_node_2_3:         x_train_1_from_4_2_to_2_3 - y_train_1_node_2_3
                                  = 0
 set_y_train_1_node_2_4:         x_train_1_from_2_3_to_2_4
                                 + x_train_1_from_4_3_to_2_4
                                 - y_train_1_node_2_4  = 0
 set_y_train_1_node_2_5:         x_train_1_from_2_4_to_2_5
                                 + x_train_1_from_4_4_to_2_5
                                 - y_train_1_node_2_5  = 0
 set_y_train_1_node_2_6:         x_train_1_from_2_5_to_2_6
                                 + x_train_1_from_4_5_to_2_6
                                 - y_train_1_node_2_6  = 0
 set_y_train_1_node_2_7:         x_train_1_from_2_6_to_2_7
                                 + x_train_1_from_4_6_to_2_7
                                 - y_train_1_node_2_7  = 0
 set_y_train_1_node_2_8:         x_train_1_from_2_7_to_2_8
                                 + x_train_1_from_4_7_to_2_8
                                 - y_train_1_node_2_8  = 0
 set_y_train_1_node_2_9:         x_train_1_from_2_8_to_2_9
                                 + x_train_1_from_4_8_to_2_9
                                 - y_train_1_node_2_9  = 0
 set_y_train_1_node_2_10:        x_train_1_from_2_9_to_2_10
                                 + x_train_1_from_4_9_to_2_10
                                 - y_train_1_node_2_10  = 0
 set_y_train_1_node_2_11:        x_train_1_from_2_10_to_2_11
                                 + x_train_1_from_4_10_to_2_11
                                 - y_train_1_node_2_11  = 0
 set_y_train_1_node_2_12:        x_train_1_from_2_11_to_2_12
                                 + x_train_1_from_4_11_to_2_12
                                 - y_train_1_node_2_12  = 0
 set_y_train_1_node_2_13:        x_train_1_from_2_12_to_2_13
                                 + x_train_1_from_4_12_to_2_13
                                 - y_train_1_node_2_13  = 0
 set_y_train_1_node_2_14:        x_train_1_from_2_13_to_2_14
                                 + x_train_1_from_4_13_to_2_14
                                 - y_train_1_node_2_14  = 0
 set_y_train_1_node_2_15:        x_train_1_from_2_14_to_2_15
                                 + x_train_1_from_4_14_to_2_15
                                 - y_train_1_node_2_15  = 0
 set_y_train_1_node_2_16:        x_train_1_from_2_15_to_2_16
                                 + x_train_1_from_4_15_to_2_16
                                 - y_train_1_node_2_16  = 0
 set_y_train_1_node_2_17:        x_train_1_from_2_16_to_2_17
                                 + x_train_1_from_4_16_to_2_17
                                 - y_train_1_node_2_17  = 0
 set_y_train_1_node_2_18:        x_train_1_from_2_17_to_2_18
                                 + x_train_1_from_4_17_to_2_18
                                 - y_train_1_node_2_18  = 0
 set_y_train_1_node_2_19:        x_train_1_from_2_18_to_2_19
                                 + x_train_1_from_4_18_to_2_19
                                 - y_train_1_node_2_19  = 0
 set_y_train_1_node_2_20:        x_train_1_from_2_19_to_2_20
                                 + x_train_1_from_4_19_to_2_20
                                 - y_train_1_node_2_20  = 0
 set_y_train_1_node_2_21:        x_train_1_from_2_20_to_2_21
                                 + x_train_1_from_4_20_to_2_21
                                 - y_train_1_node_2_21  = 0
 set_y_train_1_node_2_22:        x_train_1_from_2_21_to_2_22
                                 + x_train_1_from_4_21_to_2_22
                                 - y_train_1_node_2_22  = 0
 set_y_train_1_node_2_23:        x_train_1_from_2_22_to_2_23
                                 + x_train_1_from_4_22_to_2_23
                                 - y_train_1_node_2_23  = 0
 set_y_train_1_node_2_24:        x_train_1_from_2_23_to_2_24
                                 + x_train_1_from_4_23_to_2_24
                                 - y_train_1_node_2_24  = 0
 set_y_train_1_node_2_25:        x_train_1_from_2_24_to_2_25
                                 + x_train_1_from_4_24_to_2_25
                                 - y_train_1_node_2_25  = 0
 set_y_train_1_node_2_26:        x_train_1_from_2_25_to_2_26
                                 + x_train_1_from_4_25_to_2_26
                                 - y_train_1_node_2_26  = 0
 set_y_train_1_node_2_27:        x_train_1_from_2_26_to_2_27
                                 + x_train_1_from_4_26_to_2_27
                                 - y_train_1_node_2_27  = 0
 set_y_train_1_node_2_28:        x_train_1_from_2_27_to_2_28
                                 + x_train_1_from_4_27_to_2_28
                                 - y_train_1_node_2_28  = 0
 set_y_train_1_node_2_29:        x_train_1_from_2_28_to_2_29
                                 + x_train_1_from_4_28_to_2_29
                                 - y_train_1_node_2_29  = 0
 set_y_train_1_node_3_5:         x_train_1_from_5_4_to_3_5 - y_train_1_node_3_5
                                  = 0
 set_y_train_1_node_3_6:         x_train_1_from_3_5_to_3_6
                                 + x_train_1_from_5_5_to_3_6
                                 - y_train_1_node_3_6  = 0
 set_y_train_1_node_3_7:         x_train_1_from_3_6_to_3_7
                                 + x_train_1_from_5_6_to_3_7
                                 - y_train_1_node_3_7  = 0
 set_y_train_1_node_3_8:         x_train_1_from_3_7_to_3_8
                                 + x_train_1_from_5_7_to_3_8
                                 - y_train_1_node_3_8  = 0
 set_y_train_1_node_3_9:         x_train_1_from_3_8_to_3_9
                                 + x_train_1_from_5_8_to_3_9
                                 - y_train_1_node_3_9  = 0
 set_y_train_1_node_3_10:        x_train_1_from_3_9_to_3_10
                                 + x_train_1_from_5_9_to_3_10
                                 - y_train_1_node_3_10  = 0
 set_y_train_1_node_3_11:        x_train_1_from_3_10_to_3_11
                                 + x_train_1_from_5_10_to_3_11
                                 - y_train_1_node_3_11  = 0
 set_y_train_1_node_3_12:        x_train_1_from_3_11_to_3_12
                                 + x_train_1_from_5_11_to_3_12
                                 - y_train_1_node_3_12  = 0
 set_y_train_1_node_3_13:        x_train_1_from_3_12_to_3_13
                                 + x_train_1_from_5_12_to_3_13
                                 - y_train_1_node_3_13  = 0
 set_y_train_1_node_3_14:        x_train_1_from_3_13_to_3_14
                                 + x_train_1_from_5_13_to_3_14
                                 - y_train_1_node_3_14  = 0
 set_y_train_1_node_3_15:        x_train_1_from_3_14_to_3_15
                                 + x_train_1_from_5_14_to_3_15
                                 - y_train_1_node_3_15  = 0
 set_y_train_1_node_3_16:        x_train_1_from_3_15_to_3_16
                                 + x_train_1_from_5_15_to_3_16
                                 - y_train_1_node_3_16  = 0
 set_y_train_1_node_3_17:        x_train_1_from_3_16_to_3_17
                                 + x_train_1_from_5_16_to_3_17
                                 - y_train_1_node_3_17  = 0
 set_y_train_1_node_3_18:        x_train_1_from_3_17_to_3_18
                                 + x_train_1_from_5_17_to_3_18
                                 - y_train_1_node_3_18  = 0
 set_y_train_1_node_3_19:        x_train_1_from_3_18_to_3_19
                                 + x_train_1_from_5_18_to_3_19
                                 - y_train_1_node_3_19  = 0
 set_y_train_1_node_3_20:        x_train_1_from_3_19_to_3_20
                                 + x_train_1_from_5_19_to_3_20
                                 - y_train_1_node_3_20  = 0
 set_y_train_1_node_3_21:        x_train_1_from_3_20_to_3_21
                                 + x_train_1_from_5_20_to_3_21
                                 - y_train_1_node_3_21  = 0
 set_y_train_1_node_3_22:        x_train_1_from_3_21_to_3_22
                                 + x_train_1_from_5_21_to_3_22
                                 - y_train_1_node_3_22  = 0
 set_y_train_1_node_3_23:        x_train_1_from_3_22_to_3_23
                                 + x_train_1_from_5_22_to_3_23
                                 - y_train_1_node_3_23  = 0
 set_y_train_1_node_3_24:        x_train_1_from_3_23_to_3_24
                                 + x_train_1_from_5_23_to_3_24
                                 - y_train_1_node_3_24  = 0
 set_y_train_1_node_3_25:        x_train_1_from_3_24_to_3_25
                                 + x_train_1_from_5_24_to_3_25
                                 - y_train_1_node_3_25  = 0
 set_y_train_1_node_3_26:        x_train_1_from_3_25_to_3_26
                                 + x_train_1_from_5_25_to_3_26
                                 - y_train_1_node_3_26  = 0
 set_y_train_1_node_3_27:        x_train_1_from_3_26_to_3_27
                                 + x_train_1_from_5_26_to_3_27
                                 - y_train_1_node_3_27  = 0
 set_y_train_1_node_3_28:        x_train_1_from_3_27_to_3_28
                                 + x_train_1_from_5_27_to_3_28
                                 - y_train_1_node_3_28  = 0
 set_y_train_1_node_4_2:         x_train_1_from_6_1_to_4_2 - y_train_1_node_4_2
                                  = 0
 set_y_train_1_node_4_3:         x_train_1_from_4_2_to_4_3
                                 + x_train_1_from_6_2_to_4_3
                                 - y_train_1_node_4_3  = 0
 set_y_train_1_node_4_4:         x_train_1_from_4_3_to_4_4
                                 + x_train_1_from_6_3_to_4_4
                                 - y_train_1_node_4_4  = 0
 set_y_train_1_node_4_5:         x_train_1_from_4_4_to_4_5
                                 + x_train_1_from_6_4_to_4_5
                                 - y_train_1_node_4_5  = 0
 set_y_train_1_node_4_6:         x_train_1_from_4_5_to_4_6
                                 + x_train_1_from_6_5_to_4_6
                                 - y_train_1_node_4_6  = 0
 set_y_train_1_node_4_7:         x_train_1_from_4_6_to_4_7
                                 + x_train_1_from_6_6_to_4_7
                                 - y_train_1_node_4_7  = 0
 set_y_train_1_node_4_8:         x_train_1_from_4_7_to_4_8
                                 + x_train_1_from_6_7_to_4_8
                                 - y_train_1_node_4_8  = 0
 set_y_train_1_node_4_9:         x_train_1_from_4_8_to_4_9
                                 + x_train_1_from_6_8_to_4_9
                                 - y_train_1_node_4_9  = 0
 set_y_train_1_node_4_10:        x_train_1_from_4_9_to_4_10
                                 + x_train_1_from_6_9_to_4_10
                                 - y_train_1_node_4_10  = 0
 set_y_train_1_node_4_11:        x_train_1_from_4_10_to_4_11
                                 + x_train_1_from_6_10_to_4_11
                                 - y_train_1_node_4_11  = 0
 set_y_train_1_node_4_12:        x_train_1_from_4_11_to_4_12
                                 + x_train_1_from_6_11_to_4_12
                                 - y_train_1_node_4_12  = 0
 set_y_train_1_node_4_13:        x_train_1_from_4_12_to_4_13
                                 + x_train_1_from_6_12_to_4_13
                                 - y_train_1_node_4_13  = 0
 set_y_train_1_node_4_14:        x_train_1_from_4_13_to_4_14
                                 + x_train_1_from_6_13_to_4_14
                                 - y_train_1_node_4_14  = 0
 set_y_train_1_node_4_15:        x_train_1_from_4_14_to_4_15
                                 + x_train_1_from_6_14_to_4_15
                                 - y_train_1_node_4_15  = 0
 set_y_train_1_node_4_16:        x_train_1_from_4_15_to_4_16
                                 + x_train_1_from_6_15_to_4_16
                                 - y_train_1_node_4_16  = 0
 set_y_train_1_node_4_17:        x_train_1_from_4_16_to_4_17
                                 + x_train_1_from_6_16_to_4_17
                                 - y_train_1_node_4_17  = 0
 set_y_train_1_node_4_18:        x_train_1_from_4_17_to_4_18
                                 + x_train_1_from_6_17_to_4_18
                                 - y_train_1_node_4_18  = 0
 set_y_train_1_node_4_19:        x_train_1_from_4_18_to_4_19
                                 + x_train_1_from_6_18_to_4_19
                                 - y_train_1_node_4_19  = 0
 set_y_train_1_node_4_20:        x_train_1_from_4_19_to_4_20
                                 + x_train_1_from_6_19_to_4_20
                                 - y_train_1_node_4_20  = 0
 set_y_train_1_node_4_21:        x_train_1_from_4_20_to_4_21
                                 + x_train_1_from_6_20_to_4_21
                                 - y_train_1_node_4_21  = 0
 set_y_train_1_node_4_22:        x_train_1_from_4_21_to_4_22
                                 + x_train_1_from_6_21_to_4_22
                                 - y_train_1_node_4_22  = 0
 set_y_train_1_node_4_23:        x_train_1_from_4_22_to_4_23
                                 + x_train_1_from_6_22_to_4_23
                                 - y_train_1_node_4_23  = 0
 set_y_train_1_node_4_24:        x_train_1_from_4_23_to_4_24
                                 + x_train_1_from_6_23_to_4_24
                                 - y_train_1_node_4_24  = 0
 set_y_train_1_node_4_25:        x_train_1_from_4_24_to_4_25
                                 + x_train_1_from_6_24_to_4_25
                                 - y_train_1_node_4_25  = 0
 set_y_train_1_node_4_26:        x_train_1_from_4_25_to_4_26
                                 + x_train_1_from_6_25_to_4_26
                                 - y_train_1_node_4_26  = 0
 set_y_train_1_node_4_27:        x_train_1_from_4_26_to_4_27
                                 + x_train_1_from_6_26_to_4_27
                                 - y_train_1_node_4_27  = 0
 set_y_train_1_node_4_28:        x_train_1_from_4_27_to_4_28
                                 + x_train_1_from_6_27_to_4_28
                                 - y_train_1_node_4_28  = 0
 set_y_train_1_node_5_2:         x_train_1_from_6_1_to_5_2 - y_train_1_node_5_2
                                  = 0
 set_y_train_1_node_5_3:         x_train_1_from_5_2_to_5_3
                                 + x_train_1_from_6_2_to_5_3
                                 - y_train_1_node_5_3  = 0
 set_y_train_1_node_5_4:         x_train_1_from_5_3_to_5_4
                                 + x_train_1_from_6_3_to_5_4
                                 - y_train_1_node_5_4  = 0
 set_y_train_1_node_5_5:         x_train_1_from_5_4_to_5_5
                                 + x_train_1_from_6_4_to_5_5
                                 - y_train_1_node_5_5  = 0
 set_y_train_1_node_5_6:         x_train_1_from_5_5_to_5_6
                                 + x_train_1_from_6_5_to_5_6
                                 - y_train_1_node_5_6  = 0
 set_y_train_1_node_5_7:         x_train_1_from_5_6_to_5_7
                                 + x_train_1_from_6_6_to_5_7
                                 - y_train_1_node_5_7  = 0
 set_y_train_1_node_5_8:         x_train_1_from_5_7_to_5_8
                                 + x_train_1_from_6_7_to_5_8
                                 - y_train_1_node_5_8  = 0
 set_y_train_1_node_5_9:         x_train_1_from_5_8_to_5_9
                                 + x_train_1_from_6_8_to_5_9
                                 - y_train_1_node_5_9  = 0
 set_y_train_1_node_5_10:        x_train_1_from_5_9_to_5_10
                                 + x_train_1_from_6_9_to_5_10
                                 - y_train_1_node_5_10  = 0
 set_y_train_1_node_5_11:        x_train_1_from_5_10_to_5_11
                                 + x_train_1_from_6_10_to_5_11
                                 - y_train_1_node_5_11  = 0
 set_y_train_1_node_5_12:        x_train_1_from_5_11_to_5_12
                                 + x_train_1_from_6_11_to_5_12
                                 - y_train_1_node_5_12  = 0
 set_y_train_1_node_5_13:        x_train_1_from_5_12_to_5_13
                                 + x_train_1_from_6_12_to_5_13
                                 - y_train_1_node_5_13  = 0
 set_y_train_1_node_5_14:        x_train_1_from_5_13_to_5_14
                                 + x_train_1_from_6_13_to_5_14
                                 - y_train_1_node_5_14  = 0
 set_y_train_1_node_5_15:        x_train_1_from_5_14_to_5_15
                                 + x_train_1_from_6_14_to_5_15
                                 - y_train_1_node_5_15  = 0
 set_y_train_1_node_5_16:        x_train_1_from_5_15_to_5_16
                                 + x_train_1_from_6_15_to_5_16
                                 - y_train_1_node_5_16  = 0
 set_y_train_1_node_5_17:        x_train_1_from_5_16_to_5_17
                                 + x_train_1_from_6_16_to_5_17
                                 - y_train_1_node_5_17  = 0
 set_y_train_1_node_5_18:        x_train_1_from_5_17_to_5_18
                                 + x_train_1_from_6_17_to_5_18
                                 - y_train_1_node_5_18  = 0
 set_y_train_1_node_5_19:        x_train_1_from_5_18_to_5_19
                                 + x_train_1_from_6_18_to_5_19
                                 - y_train_1_node_5_19  = 0
 set_y_train_1_node_5_20:        x_train_1_from_5_19_to_5_20
                                 + x_train_1_from_6_19_to_5_20
                                 - y_train_1_node_5_20  = 0
 set_y_train_1_node_5_21:        x_train_1_from_5_20_to_5_21
                                 + x_train_1_from_6_20_to_5_21
                                 - y_train_1_node_5_21  = 0
 set_y_train_1_node_5_22:        x_train_1_from_5_21_to_5_22
                                 + x_train_1_from_6_21_to_5_22
                                 - y_train_1_node_5_22  = 0
 set_y_train_1_node_5_23:        x_train_1_from_5_22_to_5_23
                                 + x_train_1_from_6_22_to_5_23
                                 - y_train_1_node_5_23  = 0
 set_y_train_1_node_5_24:        x_train_1_from_5_23_to_5_24
                                 + x_train_1_from_6_23_to_5_24
                                 - y_train_1_node_5_24  = 0
 set_y_train_1_node_5_25:        x_train_1_from_5_24_to_5_25
                                 + x_train_1_from_6_24_to_5_25
                                 - y_train_1_node_5_25  = 0
 set_y_train_1_node_5_26:        x_train_1_from_5_25_to_5_26
                                 + x_train_1_from_6_25_to_5_26
                                 - y_train_1_node_5_26  = 0
 set_y_train_1_node_5_27:        x_train_1_from_5_26_to_5_27
                                 + x_train_1_from_6_26_to_5_27
                                 - y_train_1_node_5_27  = 0
 set_y_train_1_node_6_1:         x_train_1_from_sigma_to_6_1
                                 - y_train_1_node_6_1  = 0
 set_y_train_1_node_6_2:         x_train_1_from_sigma_to_6_2
                                 + x_train_1_from_6_1_to_6_2
                                 - y_train_1_node_6_2  = 0
 set_y_train_1_node_6_3:         x_train_1_from_sigma_to_6_3
                                 + x_train_1_from_6_2_to_6_3
                                 - y_train_1_node_6_3  = 0
 set_y_train_1_node_6_4:         x_train_1_from_sigma_to_6_4
                                 + x_train_1_from_6_3_to_6_4
                                 - y_train_1_node_6_4  = 0
 set_y_train_1_node_6_5:         x_train_1_from_sigma_to_6_5
                                 + x_train_1_from_6_4_to_6_5
                                 - y_train_1_node_6_5  = 0
 set_y_train_1_node_6_6:         x_train_1_from_sigma_to_6_6
                                 + x_train_1_from_6_5_to_6_6
                                 - y_train_1_node_6_6  = 0
 set_y_train_1_node_6_7:         x_train_1_from_sigma_to_6_7
                                 + x_train_1_from_6_6_to_6_7
                                 - y_train_1_node_6_7  = 0
 set_y_train_1_node_6_8:         x_train_1_from_sigma_to_6_8
                                 + x_train_1_from_6_7_to_6_8
                                 - y_train_1_node_6_8  = 0
 set_y_train_1_node_6_9:         x_train_1_from_sigma_to_6_9
                                 + x_train_1_from_6_8_to_6_9
                                 - y_train_1_node_6_9  = 0
 set_y_train_1_node_6_10:        x_train_1_from_sigma_to_6_10
                                 + x_train_1_from_6_9_to_6_10
                                 - y_train_1_node_6_10  = 0
 set_y_train_1_node_6_11:        x_train_1_from_sigma_to_6_11
                                 + x_train_1_from_6_10_to_6_11
                                 - y_train_1_node_6_11  = 0
 set_y_train_1_node_6_12:        x_train_1_from_sigma_to_6_12
                                 + x_train_1_from_6_11_to_6_12
                                 - y_train_1_node_6_12  = 0
 set_y_train_1_node_6_13:        x_train_1_from_sigma_to_6_13
                                 + x_train_1_from_6_12_to_6_13
                                 - y_train_1_node_6_13  = 0
 set_y_train_1_node_6_14:        x_train_1_from_sigma_to_6_14
                                 + x_train_1_from_6_13_to_6_14
                                 - y_train_1_node_6_14  = 0
 set_y_train_1_node_6_15:        x_train_1_from_sigma_to_6_15
                                 + x_train_1_from_6_14_to_6_15
                                 - y_train_1_node_6_15  = 0
 set_y_train_1_node_6_16:        x_train_1_from_sigma_to_6_16
                                 + x_train_1_from_6_15_to_6_16
                                 - y_train_1_node_6_16  = 0
 set_y_train_1_node_6_17:        x_train_1_from_sigma_to_6_17
                                 + x_train_1_from_6_16_to_6_17
                                 - y_train_1_node_6_17  = 0
 set_y_train_1_node_6_18:        x_train_1_from_sigma_to_6_18
                                 + x_train_1_from_6_17_to_6_18
                                 - y_train_1_node_6_18  = 0
 set_y_train_1_node_6_19:        x_train_1_from_sigma_to_6_19
                                 + x_train_1_from_6_18_to_6_19
                                 - y_train_1_node_6_19  = 0
 set_y_train_1_node_6_20:        x_train_1_from_sigma_to_6_20
                                 + x_train_1_from_6_19_to_6_20
                                 - y_train_1_node_6_20  = 0
 set_y_train_1_node_6_21:        x_train_1_from_sigma_to_6_21
                                 + x_train_1_from_6_20_to_6_21
                                 - y_train_1_node_6_21  = 0
 set_y_train_1_node_6_22:        x_train_1_from_sigma_to_6_22
                                 + x_train_1_from_6_21_to_6_22
                                 - y_train_1_node_6_22  = 0
 set_y_train_1_node_6_23:        x_train_1_from_sigma_to_6_23
                                 + x_train_1_from_6_22_to_6_23
                                 - y_train_1_node_6_23  = 0
 set_y_train_1_node_6_24:        x_train_1_from_sigma_to_6_24
                                 + x_train_1_from_6_23_to_6_24
                                 - y_train_1_node_6_24  = 0
 set_y_train_1_node_6_25:        x_train_1_from_sigma_to_6_25
                                 + x_train_1_from_6_24_to_6_25
                                 - y_train_1_node_6_25  = 0
 set_y_train_1_node_6_26:        x_train_1_from_sigma_to_6_26
                                 + x_train_1_from_6_25_to_6_26
                                 - y_train_1_node_6_26  = 0
 set_y_train_1_node_6_27:        x_train_1_from_sigma_to_6_27
                                 + x_train_1_from_6_26_to_6_27
                                 - y_train_1_node_6_27  = 0
 flow_train_0_node_0_1:          x_train_0_from_sigma_to_0_1
                                 - x_train_0_from_0_1_to_0_2
                                 - x_train_0_from_0_1_to_1_2
                                 - x_train_0_from_0_1_to_2_2  = 0
 flow_train_0_node_0_2:          x_train_0_from_sigma_to_0_2
                                 + x_train_0_from_0_1_to_0_2
                                 - x_train_0_from_0_2_to_0_3
                                 - x_train_0_from_0_2_to_1_3
                                 - x_train_0_from_0_2_to_2_3  = 0
 flow_train_0_node_0_3:          x_train_0_from_sigma_to_0_3
                                 + x_train_0_from_0_2_to_0_3
                                 - x_train_0_from_0_3_to_0_4
                                 - x_train_0_from_0_3_to_1_4
                                 - x_train_0_from_0_3_to_2_4  = 0
 flow_train_0_node_0_4:          x_train_0_from_sigma_to_0_4
                                 + x_train_0_from_0_3_to_0_4
                                 - x_train_0_from_0_4_to_0_5
                                 - x_train_0_from_0_4_to_1_5
                                 - x_train_0_from_0_4_to_2_5  = 0
 flow_train_0_node_0_5:          x_train_0_from_sigma_to_0_5
                                 + x_train_0_from_0_4_to_0_5
                                 - x_train_0_from_0_5_to_0_6
                                 - x_train_0_from_0_5_to_1_6
                                 - x_train_0_from_0_5_to_2_6  = 0
 flow_train_0_node_0_6:          x_train_0_from_sigma_to_0_6
                                 + x_train_0_from_0_5_to_0_6
                                 - x_train_0_from_0_6_to_0_7
                                 - x_train_0_from_0_6_to_1_7
                                 - x_train_0_from_0_6_to_2_7  = 0
 flow_train_0_node_0_7:          x_train_0_from_sigma_to_0_7
                                 + x_train_0_from_0_6_to_0_7
                                 - x_train_0_from_0_7_to_0_8
                                 - x_train_0_from_0_7_to_1_8
                                 - x_train_0_from_0_7_to_2_8  = 0
 flow_train_0_node_0_8:          x_train_0_from_sigma_to_0_8
                                 + x_train_0_from_0_7_to_0_8
                                 - x_train_0_from_0_8_to_0_9
                                 - x_train_0_from_0_8_to_1_9
                                 - x_train_0_from_0_8_to_2_9  = 0
 flow_train_0_node_0_9:          x_train_0_from_sigma_to_0_9
                                 + x_train_0_from_0_8_to_0_9
                                 - x_train_0_from_0_9_to_0_10
                                 - x_train_0_from_0_9_to_1_10
                                 - x_train_0_from_0_9_to_2_10  = 0
 flow_train_0_node_0_10:         x_train_0_from_sigma_to_0_10
                                 + x_train_0_from_0_9_to_0_10
                                 - x_train_0_from_0_10_to_0_11
                                 - x_train_0_from_0_10_to_1_11
                                 - x_train_0_from_0_10_to_2_11  = 0
 flow_train_0_node_0_11:         x_train_0_from_sigma_to_0_11
                                 + x_train_0_from_0_10_to_0_11
                                 - x_train_0_from_0_11_to_0_12
                                 - x_train_0_from_0_11_to_1_12
                                 - x_train_0_from_0_11_to_2_12  = 0
 flow_train_0_node_0_12:         x_train_0_from_sigma_to_0_12
                                 + x_train_0_from_0_11_to_0_12
                                 - x_train_0_from_0_12_to_0_13
                                 - x_train_0_from_0_12_to_1_13
                                 - x_train_0_from_0_12_to_2_13  = 0
 flow_train_0_node_0_13:         x_train_0_from_sigma_to_0_13
                                 + x_train_0_from_0_12_to_0_13
                                 - x_train_0_from_0_13_to_0_14
                                 - x_train_0_from_0_13_to_1_14
                                 - x_train_0_from_0_13_to_2_14  = 0
 flow_train_0_node_0_14:         x_train_0_from_sigma_to_0_14
                                 + x_train_0_from_0_13_to_0_14
                                 - x_train_0_from_0_14_to_0_15
                                 - x_train_0_from_0_14_to_1_15
                                 - x_train_0_from_0_14_to_2_15  = 0
 flow_train_0_node_0_15:         x_train_0_from_sigma_to_0_15
                                 + x_train_0_from_0_14_to_0_15
                                 - x_train_0_from_0_15_to_0_16
                                 - x_train_0_from_0_15_to_1_16
                                 - x_train_0_from_0_15_to_2_16  = 0
 flow_train_0_node_0_16:         x_train_0_from_sigma_to_0_16
                                 + x_train_0_from_0_15_to_0_16
                                 - x_train_0_from_0_16_to_0_17
                                 - x_train_0_from_0_16_to_1_17
                                 - x_train_0_from_0_16_to_2_17  = 0
 flow_train_0_node_0_17:         x_train_0_from_sigma_to_0_17
                                 + x_train_0_from_0_16_to_0_17
                                 - x_train_0_from_0_17_to_0_18
                                 - x_train_0_from_0_17_to_1_18
                                 - x_train_0_from_0_17_to_2_18  = 0
 flow_train_0_node_0_18:         x_train_0_from_sigma_to_0_18
                                 + x_train_0_from_0_17_to_0_18
                                 - x_train_0_from_0_18_to_0_19
                                 - x_train_0_from_0_18_to_1_19
                                 - x_train_0_from_0_18_to_2_19  = 0
 flow_train_0_node_0_19:         x_train_0_from_sigma_to_0_19
                                 + x_train_0_from_0_18_to_0_19
                                 - x_train_0_from_0_19_to_0_20
                                 - x_train_0_from_0_19_to_1_20
                                 - x_train_0_from_0_19_to_2_20  = 0
 flow_train_0_node_0_20:         x_train_0_from_sigma_to_0_20
                                 + x_train_0_from_0_19_to_0_20
                                 - x_train_0_from_0_20_to_0_21
                                 - x_train_0_from_0_20_to_1_21
                                 - x_train_0_from_0_20_to_2_21  = 0
 flow_train_0_node_0_21:         x_train_0_from_sigma_to_0_21
                                 + x_train_0_from_0_20_to_0_21
                                 - x_train_0_from_0_21_to_0_22
                                 - x_train_0_from_0_21_to_1_22
                                 - x_train_0_from_0_21_to_2_22  = 0
 flow_train_0_node_0_22:         x_train_0_from_sigma_to_0_22
                                 + x_train_0_from_0_21_to_0_22
                                 - x_train_0_from_0_22_to_0_23
                                 - x_train_0_from_0_22_to_1_23
                                 - x_train_0_from_0_22_to_2_23  = 0
 flow_train_0_node_0_23:         x_train_0_from_sigma_to_0_23
                                 + x_train_0_from_0_22_to_0_23
                                 - x_train_0_from_0_23_to_0_24
                                 - x_train_0_from_0_23_to_1_24
                                 - x_train_0_from_0_23_to_2_24  = 0
 flow_train_0_node_0_24:         x_train_0_from_sigma_to_0_24
                                 + x_train_0_from_0_23_to_0_24
                                 - x_train_0_from_0_24_to_0_25
                                 - x_train_0_from_0_24_to_1_25
                                 - x_train_0_from_0_24_to_2_25  = 0
 flow_train_0_node_0_25:         x_train_0_from_sigma_to_0_25
                                 + x_train_0_from_0_24_to_0_25
                                 - x_train_0_from_0_25_to_0_26
                                 - x_train_0_from_0_25_to_1_26
                                 - x_train_0_from_0_25_to_2_26  = 0
 flow_train_0_node_0_26:         x_train_0_from_sigma_to_0_26
                                 + x_train_0_from_0_25_to_0_26
                                 - x_train_0_from_0_26_to_0_27
                                 - x_train_0_from_0_26_to_1_27
                                 - x_train_0_from_0_26_to_2_27  = 0
 flow_train_0_node_0_27:         x_train_0_from_sigma_to_0_27
                                 + x_train_0_from_0_26_to_0_27
                                 - x_train_0_from_0_27_to_2_28  = 0
 flow_train_0_node_1_2:          x_train_0_from_0_1_to_1_2
                                 - x_train_0_from_1_2_to_1_3  = 0
 flow_train_0_node_1_3:          x_train_0_from_0_2_to_1_3
                                 + x_train_0_from_1_2_to_1_3
                                 - x_train_0_from_1_3_to_1_4  = 0
 flow_train_0_node_1_4:          x_train_0_from_0_3_to_1_4
                                 + x_train_0_from_1_3_to_1_4
                                 - x_train_0_from_1_4_to_1_5
                                 - x_train_0_from_1_4_to_3_5  = 0
 flow_train_0_node_1_5:          x_train_0_from_0_4_to_1_5
                                 + x_train_0_from_1_4_to_1_5
                                 - x_train_0_from_1_5_to_1_6
                                 - x_train_0_from_1_5_to_3_6  = 0
 flow_train_0_node_1_6:          x_train_0_from_0_5_to_1_6
                                 + x_train_0_from_1_5_to_1_6
                                 - x_train_0_from_1_6_to_1_7
                                 - x_train_0_from_1_6_to_3_7  = 0
 flow_train_0_node_1_7:          x_train_0_from_0_6_to_1_7
                                 + x_train_0_from_1_6_to_1_7
                                 - x_train_0_from_1_7_to_1_8
                                 - x_train_0_from_1_7_to_3_8  = 0
 flow_train_0_node_1_8:          x_train_0_from_0_7_to_1_8
                                 + x_train_0_from_1_7_to_1_8
                                 - x_train_0_from_1_8_to_1_9
                                 - x_train_0_from_1_8_to_3_9  = 0
 flow_train_0_node_1_9:          x_train_0_from_0_8_to_1_9
                                 + x_train_0_from_1_8_to_1_9
                                 - x_train_0_from_1_9_to_1_10
                                 - x_train_0_from_1_9_to_3_10  = 0
 flow_train_0_node_1_10:         x_train_0_from_0_9_to_1_10
                                 + x_train_0_from_1_9_to_1_10
                                 - x_train_0_from_1_10_to_1_11
                                 - x_train_0_from_1_10_to_3_11  = 0
 flow_train_0_node_1_11:         x_train_0_from_0_10_to_1_11
                                 + x_train_0_from_1_10_to_1_11
                                 - x_train_0_from_1_11_to_1_12
                                 - x_train_0_from_1_11_to_3_12  = 0
 flow_train_0_node_1_12:         x_train_0_from_0_11_to_1_12
                                 + x_train_0_from_1_11_to_1_12
                                 - x_train_0_from_1_12_to_1_13
                                 - x_train_0_from_1_12_to_3_13  = 0
 flow_train_0_node_1_13:         x_train_0_from_0_12_to_1_13
                                 + x_train_0_from_1_12_to_1_13
                                 - x_train_0_from_1_13_to_1_14
                                 - x_train_0_from_1_13_to_3_14  = 0
 flow_train_0_node_1_14:         x_train_0_from_0_13_to_1_14
                                 + x_train_0_from_1_13_to_1_14
                                 - x_train_0_from_1_14_to_1_15
                                 - x_train_0_from_1_14_to_3_15  = 0
 flow_train_0_node_1_15:         x_train_0_from_0_14_to_1_15
                                 + x_train_0_from_1_14_to_1_15
                                 - x_train_0_from_1_15_to_1_16
                                 - x_train_0_from_1_15_to_3_16  = 0
 flow_train_0_node_1_16:         x_train_0_from_0_15_to_1_16
                                 + x_train_0_from_1_15_to_1_16
                                 - x_train_0_from_1_16_to_1_17
                                 - x_train_0_from_1_16_to_3_17  = 0
 flow_train_0_node_1_17:         x_train_0_from_0_16_to_1_17
                                 + x_train_0_from_1_16_to_1_17
                                 - x_train_0_from_1_17_to_1_18
                                 - x_train_0_from_1_17_to_3_18  = 0
 flow_train_0_node_1_18:         x_train_0_from_0_17_to_1_18
                                 + x_train_0_from_1_17_to_1_18
                                 - x_train_0_from_1_18_to_1_19
                                 - x_train_0_from_1_18_to_3_19  = 0
 flow_train_0_node_1_19:         x_train_0_from_0_18_to_1_19
                                 + x_train_0_from_1_18_to_1_19
                                 - x_train_0_from_1_19_to_1_20
                                 - x_train_0_from_1_19_to_3_20  = 0
 flow_train_0_node_1_20:         x_train_0_from_0_19_to_1_20
                                 + x_train_0_from_1_19_to_1_20
                                 - x_train_0_from_1_20_to_1_21
                                 - x_train_0_from_1_20_to_3_21  = 0
 flow_train_0_node_1_21:         x_train_0_from_0_20_to_1_21
                                 + x_train_0_from_1_20_to_1_21
                                 - x_train_0_from_1_21_to_1_22
                                 - x_train_0_from_1_21_to_3_22  = 0
 flow_train_0_node_1_22:         x_train_0_from_0_21_to_1_22
                                 + x_train_0_from_1_21_to_1_22
                                 - x_train_0_from_1_22_to_1_23
                                 - x_train_0_from_1_22_to_3_23  = 0
 flow_train_0_node_1_23:         x_train_0_from_0_22_to_1_23
                                 + x_train_0_from_1_22_to_1_23
                                 - x_train_0_from_1_23_to_1_24
                                 - x_train_0_from_1_23_to_3_24  = 0
 flow_train_0_node_1_24:         x_train_0_from_0_23_to_1_24
                                 + x_train_0_from_1_23_to_1_24
                                 - x_train_0_from_1_24_to_1_25
                                 - x_train_0_from_1_24_to_3_25  = 0
 flow_train_0_node_1_25:         x_train_0_from_0_24_to_1_25
                                 + x_train_0_from_1_24_to_1_25
                                 - x_train_0_from_1_25_to_1_26
                                 - x_train_0_from_1_25_to_3_26  = 0
 flow_train_0_node_1_26:         x_train_0_from_0_25_to_1_26
                                 + x_train_0_from_1_25_to_1_26
                                 - x_train_0_from_1_26_to_1_27
                                 - x_train_0_from_1_26_to_3_27  = 0
 flow_train_0_node_1_27:         x_train_0_from_0_26_to_1_27
                                 + x_train_0_from_1_26_to_1_27
                                 - x_train_0_from_1_27_to_3_28  = 0
 flow_train_0_node_2_2:          x_train_0_from_0_1_to_2_2
                                 - x_train_0_from_2_2_to_2_3
                                 - x_train_0_from_2_2_to_4_3  = 0
 flow_train_0_node_2_3:          x_train_0_from_0_2_to_2_3
                                 + x_train_0_from_2_2_to_2_3
                                 - x_train_0_from_2_3_to_2_4
                                 - x_train_0_from_2_3_to_4_4  = 0
 flow_train_0_node_2_4:          x_train_0_from_0_3_to_2_4
                                 + x_train_0_from_2_3_to_2_4
                                 - x_train_0_from_2_4_to_2_5
                                 - x_train_0_from_2_4_to_4_5  = 0
 flow_train_0_node_2_5:          x_train_0_from_0_4_to_2_5
                                 + x_train_0_from_2_4_to_2_5
                                 - x_train_0_from_2_5_to_2_6
                                 - x_train_0_from_2_5_to_4_6  = 0
 flow_train_0_node_2_6:          x_train_0_from_0_5_to_2_6
                                 + x_train_0_from_2_5_to_2_6
                                 - x_train_0_from_2_6_to_2_7
                                 - x_train_0_from_2_6_to_4_7  = 0
 flow_train_0_node_2_7:          x_train_0_from_0_6_to_2_7
                                 + x_train_0_from_2_6_to_2_7
                                 - x_train_0_from_2_7_to_2_8
                                 - x_train_0_from_2_7_to_4_8  = 0
 flow_train_0_node_2_8:          x_train_0_from_0_7_to_2_8
                                 + x_train_0_from_2_7_to_2_8
                                 - x_train_0_from_2_8_to_2_9
                                 - x_train_0_from_2_8_to_4_9  = 0
 flow_train_0_node_2_9:          x_train_0_from_0_8_to_2_9
                                 + x_train_0_from_2_8_to_2_9
                                 - x_train_0_from_2_9_to_2_10
                                 - x_train_0_from_2_9_to_4_10  = 0
 flow_train_0_node_2_10:         x_train_0_from_0_9_to_2_10
                                 + x_train_0_from_2_9_to_2_10
                                 - x_train_0_from_2_10_to_2_11
                                 - x_train_0_from_2_10_to_4_11  = 0
 flow_train_0_node_2_11:         x_train_0_from_0_10_to_2_11
                                 + x_train_0_from_2_10_to_2_11
                                 - x_train_0_from_2_11_to_2_12
                                 - x_train_0_from_2_11_to_4_12  = 0
 flow_train_0_node_2_12:         x_train_0_from_0_11_to_2_12
                                 + x_train_0_from_2_11_to_2_12
                                 - x_train_0_from_2_12_to_2_13
                                 - x_train_0_from_2_12_to_4_13  = 0
 flow_train_0_node_2_13:         x_train_0_from_0_12_to_2_13
                                 + x_train_0_from_2_12_to_2_13
                                 - x_train_0_from_2_13_to_2_14
                                 - x_train_0_from_2_13_to_4_14  = 0
 flow_train_0_node_2_14:         x_train_0_from_0_13_to_2_14
                                 + x_train_0_from_2_13_to_2_14
                                 - x_train_0_from_2_14_to_2_15
                                 - x_train_0_from_2_14_to_4_15  = 0
 flow_train_0_node_2_15:         x_train_0_from_0_14_to_2_15
                                 + x_train_0_from_2_14_to_2_15
                                 - x_train_0_from_2_15_to_2_16
                                 - x_train_0_from_2_15_to_4_16  = 0
 flow_train_0_node_2_16:         x_train_0_from_0_15_to_2_16
                                 + x_train_0_from_2_15_to_2_16
                                 - x_train_0_from_2_16_to_2_17
                                 - x_train_0_from_2_16_to_4_17  = 0
 flow_train_0_node_2_17:         x_train_0_from_0_16_to_2_17
                                 + x_train_0_from_2_16_to_2_17
                                 - x_train_0_from_2_17_to_2_18
                                 - x_train_0_from_2_17_to_4_18  = 0
 flow_train_0_node_2_18:         x_train_0_from_0_17_to_2_18
                                 + x_train_0_from_2_17_to_2_18
                                 - x_train_0_from_2_18_to_2_19
                                 - x_train_0_from_2_18_to_4_19  = 0
 flow_train_0_node_2_19:         x_train_0_from_0_18_to_2_19
                                 + x_train_0_from_2_18_to_2_19
                                 - x_train_0_from_2_19_to_2_20
                                 - x_train_0_from_2_19_to_4_20  = 0
 flow_train_0_node_2_20:         x_train_0_from_0_19_to_2_20
                                 + x_train_0_from_2_19_to_2_20
                                 - x_train_0_from_2_20_to_2_21
                                 - x_train_0_from_2_20_to_4_21  = 0
 flow_train_0_node_2_21:         x_train_0_from_0_20_to_2_21
                                 + x_train_0_from_2_20_to_2_21
                                 - x_train_0_from_2_21_to_2_22
                                 - x_train_0_from_2_21_to_4_22  = 0
 flow_train_0_node_2_22:         x_train_0_from_0_21_to_2_22
                                 + x_train_0_from_2_21_to_2_22
                                 - x_train_0_from_2_22_to_2_23
                                 - x_train_0_from_2_22_to_4_23  = 0
 flow_train_0_node_2_23:         x_train_0_from_0_22_to_2_23
                                 + x_train_0_from_2_22_to_2_23
                                 - x_train_0_from_2_23_to_2_24
                                 - x_train_0_from_2_23_to_4_24  = 0
 flow_train_0_node_2_24:         x_train_0_from_0_23_to_2_24
                                 + x_train_0_from_2_23_to_2_24
                                 - x_train_0_from_2_24_to_2_25
                                 - x_train_0_from_2_24_to_4_25  = 0
 flow_train_0_node_2_25:         x_train_0_from_0_24_to_2_25
                                 + x_train_0_from_2_24_to_2_25
                                 - x_train_0_from_2_25_to_2_26
                                 - x_train_0_from_2_25_to_4_26  = 0
 flow_train_0_node_2_26:         x_train_0_from_0_25_to_2_26
                                 + x_train_0_from_2_25_to_2_26
                                 - x_train_0_from_2_26_to_2_27
                                 - x_train_0_from_2_26_to_4_27  = 0
 flow_train_0_node_2_27:         x_train_0_from_0_26_to_2_27
                                 + x_train_0_from_2_26_to_2_27
                                 - x_train_0_from_2_27_to_2_28
                                 - x_train_0_from_2_27_to_4_28  = 0
 flow_train_0_node_2_28:         x_train_0_from_0_27_to_2_28
                                 + x_train_0_from_2_27_to_2_28
                                 - x_train_0_from_2_28_to_4_29  = 0
 flow_train_0_node_3_5:          x_train_0_from_1_4_to_3_5
                                 - x_train_0_from_3_5_to_3_6
                                 - x_train_0_from_3_5_to_5_6  = 0
 flow_train_0_node_3_6:          x_train_0_from_1_5_to_3_6
                                 + x_train_0_from_3_5_to_3_6
                                 - x_train_0_from_3_6_to_3_7
                                 - x_train_0_from_3_6_to_5_7  = 0
 flow_train_0_node_3_7:          x_train_0_from_1_6_to_3_7
                                 + x_train_0_from_3_6_to_3_7
                                 - x_train_0_from_3_7_to_3_8
                                 - x_train_0_from_3_7_to_5_8  = 0
 flow_train_0_node_3_8:          x_train_0_from_1_7_to_3_8
                                 + x_train_0_from_3_7_to_3_8
                                 - x_train_0_from_3_8_to_3_9
                                 - x_train_0_from_3_8_to_5_9  = 0
 flow_train_0_node_3_9:          x_train_0_from_1_8_to_3_9
                                 + x_train_0_from_3_8_to_3_9
                                 - x_train_0_from_3_9_to_3_10
                                 - x_train_0_from_3_9_to_5_10  = 0
 flow_train_0_node_3_10:         x_train_0_from_1_9_to_3_10
                                 + x_train_0_from_3_9_to_3_10
                                 - x_train_0_from_3_10_to_3_11
                                 - x_train_0_from_3_10_to_5_11  = 0
 flow_train_0_node_3_11:         x_train_0_from_1_10_to_3_11
                                 + x_train_0_from_3_10_to_3_11
                                 - x_train_0_from_3_11_to_3_12
                                 - x_train_0_from_3_11_to_5_12  = 0
 flow_train_0_node_3_12:         x_train_0_from_1_11_to_3_12
                                 + x_train_0_from_3_11_to_3_12
                                 - x_train_0_from_3_12_to_3_13
                                 - x_train_0_from_3_12_to_5_13  = 0
 flow_train_0_node_3_13:         x_train_0_from_1_12_to_3_13
                                 + x_train_0_from_3_12_to_3_13
                                 - x_train_0_from_3_13_to_3_14
                                 - x_train_0_from_3_13_to_5_14  = 0
 flow_train_0_node_3_14:         x_train_0_from_1_13_to_3_14
                                 + x_train_0_from_3_13_to_3_14
                                 - x_train_0_from_3_14_to_3_15
                                 - x_train_0_from_3_14_to_5_15  = 0
 flow_train_0_node_3_15:         x_train_0_from_1_14_to_3_15
                                 + x_train_0_from_3_14_to_3_15
                                 - x_train_0_from_3_15_to_3_16
                                 - x_train_0_from_3_15_to_5_16  = 0
 flow_train_0_node_3_16:         x_train_0_from_1_15_to_3_16
                                 + x_train_0_from_3_15_to_3_16
                                 - x_train_0_from_3_16_to_3_17
                                 - x_train_0_from_3_16_to_5_17  = 0
 flow_train_0_node_3_17:         x_train_0_from_1_16_to_3_17
                                 + x_train_0_from_3_16_to_3_17
                                 - x_train_0_from_3_17_to_3_18
                                 - x_train_0_from_3_17_to_5_18  = 0
 flow_train_0_node_3_18:         x_train_0_from_1_17_to_3_18
                                 + x_train_0_from_3_17_to_3_18
                                 - x_train_0_from_3_18_to_3_19
                                 - x_train_0_from_3_18_to_5_19  = 0
 flow_train_0_node_3_19:         x_train_0_from_1_18_to_3_19
                                 + x_train_0_from_3_18_to_3_19
                                 - x_train_0_from_3_19_to_3_20
                                 - x_train_0_from_3_19_to_5_20  = 0
 flow_train_0_node_3_20:         x_train_0_from_1_19_to_3_20
                                 + x_train_0_from_3_19_to_3_20
                                 - x_train_0_from_3_20_to_3_21
                                 - x_train_0_from_3_20_to_5_21  = 0
 flow_train_0_node_3_21:         x_train_0_from_1_20_to_3_21
                                 + x_train_0_from_3_20_to_3_21
                                 - x_train_0_from_3_21_to_3_22
                                 - x_train_0_from_3_21_to_5_22  = 0
 flow_train_0_node_3_22:         x_train_0_from_1_21_to_3_22
                                 + x_train_0_from_3_21_to_3_22
                                 - x_train_0_from_3_22_to_3_23
                                 - x_train_0_from_3_22_to_5_23  = 0
 flow_train_0_node_3_23:         x_train_0_from_1_22_to_3_23
                                 + x_train_0_from_3_22_to_3_23
                                 - x_train_0_from_3_23_to_3_24
                                 - x_train_0_from_3_23_to_5_24  = 0
 flow_train_0_node_3_24:         x_train_0_from_1_23_to_3_24
                                 + x_train_0_from_3_23_to_3_24
                                 - x_train_0_from_3_24_to_3_25
                                 - x_train_0_from_3_24_to_5_25  = 0
 flow_train_0_node_3_25:         x_train_0_from_1_24_to_3_25
                                 + x_train_0_from_3_24_to_3_25
                                 - x_train_0_from_3_25_to_3_26
                                 - x_train_0_from_3_25_to_5_26  = 0
 flow_train_0_node_3_26:         x_train_0_from_1_25_to_3_26
                                 + x_train_0_from_3_25_to_3_26
                                 - x_train_0_from_3_26_to_3_27
                                 - x_train_0_from_3_26_to_5_27  = 0
 flow_train_0_node_3_27:         x_train_0_from_1_26_to_3_27
                                 + x_train_0_from_3_26_to_3_27
                                 - x_train_0_from_3_27_to_3_28
                                 - x_train_0_from_3_27_to_5_28  = 0
 flow_train_0_node_3_28:         x_train_0_from_1_27_to_3_28
                                 + x_train_0_from_3_27_to_3_28
                                 - x_train_0_from_3_28_to_5_29  = 0
 flow_train_0_node_4_3:          x_train_0_from_2_2_to_4_3
                                 - x_train_0_from_4_3_to_4_4
                                 - x_train_0_from_4_3_to_6_4  = 0
 flow_train_0_node_4_4:          x_train_0_from_2_3_to_4_4
                                 + x_train_0_from_4_3_to_4_4
                                 - x_train_0_from_4_4_to_4_5
                                 - x_train_0_from_4_4_to_6_5  = 0
 flow_train_0_node_4_5:          x_train_0_from_2_4_to_4_5
                                 + x_train_0_from_4_4_to_4_5
                                 - x_train_0_from_4_5_to_4_6
                                 - x_train_0_from_4_5_to_6_6  = 0
 flow_train_0_node_4_6:          x_train_0_from_2_5_to_4_6
                                 + x_train_0_from_4_5_to_4_6
                                 - x_train_0_from_4_6_to_4_7
                                 - x_train_0_from_4_6_to_6_7  = 0
 flow_train_0_node_4_7:          x_train_0_from_2_6_to_4_7
                                 + x_train_0_from_4_6_to_4_7
                                 - x_train_0_from_4_7_to_4_8
                                 - x_train_0_from_4_7_to_6_8  = 0
 flow_train_0_node_4_8:          x_train_0_from_2_7_to_4_8
                                 + x_train_0_from_4_7_to_4_8
                                 - x_train_0_from_4_8_to_4_9
                                 - x_train_0_from_4_8_to_6_9  = 0
 flow_train_0_node_4_9:          x_train_0_from_2_8_to_4_9
                                 + x_train_0_from_4_8_to_4_9
                                 - x_train_0_from_4_9_to_4_10
                                 - x_train_0_from_4_9_to_6_10  = 0
 flow_train_0_node_4_10:         x_train_0_from_2_9_to_4_10
                                 + x_train_0_from_4_9_to_4_10
                                 - x_train_0_from_4_10_to_4_11
                                 - x_train_0_from_4_10_to_6_11  = 0
 flow_train_0_node_4_11:         x_train_0_from_2_10_to_4_11
                                 + x_train_0_from_4_10_to_4_11
                                 - x_train_0_from_4_11_to_4_12
                                 - x_train_0_from_4_11_to_6_12  = 0
 flow_train_0_node_4_12:         x_train_0_from_2_11_to_4_12
                                 + x_train_0_from_4_11_to_4_12
                                 - x_train_0_from_4_12_to_4_13
                                 - x_train_0_from_4_12_to_6_13  = 0
 flow_train_0_node_4_13:         x_train_0_from_2_12_to_4_13
                                 + x_train_0_from_4_12_to_4_13
                                 - x_train_0_from_4_13_to_4_14
                                 - x_train_0_from_4_13_to_6_14  = 0
 flow_train_0_node_4_14:         x_train_0_from_2_13_to_4_14
                                 + x_train_0_from_4_13_to_4_14
                                 - x_train_0_from_4_14_to_4_15
                                 - x_train_0_from_4_14_to_6_15  = 0
 flow_train_0_node_4_15:         x_train_0_from_2_14_to_4_15
                                 + x_train_0_from_4_14_to_4_15
                                 - x_train_0_from_4_15_to_4_16
                                 - x_train_0_from_4_15_to_6_16  = 0
 flow_train_0_node_4_16:         x_train_0_from_2_15_to_4_16
                                 + x_train_0_from_4_15_to_4_16
                                 - x_train_0_from_4_16_to_4_17
                                 - x_train_0_from_4_16_to_6_17  = 0
 flow_train_0_node_4_17:         x_train_0_from_2_16_to_4_17
                                 + x_train_0_from_4_16_to_4_17
                                 - x_train_0_from_4_17_to_4_18
                                 - x_train_0_from_4_17_to_6_18  = 0
 flow_train_0_node_4_18:         x_train_0_from_2_17_to_4_18
                                 + x_train_0_from_4_17_to_4_18
                                 - x_train_0_from_4_18_to_4_19
                                 - x_train_0_from_4_18_to_6_19  = 0
 flow_train_0_node_4_19:         x_train_0_from_2_18_to_4_19
                                 + x_train_0_from_4_18_to_4_19
                                 - x_train_0_from_4_19_to_4_20
                                 - x_train_0_from_4_19_to_6_20  = 0
 flow_train_0_node_4_20:         x_train_0_from_2_19_to_4_20
                                 + x_train_0_from_4_19_to_4_20
                                 - x_train_0_from_4_20_to_4_21
                                 - x_train_0_from_4_20_to_6_21  = 0
 flow_train_0_node_4_21:         x_train_0_from_2_20_to_4_21
                                 + x_train_0_from_4_20_to_4_21
                                 - x_train_0_from_4_21_to_4_22
                                 - x_train_0_from_4_21_to_6_22  = 0
 flow_train_0_node_4_22:         x_train_0_from_2_21_to_4_22
                                 + x_train_0_from_4_21_to_4_22
                                 - x_train_0_from_4_22_to_4_23
                                 - x_train_0_from_4_22_to_6_23  = 0
 flow_train_0_node_4_23:         x_train_0_from_2_22_to_4_23
                                 + x_train_0_from_4_22_to_4_23
                                 - x_train_0_from_4_23_to_4_24
                                 - x_train_0_from_4_23_to_6_24  = 0
 flow_train_0_node_4_24:         x_train_0_from_2_23_to_4_24
                                 + x_train_0_from_4_23_to_4_24
                                 - x_train_0_from_4_24_to_4_25
                                 - x_train_0_from_4_24_to_6_25  = 0
 flow_train_0_node_4_25:         x_train_0_from_2_24_to_4_25
                                 + x_train_0_from_4_24_to_4_25
                                 - x_train_0_from_4_25_to_4_26
                                 - x_train_0_from_4_25_to_6_26  = 0
 flow_train_0_node_4_26:         x_train_0_from_2_25_to_4_26
                                 + x_train_0_from_4_25_to_4_26
                                 - x_train_0_from_4_26_to_4_27
                                 - x_train_0_from_4_26_to_6_27  = 0
 flow_train_0_node_4_27:         x_train_0_from_2_26_to_4_27
                                 + x_train_0_from_4_26_to_4_27
                                 - x_train_0_from_4_27_to_4_28
                                 - x_train_0_from_4_27_to_6_28  = 0
 flow_train_0_node_4_28:         x_train_0_from_2_27_to_4_28
                                 + x_train_0_from_4_27_to_4_28
                                 - x_train_0_from_4_28_to_4_29
                                 - x_train_0_from_4_28_to_6_29  = 0
 flow_train_0_node_4_29:         x_train_0_from_2_28_to_4_29
                                 + x_train_0_from_4_28_to_4_29
                                 - x_train_0_from_4_29_to_6_30  = 0
 flow_train_0_node_5_6:          x_train_0_from_3_5_to_5_6
                                 - x_train_0_from_5_6_to_5_7
                                 - x_train_0_from_5_6_to_6_7  = 0
 flow_train_0_node_5_7:          x_train_0_from_3_6_to_5_7
                                 + x_train_0_from_5_6_to_5_7
                                 - x_train_0_from_5_7_to_5_8
                                 - x_train_0_from_5_7_to_6_8  = 0
 flow_train_0_node_5_8:          x_train_0_from_3_7_to_5_8
                                 + x_train_0_from_5_7_to_5_8
                                 - x_train_0_from_5_8_to_5_9
                                 - x_train_0_from_5_8_to_6_9  = 0
 flow_train_0_node_5_9:          x_train_0_from_3_8_to_5_9
                                 + x_train_0_from_5_8_to_5_9
                                 - x_train_0_from_5_9_to_5_10
                                 - x_train_0_from_5_9_to_6_10  = 0
 flow_train_0_node_5_10:         x_train_0_from_3_9_to_5_10
                                 + x_train_0_from_5_9_to_5_10
                                 - x_train_0_from_5_10_to_5_11
                                 - x_train_0_from_5_10_to_6_11  = 0
 flow_train_0_node_5_11:         x_train_0_from_3_10_to_5_11
                                 + x_train_0_from_5_10_to_5_11
                                 - x_train_0_from_5_11_to_5_12
                                 - x_train_0_from_5_11_to_6_12  = 0
 flow_train_0_node_5_12:         x_train_0_from_3_11_to_5_12
                                 + x_train_0_from_5_11_to_5_12
                                 - x_train_0_from_5_12_to_5_13
                                 - x_train_0_from_5_12_to_6_13  = 0
 flow_train_0_node_5_13:         x_train_0_from_3_12_to_5_13
                                 + x_train_0_from_5_12_to_5_13
                                 - x_train_0_from_5_13_to_5_14
                                 - x_train_0_from_5_13_to_6_14  = 0
 flow_train_0_node_5_14:         x_train_0_from_3_13_to_5_14
                                 + x_train_0_from_5_13_to_5_14
                                 - x_train_0_from_5_14_to_5_15
                                 - x_train_0_from_5_14_to_6_15  = 0
 flow_train_0_node_5_15:         x_train_0_from_3_14_to_5_15
                                 + x_train_0_from_5_14_to_5_15
                                 - x_train_0_from_5_15_to_5_16
                                 - x_train_0_from_5_15_to_6_16  = 0
 flow_train_0_node_5_16:         x_train_0_from_3_15_to_5_16
                                 + x_train_0_from_5_15_to_5_16
                                 - x_train_0_from_5_16_to_5_17
                                 - x_train_0_from_5_16_to_6_17  = 0
 flow_train_0_node_5_17:         x_train_0_from_3_16_to_5_17
                                 + x_train_0_from_5_16_to_5_17
                                 - x_train_0_from_5_17_to_5_18
                                 - x_train_0_from_5_17_to_6_18  = 0
 flow_train_0_node_5_18:         x_train_0_from_3_17_to_5_18
                                 + x_train_0_from_5_17_to_5_18
                                 - x_train_0_from_5_18_to_5_19
                                 - x_train_0_from_5_18_to_6_19  = 0
 flow_train_0_node_5_19:         x_train_0_from_3_18_to_5_19
                                 + x_train_0_from_5_18_to_5_19
                                 - x_train_0_from_5_19_to_5_20
                                 - x_train_0_from_5_19_to_6_20  = 0
 flow_train_0_node_5_20:         x_train_0_from_3_19_to_5_20
                                 + x_train_0_from_5_19_to_5_20
                                 - x_train_0_from_5_20_to_5_21
                                 - x_train_0_from_5_20_to_6_21  = 0
 flow_train_0_node_5_21:         x_train_0_from_3_20_to_5_21
                                 + x_train_0_from_5_20_to_5_21
                                 - x_train_0_from_5_21_to_5_22
                                 - x_train_0_from_5_21_to_6_22  = 0
 flow_train_0_node_5_22:         x_train_0_from_3_21_to_5_22
                                 + x_train_0_from_5_21_to_5_22
                                 - x_train_0_from_5_22_to_5_23
                                 - x_train_0_from_5_22_to_6_23  = 0
 flow_train_0_node_5_23:         x_train_0_from_3_22_to_5_23
                                 + x_train_0_from_5_22_to_5_23
                                 - x_train_0_from_5_23_to_5_24
                                 - x_train_0_from_5_23_to_6_24  = 0
 flow_train_0_node_5_24:         x_train_0_from_3_23_to_5_24
                                 + x_train_0_from_5_23_to_5_24
                                 - x_train_0_from_5_24_to_5_25
                                 - x_train_0_from_5_24_to_6_25  = 0
 flow_train_0_node_5_25:         x_train_0_from_3_24_to_5_25
                                 + x_train_0_from_5_24_to_5_25
                                 - x_train_0_from_5_25_to_5_26
                                 - x_train_0_from_5_25_to_6_26  = 0
 flow_train_0_node_5_26:         x_train_0_from_3_25_to_5_26
                                 + x_train_0_from_5_25_to_5_26
                                 - x_train_0_from_5_26_to_5_27
                                 - x_train_0_from_5_26_to_6_27  = 0
 flow_train_0_node_5_27:         x_train_0_from_3_26_to_5_27
                                 + x_train_0_from_5_26_to_5_27
                                 - x_train_0_from_5_27_to_5_28
                                 - x_train_0_from_5_27_to_6_28  = 0
 flow_train_0_node_5_28:         x_train_0_from_3_27_to_5_28
                                 + x_train_0_from_5_27_to_5_28
                                 - x_train_0_from_5_28_to_5_29
                                 - x_train_0_from_5_28_to_6_29  = 0
 flow_train_0_node_5_29:         x_train_0_from_3_28_to_5_29
                                 + x_train_0_from_5_28_to_5_29
                                 - x_train_0_from_5_29_to_6_30  = 0
 flow_train_0_node_6_4:          - x_train_0_from_6_4_to_tau
                                 + x_train_0_from_4_3_to_6_4
                                 - x_train_0_from_6_4_to_6_5  = 0
 flow_train_0_node_6_5:          - x_train_0_from_6_5_to_tau
                                 + x_train_0_from_4_4_to_6_5
                                 + x_train_0_from_6_4_to_6_5
                                 - x_train_0_from_6_5_to_6_6  = 0
 flow_train_0_node_6_6:          - x_train_0_from_6_6_to_tau
                                 + x_train_0_from_4_5_to_6_6
                                 + x_train_0_from_6_5_to_6_6
                                 - x_train_0_from_6_6_to_6_7  = 0
 flow_train_0_node_6_7:          - x_train_0_from_6_7_to_tau
                                 + x_train_0_from_4_6_to_6_7
                                 + x_train_0_from_5_6_to_6_7
                                 + x_train_0_from_6_6_to_6_7
                                 - x_train_0_from_6_7_to_6_8  = 0
 flow_train_0_node_6_8:          - x_train_0_from_6_8_to_tau
                                 + x_train_0_from_4_7_to_6_8
                                 + x_train_0_from_5_7_to_6_8
                                 + x_train_0_from_6_7_to_6_8
                                 - x_train_0_from_6_8_to_6_9  = 0
 flow_train_0_node_6_9:          - x_train_0_from_6_9_to_tau
                                 + x_train_0_from_4_8_to_6_9
                                 + x_train_0_from_5_8_to_6_9
                                 + x_train_0_from_6_8_to_6_9
                                 - x_train_0_from_6_9_to_6_10  = 0
 flow_train_0_node_6_10:         - x_train_0_from_6_10_to_tau
                                 + x_train_0_from_4_9_to_6_10
                                 + x_train_0_from_5_9_to_6_10
                                 + x_train_0_from_6_9_to_6_10
                                 - x_train_0_from_6_10_to_6_11  = 0
 flow_train_0_node_6_11:         - x_train_0_from_6_11_to_tau
                                 + x_train_0_from_4_10_to_6_11
                                 + x_train_0_from_5_10_to_6_11
                                 + x_train_0_from_6_10_to_6_11
                                 - x_train_0_from_6_11_to_6_12  = 0
 flow_train_0_node_6_12:         - x_train_0_from_6_12_to_tau
                                 + x_train_0_from_4_11_to_6_12
                                 + x_train_0_from_5_11_to_6_12
                                 + x_train_0_from_6_11_to_6_12
                                 - x_train_0_from_6_12_to_6_13  = 0
 flow_train_0_node_6_13:         - x_train_0_from_6_13_to_tau
                                 + x_train_0_from_4_12_to_6_13
                                 + x_train_0_from_5_12_to_6_13
                                 + x_train_0_from_6_12_to_6_13
                                 - x_train_0_from_6_13_to_6_14  = 0
 flow_train_0_node_6_14:         - x_train_0_from_6_14_to_tau
                                 + x_train_0_from_4_13_to_6_14
                                 + x_train_0_from_5_13_to_6_14
                                 + x_train_0_from_6_13_to_6_14
                                 - x_train_0_from_6_14_to_6_15  = 0
 flow_train_0_node_6_15:         - x_train_0_from_6_15_to_tau
                                 + x_train_0_from_4_14_to_6_15
                                 + x_train_0_from_5_14_to_6_15
                                 + x_train_0_from_6_14_to_6_15
                                 - x_train_0_from_6_15_to_6_16  = 0
 flow_train_0_node_6_16:         - x_train_0_from_6_16_to_tau
                                 + x_train_0_from_4_15_to_6_16
                                 + x_train_0_from_5_15_to_6_16
                                 + x_train_0_from_6_15_to_6_16
                                 - x_train_0_from_6_16_to_6_17  = 0
 flow_train_0_node_6_17:         - x_train_0_from_6_17_to_tau
                                 + x_train_0_from_4_16_to_6_17
                                 + x_train_0_from_5_16_to_6_17
                                 + x_train_0_from_6_16_to_6_17
                                 - x_train_0_from_6_17_to_6_18  = 0
 flow_train_0_node_6_18:         - x_train_0_from_6_18_to_tau
                                 + x_train_0_from_4_17_to_6_18
                                 + x_train_0_from_5_17_to_6_18
                                 + x_train_0_from_6_17_to_6_18
                                 - x_train_0_from_6_18_to_6_19  = 0
 flow_train_0_node_6_19:         - x_train_0_from_6_19_to_tau
                                 + x_train_0_from_4_18_to_6_19
                                 + x_train_0_from_5_18_to_6_19
                                 + x_train_0_from_6_18_to_6_19
                                 - x_train_0_from_6_19_to_6_20  = 0
 flow_train_0_node_6_20:         - x_train_0_from_6_20_to_tau
                                 + x_train_0_from_4_19_to_6_20
                                 + x_train_0_from_5_19_to_6_20
                                 + x_train_0_from_6_19_to_6_20
                                 - x_train_0_from_6_20_to_6_21  = 0
 flow_train_0_node_6_21:         - x_train_0_from_6_21_to_tau
                                 + x_train_0_from_4_20_to_6_21
                                 + x_train_0_from_5_20_to_6_21
                                 + x_train_0_from_6_20_to_6_21
                                 - x_train_0_from_6_21_to_6_22  = 0
 flow_train_0_node_6_22:         - x_train_0_from_6_22_to_tau
                                 + x_train_0_from_4_21_to_6_22
                                 + x_train_0_from_5_21_to_6_22
                                 + x_train_0_from_6_21_to_6_22
                                 - x_train_0_from_6_22_to_6_23  = 0
 flow_train_0_node_6_23:         - x_train_0_from_6_23_to_tau
                                 + x_train_0_from_4_22_to_6_23
                                 + x_train_0_from_5_22_to_6_23
                                 + x_train_0_from_6_22_to_6_23
                                 - x_train_0_from_6_23_to_6_24  = 0
 flow_train_0_node_6_24:         - x_train_0_from_6_24_to_tau
                                 + x_train_0_from_4_23_to_6_24
                                 + x_train_0_from_5_23_to_6_24
                                 + x_train_0_from_6_23_to_6_24
                                 - x_train_0_from_6_24_to_6_25  = 0
 flow_train_0_node_6_25:         - x_train_0_from_6_25_to_tau
                                 + x_train_0_from_4_24_to_6_25
                                 + x_train_0_from_5_24_to_6_25
                                 + x_train_0_from_6_24_to_6_25
                                 - x_train_0_from_6_25_to_6_26  = 0
 flow_train_0_node_6_26:         - x_train_0_from_6_26_to_tau
                                 + x_train_0_from_4_25_to_6_26
                                 + x_train_0_from_5_25_to_6_26
                                 + x_train_0_from_6_25_to_6_26
                                 - x_train_0_from_6_26_to_6_27  = 0
 flow_train_0_node_6_27:         - x_train_0_from_6_27_to_tau
                                 + x_train_0_from_4_26_to_6_27
                                 + x_train_0_from_5_26_to_6_27
                                 + x_train_0_from_6_26_to_6_27
                                 - x_train_0_from_6_27_to_6_28  = 0
 flow_train_0_node_6_28:         - x_train_0_from_6_28_to_tau
                                 + x_train_0_from_4_27_to_6_28
                                 + x_train_0_from_5_27_to_6_28
                                 + x_train_0_from_6_27_to_6_28
                                 - x_train_0_from_6_28_to_6_29  = 0
 flow_train_0_node_6_29:         - x_train_0_from_6_29_to_tau
                                 + x_train_0_from_4_28_to_6_29
                                 + x_train_0_from_5_28_to_6_29
                                 + x_train_0_from_6_28_to_6_29
                                 - x_train_0_from_6_29_to_6_30  = 0
 flow_train_0_node_6_30:         - x_train_0_from_6_30_to_tau
                                 + x_train_0_from_4_29_to_6_30
                                 + x_train_0_from_5_29_to_6_30
                                 + x_train_0_from_6_29_to_6_30  = 0
 flow_train_1_node_0_4:          - x_train_1_from_0_4_to_tau
                                 + x_train_1_from_2_3_to_0_4
                                 - x_train_1_from_0_4_to_0_5  = 0
 flow_train_1_node_0_5:          - x_train_1_from_0_5_to_tau
                                 + x_train_1_from_0_4_to_0_5
                                 + x_train_1_from_2_4_to_0_5
                                 - x_train_1_from_0_5_to_0_6  = 0
 flow_train_1_node_0_6:          - x_train_1_from_0_6_to_tau
                                 + x_train_1_from_0_5_to_0_6
                                 + x_train_1_from_2_5_to_0_6
                                 - x_train_1_from_0_6_to_0_7  = 0
 flow_train_1_node_0_7:          - x_train_1_from_0_7_to_tau
                                 + x_train_1_from_0_6_to_0_7
                                 + x_train_1_from_1_6_to_0_7
                                 + x_train_1_from_2_6_to_0_7
                                 - x_train_1_from_0_7_to_0_8  = 0
 flow_train_1_node_0_8:          - x_train_1_from_0_8_to_tau
                                 + x_train_1_from_0_7_to_0_8
                                 + x_train_1_from_1_7_to_0_8
                                 + x_train_1_from_2_7_to_0_8
                                 - x_train_1_from_0_8_to_0_9  = 0
 flow_train_1_node_0_9:          - x_train_1_from_0_9_to_tau
                                 + x_train_1_from_0_8_to_0_9
                                 + x_train_1_from_1_8_to_0_9
                                 + x_train_1_from_2_8_to_0_9
                                 - x_train_1_from_0_9_to_0_10  = 0
 flow_train_1_node_0_10:         - x_train_1_from_0_10_to_tau
                                 + x_train_1_from_0_9_to_0_10
                                 + x_train_1_from_1_9_to_0_10
                                 + x_train_1_from_2_9_to_0_10
                                 - x_train_1_from_0_10_to_0_11  = 0
 flow_train_1_node_0_11:         - x_train_1_from_0_11_to_tau
                                 + x_train_1_from_0_10_to_0_11
                                 + x_train_1_from_1_10_to_0_11
                                 + x_train_1_from_2_10_to_0_11
                                 - x_train_1_from_0_11_to_0_12  = 0
 flow_train_1_node_0_12:         - x_train_1_from_0_12_to_tau
                                 + x_train_1_from_0_11_to_0_12
                                 + x_train_1_from_1_11_to_0_12
                                 + x_train_1_from_2_11_to_0_12
                                 - x_train_1_from_0_12_to_0_13  = 0
 flow_train_1_node_0_13:         - x_train_1_from_0_13_to_tau
                                 + x_train_1_from_0_12_to_0_13
                                 + x_train_1_from_1_12_to_0_13
                                 + x_train_1_from_2_12_to_0_13
                                 - x_train_1_from_0_13_to_0_14  = 0
 flow_train_1_node_0_14:         - x_train_1_from_0_14_to_tau
                                 + x_train_1_from_0_13_to_0_14
                                 + x_train_1_from_1_13_to_0_14
                                 + x_train_1_from_2_13_to_0_14
                                 - x_train_1_from_0_14_to_0_15  = 0
 flow_train_1_node_0_15:         - x_train_1_from_0_15_to_tau
                                 + x_train_1_from_0_14_to_0_15
                                 + x_train_1_from_1_14_to_0_15
                                 + x_train_1_from_2_14_to_0_15
                                 - x_train_1_from_0_15_to_0_16  = 0
 flow_train_1_node_0_16:         - x_train_1_from_0_16_to_tau
                                 + x_train_1_from_0_15_to_0_16
                                 + x_train_1_from_1_15_to_0_16
                                 + x_train_1_from_2_15_to_0_16
                                 - x_train_1_from_0_16_to_0_17  = 0
 flow_train_1_node_0_17:         - x_train_1_from_0_17_to_tau
                                 + x_train_1_from_0_16_to_0_17
                                 + x_train_1_from_1_16_to_0_17
                                 + x_train_1_from_2_16_to_0_17
                                 - x_train_1_from_0_17_to_0_18  = 0
 flow_train_1_node_0_18:         - x_train_1_from_0_18_to_tau
                                 + x_train_1_from_0_17_to_0_18
                                 + x_train_1_from_1_17_to_0_18
                                 + x_train_1_from_2_17_to_0_18
                                 - x_train_1_from_0_18_to_0_19  = 0
 flow_train_1_node_0_19:         - x_train_1_from_0_19_to_tau
                                 + x_train_1_from_0_18_to_0_19
                                 + x_train_1_from_1_18_to_0_19
                                 + x_train_1_from_2_18_to_0_19
                                 - x_train_1_from_0_19_to_0_20  = 0
 flow_train_1_node_0_20:         - x_train_1_from_0_20_to_tau
                                 + x_train_1_from_0_19_to_0_20
                                 + x_train_1_from_1_19_to_0_20
                                 + x_train_1_from_2_19_to_0_20
                                 - x_train_1_from_0_20_to_0_21  = 0
 flow_train_1_node_0_21:         - x_train_1_from_0_21_to_tau
                                 + x_train_1_from_0_20_to_0_21
                                 + x_train_1_from_1_20_to_0_21
                                 + x_train_1_from_2_20_to_0_21
                                 - x_train_1_from_0_21_to_0_22  = 0
 flow_train_1_node_0_22:         - x_train_1_from_0_22_to_tau
                                 + x_train_1_from_0_21_to_0_22
                                 + x_train_1_from_1_21_to_0_22
                                 + x_train_1_from_2_21_to_0_22
                                 - x_train_1_from_0_22_to_0_23  = 0
 flow_train_1_node_0_23:         - x_train_1_from_0_23_to_tau
                                 + x_train_1_from_0_22_to_0_23
                                 + x_train_1_from_1_22_to_0_23
                                 + x_train_1_from_2_22_to_0_23
                                 - x_train_1_from_0_23_to_0_24  = 0
 flow_train_1_node_0_24:         - x_train_1_from_0_24_to_tau
                                 + x_train_1_from_0_23_to_0_24
                                 + x_train_1_from_1_23_to_0_24
                                 + x_train_1_from_2_23_to_0_24
                                 - x_train_1_from_0_24_to_0_25  = 0
 flow_train_1_node_0_25:         - x_train_1_from_0_25_to_tau
                                 + x_train_1_from_0_24_to_0_25
                                 + x_train_1_from_1_24_to_0_25
                                 + x_train_1_from_2_24_to_0_25
                                 - x_train_1_from_0_25_to_0_26  = 0
 flow_train_1_node_0_26:         - x_train_1_from_0_26_to_tau
                                 + x_train_1_from_0_25_to_0_26
                                 + x_train_1_from_1_25_to_0_26
                                 + x_train_1_from_2_25_to_0_26
                                 - x_train_1_from_0_26_to_0_27  = 0
 flow_train_1_node_0_27:         - x_train_1_from_0_27_to_tau
                                 + x_train_1_from_0_26_to_0_27
                                 + x_train_1_from_1_26_to_0_27
                                 + x_train_1_from_2_26_to_0_27
                                 - x_train_1_from_0_27_to_0_28  = 0
 flow_train_1_node_0_28:         - x_train_1_from_0_28_to_tau
                                 + x_train_1_from_0_27_to_0_28
                                 + x_train_1_from_1_27_to_0_28
                                 + x_train_1_from_2_27_to_0_28
                                 - x_train_1_from_0_28_to_0_29  = 0
 flow_train_1_node_0_29:         - x_train_1_from_0_29_to_tau
                                 + x_train_1_from_0_28_to_0_29
                                 + x_train_1_from_1_28_to_0_29
                                 + x_train_1_from_2_28_to_0_29
                                 - x_train_1_from_0_29_to_0_30  = 0
 flow_train_1_node_0_30:         - x_train_1_from_0_30_to_tau
                                 + x_train_1_from_0_29_to_0_30
                                 + x_train_1_from_1_29_to_0_30
                                 + x_train_1_from_2_29_to_0_30  = 0
 flow_train_1_node_1_6:          - x_train_1_from_1_6_to_0_7
                                 + x_train_1_from_3_5_to_1_6
                                 - x_train_1_from_1_6_to_1_7  = 0
 flow_train_1_node_1_7:          - x_train_1_from_1_7_to_0_8
                                 + x_train_1_from_1_6_to_1_7
                                 + x_train_1_from_3_6_to_1_7
                                 - x_train_1_from_1_7_to_1_8  = 0
 flow_train_1_node_1_8:          - x_train_1_from_1_8_to_0_9
                                 + x_train_1_from_1_7_to_1_8
                                 + x_train_1_from_3_7_to_1_8
                                 - x_train_1_from_1_8_to_1_9  = 0
 flow_train_1_node_1_9:          - x_train_1_from_1_9_to_0_10
                                 + x_train_1_from_1_8_to_1_9
                                 + x_train_1_from_3_8_to_1_9
                                 - x_train_1_from_1_9_to_1_10  = 0
 flow_train_1_node_1_10:         - x_train_1_from_1_10_to_0_11
                                 + x_train_1_from_1_9_to_1_10
                                 + x_train_1_from_3_9_to_1_10
                                 - x_train_1_from_1_10_to_1_11  = 0
 flow_train_1_node_1_11:         - x_train_1_from_1_11_to_0_12
                                 + x_train_1_from_1_10_to_1_11
                                 + x_train_1_from_3_10_to_1_11
                                 - x_train_1_from_1_11_to_1_12  = 0
 flow_train_1_node_1_12:         - x_train_1_from_1_12_to_0_13
                                 + x_train_1_from_1_11_to_1_12
                                 + x_train_1_from_3_11_to_1_12
                                 - x_train_1_from_1_12_to_1_13  = 0
 flow_train_1_node_1_13:         - x_train_1_from_1_13_to_0_14
                                 + x_train_1_from_1_12_to_1_13
                                 + x_train_1_from_3_12_to_1_13
                                 - x_train_1_from_1_13_to_1_14  = 0
 flow_train_1_node_1_14:         - x_train_1_from_1_14_to_0_15
                                 + x_train_1_from_1_13_to_1_14
                                 + x_train_1_from_3_13_to_1_14
                                 - x_train_1_from_1_14_to_1_15  = 0
 flow_train_1_node_1_15:         - x_train_1_from_1_15_to_0_16
                                 + x_train_1_from_1_14_to_1_15
                                 + x_train_1_from_3_14_to_1_15
                                 - x_train_1_from_1_15_to_1_16  = 0
 flow_train_1_node_1_16:         - x_train_1_from_1_16_to_0_17
                                 + x_train_1_from_1_15_to_1_16
                                 + x_train_1_from_3_15_to_1_16
                                 - x_train_1_from_1_16_to_1_17  = 0
 flow_train_1_node_1_17:         - x_train_1_from_1_17_to_0_18
                                 + x_train_1_from_1_16_to_1_17
                                 + x_train_1_from_3_16_to_1_17
                                 - x_train_1_from_1_17_to_1_18  = 0
 flow_train_1_node_1_18:         - x_train_1_from_1_18_to_0_19
                                 + x_train_1_from_1_17_to_1_18
                                 + x_train_1_from_3_17_to_1_18
                                 - x_train_1_from_1_18_to_1_19  = 0
 flow_train_1_node_1_19:         - x_train_1_from_1_19_to_0_20
                                 + x_train_1_from_1_18_to_1_19
                                 + x_train_1_from_3_18_to_1_19
                                 - x_train_1_from_1_19_to_1_20  = 0
 flow_train_1_node_1_20:         - x_train_1_from_1_20_to_0_21
                                 + x_train_1_from_1_19_to_1_20
                                 + x_train_1_from_3_19_to_1_20
                                 - x_train_1_from_1_20_to_1_21  = 0
 flow_train_1_node_1_21:         - x_train_1_from_1_21_to_0_22
                                 + x_train_1_from_1_20_to_1_21
                                 + x_train_1_from_3_20_to_1_21
                                 - x_train_1_from_1_21_to_1_22  = 0
 flow_train_1_node_1_22:         - x_train_1_from_1_22_to_0_23
                                 + x_train_1_from_1_21_to_1_22
                                 + x_train_1_from_3_21_to_1_22
                                 - x_train_1_from_1_22_to_1_23  = 0
 flow_train_1_node_1_23:         - x_train_1_from_1_23_to_0_24
                                 + x_train_1_from_1_22_to_1_23
                                 + x_train_1_from_3_22_to_1_23
                                 - x_train_1_from_1_23_to_1_24  = 0
 flow_train_1_node_1_24:         - x_train_1_from_1_24_to_0_25
                                 + x_train_1_from_1_23_to_1_24
                                 + x_train_1_from_3_23_to_1_24
                                 - x_train_1_from_1_24_to_1_25  = 0
 flow_train_1_node_1_25:         - x_train_1_from_1_25_to_0_26
                                 + x_train_1_from_1_24_to_1_25
                                 + x_train_1_from_3_24_to_1_25
                                 - x_train_1_from_1_25_to_1_26  = 0
 flow_train_1_node_1_26:         - x_train_1_from_1_26_to_0_27
                                 + x_train_1_from_1_25_to_1_26
                                 + x_train_1_from_3_25_to_1_26
                                 - x_train_1_from_1_26_to_1_27  = 0
 flow_train_1_node_1_27:         - x_train_1_from_1_27_to_0_28
                                 + x_train_1_from_1_26_to_1_27
                                 + x_train_1_from_3_26_to_1_27
                                 - x_train_1_from_1_27_to_1_28  = 0
 flow_train_1_node_1_28:         - x_train_1_from_1_28_to_0_29
                                 + x_train_1_from_1_27_to_1_28
                                 + x_train_1_from_3_27_to_1_28
                                 - x_train_1_from_1_28_to_1_29  = 0
 flow_train_1_node_1_29:         - x_train_1_from_1_29_to_0_30
                                 + x_train_1_from_1_28_to_1_29
                                 + x_train_1_from_3_28_to_1_29  = 0
 flow_train_1_node_2_3:          - x_train_1_from_2_3_to_0_4
                                 + x_train_1_from_4_2_to_2_3
                                 - x_train_1_from_2_3_to_2_4  = 0
 flow_train_1_node_2_4:          - x_train_1_from_2_4_to_0_5
                                 + x_train_1_from_2_3_to_2_4
                                 + x_train_1_from_4_3_to_2_4
                                 - x_train_1_from_2_4_to_2_5  = 0
 flow_train_1_node_2_5:          - x_train_1_from_2_5_to_0_6
                                 + x_train_1_from_2_4_to_2_5
                                 + x_train_1_from_4_4_to_2_5
                                 - x_train_1_from_2_5_to_2_6  = 0
 flow_train_1_node_2_6:          - x_train_1_from_2_6_to_0_7
                                 + x_train_1_from_2_5_to_2_6
                                 + x_train_1_from_4_5_to_2_6
                                 - x_train_1_from_2_6_to_2_7  = 0
 flow_train_1_node_2_7:          - x_train_1_from_2_7_to_0_8
                                 + x_train_1_from_2_6_to_2_7
                                 + x_train_1_from_4_6_to_2_7
                                 - x_train_1_from_2_7_to_2_8  = 0
 flow_train_1_node_2_8:          - x_train_1_from_2_8_to_0_9
                                 + x_train_1_from_2_7_to_2_8
                                 + x_train_1_from_4_7_to_2_8
                                 - x_train_1_from_2_8_to_2_9  = 0
 flow_train_1_node_2_9:          - x_train_1_from_2_9_to_0_10
                                 + x_train_1_from_2_8_to_2_9
                                 + x_train_1_from_4_8_to_2_9
                                 - x_train_1_from_2_9_to_2_10  = 0
 flow_train_1_node_2_10:         - x_train_1_from_2_10_to_0_11
                                 + x_train_1_from_2_9_to_2_10
                                 + x_train_1_from_4_9_to_2_10
                                 - x_train_1_from_2_10_to_2_11  = 0
 flow_train_1_node_2_11:         - x_train_1_from_2_11_to_0_12
                                 + x_train_1_from_2_10_to_2_11
                                 + x_train_1_from_4_10_to_2_11
                                 - x_train_1_from_2_11_to_2_12  = 0
 flow_train_1_node_2_12:         - x_train_1_from_2_12_to_0_13
                                 + x_train_1_from_2_11_to_2_12
                                 + x_train_1_from_4_11_to_2_12
                                 - x_train_1_from_2_12_to_2_13  = 0
 flow_train_1_node_2_13:         - x_train_1_from_2_13_to_0_14
                                 + x_train_1_from_2_12_to_2_13
                                 + x_train_1_from_4_12_to_2_13
                                 - x_train_1_from_2_13_to_2_14  = 0
 flow_train_1_node_2_14:         - x_train_1_from_2_14_to_0_15
                                 + x_train_1_from_2_13_to_2_14
                                 + x_train_1_from_4_13_to_2_14
                                 - x_train_1_from_2_14_to_2_15  = 0
 flow_train_1_node_2_15:         - x_train_1_from_2_15_to_0_16
                                 + x_train_1_from_2_14_to_2_15
                                 + x_train_1_from_4_14_to_2_15
                                 - x_train_1_from_2_15_to_2_16  = 0
 flow_train_1_node_2_16:         - x_train_1_from_2_16_to_0_17
                                 + x_train_1_from_2_15_to_2_16
                                 + x_train_1_from_4_15_to_2_16
                                 - x_train_1_from_2_16_to_2_17  = 0
 flow_train_1_node_2_17:         - x_train_1_from_2_17_to_0_18
                                 + x_train_1_from_2_16_to_2_17
                                 + x_train_1_from_4_16_to_2_17
                                 - x_train_1_from_2_17_to_2_18  = 0
 flow_train_1_node_2_18:         - x_train_1_from_2_18_to_0_19
                                 + x_train_1_from_2_17_to_2_18
                                 + x_train_1_from_4_17_to_2_18
                                 - x_train_1_from_2_18_to_2_19  = 0
 flow_train_1_node_2_19:         - x_train_1_from_2_19_to_0_20
                                 + x_train_1_from_2_18_to_2_19
                                 + x_train_1_from_4_18_to_2_19
                                 - x_train_1_from_2_19_to_2_20  = 0
 flow_train_1_node_2_20:         - x_train_1_from_2_20_to_0_21
                                 + x_train_1_from_2_19_to_2_20
                                 + x_train_1_from_4_19_to_2_20
                                 - x_train_1_from_2_20_to_2_21  = 0
 flow_train_1_node_2_21:         - x_train_1_from_2_21_to_0_22
                                 + x_train_1_from_2_20_to_2_21
                                 + x_train_1_from_4_20_to_2_21
                                 - x_train_1_from_2_21_to_2_22  = 0
 flow_train_1_node_2_22:         - x_train_1_from_2_22_to_0_23
                                 + x_train_1_from_2_21_to_2_22
                                 + x_train_1_from_4_21_to_2_22
                                 - x_train_1_from_2_22_to_2_23  = 0
 flow_train_1_node_2_23:         - x_train_1_from_2_23_to_0_24
                                 + x_train_1_from_2_22_to_2_23
                                 + x_train_1_from_4_22_to_2_23
                                 - x_train_1_from_2_23_to_2_24  = 0
 flow_train_1_node_2_24:         - x_train_1_from_2_24_to_0_25
                                 + x_train_1_from_2_23_to_2_24
                                 + x_train_1_from_4_23_to_2_24
                                 - x_train_1_from_2_24_to_2_25  = 0
 flow_train_1_node_2_25:         - x_train_1_from_2_25_to_0_26
                                 + x_train_1_from_2_24_to_2_25
                                 + x_train_1_from_4_24_to_2_25
                                 - x_train_1_from_2_25_to_2_26  = 0
 flow_train_1_node_2_26:         - x_train_1_from_2_26_to_0_27
                                 + x_train_1_from_2_25_to_2_26
                                 + x_train_1_from_4_25_to_2_26
                                 - x_train_1_from_2_26_to_2_27  = 0
 flow_train_1_node_2_27:         - x_train_1_from_2_27_to_0_28
                                 + x_train_1_from_2_26_to_2_27
                                 + x_train_1_from_4_26_to_2_27
                                 - x_train_1_from_2_27_to_2_28  = 0
 flow_train_1_node_2_28:         - x_train_1_from_2_28_to_0_29
                                 + x_train_1_from_2_27_to_2_28
                                 + x_train_1_from_4_27_to_2_28
                                 - x_train_1_from_2_28_to_2_29  = 0
 flow_train_1_node_2_29:         - x_train_1_from_2_29_to_0_30
                                 + x_train_1_from_2_28_to_2_29
                                 + x_train_1_from_4_28_to_2_29  = 0
 flow_train_1_node_3_5:          - x_train_1_from_3_5_to_1_6
                                 + x_train_1_from_5_4_to_3_5
                                 - x_train_1_from_3_5_to_3_6  = 0
 flow_train_1_node_3_6:          - x_train_1_from_3_6_to_1_7
                                 + x_train_1_from_3_5_to_3_6
                                 + x_train_1_from_5_5_to_3_6
                                 - x_train_1_from_3_6_to_3_7  = 0
 flow_train_1_node_3_7:          - x_train_1_from_3_7_to_1_8
                                 + x_train_1_from_3_6_to_3_7
                                 + x_train_1_from_5_6_to_3_7
                                 - x_train_1_from_3_7_to_3_8  = 0
 flow_train_1_node_3_8:          - x_train_1_from_3_8_to_1_9
                                 + x_train_1_from_3_7_to_3_8
                                 + x_train_1_from_5_7_to_3_8
                                 - x_train_1_from_3_8_to_3_9  = 0
 flow_train_1_node_3_9:          - x_train_1_from_3_9_to_1_10
                                 + x_train_1_from_3_8_to_3_9
                                 + x_train_1_from_5_8_to_3_9
                                 - x_train_1_from_3_9_to_3_10  = 0
 flow_train_1_node_3_10:         - x_train_1_from_3_10_to_1_11
                                 + x_train_1_from_3_9_to_3_10
                                 + x_train_1_from_5_9_to_3_10
                                 - x_train_1_from_3_10_to_3_11  = 0
 flow_train_1_node_3_11:         - x_train_1_from_3_11_to_1_12
                                 + x_train_1_from_3_10_to_3_11
                                 + x_train_1_from_5_10_to_3_11
                                 - x_train_1_from_3_11_to_3_12  = 0
 flow_train_1_node_3_12:         - x_train_1_from_3_12_to_1_13
                                 + x_train_1_from_3_11_to_3_12
                                 + x_train_1_from_5_11_to_3_12
                                 - x_train_1_from_3_12_to_3_13  = 0
 flow_train_1_node_3_13:         - x_train_1_from_3_13_to_1_14
                                 + x_train_1_from_3_12_to_3_13
                                 + x_train_1_from_5_12_to_3_13
                                 - x_train_1_from_3_13_to_3_14  = 0
 flow_train_1_node_3_14:         - x_train_1_from_3_14_to_1_15
                                 + x_train_1_from_3_13_to_3_14
                                 + x_train_1_from_5_13_to_3_14
                                 - x_train_1_from_3_14_to_3_15  = 0
 flow_train_1_node_3_15:         - x_train_1_from_3_15_to_1_16
                                 + x_train_1_from_3_14_to_3_15
                                 + x_train_1_from_5_14_to_3_15
                                 - x_train_1_from_3_15_to_3_16  = 0
 flow_train_1_node_3_16:         - x_train_1_from_3_16_to_1_17
                                 + x_train_1_from_3_15_to_3_16
                                 + x_train_1_from_5_15_to_3_16
                                 - x_train_1_from_3_16_to_3_17  = 0
 flow_train_1_node_3_17:         - x_train_1_from_3_17_to_1_18
                                 + x_train_1_from_3_16_to_3_17
                                 + x_train_1_from_5_16_to_3_17
                                 - x_train_1_from_3_17_to_3_18  = 0
 flow_train_1_node_3_18:         - x_train_1_from_3_18_to_1_19
                                 + x_train_1_from_3_17_to_3_18
                                 + x_train_1_from_5_17_to_3_18
                                 - x_train_1_from_3_18_to_3_19  = 0
 flow_train_1_node_3_19:         - x_train_1_from_3_19_to_1_20
                                 + x_train_1_from_3_18_to_3_19
                                 + x_train_1_from_5_18_to_3_19
                                 - x_train_1_from_3_19_to_3_20  = 0
 flow_train_1_node_3_20:         - x_train_1_from_3_20_to_1_21
                                 + x_train_1_from_3_19_to_3_20
                                 + x_train_1_from_5_19_to_3_20
                                 - x_train_1_from_3_20_to_3_21  = 0
 flow_train_1_node_3_21:         - x_train_1_from_3_21_to_1_22
                                 + x_train_1_from_3_20_to_3_21
                                 + x_train_1_from_5_20_to_3_21
                                 - x_train_1_from_3_21_to_3_22  = 0
 flow_train_1_node_3_22:         - x_train_1_from_3_22_to_1_23
                                 + x_train_1_from_3_21_to_3_22
                                 + x_train_1_from_5_21_to_3_22
                                 - x_train_1_from_3_22_to_3_23  = 0
 flow_train_1_node_3_23:         - x_train_1_from_3_23_to_1_24
                                 + x_train_1_from_3_22_to_3_23
                                 + x_train_1_from_5_22_to_3_23
                                 - x_train_1_from_3_23_to_3_24  = 0
 flow_train_1_node_3_24:         - x_train_1_from_3_24_to_1_25
                                 + x_train_1_from_3_23_to_3_24
                                 + x_train_1_from_5_23_to_3_24
                                 - x_train_1_from_3_24_to_3_25  = 0
 flow_train_1_node_3_25:         - x_train_1_from_3_25_to_1_26
                                 + x_train_1_from_3_24_to_3_25
                                 + x_train_1_from_5_24_to_3_25
                                 - x_train_1_from_3_25_to_3_26  = 0
 flow_train_1_node_3_26:         - x_train_1_from_3_26_to_1_27
                                 + x_train_1_from_3_25_to_3_26
                                 + x_train_1_from_5_25_to_3_26
                                 - x_train_1_from_3_26_to_3_27  = 0
 flow_train_1_node_3_27:         - x_train_1_from_3_27_to_1_28
                                 + x_train_1_from_3_26_to_3_27
                                 + x_train_1_from_5_26_to_3_27
                                 - x_train_1_from_3_27_to_3_28  = 0
 flow_train_1_node_3_28:         - x_train_1_from_3_28_to_1_29
                                 + x_train_1_from_3_27_to_3_28
                                 + x_train_1_from_5_27_to_3_28  = 0
 flow_train_1_node_4_2:          - x_train_1_from_4_2_to_2_3
                                 + x_train_1_from_6_1_to_4_2
                                 - x_train_1_from_4_2_to_4_3  = 0
 flow_train_1_node_4_3:          - x_train_1_from_4_3_to_2_4
                                 + x_train_1_from_4_2_to_4_3
                                 + x_train_1_from_6_2_to_4_3
                                 - x_train_1_from_4_3_to_4_4  = 0
 flow_train_1_node_4_4:          - x_train_1_from_4_4_to_2_5
                                 + x_train_1_from_4_3_to_4_4
                                 + x_train_1_from_6_3_to_4_4
                                 - x_train_1_from_4_4_to_4_5  = 0
 flow_train_1_node_4_5:          - x_train_1_from_4_5_to_2_6
                                 + x_train_1_from_4_4_to_4_5
                                 + x_train_1_from_6_4_to_4_5
                                 - x_train_1_from_4_5_to_4_6  = 0
 flow_train_1_node_4_6:          - x_train_1_from_4_6_to_2_7
                                 + x_train_1_from_4_5_to_4_6
                                 + x_train_1_from_6_5_to_4_6
                                 - x_train_1_from_4_6_to_4_7  = 0
 flow_train_1_node_4_7:          - x_train_1_from_4_7_to_2_8
                                 + x_train_1_from_4_6_to_4_7
                                 + x_train_1_from_6_6_to_4_7
                                 - x_train_1_from_4_7_to_4_8  = 0
 flow_train_1_node_4_8:          - x_train_1_from_4_8_to_2_9
                                 + x_train_1_from_4_7_to_4_8
                                 + x_train_1_from_6_7_to_4_8
                                 - x_train_1_from_4_8_to_4_9  = 0
 flow_train_1_node_4_9:          - x_train_1_from_4_9_to_2_10
                                 + x_train_1_from_4_8_to_4_9
                                 + x_train_1_from_6_8_to_4_9
                                 - x_train_1_from_4_9_to_4_10  = 0
 flow_train_1_node_4_10:         - x_train_1_from_4_10_to_2_11
                                 + x_train_1_from_4_9_to_4_10
                                 + x_train_1_from_6_9_to_4_10
                                 - x_train_1_from_4_10_to_4_11  = 0
 flow_train_1_node_4_11:         - x_train_1_from_4_11_to_2_12
                                 + x_train_1_from_4_10_to_4_11
                                 + x_train_1_from_6_10_to_4_11
                                 - x_train_1_from_4_11_to_4_12  = 0
 flow_train_1_node_4_12:         - x_train_1_from_4_12_to_2_13
                                 + x_train_1_from_4_11_to_4_12
                                 + x_train_1_from_6_11_to_4_12
                                 - x_train_1_from_4_12_to_4_13  = 0
 flow_train_1_node_4_13:         - x_train_1_from_4_13_to_2_14
                                 + x_train_1_from_4_12_to_4_13
                                 + x_train_1_from_6_12_to_4_13
                                 - x_train_1_from_4_13_to_4_14  = 0
 flow_train_1_node_4_14:         - x_train_1_from_4_14_to_2_15
                                 + x_train_1_from_4_13_to_4_14
                                 + x_train_1_from_6_13_to_4_14
                                 - x_train_1_from_4_14_to_4_15  = 0
 flow_train_1_node_4_15:         - x_train_1_from_4_15_to_2_16
                                 + x_train_1_from_4_14_to_4_15
                                 + x_train_1_from_6_14_to_4_15
                                 - x_train_1_from_4_15_to_4_16  = 0
 flow_train_1_node_4_16:         - x_train_1_from_4_16_to_2_17
                                 + x_train_1_from_4_15_to_4_16
                                 + x_train_1_from_6_15_to_4_16
                                 - x_train_1_from_4_16_to_4_17  = 0
 flow_train_1_node_4_17:         - x_train_1_from_4_17_to_2_18
                                 + x_train_1_from_4_16_to_4_17
                                 + x_train_1_from_6_16_to_4_17
                                 - x_train_1_from_4_17_to_4_18  = 0
 flow_train_1_node_4_18:         - x_train_1_from_4_18_to_2_19
                                 + x_train_1_from_4_17_to_4_18
                                 + x_train_1_from_6_17_to_4_18
                                 - x_train_1_from_4_18_to_4_19  = 0
 flow_train_1_node_4_19:         - x_train_1_from_4_19_to_2_20
                                 + x_train_1_from_4_18_to_4_19
                                 + x_train_1_from_6_18_to_4_19
                                 - x_train_1_from_4_19_to_4_20  = 0
 flow_train_1_node_4_20:         - x_train_1_from_4_20_to_2_21
                                 + x_train_1_from_4_19_to_4_20
                                 + x_train_1_from_6_19_to_4_20
                                 - x_train_1_from_4_20_to_4_21  = 0
 flow_train_1_node_4_21:         - x_train_1_from_4_21_to_2_22
                                 + x_train_1_from_4_20_to_4_21
                                 + x_train_1_from_6_20_to_4_21
                                 - x_train_1_from_4_21_to_4_22  = 0
 flow_train_1_node_4_22:         - x_train_1_from_4_22_to_2_23
                                 + x_train_1_from_4_21_to_4_22
                                 + x_train_1_from_6_21_to_4_22
                                 - x_train_1_from_4_22_to_4_23  = 0
 flow_train_1_node_4_23:         - x_train_1_from_4_23_to_2_24
                                 + x_train_1_from_4_22_to_4_23
                                 + x_train_1_from_6_22_to_4_23
                                 - x_train_1_from_4_23_to_4_24  = 0
 flow_train_1_node_4_24:         - x_train_1_from_4_24_to_2_25
                                 + x_train_1_from_4_23_to_4_24
                                 + x_train_1_from_6_23_to_4_24
                                 - x_train_1_from_4_24_to_4_25  = 0
 flow_train_1_node_4_25:         - x_train_1_from_4_25_to_2_26
                                 + x_train_1_from_4_24_to_4_25
                                 + x_train_1_from_6_24_to_4_25
                                 - x_train_1_from_4_25_to_4_26  = 0
 flow_train_1_node_4_26:         - x_train_1_from_4_26_to_2_27
                                 + x_train_1_from_4_25_to_4_26
                                 + x_train_1_from_6_25_to_4_26
                                 - x_train_1_from_4_26_to_4_27  = 0
 flow_train_1_node_4_27:         - x_train_1_from_4_27_to_2_28
                                 + x_train_1_from_4_26_to_4_27
                                 + x_train_1_from_6_26_to_4_27
                                 - x_train_1_from_4_27_to_4_28  = 0
 flow_train_1_node_4_28:         - x_train_1_from_4_28_to_2_29
                                 + x_train_1_from_4_27_to_4_28
                                 + x_train_1_from_6_27_to_4_28  = 0
 flow_train_1_node_5_2:          x_train_1_from_6_1_to_5_2
                                 - x_train_1_from_5_2_to_5_3  = 0
 flow_train_1_node_5_3:          x_train_1_from_5_2_to_5_3
                                 + x_train_1_from_6_2_to_5_3
                                 - x_train_1_from_5_3_to_5_4  = 0
 flow_train_1_node_5_4:          - x_train_1_from_5_4_to_3_5
                                 + x_train_1_from_5_3_to_5_4
                                 + x_train_1_from_6_3_to_5_4
                                 - x_train_1_from_5_4_to_5_5  = 0
 flow_train_1_node_5_5:          - x_train_1_from_5_5_to_3_6
                                 + x_train_1_from_5_4_to_5_5
                                 + x_train_1_from_6_4_to_5_5
                                 - x_train_1_from_5_5_to_5_6  = 0
 flow_train_1_node_5_6:          - x_train_1_from_5_6_to_3_7
                                 + x_train_1_from_5_5_to_5_6
                                 + x_train_1_from_6_5_to_5_6
                                 - x_train_1_from_5_6_to_5_7  = 0
 flow_train_1_node_5_7:          - x_train_1_from_5_7_to_3_8
                                 + x_train_1_from_5_6_to_5_7
                                 + x_train_1_from_6_6_to_5_7
                                 - x_train_1_from_5_7_to_5_8  = 0
 flow_train_1_node_5_8:          - x_train_1_from_5_8_to_3_9
                                 + x_train_1_from_5_7_to_5_8
                                 + x_train_1_from_6_7_to_5_8
                                 - x_train_1_from_5_8_to_5_9  = 0
 flow_train_1_node_5_9:          - x_train_1_from_5_9_to_3_10
                                 + x_train_1_from_5_8_to_5_9
                                 + x_train_1_from_6_8_to_5_9
                                 - x_train_1_from_5_9_to_5_10  = 0
 flow_train_1_node_5_10:         - x_train_1_from_5_10_to_3_11
                                 + x_train_1_from_5_9_to_5_10
                                 + x_train_1_from_6_9_to_5_10
                                 - x_train_1_from_5_10_to_5_11  = 0
 flow_train_1_node_5_11:         - x_train_1_from_5_11_to_3_12
                                 + x_train_1_from_5_10_to_5_11
                                 + x_train_1_from_6_10_to_5_11
                                 - x_train_1_from_5_11_to_5_12  = 0
 flow_train_1_node_5_12:         - x_train_1_from_5_12_to_3_13
                                 + x_train_1_from_5_11_to_5_12
                                 + x_train_1_from_6_11_to_5_12
                                 - x_train_1_from_5_12_to_5_13  = 0
 flow_train_1_node_5_13:         - x_train_1_from_5_13_to_3_14
                                 + x_train_1_from_5_12_to_5_13
                                 + x_train_1_from_6_12_to_5_13
                                 - x_train_1_from_5_13_to_5_14  = 0
 flow_train_1_node_5_14:         - x_train_1_from_5_14_to_3_15
                                 + x_train_1_from_5_13_to_5_14
                                 + x_train_1_from_6_13_to_5_14
                                 - x_train_1_from_5_14_to_5_15  = 0
 flow_train_1_node_5_15:         - x_train_1_from_5_15_to_3_16
                                 + x_train_1_from_5_14_to_5_15
                                 + x_train_1_from_6_14_to_5_15
                                 - x_train_1_from_5_15_to_5_16  = 0
 flow_train_1_node_5_16:         - x_train_1_from_5_16_to_3_17
                                 + x_train_1_from_5_15_to_5_16
                                 + x_train_1_from_6_15_to_5_16
                                 - x_train_1_from_5_16_to_5_17  = 0
 flow_train_1_node_5_17:         - x_train_1_from_5_17_to_3_18
                                 + x_train_1_from_5_16_to_5_17
                                 + x_train_1_from_6_16_to_5_17
                                 - x_train_1_from_5_17_to_5_18  = 0
 flow_train_1_node_5_18:         - x_train_1_from_5_18_to_3_19
                                 + x_train_1_from_5_17_to_5_18
                                 + x_train_1_from_6_17_to_5_18
                                 - x_train_1_from_5_18_to_5_19  = 0
 flow_train_1_node_5_19:         - x_train_1_from_5_19_to_3_20
                                 + x_train_1_from_5_18_to_5_19
                                 + x_train_1_from_6_18_to_5_19
                                 - x_train_1_from_5_19_to_5_20  = 0
 flow_train_1_node_5_20:         - x_train_1_from_5_20_to_3_21
                                 + x_train_1_from_5_19_to_5_20
                                 + x_train_1_from_6_19_to_5_20
                                 - x_train_1_from_5_20_to_5_21  = 0
 flow_train_1_node_5_21:         - x_train_1_from_5_21_to_3_22
                                 + x_train_1_from_5_20_to_5_21
                                 + x_train_1_from_6_20_to_5_21
                                 - x_train_1_from_5_21_to_5_22  = 0
 flow_train_1_node_5_22:         - x_train_1_from_5_22_to_3_23
                                 + x_train_1_from_5_21_to_5_22
                                 + x_train_1_from_6_21_to_5_22
                                 - x_train_1_from_5_22_to_5_23  = 0
 flow_train_1_node_5_23:         - x_train_1_from_5_23_to_3_24
                                 + x_train_1_from_5_22_to_5_23
                                 + x_train_1_from_6_22_to_5_23
                                 - x_train_1_from_5_23_to_5_24  = 0
 flow_train_1_node_5_24:         - x_train_1_from_5_24_to_3_25
                                 + x_train_1_from_5_23_to_5_24
                                 + x_train_1_from_6_23_to_5_24
                                 - x_train_1_from_5_24_to_5_25  = 0
 flow_train_1_node_5_25:         - x_train_1_from_5_25_to_3_26
                                 + x_train_1_from_5_24_to_5_25
                                 + x_train_1_from_6_24_to_5_25
                                 - x_train_1_from_5_25_to_5_26  = 0
 flow_train_1_node_5_26:         - x_train_1_from_5_26_to_3_27
                                 + x_train_1_from_5_25_to_5_26
                                 + x_train_1_from_6_25_to_5_26
                                 - x_train_1_from_5_26_to_5_27  = 0
 flow_train_1_node_5_27:         - x_train_1_from_5_27_to_3_28
                                 + x_train_1_from_5_26_to_5_27
                                 + x_train_1_from_6_26_to_5_27  = 0
 flow_train_1_node_6_1:          x_train_1_from_sigma_to_6_1
                                 - x_train_1_from_6_1_to_4_2
                                 - x_train_1_from_6_1_to_5_2
                                 - x_train_1_from_6_1_to_6_2  = 0
 flow_train_1_node_6_2:          x_train_1_from_sigma_to_6_2
                                 - x_train_1_from_6_2_to_4_3
                                 - x_train_1_from_6_2_to_5_3
                                 + x_train_1_from_6_1_to_6_2
                                 - x_train_1_from_6_2_to_6_3  = 0
 flow_train_1_node_6_3:          x_train_1_from_sigma_to_6_3
                                 - x_train_1_from_6_3_to_4_4
                                 - x_train_1_from_6_3_to_5_4
                                 + x_train_1_from_6_2_to_6_3
                                 - x_train_1_from_6_3_to_6_4  = 0
 flow_train_1_node_6_4:          x_train_1_from_sigma_to_6_4
                                 - x_train_1_from_6_4_to_4_5
                                 - x_train_1_from_6_4_to_5_5
                                 + x_train_1_from_6_3_to_6_4
                                 - x_train_1_from_6_4_to_6_5  = 0
 flow_train_1_node_6_5:          x_train_1_from_sigma_to_6_5
                                 - x_train_1_from_6_5_to_4_6
                                 - x_train_1_from_6_5_to_5_6
                                 + x_train_1_from_6_4_to_6_5
                                 - x_train_1_from_6_5_to_6_6  = 0
 flow_train_1_node_6_6:          x_train_1_from_sigma_to_6_6
                                 - x_train_1_from_6_6_to_4_7
                                 - x_train_1_from_6_6_to_5_7
                                 + x_train_1_from_6_5_to_6_6
                                 - x_train_1_from_6_6_to_6_7  = 0
 flow_train_1_node_6_7:          x_train_1_from_sigma_to_6_7
                                 - x_train_1_from_6_7_to_4_8
                                 - x_train_1_from_6_7_to_5_8
                                 + x_train_1_from_6_6_to_6_7
                                 - x_train_1_from_6_7_to_6_8  = 0
 flow_train_1_node_6_8:          x_train_1_from_sigma_to_6_8
                                 - x_train_1_from_6_8_to_4_9
                                 - x_train_1_from_6_8_to_5_9
                                 + x_train_1_from_6_7_to_6_8
                                 - x_train_1_from_6_8_to_6_9  = 0
 flow_train_1_node_6_9:          x_train_1_from_sigma_to_6_9
                                 - x_train_1_from_6_9_to_4_10
                                 - x_train_1_from_6_9_to_5_10
                                 + x_train_1_from_6_8_to_6_9
                                 - x_train_1_from_6_9_to_6_10  = 0
 flow_train_1_node_6_10:         x_train_1_from_sigma_to_6_10
                                 - x_train_1_from_6_10_to_4_11
                                 - x_train_1_from_6_10_to_5_11
                                 + x_train_1_from_6_9_to_6_10
                                 - x_train_1_from_6_10_to_6_11  = 0
 flow_train_1_node_6_11:         x_train_1_from_sigma_to_6_11
                                 - x_train_1_from_6_11_to_4_12
                                 - x_train_1_from_6_11_to_5_12
                                 + x_train_1_from_6_10_to_6_11
                                 - x_train_1_from_6_11_to_6_12  = 0
 flow_train_1_node_6_12:         x_train_1_from_sigma_to_6_12
                                 - x_train_1_from_6_12_to_4_13
                                 - x_train_1_from_6_12_to_5_13
                                 + x_train_1_from_6_11_to_6_12
                                 - x_train_1_from_6_12_to_6_13  = 0
 flow_train_1_node_6_13:         x_train_1_from_sigma_to_6_13
                                 - x_train_1_from_6_13_to_4_14
                                 - x_train_1_from_6_13_to_5_14
                                 + x_train_1_from_6_12_to_6_13
                                 - x_train_1_from_6_13_to_6_14  = 0
 flow_train_1_node_6_14:         x_train_1_from_sigma_to_6_14
                                 - x_train_1_from_6_14_to_4_15
                                 - x_train_1_from_6_14_to_5_15
                                 + x_train_1_from_6_13_to_6_14
                                 - x_train_1_from_6_14_to_6_15  = 0
 flow_train_1_node_6_15:         x_train_1_from_sigma_to_6_15
                                 - x_train_1_from_6_15_to_4_16
                                 - x_train_1_from_6_15_to_5_16
                                 + x_train_1_from_6_14_to_6_15
                                 - x_train_1_from_6_15_to_6_16  = 0
 flow_train_1_node_6_16:         x_train_1_from_sigma_to_6_16
                                 - x_train_1_from_6_16_to_4_17
                                 - x_train_1_from_6_16_to_5_17
                                 + x_train_1_from_6_15_to_6_16
                                 - x_train_1_from_6_16_to_6_17  = 0
 flow_train_1_node_6_17:         x_train_1_from_sigma_to_6_17
                                 - x_train_1_from_6_17_to_4_18
                                 - x_train_1_from_6_17_to_5_18
                                 + x_train_1_from_6_16_to_6_17
                                 - x_train_1_from_6_17_to_6_18  = 0
 flow_train_1_node_6_18:         x_train_1_from_sigma_to_6_18
                                 - x_train_1_from_6_18_to_4_19
                                 - x_train_1_from_6_18_to_5_19
                                 + x_train_1_from_6_17_to_6_18
                                 - x_train_1_from_6_18_to_6_19  = 0
 flow_train_1_node_6_19:         x_train_1_from_sigma_to_6_19
                                 - x_train_1_from_6_19_to_4_20
                                 - x_train_1_from_6_19_to_5_20
                                 + x_train_1_from_6_18_to_6_19
                                 - x_train_1_from_6_19_to_6_20  = 0
 flow_train_1_node_6_20:         x_train_1_from_sigma_to_6_20
                                 - x_train_1_from_6_20_to_4_21
                                 - x_train_1_from_6_20_to_5_21
                                 + x_train_1_from_6_19_to_6_20
                                 - x_train_1_from_6_20_to_6_21  = 0
 flow_train_1_node_6_21:         x_train_1_from_sigma_to_6_21
                                 - x_train_1_from_6_21_to_4_22
                                 - x_train_1_from_6_21_to_5_22
                                 + x_train_1_from_6_20_to_6_21
                                 - x_train_1_from_6_21_to_6_22  = 0
 flow_train_1_node_6_22:         x_train_1_from_sigma_to_6_22
                                 - x_train_1_from_6_22_to_4_23
                                 - x_train_1_from_6_22_to_5_23
                                 + x_train_1_from_6_21_to_6_22
                                 - x_train_1_from_6_22_to_6_23  = 0
 flow_train_1_node_6_23:         x_train_1_from_sigma_to_6_23
                                 - x_train_1_from_6_23_to_4_24
                                 - x_train_1_from_6_23_to_5_24
                                 + x_train_1_from_6_22_to_6_23
                                 - x_train_1_from_6_23_to_6_24  = 0
 flow_train_1_node_6_24:         x_train_1_from_sigma_to_6_24
                                 - x_train_1_from_6_24_to_4_25
                                 - x_train_1_from_6_24_to_5_25
                                 + x_train_1_from_6_23_to_6_24
                                 - x_train_1_from_6_24_to_6_25  = 0
 flow_train_1_node_6_25:         x_train_1_from_sigma_to_6_25
                                 - x_train_1_from_6_25_to_4_26
                                 - x_train_1_from_6_25_to_5_26
                                 + x_train_1_from_6_24_to_6_25
                                 - x_train_1_from_6_25_to_6_26  = 0
 flow_train_1_node_6_26:         x_train_1_from_sigma_to_6_26
                                 - x_train_1_from_6_26_to_4_27
                                 - x_train_1_from_6_26_to_5_27
                                 + x_train_1_from_6_25_to_6_26
                                 - x_train_1_from_6_26_to_6_27  = 0
 flow_train_1_node_6_27:         x_train_1_from_sigma_to_6_27
                                 - x_train_1_from_6_27_to_4_28
                                 + x_train_1_from_6_26_to_6_27  = 0
 z_sa_train_0_schedule_0:        z_train_0_segment_1  = 1
 set_z_train_0_segment_0:        - 30 z_train_0_segment_0 + y_train_0_node_0_1
                                 + y_train_0_node_0_2 + y_train_0_node_0_3
                                 + y_train_0_node_0_4 + y_train_0_node_0_5
                                 + y_train_0_node_0_6 + y_train_0_node_0_7
                                 + y_train_0_node_0_8 + y_train_0_node_0_9
                                 + y_train_0_node_0_10 + y_train_0_node_0_11
                                 + y_train_0_node_0_12 + y_train_0_node_0_13
                                 + y_train_0_node_0_14 + y_train_0_node_0_15
                                 + y_train_0_node_0_16 + y_train_0_node_0_17
                                 + y_train_0_node_0_18 + y_train_0_node_0_19
                                 + y_train_0_node_0_20 + y_train_0_node_0_21
                                 + y_train_0_node_0_22 + y_train_0_node_0_23
                                 + y_train_0_node_0_24 + y_train_0_node_0_25
                                 + y_train_0_node_0_26 + y_train_0_node_0_27
                                 <= 0
 set_z_train_0_segment_1:        - 30 z_train_0_segment_1 + y_train_0_node_1_2
                                 + y_train_0_node_1_3 + y_train_0_node_1_4
                                 + y_train_0_node_1_5 + y_train_0_node_1_6
                                 + y_train_0_node_1_7 + y_train_0_node_1_8
                                 + y_train_0_node_1_9 + y_train_0_node_1_10
                                 + y_train_0_node_1_11 + y_train_0_node_1_12
                                 + y_train_0_node_1_13 + y_train_0_node_1_14
                                 + y_train_0_node_1_15 + y_train_0_node_1_16
                                 + y_train_0_node_1_17 + y_train_0_node_1_18
                                 + y_train_0_node_1_19 + y_train_0_node_1_20
                                 + y_train_0_node_1_21 + y_train_0_node_1_22
                                 + y_train_0_node_1_23 + y_train_0_node_1_24
                                 + y_train_0_node_1_25 + y_train_0_node_1_26
                                 + y_train_0_node_1_27 <= 0
 set_z_train_0_segment_2:        - 30 z_train_0_segment_2 + y_train_0_node_2_2
                                 + y_train_0_node_2_3 + y_train_0_node_2_4
                                 + y_train_0_node_2_5 + y_train_0_node_2_6
                                 + y_train_0_node_2_7 + y_train_0_node_2_8
                                 + y_train_0_node_2_9 + y_train_0_node_2_10
                                 + y_train_0_node_2_11 + y_train_0_node_2_12
                                 + y_train_0_node_2_13 + y_train_0_node_2_14
                                 + y_train_0_node_2_15 + y_train_0_node_2_16
                                 + y_train_0_node_2_17 + y_train_0_node_2_18
                                 + y_train_0_node_2_19 + y_train_0_node_2_20
                                 + y_train_0_node_2_21 + y_train_0_node_2_22
                                 + y_train_0_node_2_23 + y_train_0_node_2_24
                                 + y_train_0_node_2_25 + y_train_0_node_2_26
                                 + y_train_0_node_2_27 + y_train_0_node_2_28
                                 <= 0
 set_z_train_0_segment_3:        - 30 z_train_0_segment_3 + y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 <= 0
 set_z_train_0_segment_4:        - 30 z_train_0_segment_4 + y_train_0_node_4_3
                                 + y_train_0_node_4_4 + y_train_0_node_4_5
                                 + y_train_0_node_4_6 + y_train_0_node_4_7
                                 + y_train_0_node_4_8 + y_train_0_node_4_9
                                 + y_train_0_node_4_10 + y_train_0_node_4_11
                                 + y_train_0_node_4_12 + y_train_0_node_4_13
                                 + y_train_0_node_4_14 + y_train_0_node_4_15
                                 + y_train_0_node_4_16 + y_train_0_node_4_17
                                 + y_train_0_node_4_18 + y_train_0_node_4_19
                                 + y_train_0_node_4_20 + y_train_0_node_4_21
                                 + y_train_0_node_4_22 + y_train_0_node_4_23
                                 + y_train_0_node_4_24 + y_train_0_node_4_25
                                 + y_train_0_node_4_26 + y_train_0_node_4_27
                                 + y_train_0_node_4_28 + y_train_0_node_4_29
                                 <= 0
 set_z_train_0_segment_5:        - 30 z_train_0_segment_5 + y_train_0_node_5_6
                                 + y_train_0_node_5_7 + y_train_0_node_5_8
                                 + y_train_0_node_5_9 + y_train_0_node_5_10
                                 + y_train_0_node_5_11 + y_train_0_node_5_12
                                 + y_train_0_node_5_13 + y_train_0_node_5_14
                                 + y_train_0_node_5_15 + y_train_0_node_5_16
                                 + y_train_0_node_5_17 + y_train_0_node_5_18
                                 + y_train_0_node_5_19 + y_train_0_node_5_20
                                 + y_train_0_node_5_21 + y_train_0_node_5_22
                                 + y_train_0_node_5_23 + y_train_0_node_5_24
                                 + y_train_0_node_5_25 + y_train_0_node_5_26
                                 + y_train_0_node_5_27 + y_train_0_node_5_28
                                 + y_train_0_node_5_29 <= 0
 set_z_train_0_segment_6:        - 30 z_train_0_segment_6 + y_train_0_node_6_4
                                 + y_train_0_node_6_5 + y_train_0_node_6_6
                                 + y_train_0_node_6_7 + y_train_0_node_6_8
                                 + y_train_0_node_6_9 + y_train_0_node_6_10
                                 + y_train_0_node_6_11 + y_train_0_node_6_12
                                 + y_train_0_node_6_13 + y_train_0_node_6_14
                                 + y_train_0_node_6_15 + y_train_0_node_6_16
                                 + y_train_0_node_6_17 + y_train_0_node_6_18
                                 + y_train_0_node_6_19 + y_train_0_node_6_20
                                 + y_train_0_node_6_21 + y_train_0_node_6_22
                                 + y_train_0_node_6_23 + y_train_0_node_6_24
                                 + y_train_0_node_6_25 + y_train_0_node_6_26
                                 + y_train_0_node_6_27 + y_train_0_node_6_28
                                 + y_train_0_node_6_29 + y_train_0_node_6_30
                                 <= 0
 set_z_train_1_segment_0:        - 30 z_train_1_segment_0 + y_train_1_node_0_4
                                 + y_train_1_node_0_5 + y_train_1_node_0_6
                                 + y_train_1_node_0_7 + y_train_1_node_0_8
                                 + y_train_1_node_0_9 + y_train_1_node_0_10
                                 + y_train_1_node_0_11 + y_train_1_node_0_12
                                 + y_train_1_node_0_13 + y_train_1_node_0_14
                                 + y_train_1_node_0_15 + y_train_1_node_0_16
                                 + y_train_1_node_0_17 + y_train_1_node_0_18
                                 + y_train_1_node_0_19 + y_train_1_node_0_20
                                 + y_train_1_node_0_21 + y_train_1_node_0_22
                                 + y_train_1_node_0_23 + y_train_1_node_0_24
                                 + y_train_1_node_0_25 + y_train_1_node_0_26
                                 + y_train_1_node_0_27 + y_train_1_node_0_28
                                 + y_train_1_node_0_29 + y_train_1_node_0_30
                                 <= 0
 set_z_train_1_segment_1:        - 30 z_train_1_segment_1 + y_train_1_node_1_6
                                 + y_train_1_node_1_7 + y_train_1_node_1_8
                                 + y_train_1_node_1_9 + y_train_1_node_1_10
                                 + y_train_1_node_1_11 + y_train_1_node_1_12
                                 + y_train_1_node_1_13 + y_train_1_node_1_14
                                 + y_train_1_node_1_15 + y_train_1_node_1_16
                                 + y_train_1_node_1_17 + y_train_1_node_1_18
                                 + y_train_1_node_1_19 + y_train_1_node_1_20
                                 + y_train_1_node_1_21 + y_train_1_node_1_22
                                 + y_train_1_node_1_23 + y_train_1_node_1_24
                                 + y_train_1_node_1_25 + y_train_1_node_1_26
                                 + y_train_1_node_1_27 + y_train_1_node_1_28
                                 + y_train_1_node_1_29 <= 0
 set_z_train_1_segment_2:        - 30 z_train_1_segment_2 + y_train_1_node_2_3
                                 + y_train_1_node_2_4 + y_train_1_node_2_5
                                 + y_train_1_node_2_6 + y_train_1_node_2_7
                                 + y_train_1_node_2_8 + y_train_1_node_2_9
                                 + y_train_1_node_2_10 + y_train_1_node_2_11
                                 + y_train_1_node_2_12 + y_train_1_node_2_13
                                 + y_train_1_node_2_14 + y_train_1_node_2_15
                                 + y_train_1_node_2_16 + y_train_1_node_2_17
                                 + y_train_1_node_2_18 + y_train_1_node_2_19
                                 + y_train_1_node_2_20 + y_train_1_node_2_21
                                 + y_train_1_node_2_22 + y_train_1_node_2_23
                                 + y_train_1_node_2_24 + y_train_1_node_2_25
                                 + y_train_1_node_2_26 + y_train_1_node_2_27
                                 + y_train_1_node_2_28 + y_train_1_node_2_29
                                 <= 0
 set_z_train_1_segment_3:        - 30 z_train_1_segment_3 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 set_z_train_1_segment_4:        - 30 z_train_1_segment_4 + y_train_1_node_4_2
                                 + y_train_1_node_4_3 + y_train_1_node_4_4
                                 + y_train_1_node_4_5 + y_train_1_node_4_6
                                 + y_train_1_node_4_7 + y_train_1_node_4_8
                                 + y_train_1_node_4_9 + y_train_1_node_4_10
                                 + y_train_1_node_4_11 + y_train_1_node_4_12
                                 + y_train_1_node_4_13 + y_train_1_node_4_14
                                 + y_train_1_node_4_15 + y_train_1_node_4_16
                                 + y_train_1_node_4_17 + y_train_1_node_4_18
                                 + y_train_1_node_4_19 + y_train_1_node_4_20
                                 + y_train_1_node_4_21 + y_train_1_node_4_22
                                 + y_train_1_node_4_23 + y_train_1_node_4_24
                                 + y_train_1_node_4_25 + y_train_1_node_4_26
                                 + y_train_1_node_4_27 + y_train_1_node_4_28
                                 <= 0
 set_z_train_1_segment_5:        - 30 z_train_1_segment_5 + y_train_1_node_5_2
                                 + y_train_1_node_5_3 + y_train_1_node_5_4
                                 + y_train_1_node_5_5 + y_train_1_node_5_6
                                 + y_train_1_node_5_7 + y_train_1_node_5_8
                                 + y_train_1_node_5_9 + y_train_1_node_5_10
                                 + y_train_1_node_5_11 + y_train_1_node_5_12
                                 + y_train_1_node_5_13 + y_train_1_node_5_14
                                 + y_train_1_node_5_15 + y_train_1_node_5_16
                                 + y_train_1_node_5_17 + y_train_1_node_5_18
                                 + y_train_1_node_5_19 + y_train_1_node_5_20
                                 + y_train_1_node_5_21 + y_train_1_node_5_22
                                 + y_train_1_node_5_23 + y_train_1_node_5_24
                                 + y_train_1_node_5_25 + y_train_1_node_5_26
                                 + y_train_1_node_5_27 <= 0
 set_z_train_1_segment_6:        - 30 z_train_1_segment_6 + y_train_1_node_6_1
                                 + y_train_1_node_6_2 + y_train_1_node_6_3
                                 + y_train_1_node_6_4 + y_train_1_node_6_5
                                 + y_train_1_node_6_6 + y_train_1_node_6_7
                                 + y_train_1_node_6_8 + y_train_1_node_6_9
                                 + y_train_1_node_6_10 + y_train_1_node_6_11
                                 + y_train_1_node_6_12 + y_train_1_node_6_13
                                 + y_train_1_node_6_14 + y_train_1_node_6_15
                                 + y_train_1_node_6_16 + y_train_1_node_6_17
                                 + y_train_1_node_6_18 + y_train_1_node_6_19
                                 + y_train_1_node_6_20 + y_train_1_node_6_21
                                 + y_train_1_node_6_22 + y_train_1_node_6_23
                                 + y_train_1_node_6_24 + y_train_1_node_6_25
                                 + y_train_1_node_6_26 + y_train_1_node_6_27
                                 <= 0
 set_theta_train_0_segment_0:    theta_train_0_segment_0
                                 + x_train_0_from_sigma_to_0_1
                                 + 2 x_train_0_from_sigma_to_0_2
                                 + 3 x_train_0_from_sigma_to_0_3
                                 + 4 x_train_0_from_sigma_to_0_4
                                 + 5 x_train_0_from_sigma_to_0_5
                                 + 6 x_train_0_from_sigma_to_0_6
                                 + 7 x_train_0_from_sigma_to_0_7
                                 + 8 x_train_0_from_sigma_to_0_8
                                 + 9 x_train_0_from_sigma_to_0_9
                                 + 10 x_train_0_from_sigma_to_0_10
                                 + 11 x_train_0_from_sigma_to_0_11
                                 + 12 x_train_0_from_sigma_to_0_12
                                 + 13 x_train_0_from_sigma_to_0_13
                                 + 14 x_train_0_from_sigma_to_0_14
                                 + 15 x_train_0_from_sigma_to_0_15
                                 + 16 x_train_0_from_sigma_to_0_16
                                 + 17 x_train_0_from_sigma_to_0_17
                                 + 18 x_train_0_from_sigma_to_0_18
                                 + 19 x_train_0_from_sigma_to_0_19
                                 + 20 x_train_0_from_sigma_to_0_20
                                 + 21 x_train_0_from_sigma_to_0_21
                                 + 22 x_train_0_from_sigma_to_0_22
                                 + 23 x_train_0_from_sigma_to_0_23
                                 + 24 x_train_0_from_sigma_to_0_24
                                 + 25 x_train_0_from_sigma_to_0_25
                                 + 26 x_train_0_from_sigma_to_0_26
                                 + 27 x_train_0_from_sigma_to_0_27
                                 - x_train_0_from_0_1_to_1_2
                                 - 2 x_train_0_from_0_2_to_1_3
                                 - 3 x_train_0_from_0_3_to_1_4
                                 - 4 x_train_0_from_0_4_to_1_5
                                 - 5 x_train_0_from_0_5_to_1_6
                                 - 6 x_train_0_from_0_6_to_1_7
                                 - 7 x_train_0_from_0_7_to_1_8
                                 - 8 x_train_0_from_0_8_to_1_9
                                 - 9 x_train_0_from_0_9_to_1_10
                                 - 10 x_train_0_from_0_10_to_1_11
                                 - 11 x_train_0_from_0_11_to_1_12
                                 - 12 x_train_0_from_0_12_to_1_13
                                 - 13 x_train_0_from_0_13_to_1_14
                                 - 14 x_train_0_from_0_14_to_1_15
                                 - 15 x_train_0_from_0_15_to_1_16
                                 - 16 x_train_0_from_0_16_to_1_17
                                 - 17 x_train_0_from_0_17_to_1_18
                                 - 18 x_train_0_from_0_18_to_1_19
                                 - 19 x_train_0_from_0_19_to_1_20
                                 - 20 x_train_0_from_0_20_to_1_21
                                 - 21 x_train_0_from_0_21_to_1_22
                                 - 22 x_train_0_from_0_22_to_1_23
                                 - 23 x_train_0_from_0_23_to_1_24
                                 - 24 x_train_0_from_0_24_to_1_25
                                 - 25 x_train_0_from_0_25_to_1_26
                                 - 26 x_train_0_from_0_26_to_1_27
                                 - x_train_0_from_0_1_to_2_2
                                 - 2 x_train_0_from_0_2_to_2_3
                                 - 3 x_train_0_from_0_3_to_2_4
                                 - 4 x_train_0_from_0_4_to_2_5
                                 - 5 x_train_0_from_0_5_to_2_6
                                 - 6 x_train_0_from_0_6_to_2_7
                                 - 7 x_train_0_from_0_7_to_2_8
                                 - 8 x_train_0_from_0_8_to_2_9
                                 - 9 x_train_0_from_0_9_to_2_10
                                 - 10 x_train_0_from_0_10_to_2_11
                                 - 11 x_train_0_from_0_11_to_2_12
                                 - 12 x_train_0_from_0_12_to_2_13
                                 - 13 x_train_0_from_0_13_to_2_14
                                 - 14 x_train_0_from_0_14_to_2_15
                                 - 15 x_train_0_from_0_15_to_2_16
                                 - 16 x_train_0_from_0_16_to_2_17
                                 - 17 x_train_0_from_0_17_to_2_18
                                 - 18 x_train_0_from_0_18_to_2_19
                                 - 19 x_train_0_from_0_19_to_2_20
                                 - 20 x_train_0_from_0_20_to_2_21
                                 - 21 x_train_0_from_0_21_to_2_22
                                 - 22 x_train_0_from_0_22_to_2_23
                                 - 23 x_train_0_from_0_23_to_2_24
                                 - 24 x_train_0_from_0_24_to_2_25
                                 - 25 x_train_0_from_0_25_to_2_26
                                 - 26 x_train_0_from_0_26_to_2_27
                                 - 27 x_train_0_from_0_27_to_2_28  = 0
 set_theta_train_0_segment_1:    theta_train_0_segment_1
                                 + 2 x_train_0_from_0_1_to_1_2
                                 + 3 x_train_0_from_0_2_to_1_3
                                 + 4 x_train_0_from_0_3_to_1_4
                                 + 5 x_train_0_from_0_4_to_1_5
                                 + 6 x_train_0_from_0_5_to_1_6
                                 + 7 x_train_0_from_0_6_to_1_7
                                 + 8 x_train_0_from_0_7_to_1_8
                                 + 9 x_train_0_from_0_8_to_1_9
                                 + 10 x_train_0_from_0_9_to_1_10
                                 + 11 x_train_0_from_0_10_to_1_11
                                 + 12 x_train_0_from_0_11_to_1_12
                                 + 13 x_train_0_from_0_12_to_1_13
                                 + 14 x_train_0_from_0_13_to_1_14
                                 + 15 x_train_0_from_0_14_to_1_15
                                 + 16 x_train_0_from_0_15_to_1_16
                                 + 17 x_train_0_from_0_16_to_1_17
                                 + 18 x_train_0_from_0_17_to_1_18
                                 + 19 x_train_0_from_0_18_to_1_19
                                 + 20 x_train_0_from_0_19_to_1_20
                                 + 21 x_train_0_from_0_20_to_1_21
                                 + 22 x_train_0_from_0_21_to_1_22
                                 + 23 x_train_0_from_0_22_to_1_23
                                 + 24 x_train_0_from_0_23_to_1_24
                                 + 25 x_train_0_from_0_24_to_1_25
                                 + 26 x_train_0_from_0_25_to_1_26
                                 + 27 x_train_0_from_0_26_to_1_27
                                 - 4 x_train_0_from_1_4_to_3_5
                                 - 5 x_train_0_from_1_5_to_3_6
                                 - 6 x_train_0_from_1_6_to_3_7
                                 - 7 x_train_0_from_1_7_to_3_8
                                 - 8 x_train_0_from_1_8_to_3_9
                                 - 9 x_train_0_from_1_9_to_3_10
                                 - 10 x_train_0_from_1_10_to_3_11
                                 - 11 x_train_0_from_1_11_to_3_12
                                 - 12 x_train_0_from_1_12_to_3_13
                                 - 13 x_train_0_from_1_13_to_3_14
                                 - 14 x_train_0_from_1_14_to_3_15
                                 - 15 x_train_0_from_1_15_to_3_16
                                 - 16 x_train_0_from_1_16_to_3_17
                                 - 17 x_train_0_from_1_17_to_3_18
                                 - 18 x_train_0_from_1_18_to_3_19
                                 - 19 x_train_0_from_1_19_to_3_20
                                 - 20 x_train_0_from_1_20_to_3_21
                                 - 21 x_train_0_from_1_21_to_3_22
                                 - 22 x_train_0_from_1_22_to_3_23
                                 - 23 x_train_0_from_1_23_to_3_24
                                 - 24 x_train_0_from_1_24_to_3_25
                                 - 25 x_train_0_from_1_25_to_3_26
                                 - 26 x_train_0_from_1_26_to_3_27
                                 - 27 x_train_0_from_1_27_to_3_28  = 0
 set_theta_train_0_segment_2:    theta_train_0_segment_2
                                 + 2 x_train_0_from_0_1_to_2_2
                                 + 3 x_train_0_from_0_2_to_2_3
                                 + 4 x_train_0_from_0_3_to_2_4
                                 + 5 x_train_0_from_0_4_to_2_5
                                 + 6 x_train_0_from_0_5_to_2_6
                                 + 7 x_train_0_from_0_6_to_2_7
                                 + 8 x_train_0_from_0_7_to_2_8
                                 + 9 x_train_0_from_0_8_to_2_9
                                 + 10 x_train_0_from_0_9_to_2_10
                                 + 11 x_train_0_from_0_10_to_2_11
                                 + 12 x_train_0_from_0_11_to_2_12
                                 + 13 x_train_0_from_0_12_to_2_13
                                 + 14 x_train_0_from_0_13_to_2_14
                                 + 15 x_train_0_from_0_14_to_2_15
                                 + 16 x_train_0_from_0_15_to_2_16
                                 + 17 x_train_0_from_0_16_to_2_17
                                 + 18 x_train_0_from_0_17_to_2_18
                                 + 19 x_train_0_from_0_18_to_2_19
                                 + 20 x_train_0_from_0_19_to_2_20
                                 + 21 x_train_0_from_0_20_to_2_21
                                 + 22 x_train_0_from_0_21_to_2_22
                                 + 23 x_train_0_from_0_22_to_2_23
                                 + 24 x_train_0_from_0_23_to_2_24
                                 + 25 x_train_0_from_0_24_to_2_25
                                 + 26 x_train_0_from_0_25_to_2_26
                                 + 27 x_train_0_from_0_26_to_2_27
                                 + 28 x_train_0_from_0_27_to_2_28
                                 - 2 x_train_0_from_2_2_to_4_3
                                 - 3 x_train_0_from_2_3_to_4_4
                                 - 4 x_train_0_from_2_4_to_4_5
                                 - 5 x_train_0_from_2_5_to_4_6
                                 - 6 x_train_0_from_2_6_to_4_7
                                 - 7 x_train_0_from_2_7_to_4_8
                                 - 8 x_train_0_from_2_8_to_4_9
                                 - 9 x_train_0_from_2_9_to_4_10
                                 - 10 x_train_0_from_2_10_to_4_11
                                 - 11 x_train_0_from_2_11_to_4_12
                                 - 12 x_train_0_from_2_12_to_4_13
                                 - 13 x_train_0_from_2_13_to_4_14
                                 - 14 x_train_0_from_2_14_to_4_15
                                 - 15 x_train_0_from_2_15_to_4_16
                                 - 16 x_train_0_from_2_16_to_4_17
                                 - 17 x_train_0_from_2_17_to_4_18
                                 - 18 x_train_0_from_2_18_to_4_19
                                 - 19 x_train_0_from_2_19_to_4_20
                                 - 20 x_train_0_from_2_20_to_4_21
                                 - 21 x_train_0_from_2_21_to_4_22
                                 - 22 x_train_0_from_2_22_to_4_23
                                 - 23 x_train_0_from_2_23_to_4_24
                                 - 24 x_train_0_from_2_24_to_4_25
                                 - 25 x_train_0_from_2_25_to_4_26
                                 - 26 x_train_0_from_2_26_to_4_27
                                 - 27 x_train_0_from_2_27_to_4_28
                                 - 28 x_train_0_from_2_28_to_4_29  = 0
 set_theta_train_0_segment_3:    theta_train_0_segment_3
                                 + 5 x_train_0_from_1_4_to_3_5
                                 + 6 x_train_0_from_1_5_to_3_6
                                 + 7 x_train_0_from_1_6_to_3_7
                                 + 8 x_train_0_from_1_7_to_3_8
                                 + 9 x_train_0_from_1_8_to_3_9
                                 + 10 x_train_0_from_1_9_to_3_10
                                 + 11 x_train_0_from_1_10_to_3_11
                                 + 12 x_train_0_from_1_11_to_3_12
                                 + 13 x_train_0_from_1_12_to_3_13
                                 + 14 x_train_0_from_1_13_to_3_14
                                 + 15 x_train_0_from_1_14_to_3_15
                                 + 16 x_train_0_from_1_15_to_3_16
                                 + 17 x_train_0_from_1_16_to_3_17
                                 + 18 x_train_0_from_1_17_to_3_18
                                 + 19 x_train_0_from_1_18_to_3_19
                                 + 20 x_train_0_from_1_19_to_3_20
                                 + 21 x_train_0_from_1_20_to_3_21
                                 + 22 x_train_0_from_1_21_to_3_22
                                 + 23 x_train_0_from_1_22_to_3_23
                                 + 24 x_train_0_from_1_23_to_3_24
                                 + 25 x_train_0_from_1_24_to_3_25
                                 + 26 x_train_0_from_1_25_to_3_26
                                 + 27 x_train_0_from_1_26_to_3_27
                                 + 28 x_train_0_from_1_27_to_3_28
                                 - 5 x_train_0_from_3_5_to_5_6
                                 - 6 x_train_0_from_3_6_to_5_7
                                 - 7 x_train_0_from_3_7_to_5_8
                                 - 8 x_train_0_from_3_8_to_5_9
                                 - 9 x_train_0_from_3_9_to_5_10
                                 - 10 x_train_0_from_3_10_to_5_11
                                 - 11 x_train_0_from_3_11_to_5_12
                                 - 12 x_train_0_from_3_12_to_5_13
                                 - 13 x_train_0_from_3_13_to_5_14
                                 - 14 x_train_0_from_3_14_to_5_15
                                 - 15 x_train_0_from_3_15_to_5_16
                                 - 16 x_train_0_from_3_16_to_5_17
                                 - 17 x_train_0_from_3_17_to_5_18
                                 - 18 x_train_0_from_3_18_to_5_19
                                 - 19 x_train_0_from_3_19_to_5_20
                                 - 20 x_train_0_from_3_20_to_5_21
                                 - 21 x_train_0_from_3_21_to_5_22
                                 - 22 x_train_0_from_3_22_to_5_23
                                 - 23 x_train_0_from_3_23_to_5_24
                                 - 24 x_train_0_from_3_24_to_5_25
                                 - 25 x_train_0_from_3_25_to_5_26
                                 - 26 x_train_0_from_3_26_to_5_27
                                 - 27 x_train_0_from_3_27_to_5_28
                                 - 28 x_train_0_from_3_28_to_5_29  = 0
 set_theta_train_0_segment_4:    theta_train_0_segment_4
                                 + 3 x_train_0_from_2_2_to_4_3
                                 + 4 x_train_0_from_2_3_to_4_4
                                 + 5 x_train_0_from_2_4_to_4_5
                                 + 6 x_train_0_from_2_5_to_4_6
                                 + 7 x_train_0_from_2_6_to_4_7
                                 + 8 x_train_0_from_2_7_to_4_8
                                 + 9 x_train_0_from_2_8_to_4_9
                                 + 10 x_train_0_from_2_9_to_4_10
                                 + 11 x_train_0_from_2_10_to_4_11
                                 + 12 x_train_0_from_2_11_to_4_12
                                 + 13 x_train_0_from_2_12_to_4_13
                                 + 14 x_train_0_from_2_13_to_4_14
                                 + 15 x_train_0_from_2_14_to_4_15
                                 + 16 x_train_0_from_2_15_to_4_16
                                 + 17 x_train_0_from_2_16_to_4_17
                                 + 18 x_train_0_from_2_17_to_4_18
                                 + 19 x_train_0_from_2_18_to_4_19
                                 + 20 x_train_0_from_2_19_to_4_20
                                 + 21 x_train_0_from_2_20_to_4_21
                                 + 22 x_train_0_from_2_21_to_4_22
                                 + 23 x_train_0_from_2_22_to_4_23
                                 + 24 x_train_0_from_2_23_to_4_24
                                 + 25 x_train_0_from_2_24_to_4_25
                                 + 26 x_train_0_from_2_25_to_4_26
                                 + 27 x_train_0_from_2_26_to_4_27
                                 + 28 x_train_0_from_2_27_to_4_28
                                 + 29 x_train_0_from_2_28_to_4_29
                                 - 3 x_train_0_from_4_3_to_6_4
                                 - 4 x_train_0_from_4_4_to_6_5
                                 - 5 x_train_0_from_4_5_to_6_6
                                 - 6 x_train_0_from_4_6_to_6_7
                                 - 7 x_train_0_from_4_7_to_6_8
                                 - 8 x_train_0_from_4_8_to_6_9
                                 - 9 x_train_0_from_4_9_to_6_10
                                 - 10 x_train_0_from_4_10_to_6_11
                                 - 11 x_train_0_from_4_11_to_6_12
                                 - 12 x_train_0_from_4_12_to_6_13
                                 - 13 x_train_0_from_4_13_to_6_14
                                 - 14 x_train_0_from_4_14_to_6_15
                                 - 15 x_train_0_from_4_15_to_6_16
                                 - 16 x_train_0_from_4_16_to_6_17
                                 - 17 x_train_0_from_4_17_to_6_18
                                 - 18 x_train_0_from_4_18_to_6_19
                                 - 19 x_train_0_from_4_19_to_6_20
                                 - 20 x_train_0_from_4_20_to_6_21
                                 - 21 x_train_0_from_4_21_to_6_22
                                 - 22 x_train_0_from_4_22_to_6_23
                                 - 23 x_train_0_from_4_23_to_6_24
                                 - 24 x_train_0_from_4_24_to_6_25
                                 - 25 x_train_0_from_4_25_to_6_26
                                 - 26 x_train_0_from_4_26_to_6_27
                                 - 27 x_train_0_from_4_27_to_6_28
                                 - 28 x_train_0_from_4_28_to_6_29
                                 - 29 x_train_0_from_4_29_to_6_30  = 0
 set_theta_train_0_segment_5:    theta_train_0_segment_5
                                 + 6 x_train_0_from_3_5_to_5_6
                                 + 7 x_train_0_from_3_6_to_5_7
                                 + 8 x_train_0_from_3_7_to_5_8
                                 + 9 x_train_0_from_3_8_to_5_9
                                 + 10 x_train_0_from_3_9_to_5_10
                                 + 11 x_train_0_from_3_10_to_5_11
                                 + 12 x_train_0_from_3_11_to_5_12
                                 + 13 x_train_0_from_3_12_to_5_13
                                 + 14 x_train_0_from_3_13_to_5_14
                                 + 15 x_train_0_from_3_14_to_5_15
                                 + 16 x_train_0_from_3_15_to_5_16
                                 + 17 x_train_0_from_3_16_to_5_17
                                 + 18 x_train_0_from_3_17_to_5_18
                                 + 19 x_train_0_from_3_18_to_5_19
                                 + 20 x_train_0_from_3_19_to_5_20
                                 + 21 x_train_0_from_3_20_to_5_21
                                 + 22 x_train_0_from_3_21_to_5_22
                                 + 23 x_train_0_from_3_22_to_5_23
                                 + 24 x_train_0_from_3_23_to_5_24
                                 + 25 x_train_0_from_3_24_to_5_25
                                 + 26 x_train_0_from_3_25_to_5_26
                                 + 27 x_train_0_from_3_26_to_5_27
                                 + 28 x_train_0_from_3_27_to_5_28
                                 + 29 x_train_0_from_3_28_to_5_29
                                 - 6 x_train_0_from_5_6_to_6_7
                                 - 7 x_train_0_from_5_7_to_6_8
                                 - 8 x_train_0_from_5_8_to_6_9
                                 - 9 x_train_0_from_5_9_to_6_10
                                 - 10 x_train_0_from_5_10_to_6_11
                                 - 11 x_train_0_from_5_11_to_6_12
                                 - 12 x_train_0_from_5_12_to_6_13
                                 - 13 x_train_0_from_5_13_to_6_14
                                 - 14 x_train_0_from_5_14_to_6_15
                                 - 15 x_train_0_from_5_15_to_6_16
                                 - 16 x_train_0_from_5_16_to_6_17
                                 - 17 x_train_0_from_5_17_to_6_18
                                 - 18 x_train_0_from_5_18_to_6_19
                                 - 19 x_train_0_from_5_19_to_6_20
                                 - 20 x_train_0_from_5_20_to_6_21
                                 - 21 x_train_0_from_5_21_to_6_22
                                 - 22 x_train_0_from_5_22_to_6_23
                                 - 23 x_train_0_from_5_23_to_6_24
                                 - 24 x_train_0_from_5_24_to_6_25
                                 - 25 x_train_0_from_5_25_to_6_26
                                 - 26 x_train_0_from_5_26_to_6_27
                                 - 27 x_train_0_from_5_27_to_6_28
                                 - 28 x_train_0_from_5_28_to_6_29
                                 - 29 x_train_0_from_5_29_to_6_30  = 0
 set_theta_train_0_segment_6:    theta_train_0_segment_6
                                 - 4 x_train_0_from_6_4_to_tau
                                 - 5 x_train_0_from_6_5_to_tau
                                 - 6 x_train_0_from_6_6_to_tau
                                 - 7 x_train_0_from_6_7_to_tau
                                 - 8 x_train_0_from_6_8_to_tau
                                 - 9 x_train_0_from_6_9_to_tau
                                 - 10 x_train_0_from_6_10_to_tau
                                 - 11 x_train_0_from_6_11_to_tau
                                 - 12 x_train_0_from_6_12_to_tau
                                 - 13 x_train_0_from_6_13_to_tau
                                 - 14 x_train_0_from_6_14_to_tau
                                 - 15 x_train_0_from_6_15_to_tau
                                 - 16 x_train_0_from_6_16_to_tau
                                 - 17 x_train_0_from_6_17_to_tau
                                 - 18 x_train_0_from_6_18_to_tau
                                 - 19 x_train_0_from_6_19_to_tau
                                 - 20 x_train_0_from_6_20_to_tau
                                 - 21 x_train_0_from_6_21_to_tau
                                 - 22 x_train_0_from_6_22_to_tau
                                 - 23 x_train_0_from_6_23_to_tau
                                 - 24 x_train_0_from_6_24_to_tau
                                 - 25 x_train_0_from_6_25_to_tau
                                 - 26 x_train_0_from_6_26_to_tau
                                 - 27 x_train_0_from_6_27_to_tau
                                 - 28 x_train_0_from_6_28_to_tau
                                 - 29 x_train_0_from_6_29_to_tau
                                 - 30 x_train_0_from_6_30_to_tau
                                 + 4 x_train_0_from_4_3_to_6_4
                                 + 5 x_train_0_from_4_4_to_6_5
                                 + 6 x_train_0_from_4_5_to_6_6
                                 + 7 x_train_0_from_4_6_to_6_7
                                 + 7 x_train_0_from_5_6_to_6_7
                                 + 8 x_train_0_from_4_7_to_6_8
                                 + 8 x_train_0_from_5_7_to_6_8
                                 + 9 x_train_0_from_4_8_to_6_9
                                 + 9 x_train_0_from_5_8_to_6_9
                                 + 10 x_train_0_from_4_9_to_6_10
                                 + 10 x_train_0_from_5_9_to_6_10
                                 + 11 x_train_0_from_4_10_to_6_11
                                 + 11 x_train_0_from_5_10_to_6_11
                                 + 12 x_train_0_from_4_11_to_6_12
                                 + 12 x_train_0_from_5_11_to_6_12
                                 + 13 x_train_0_from_4_12_to_6_13
                                 + 13 x_train_0_from_5_12_to_6_13
                                 + 14 x_train_0_from_4_13_to_6_14
                                 + 14 x_train_0_from_5_13_to_6_14
                                 + 15 x_train_0_from_4_14_to_6_15
                                 + 15 x_train_0_from_5_14_to_6_15
                                 + 16 x_train_0_from_4_15_to_6_16
                                 + 16 x_train_0_from_5_15_to_6_16
                                 + 17 x_train_0_from_4_16_to_6_17
                                 + 17 x_train_0_from_5_16_to_6_17
                                 + 18 x_train_0_from_4_17_to_6_18
                                 + 18 x_train_0_from_5_17_to_6_18
                                 + 19 x_train_0_from_4_18_to_6_19
                                 + 19 x_train_0_from_5_18_to_6_19
                                 + 20 x_train_0_from_4_19_to_6_20
                                 + 20 x_train_0_from_5_19_to_6_20
                                 + 21 x_train_0_from_4_20_to_6_21
                                 + 21 x_train_0_from_5_20_to_6_21
                                 + 22 x_train_0_from_4_21_to_6_22
                                 + 22 x_train_0_from_5_21_to_6_22
                                 + 23 x_train_0_from_4_22_to_6_23
                                 + 23 x_train_0_from_5_22_to_6_23
                                 + 24 x_train_0_from_4_23_to_6_24
                                 + 24 x_train_0_from_5_23_to_6_24
                                 + 25 x_train_0_from_4_24_to_6_25
                                 + 25 x_train_0_from_5_24_to_6_25
                                 + 26 x_train_0_from_4_25_to_6_26
                                 + 26 x_train_0_from_5_25_to_6_26
                                 + 27 x_train_0_from_4_26_to_6_27
                                 + 27 x_train_0_from_5_26_to_6_27
                                 + 28 x_train_0_from_4_27_to_6_28
                                 + 28 x_train_0_from_5_27_to_6_28
                                 + 29 x_train_0_from_4_28_to_6_29
                                 + 29 x_train_0_from_5_28_to_6_29
                                 + 30 x_train_0_from_4_29_to_6_30
                                 + 30 x_train_0_from_5_29_to_6_30  = 0
 set_theta_train_1_segment_0:    theta_train_1_segment_0
                                 - 4 x_train_1_from_0_4_to_tau
                                 - 5 x_train_1_from_0_5_to_tau
                                 - 6 x_train_1_from_0_6_to_tau
                                 - 7 x_train_1_from_0_7_to_tau
                                 - 8 x_train_1_from_0_8_to_tau
                                 - 9 x_train_1_from_0_9_to_tau
                                 - 10 x_train_1_from_0_10_to_tau
                                 - 11 x_train_1_from_0_11_to_tau
                                 - 12 x_train_1_from_0_12_to_tau
                                 - 13 x_train_1_from_0_13_to_tau
                                 - 14 x_train_1_from_0_14_to_tau
                                 - 15 x_train_1_from_0_15_to_tau
                                 - 16 x_train_1_from_0_16_to_tau
                                 - 17 x_train_1_from_0_17_to_tau
                                 - 18 x_train_1_from_0_18_to_tau
                                 - 19 x_train_1_from_0_19_to_tau
                                 - 20 x_train_1_from_0_20_to_tau
                                 - 21 x_train_1_from_0_21_to_tau
                                 - 22 x_train_1_from_0_22_to_tau
                                 - 23 x_train_1_from_0_23_to_tau
                                 - 24 x_train_1_from_0_24_to_tau
                                 - 25 x_train_1_from_0_25_to_tau
                                 - 26 x_train_1_from_0_26_to_tau
                                 - 27 x_train_1_from_0_27_to_tau
                                 - 28 x_train_1_from_0_28_to_tau
                                 - 29 x_train_1_from_0_29_to_tau
                                 - 30 x_train_1_from_0_30_to_tau
                                 + 4 x_train_1_from_2_3_to_0_4
                                 + 5 x_train_1_from_2_4_to_0_5
                                 + 6 x_train_1_from_2_5_to_0_6
                                 + 7 x_train_1_from_1_6_to_0_7
                                 + 7 x_train_1_from_2_6_to_0_7
                                 + 8 x_train_1_from_1_7_to_0_8
                                 + 8 x_train_1_from_2_7_to_0_8
                                 + 9 x_train_1_from_1_8_to_0_9
                                 + 9 x_train_1_from_2_8_to_0_9
                                 + 10 x_train_1_from_1_9_to_0_10
                                 + 10 x_train_1_from_2_9_to_0_10
                                 + 11 x_train_1_from_1_10_to_0_11
                                 + 11 x_train_1_from_2_10_to_0_11
                                 + 12 x_train_1_from_1_11_to_0_12
                                 + 12 x_train_1_from_2_11_to_0_12
                                 + 13 x_train_1_from_1_12_to_0_13
                                 + 13 x_train_1_from_2_12_to_0_13
                                 + 14 x_train_1_from_1_13_to_0_14
                                 + 14 x_train_1_from_2_13_to_0_14
                                 + 15 x_train_1_from_1_14_to_0_15
                                 + 15 x_train_1_from_2_14_to_0_15
                                 + 16 x_train_1_from_1_15_to_0_16
                                 + 16 x_train_1_from_2_15_to_0_16
                                 + 17 x_train_1_from_1_16_to_0_17
                                 + 17 x_train_1_from_2_16_to_0_17
                                 + 18 x_train_1_from_1_17_to_0_18
                                 + 18 x_train_1_from_2_17_to_0_18
                                 + 19 x_train_1_from_1_18_to_0_19
                                 + 19 x_train_1_from_2_18_to_0_19
                                 + 20 x_train_1_from_1_19_to_0_20
                                 + 20 x_train_1_from_2_19_to_0_20
                                 + 21 x_train_1_from_1_20_to_0_21
                                 + 21 x_train_1_from_2_20_to_0_21
                                 + 22 x_train_1_from_1_21_to_0_22
                                 + 22 x_train_1_from_2_21_to_0_22
                                 + 23 x_train_1_from_1_22_to_0_23
                                 + 23 x_train_1_from_2_22_to_0_23
                                 + 24 x_train_1_from_1_23_to_0_24
                                 + 24 x_train_1_from_2_23_to_0_24
                                 + 25 x_train_1_from_1_24_to_0_25
                                 + 25 x_train_1_from_2_24_to_0_25
                                 + 26 x_train_1_from_1_25_to_0_26
                                 + 26 x_train_1_from_2_25_to_0_26
                                 + 27 x_train_1_from_1_26_to_0_27
                                 + 27 x_train_1_from_2_26_to_0_27
                                 + 28 x_train_1_from_1_27_to_0_28
                                 + 28 x_train_1_from_2_27_to_0_28
                                 + 29 x_train_1_from_1_28_to_0_29
                                 + 29 x_train_1_from_2_28_to_0_29
                                 + 30 x_train_1_from_1_29_to_0_30
                                 + 30 x_train_1_from_2_29_to_0_30  = 0
 set_theta_train_1_segment_1:    theta_train_1_segment_1
                                 - 6 x_train_1_from_1_6_to_0_7
                                 - 7 x_train_1_from_1_7_to_0_8
                                 - 8 x_train_1_from_1_8_to_0_9
                                 - 9 x_train_1_from_1_9_to_0_10
                                 - 10 x_train_1_from_1_10_to_0_11
                                 - 11 x_train_1_from_1_11_to_0_12
                                 - 12 x_train_1_from_1_12_to_0_13
                                 - 13 x_train_1_from_1_13_to_0_14
                                 - 14 x_train_1_from_1_14_to_0_15
                                 - 15 x_train_1_from_1_15_to_0_16
                                 - 16 x_train_1_from_1_16_to_0_17
                                 - 17 x_train_1_from_1_17_to_0_18
                                 - 18 x_train_1_from_1_18_to_0_19
                                 - 19 x_train_1_from_1_19_to_0_20
                                 - 20 x_train_1_from_1_20_to_0_21
                                 - 21 x_train_1_from_1_21_to_0_22
                                 - 22 x_train_1_from_1_22_to_0_23
                                 - 23 x_train_1_from_1_23_to_0_24
                                 - 24 x_train_1_from_1_24_to_0_25
                                 - 25 x_train_1_from_1_25_to_0_26
                                 - 26 x_train_1_from_1_26_to_0_27
                                 - 27 x_train_1_from_1_27_to_0_28
                                 - 28 x_train_1_from_1_28_to_0_29
                                 - 29 x_train_1_from_1_29_to_0_30
                                 + 6 x_train_1_from_3_5_to_1_6
                                 + 7 x_train_1_from_3_6_to_1_7
                                 + 8 x_train_1_from_3_7_to_1_8
                                 + 9 x_train_1_from_3_8_to_1_9
                                 + 10 x_train_1_from_3_9_to_1_10
                                 + 11 x_train_1_from_3_10_to_1_11
                                 + 12 x_train_1_from_3_11_to_1_12
                                 + 13 x_train_1_from_3_12_to_1_13
                                 + 14 x_train_1_from_3_13_to_1_14
                                 + 15 x_train_1_from_3_14_to_1_15
                                 + 16 x_train_1_from_3_15_to_1_16
                                 + 17 x_train_1_from_3_16_to_1_17
                                 + 18 x_train_1_from_3_17_to_1_18
                                 + 19 x_train_1_from_3_18_to_1_19
                                 + 20 x_train_1_from_3_19_to_1_20
                                 + 21 x_train_1_from_3_20_to_1_21
                                 + 22 x_train_1_from_3_21_to_1_22
                                 + 23 x_train_1_from_3_22_to_1_23
                                 + 24 x_train_1_from_3_23_to_1_24
                                 + 25 x_train_1_from_3_24_to_1_25
                                 + 26 x_train_1_from_3_25_to_1_26
                                 + 27 x_train_1_from_3_26_to_1_27
                                 + 28 x_train_1_from_3_27_to_1_28
                                 + 29 x_train_1_from_3_28_to_1_29  = 0
 set_theta_train_1_segment_2:    theta_train_1_segment_2
                                 - 3 x_train_1_from_2_3_to_0_4
                                 - 4 x_train_1_from_2_4_to_0_5
                                 - 5 x_train_1_from_2_5_to_0_6
                                 - 6 x_train_1_from_2_6_to_0_7
                                 - 7 x_train_1_from_2_7_to_0_8
                                 - 8 x_train_1_from_2_8_to_0_9
                                 - 9 x_train_1_from_2_9_to_0_10
                                 - 10 x_train_1_from_2_10_to_0_11
                                 - 11 x_train_1_from_2_11_to_0_12
                                 - 12 x_train_1_from_2_12_to_0_13
                                 - 13 x_train_1_from_2_13_to_0_14
                                 - 14 x_train_1_from_2_14_to_0_15
                                 - 15 x_train_1_from_2_15_to_0_16
                                 - 16 x_train_1_from_2_16_to_0_17
                                 - 17 x_train_1_from_2_17_to_0_18
                                 - 18 x_train_1_from_2_18_to_0_19
                                 - 19 x_train_1_from_2_19_to_0_20
                                 - 20 x_train_1_from_2_20_to_0_21
                                 - 21 x_train_1_from_2_21_to_0_22
                                 - 22 x_train_1_from_2_22_to_0_23
                                 - 23 x_train_1_from_2_23_to_0_24
                                 - 24 x_train_1_from_2_24_to_0_25
                                 - 25 x_train_1_from_2_25_to_0_26
                                 - 26 x_train_1_from_2_26_to_0_27
                                 - 27 x_train_1_from_2_27_to_0_28
                                 - 28 x_train_1_from_2_28_to_0_29
                                 - 29 x_train_1_from_2_29_to_0_30
                                 + 3 x_train_1_from_4_2_to_2_3
                                 + 4 x_train_1_from_4_3_to_2_4
                                 + 5 x_train_1_from_4_4_to_2_5
                                 + 6 x_train_1_from_4_5_to_2_6
                                 + 7 x_train_1_from_4_6_to_2_7
                                 + 8 x_train_1_from_4_7_to_2_8
                                 + 9 x_train_1_from_4_8_to_2_9
                                 + 10 x_train_1_from_4_9_to_2_10
                                 + 11 x_train_1_from_4_10_to_2_11
                                 + 12 x_train_1_from_4_11_to_2_12
                                 + 13 x_train_1_from_4_12_to_2_13
                                 + 14 x_train_1_from_4_13_to_2_14
                                 + 15 x_train_1_from_4_14_to_2_15
                                 + 16 x_train_1_from_4_15_to_2_16
                                 + 17 x_train_1_from_4_16_to_2_17
                                 + 18 x_train_1_from_4_17_to_2_18
                                 + 19 x_train_1_from_4_18_to_2_19
                                 + 20 x_train_1_from_4_19_to_2_20
                                 + 21 x_train_1_from_4_20_to_2_21
                                 + 22 x_train_1_from_4_21_to_2_22
                                 + 23 x_train_1_from_4_22_to_2_23
                                 + 24 x_train_1_from_4_23_to_2_24
                                 + 25 x_train_1_from_4_24_to_2_25
                                 + 26 x_train_1_from_4_25_to_2_26
                                 + 27 x_train_1_from_4_26_to_2_27
                                 + 28 x_train_1_from_4_27_to_2_28
                                 + 29 x_train_1_from_4_28_to_2_29  = 0
 set_theta_train_1_segment_3:    theta_train_1_segment_3
                                 - 5 x_train_1_from_3_5_to_1_6
                                 - 6 x_train_1_from_3_6_to_1_7
                                 - 7 x_train_1_from_3_7_to_1_8
                                 - 8 x_train_1_from_3_8_to_1_9
                                 - 9 x_train_1_from_3_9_to_1_10
                                 - 10 x_train_1_from_3_10_to_1_11
                                 - 11 x_train_1_from_3_11_to_1_12
                                 - 12 x_train_1_from_3_12_to_1_13
                                 - 13 x_train_1_from_3_13_to_1_14
                                 - 14 x_train_1_from_3_14_to_1_15
                                 - 15 x_train_1_from_3_15_to_1_16
                                 - 16 x_train_1_from_3_16_to_1_17
                                 - 17 x_train_1_from_3_17_to_1_18
                                 - 18 x_train_1_from_3_18_to_1_19
                                 - 19 x_train_1_from_3_19_to_1_20
                                 - 20 x_train_1_from_3_20_to_1_21
                                 - 21 x_train_1_from_3_21_to_1_22
                                 - 22 x_train_1_from_3_22_to_1_23
                                 - 23 x_train_1_from_3_23_to_1_24
                                 - 24 x_train_1_from_3_24_to_1_25
                                 - 25 x_train_1_from_3_25_to_1_26
                                 - 26 x_train_1_from_3_26_to_1_27
                                 - 27 x_train_1_from_3_27_to_1_28
                                 - 28 x_train_1_from_3_28_to_1_29
                                 + 5 x_train_1_from_5_4_to_3_5
                                 + 6 x_train_1_from_5_5_to_3_6
                                 + 7 x_train_1_from_5_6_to_3_7
                                 + 8 x_train_1_from_5_7_to_3_8
                                 + 9 x_train_1_from_5_8_to_3_9
                                 + 10 x_train_1_from_5_9_to_3_10
                                 + 11 x_train_1_from_5_10_to_3_11
                                 + 12 x_train_1_from_5_11_to_3_12
                                 + 13 x_train_1_from_5_12_to_3_13
                                 + 14 x_train_1_from_5_13_to_3_14
                                 + 15 x_train_1_from_5_14_to_3_15
                                 + 16 x_train_1_from_5_15_to_3_16
                                 + 17 x_train_1_from_5_16_to_3_17
                                 + 18 x_train_1_from_5_17_to_3_18
                                 + 19 x_train_1_from_5_18_to_3_19
                                 + 20 x_train_1_from_5_19_to_3_20
                                 + 21 x_train_1_from_5_20_to_3_21
                                 + 22 x_train_1_from_5_21_to_3_22
                                 + 23 x_train_1_from_5_22_to_3_23
                                 + 24 x_train_1_from_5_23_to_3_24
                                 + 25 x_train_1_from_5_24_to_3_25
                                 + 26 x_train_1_from_5_25_to_3_26
                                 + 27 x_train_1_from_5_26_to_3_27
                                 + 28 x_train_1_from_5_27_to_3_28  = 0
 set_theta_train_1_segment_4:    theta_train_1_segment_4
                                 - 2 x_train_1_from_4_2_to_2_3
                                 - 3 x_train_1_from_4_3_to_2_4
                                 - 4 x_train_1_from_4_4_to_2_5
                                 - 5 x_train_1_from_4_5_to_2_6
                                 - 6 x_train_1_from_4_6_to_2_7
                                 - 7 x_train_1_from_4_7_to_2_8
                                 - 8 x_train_1_from_4_8_to_2_9
                                 - 9 x_train_1_from_4_9_to_2_10
                                 - 10 x_train_1_from_4_10_to_2_11
                                 - 11 x_train_1_from_4_11_to_2_12
                                 - 12 x_train_1_from_4_12_to_2_13
                                 - 13 x_train_1_from_4_13_to_2_14
                                 - 14 x_train_1_from_4_14_to_2_15
                                 - 15 x_train_1_from_4_15_to_2_16
                                 - 16 x_train_1_from_4_16_to_2_17
                                 - 17 x_train_1_from_4_17_to_2_18
                                 - 18 x_train_1_from_4_18_to_2_19
                                 - 19 x_train_1_from_4_19_to_2_20
                                 - 20 x_train_1_from_4_20_to_2_21
                                 - 21 x_train_1_from_4_21_to_2_22
                                 - 22 x_train_1_from_4_22_to_2_23
                                 - 23 x_train_1_from_4_23_to_2_24
                                 - 24 x_train_1_from_4_24_to_2_25
                                 - 25 x_train_1_from_4_25_to_2_26
                                 - 26 x_train_1_from_4_26_to_2_27
                                 - 27 x_train_1_from_4_27_to_2_28
                                 - 28 x_train_1_from_4_28_to_2_29
                                 + 2 x_train_1_from_6_1_to_4_2
                                 + 3 x_train_1_from_6_2_to_4_3
                                 + 4 x_train_1_from_6_3_to_4_4
                                 + 5 x_train_1_from_6_4_to_4_5
                                 + 6 x_train_1_from_6_5_to_4_6
                                 + 7 x_train_1_from_6_6_to_4_7
                                 + 8 x_train_1_from_6_7_to_4_8
                                 + 9 x_train_1_from_6_8_to_4_9
                                 + 10 x_train_1_from_6_9_to_4_10
                                 + 11 x_train_1_from_6_10_to_4_11
                                 + 12 x_train_1_from_6_11_to_4_12
                                 + 13 x_train_1_from_6_12_to_4_13
                                 + 14 x_train_1_from_6_13_to_4_14
                                 + 15 x_train_1_from_6_14_to_4_15
                                 + 16 x_train_1_from_6_15_to_4_16
                                 + 17 x_train_1_from_6_16_to_4_17
                                 + 18 x_train_1_from_6_17_to_4_18
                                 + 19 x_train_1_from_6_18_to_4_19
                                 + 20 x_train_1_from_6_19_to_4_20
                                 + 21 x_train_1_from_6_20_to_4_21
                                 + 22 x_train_1_from_6_21_to_4_22
                                 + 23 x_train_1_from_6_22_to_4_23
                                 + 24 x_train_1_from_6_23_to_4_24
                                 + 25 x_train_1_from_6_24_to_4_25
                                 + 26 x_train_1_from_6_25_to_4_26
                                 + 27 x_train_1_from_6_26_to_4_27
                                 + 28 x_train_1_from_6_27_to_4_28  = 0
 set_theta_train_1_segment_5:    theta_train_1_segment_5
                                 - 4 x_train_1_from_5_4_to_3_5
                                 - 5 x_train_1_from_5_5_to_3_6
                                 - 6 x_train_1_from_5_6_to_3_7
                                 - 7 x_train_1_from_5_7_to_3_8
                                 - 8 x_train_1_from_5_8_to_3_9
                                 - 9 x_train_1_from_5_9_to_3_10
                                 - 10 x_train_1_from_5_10_to_3_11
                                 - 11 x_train_1_from_5_11_to_3_12
                                 - 12 x_train_1_from_5_12_to_3_13
                                 - 13 x_train_1_from_5_13_to_3_14
                                 - 14 x_train_1_from_5_14_to_3_15
                                 - 15 x_train_1_from_5_15_to_3_16
                                 - 16 x_train_1_from_5_16_to_3_17
                                 - 17 x_train_1_from_5_17_to_3_18
                                 - 18 x_train_1_from_5_18_to_3_19
                                 - 19 x_train_1_from_5_19_to_3_20
                                 - 20 x_train_1_from_5_20_to_3_21
                                 - 21 x_train_1_from_5_21_to_3_22
                                 - 22 x_train_1_from_5_22_to_3_23
                                 - 23 x_train_1_from_5_23_to_3_24
                                 - 24 x_train_1_from_5_24_to_3_25
                                 - 25 x_train_1_from_5_25_to_3_26
                                 - 26 x_train_1_from_5_26_to_3_27
                                 - 27 x_train_1_from_5_27_to_3_28
                                 + 2 x_train_1_from_6_1_to_5_2
                                 + 3 x_train_1_from_6_2_to_5_3
                                 + 4 x_train_1_from_6_3_to_5_4
                                 + 5 x_train_1_from_6_4_to_5_5
                                 + 6 x_train_1_from_6_5_to_5_6
                                 + 7 x_train_1_from_6_6_to_5_7
                                 + 8 x_train_1_from_6_7_to_5_8
                                 + 9 x_train_1_from_6_8_to_5_9
                                 + 10 x_train_1_from_6_9_to_5_10
                                 + 11 x_train_1_from_6_10_to_5_11
                                 + 12 x_train_1_from_6_11_to_5_12
                                 + 13 x_train_1_from_6_12_to_5_13
                                 + 14 x_train_1_from_6_13_to_5_14
                                 + 15 x_train_1_from_6_14_to_5_15
                                 + 16 x_train_1_from_6_15_to_5_16
                                 + 17 x_train_1_from_6_16_to_5_17
                                 + 18 x_train_1_from_6_17_to_5_18
                                 + 19 x_train_1_from_6_18_to_5_19
                                 + 20 x_train_1_from_6_19_to_5_20
                                 + 21 x_train_1_from_6_20_to_5_21
                                 + 22 x_train_1_from_6_21_to_5_22
                                 + 23 x_train_1_from_6_22_to_5_23
                                 + 24 x_train_1_from_6_23_to_5_24
                                 + 25 x_train_1_from_6_24_to_5_25
                                 + 26 x_train_1_from_6_25_to_5_26
                                 + 27 x_train_1_from_6_26_to_5_27  = 0
 set_theta_train_1_segment_6:    theta_train_1_segment_6
                                 + x_train_1_from_sigma_to_6_1
                                 + 2 x_train_1_from_sigma_to_6_2
                                 + 3 x_train_1_from_sigma_to_6_3
                                 + 4 x_train_1_from_sigma_to_6_4
                                 + 5 x_train_1_from_sigma_to_6_5
                                 + 6 x_train_1_from_sigma_to_6_6
                                 + 7 x_train_1_from_sigma_to_6_7
                                 + 8 x_train_1_from_sigma_to_6_8
                                 + 9 x_train_1_from_sigma_to_6_9
                                 + 10 x_train_1_from_sigma_to_6_10
                                 + 11 x_train_1_from_sigma_to_6_11
                                 + 12 x_train_1_from_sigma_to_6_12
                                 + 13 x_train_1_from_sigma_to_6_13
                                 + 14 x_train_1_from_sigma_to_6_14
                                 + 15 x_train_1_from_sigma_to_6_15
                                 + 16 x_train_1_from_sigma_to_6_16
                                 + 17 x_train_1_from_sigma_to_6_17
                                 + 18 x_train_1_from_sigma_to_6_18
                                 + 19 x_train_1_from_sigma_to_6_19
                                 + 20 x_train_1_from_sigma_to_6_20
                                 + 21 x_train_1_from_sigma_to_6_21
                                 + 22 x_train_1_from_sigma_to_6_22
                                 + 23 x_train_1_from_sigma_to_6_23
                                 + 24 x_train_1_from_sigma_to_6_24
                                 + 25 x_train_1_from_sigma_to_6_25
                                 + 26 x_train_1_from_sigma_to_6_26
                                 + 27 x_train_1_from_sigma_to_6_27
                                 - x_train_1_from_6_1_to_4_2
                                 - 2 x_train_1_from_6_2_to_4_3
                                 - 3 x_train_1_from_6_3_to_4_4
                                 - 4 x_train_1_from_6_4_to_4_5
                                 - 5 x_train_1_from_6_5_to_4_6
                                 - 6 x_train_1_from_6_6_to_4_7
                                 - 7 x_train_1_from_6_7_to_4_8
                                 - 8 x_train_1_from_6_8_to_4_9
                                 - 9 x_train_1_from_6_9_to_4_10
                                 - 10 x_train_1_from_6_10_to_4_11
                                 - 11 x_train_1_from_6_11_to_4_12
                                 - 12 x_train_1_from_6_12_to_4_13
                                 - 13 x_train_1_from_6_13_to_4_14
                                 - 14 x_train_1_from_6_14_to_4_15
                                 - 15 x_train_1_from_6_15_to_4_16
                                 - 16 x_train_1_from_6_16_to_4_17
                                 - 17 x_train_1_from_6_17_to_4_18
                                 - 18 x_train_1_from_6_18_to_4_19
                                 - 19 x_train_1_from_6_19_to_4_20
                                 - 20 x_train_1_from_6_20_to_4_21
                                 - 21 x_train_1_from_6_21_to_4_22
                                 - 22 x_train_1_from_6_22_to_4_23
                                 - 23 x_train_1_from_6_23_to_4_24
                                 - 24 x_train_1_from_6_24_to_4_25
                                 - 25 x_train_1_from_6_25_to_4_26
                                 - 26 x_train_1_from_6_26_to_4_27
                                 - 27 x_train_1_from_6_27_to_4_28
                                 - x_train_1_from_6_1_to_5_2
                                 - 2 x_train_1_from_6_2_to_5_3
                                 - 3 x_train_1_from_6_3_to_5_4
                                 - 4 x_train_1_from_6_4_to_5_5
                                 - 5 x_train_1_from_6_5_to_5_6
                                 - 6 x_train_1_from_6_6_to_5_7
                                 - 7 x_train_1_from_6_7_to_5_8
                                 - 8 x_train_1_from_6_8_to_5_9
                                 - 9 x_train_1_from_6_9_to_5_10
                                 - 10 x_train_1_from_6_10_to_5_11
                                 - 11 x_train_1_from_6_11_to_5_12
                                 - 12 x_train_1_from_6_12_to_5_13
                                 - 13 x_train_1_from_6_13_to_5_14
                                 - 14 x_train_1_from_6_14_to_5_15
                                 - 15 x_train_1_from_6_15_to_5_16
                                 - 16 x_train_1_from_6_16_to_5_17
                                 - 17 x_train_1_from_6_17_to_5_18
                                 - 18 x_train_1_from_6_18_to_5_19
                                 - 19 x_train_1_from_6_19_to_5_20
                                 - 20 x_train_1_from_6_20_to_5_21
                                 - 21 x_train_1_from_6_21_to_5_22
                                 - 22 x_train_1_from_6_22_to_5_23
                                 - 23 x_train_1_from_6_23_to_5_24
                                 - 24 x_train_1_from_6_24_to_5_25
                                 - 25 x_train_1_from_6_25_to_5_26
                                 - 26 x_train_1_from_6_26_to_5_27  = 0
 wt_1_train_0:                   - d_train_0 - 4 x_train_0_from_6_4_to_tau
                                 - 5 x_train_0_from_6_5_to_tau
                                 - 6 x_train_0_from_6_6_to_tau
                                 - 7 x_train_0_from_6_7_to_tau
                                 - 8 x_train_0_from_6_8_to_tau
                                 - 9 x_train_0_from_6_9_to_tau
                                 - 10 x_train_0_from_6_10_to_tau
                                 - 11 x_train_0_from_6_11_to_tau
                                 - 12 x_train_0_from_6_12_to_tau
                                 - 13 x_train_0_from_6_13_to_tau
                                 - 14 x_train_0_from_6_14_to_tau
                                 - 15 x_train_0_from_6_15_to_tau
                                 - 16 x_train_0_from_6_16_to_tau
                                 - 17 x_train_0_from_6_17_to_tau
                                 - 18 x_train_0_from_6_18_to_tau
                                 - 19 x_train_0_from_6_19_to_tau
                                 - 20 x_train_0_from_6_20_to_tau
                                 - 21 x_train_0_from_6_21_to_tau
                                 - 22 x_train_0_from_6_22_to_tau
                                 - 23 x_train_0_from_6_23_to_tau
                                 - 24 x_train_0_from_6_24_to_tau
                                 - 25 x_train_0_from_6_25_to_tau
                                 - 26 x_train_0_from_6_26_to_tau
                                 - 27 x_train_0_from_6_27_to_tau
                                 - 28 x_train_0_from_6_28_to_tau
                                 - 29 x_train_0_from_6_29_to_tau
                                 - 30 x_train_0_from_6_30_to_tau <= -26
 wt_1_train_1:                   - d_train_1 - 4 x_train_1_from_0_4_to_tau
                                 - 5 x_train_1_from_0_5_to_tau
                                 - 6 x_train_1_from_0_6_to_tau
                                 - 7 x_train_1_from_0_7_to_tau
                                 - 8 x_train_1_from_0_8_to_tau
                                 - 9 x_train_1_from_0_9_to_tau
                                 - 10 x_train_1_from_0_10_to_tau
                                 - 11 x_train_1_from_0_11_to_tau
                                 - 12 x_train_1_from_0_12_to_tau
                                 - 13 x_train_1_from_0_13_to_tau
                                 - 14 x_train_1_from_0_14_to_tau
                                 - 15 x_train_1_from_0_15_to_tau
                                 - 16 x_train_1_from_0_16_to_tau
                                 - 17 x_train_1_from_0_17_to_tau
                                 - 18 x_train_1_from_0_18_to_tau
                                 - 19 x_train_1_from_0_19_to_tau
                                 - 20 x_train_1_from_0_20_to_tau
                                 - 21 x_train_1_from_0_21_to_tau
                                 - 22 x_train_1_from_0_22_to_tau
                                 - 23 x_train_1_from_0_23_to_tau
                                 - 24 x_train_1_from_0_24_to_tau
                                 - 25 x_train_1_from_0_25_to_tau
                                 - 26 x_train_1_from_0_26_to_tau
                                 - 27 x_train_1_from_0_27_to_tau
                                 - 28 x_train_1_from_0_28_to_tau
                                 - 29 x_train_1_from_0_29_to_tau
                                 - 30 x_train_1_from_0_30_to_tau <= -24
 wt_2_train_0:                   - d_train_0 + 4 x_train_0_from_6_4_to_tau
                                 + 5 x_train_0_from_6_5_to_tau
                                 + 6 x_train_0_from_6_6_to_tau
                                 + 7 x_train_0_from_6_7_to_tau
                                 + 8 x_train_0_from_6_8_to_tau
                                 + 9 x_train_0_from_6_9_to_tau
                                 + 10 x_train_0_from_6_10_to_tau
                                 + 11 x_train_0_from_6_11_to_tau
                                 + 12 x_train_0_from_6_12_to_tau
                                 + 13 x_train_0_from_6_13_to_tau
                                 + 14 x_train_0_from_6_14_to_tau
                                 + 15 x_train_0_from_6_15_to_tau
                                 + 16 x_train_0_from_6_16_to_tau
                                 + 17 x_train_0_from_6_17_to_tau
                                 + 18 x_train_0_from_6_18_to_tau
                                 + 19 x_train_0_from_6_19_to_tau
                                 + 20 x_train_0_from_6_20_to_tau
                                 + 21 x_train_0_from_6_21_to_tau
                                 + 22 x_train_0_from_6_22_to_tau
                                 + 23 x_train_0_from_6_23_to_tau
                                 + 24 x_train_0_from_6_24_to_tau
                                 + 25 x_train_0_from_6_25_to_tau
                                 + 26 x_train_0_from_6_26_to_tau
                                 + 27 x_train_0_from_6_27_to_tau
                                 + 28 x_train_0_from_6_28_to_tau
                                 + 29 x_train_0_from_6_29_to_tau
                                 + 30 x_train_0_from_6_30_to_tau <= 38
 wt_2_train_1:                   - d_train_1 + 4 x_train_1_from_0_4_to_tau
                                 + 5 x_train_1_from_0_5_to_tau
                                 + 6 x_train_1_from_0_6_to_tau
                                 + 7 x_train_1_from_0_7_to_tau
                                 + 8 x_train_1_from_0_8_to_tau
                                 + 9 x_train_1_from_0_9_to_tau
                                 + 10 x_train_1_from_0_10_to_tau
                                 + 11 x_train_1_from_0_11_to_tau
                                 + 12 x_train_1_from_0_12_to_tau
                                 + 13 x_train_1_from_0_13_to_tau
                                 + 14 x_train_1_from_0_14_to_tau
                                 + 15 x_train_1_from_0_15_to_tau
                                 + 16 x_train_1_from_0_16_to_tau
                                 + 17 x_train_1_from_0_17_to_tau
                                 + 18 x_train_1_from_0_18_to_tau
                                 + 19 x_train_1_from_0_19_to_tau
                                 + 20 x_train_1_from_0_20_to_tau
                                 + 21 x_train_1_from_0_21_to_tau
                                 + 22 x_train_1_from_0_22_to_tau
                                 + 23 x_train_1_from_0_23_to_tau
                                 + 24 x_train_1_from_0_24_to_tau
                                 + 25 x_train_1_from_0_25_to_tau
                                 + 26 x_train_1_from_0_26_to_tau
                                 + 27 x_train_1_from_0_27_to_tau
                                 + 28 x_train_1_from_0_28_to_tau
                                 + 29 x_train_1_from_0_29_to_tau
                                 + 30 x_train_1_from_0_30_to_tau <= 36
 sa_delay_train_0_schedule_for_1: - _e_train_0_segment_1
                                 + 2 x_train_0_from_0_1_to_1_2
                                 + 3 x_train_0_from_0_2_to_1_3
                                 + 4 x_train_0_from_0_3_to_1_4
                                 + 5 x_train_0_from_0_4_to_1_5
                                 + 6 x_train_0_from_0_5_to_1_6
                                 + 7 x_train_0_from_0_6_to_1_7
                                 + 8 x_train_0_from_0_7_to_1_8
                                 + 9 x_train_0_from_0_8_to_1_9
                                 + 10 x_train_0_from_0_9_to_1_10
                                 + 11 x_train_0_from_0_10_to_1_11
                                 + 12 x_train_0_from_0_11_to_1_12
                                 + 13 x_train_0_from_0_12_to_1_13
                                 + 14 x_train_0_from_0_13_to_1_14
                                 + 15 x_train_0_from_0_14_to_1_15
                                 + 16 x_train_0_from_0_15_to_1_16
                                 + 17 x_train_0_from_0_16_to_1_17
                                 + 18 x_train_0_from_0_17_to_1_18
                                 + 19 x_train_0_from_0_18_to_1_19
                                 + 20 x_train_0_from_0_19_to_1_20
                                 + 21 x_train_0_from_0_20_to_1_21
                                 + 22 x_train_0_from_0_21_to_1_22
                                 + 23 x_train_0_from_0_22_to_1_23
                                 + 24 x_train_0_from_0_23_to_1_24
                                 + 25 x_train_0_from_0_24_to_1_25
                                 + 26 x_train_0_from_0_25_to_1_26
                                 + 27 x_train_0_from_0_26_to_1_27 <= 18
 min_time_train_0_segment_0:     z_train_0_segment_0 - theta_train_0_segment_0
                                 <= 0
 min_time_train_0_segment_1:     z_train_0_segment_1 - theta_train_0_segment_1
                                 <= 0
 min_time_train_0_segment_2:     z_train_0_segment_2 - theta_train_0_segment_2
                                 <= 0
 min_time_train_0_segment_3:     z_train_0_segment_3 - theta_train_0_segment_3
                                 <= 0
 min_time_train_0_segment_4:     z_train_0_segment_4 - theta_train_0_segment_4
                                 <= 0
 min_time_train_0_segment_5:     z_train_0_segment_5 - theta_train_0_segment_5
                                 <= 0
 min_time_train_0_segment_6:     2 z_train_0_segment_6
                                 - theta_train_0_segment_6 <= 0
 min_time_train_1_segment_0:     z_train_1_segment_0 - theta_train_1_segment_0
                                 <= 0
 min_time_train_1_segment_1:     z_train_1_segment_1 - theta_train_1_segment_1
                                 <= 0
 min_time_train_1_segment_2:     z_train_1_segment_2 - theta_train_1_segment_2
                                 <= 0
 min_time_train_1_segment_3:     z_train_1_segment_3 - theta_train_1_segment_3
                                 <= 0
 min_time_train_1_segment_4:     z_train_1_segment_4 - theta_train_1_segment_4
                                 <= 0
 min_time_train_1_segment_5:     z_train_1_segment_5 - theta_train_1_segment_5
                                 <= 0
 min_time_train_1_segment_6:     2 z_train_1_segment_6
                                 - theta_train_1_segment_6 <= 0
 headway_train_0_segment_0_time_1: 3 y_train_0_node_0_1 <= 3
 headway_train_0_segment_0_time_2: 3 y_train_0_node_0_2 <= 3
 headway_train_0_segment_0_time_3: 3 y_train_0_node_0_3 + y_train_1_node_0_4
                                 <= 3
 headway_train_0_segment_0_time_4: 3 y_train_0_node_0_4 + y_train_1_node_0_4
                                 + y_train_1_node_0_5 <= 3
 headway_train_0_segment_0_time_5: 3 y_train_0_node_0_5 + y_train_1_node_0_4
                                 + y_train_1_node_0_5 + y_train_1_node_0_6 <= 3
 headway_train_0_segment_0_time_6: 3 y_train_0_node_0_6 + y_train_1_node_0_5
                                 + y_train_1_node_0_6 + y_train_1_node_0_7 <= 3
 headway_train_0_segment_0_time_7: 3 y_train_0_node_0_7 + y_train_1_node_0_6
                                 + y_train_1_node_0_7 + y_train_1_node_0_8 <= 3
 headway_train_0_segment_0_time_8: 3 y_train_0_node_0_8 + y_train_1_node_0_7
                                 + y_train_1_node_0_8 + y_train_1_node_0_9 <= 3
 headway_train_0_segment_0_time_9: 3 y_train_0_node_0_9 + y_train_1_node_0_8
                                 + y_train_1_node_0_9 + y_train_1_node_0_10
                                 <= 3
 headway_train_0_segment_0_time_10: 3 y_train_0_node_0_10 + y_train_1_node_0_9
                                 + y_train_1_node_0_10 + y_train_1_node_0_11
                                 <= 3
 headway_train_0_segment_0_time_11: 3 y_train_0_node_0_11 + y_train_1_node_0_10
                                 + y_train_1_node_0_11 + y_train_1_node_0_12
                                 <= 3
 headway_train_0_segment_0_time_12: 3 y_train_0_node_0_12 + y_train_1_node_0_11
                                 + y_train_1_node_0_12 + y_train_1_node_0_13
                                 <= 3
 headway_train_0_segment_0_time_13: 3 y_train_0_node_0_13 + y_train_1_node_0_12
                                 + y_train_1_node_0_13 + y_train_1_node_0_14
                                 <= 3
 headway_train_0_segment_0_time_14: 3 y_train_0_node_0_14 + y_train_1_node_0_13
                                 + y_train_1_node_0_14 + y_train_1_node_0_15
                                 <= 3
 headway_train_0_segment_0_time_15: 3 y_train_0_node_0_15 + y_train_1_node_0_14
                                 + y_train_1_node_0_15 + y_train_1_node_0_16
                                 <= 3
 headway_train_0_segment_0_time_16: 3 y_train_0_node_0_16 + y_train_1_node_0_15
                                 + y_train_1_node_0_16 + y_train_1_node_0_17
                                 <= 3
 headway_train_0_segment_0_time_17: 3 y_train_0_node_0_17 + y_train_1_node_0_16
                                 + y_train_1_node_0_17 + y_train_1_node_0_18
                                 <= 3
 headway_train_0_segment_0_time_18: 3 y_train_0_node_0_18 + y_train_1_node_0_17
                                 + y_train_1_node_0_18 + y_train_1_node_0_19
                                 <= 3
 headway_train_0_segment_0_time_19: 3 y_train_0_node_0_19 + y_train_1_node_0_18
                                 + y_train_1_node_0_19 + y_train_1_node_0_20
                                 <= 3
 headway_train_0_segment_0_time_20: 3 y_train_0_node_0_20 + y_train_1_node_0_19
                                 + y_train_1_node_0_20 + y_train_1_node_0_21
                                 <= 3
 headway_train_0_segment_0_time_21: 3 y_train_0_node_0_21 + y_train_1_node_0_20
                                 + y_train_1_node_0_21 + y_train_1_node_0_22
                                 <= 3
 headway_train_0_segment_0_time_22: 3 y_train_0_node_0_22 + y_train_1_node_0_21
                                 + y_train_1_node_0_22 + y_train_1_node_0_23
                                 <= 3
 headway_train_0_segment_0_time_23: 3 y_train_0_node_0_23 + y_train_1_node_0_22
                                 + y_train_1_node_0_23 + y_train_1_node_0_24
                                 <= 3
 headway_train_0_segment_0_time_24: 3 y_train_0_node_0_24 + y_train_1_node_0_23
                                 + y_train_1_node_0_24 + y_train_1_node_0_25
                                 <= 3
 headway_train_0_segment_0_time_25: 3 y_train_0_node_0_25 + y_train_1_node_0_24
                                 + y_train_1_node_0_25 + y_train_1_node_0_26
                                 <= 3
 headway_train_0_segment_0_time_26: 3 y_train_0_node_0_26 + y_train_1_node_0_25
                                 + y_train_1_node_0_26 + y_train_1_node_0_27
                                 <= 3
 headway_train_0_segment_0_time_27: 3 y_train_0_node_0_27 + y_train_1_node_0_26
                                 + y_train_1_node_0_27 + y_train_1_node_0_28
                                 <= 3
 headway_train_0_segment_1_time_2: 3 y_train_0_node_1_2 <= 3
 headway_train_0_segment_1_time_3: 3 y_train_0_node_1_3 <= 3
 headway_train_0_segment_1_time_4: 3 y_train_0_node_1_4 <= 3
 headway_train_0_segment_1_time_5: 3 y_train_0_node_1_5 + y_train_1_node_1_6
                                 <= 3
 headway_train_0_segment_1_time_6: 3 y_train_0_node_1_6 + y_train_1_node_1_6
                                 + y_train_1_node_1_7 <= 3
 headway_train_0_segment_1_time_7: 3 y_train_0_node_1_7 + y_train_1_node_1_6
                                 + y_train_1_node_1_7 + y_train_1_node_1_8 <= 3
 headway_train_0_segment_1_time_8: 3 y_train_0_node_1_8 + y_train_1_node_1_7
                                 + y_train_1_node_1_8 + y_train_1_node_1_9 <= 3
 headway_train_0_segment_1_time_9: 3 y_train_0_node_1_9 + y_train_1_node_1_8
                                 + y_train_1_node_1_9 + y_train_1_node_1_10
                                 <= 3
 headway_train_0_segment_1_time_10: 3 y_train_0_node_1_10 + y_train_1_node_1_9
                                 + y_train_1_node_1_10 + y_train_1_node_1_11
                                 <= 3
 headway_train_0_segment_1_time_11: 3 y_train_0_node_1_11 + y_train_1_node_1_10
                                 + y_train_1_node_1_11 + y_train_1_node_1_12
                                 <= 3
 headway_train_0_segment_1_time_12: 3 y_train_0_node_1_12 + y_train_1_node_1_11
                                 + y_train_1_node_1_12 + y_train_1_node_1_13
                                 <= 3
 headway_train_0_segment_1_time_13: 3 y_train_0_node_1_13 + y_train_1_node_1_12
                                 + y_train_1_node_1_13 + y_train_1_node_1_14
                                 <= 3
 headway_train_0_segment_1_time_14: 3 y_train_0_node_1_14 + y_train_1_node_1_13
                                 + y_train_1_node_1_14 + y_train_1_node_1_15
                                 <= 3
 headway_train_0_segment_1_time_15: 3 y_train_0_node_1_15 + y_train_1_node_1_14
                                 + y_train_1_node_1_15 + y_train_1_node_1_16
                                 <= 3
 headway_train_0_segment_1_time_16: 3 y_train_0_node_1_16 + y_train_1_node_1_15
                                 + y_train_1_node_1_16 + y_train_1_node_1_17
                                 <= 3
 headway_train_0_segment_1_time_17: 3 y_train_0_node_1_17 + y_train_1_node_1_16
                                 + y_train_1_node_1_17 + y_train_1_node_1_18
                                 <= 3
 headway_train_0_segment_1_time_18: 3 y_train_0_node_1_18 + y_train_1_node_1_17
                                 + y_train_1_node_1_18 + y_train_1_node_1_19
                                 <= 3
 headway_train_0_segment_1_time_19: 3 y_train_0_node_1_19 + y_train_1_node_1_18
                                 + y_train_1_node_1_19 + y_train_1_node_1_20
                                 <= 3
 headway_train_0_segment_1_time_20: 3 y_train_0_node_1_20 + y_train_1_node_1_19
                                 + y_train_1_node_1_20 + y_train_1_node_1_21
                                 <= 3
 headway_train_0_segment_1_time_21: 3 y_train_0_node_1_21 + y_train_1_node_1_20
                                 + y_train_1_node_1_21 + y_train_1_node_1_22
                                 <= 3
 headway_train_0_segment_1_time_22: 3 y_train_0_node_1_22 + y_train_1_node_1_21
                                 + y_train_1_node_1_22 + y_train_1_node_1_23
                                 <= 3
 headway_train_0_segment_1_time_23: 3 y_train_0_node_1_23 + y_train_1_node_1_22
                                 + y_train_1_node_1_23 + y_train_1_node_1_24
                                 <= 3
 headway_train_0_segment_1_time_24: 3 y_train_0_node_1_24 + y_train_1_node_1_23
                                 + y_train_1_node_1_24 + y_train_1_node_1_25
                                 <= 3
 headway_train_0_segment_1_time_25: 3 y_train_0_node_1_25 + y_train_1_node_1_24
                                 + y_train_1_node_1_25 + y_train_1_node_1_26
                                 <= 3
 headway_train_0_segment_1_time_26: 3 y_train_0_node_1_26 + y_train_1_node_1_25
                                 + y_train_1_node_1_26 + y_train_1_node_1_27
                                 <= 3
 headway_train_0_segment_1_time_27: 3 y_train_0_node_1_27 + y_train_1_node_1_26
                                 + y_train_1_node_1_27 + y_train_1_node_1_28
                                 <= 3
 headway_train_0_segment_2_time_2: 3 y_train_0_node_2_2 + y_train_1_node_2_3
                                 <= 3
 headway_train_0_segment_2_time_3: 3 y_train_0_node_2_3 + y_train_1_node_2_3
                                 + y_train_1_node_2_4 <= 3
 headway_train_0_segment_2_time_4: 3 y_train_0_node_2_4 + y_train_1_node_2_3
                                 + y_train_1_node_2_4 + y_train_1_node_2_5 <= 3
 headway_train_0_segment_2_time_5: 3 y_train_0_node_2_5 + y_train_1_node_2_4
                                 + y_train_1_node_2_5 + y_train_1_node_2_6 <= 3
 headway_train_0_segment_2_time_6: 3 y_train_0_node_2_6 + y_train_1_node_2_5
                                 + y_train_1_node_2_6 + y_train_1_node_2_7 <= 3
 headway_train_0_segment_2_time_7: 3 y_train_0_node_2_7 + y_train_1_node_2_6
                                 + y_train_1_node_2_7 + y_train_1_node_2_8 <= 3
 headway_train_0_segment_2_time_8: 3 y_train_0_node_2_8 + y_train_1_node_2_7
                                 + y_train_1_node_2_8 + y_train_1_node_2_9 <= 3
 headway_train_0_segment_2_time_9: 3 y_train_0_node_2_9 + y_train_1_node_2_8
                                 + y_train_1_node_2_9 + y_train_1_node_2_10
                                 <= 3
 headway_train_0_segment_2_time_10: 3 y_train_0_node_2_10 + y_train_1_node_2_9
                                 + y_train_1_node_2_10 + y_train_1_node_2_11
                                 <= 3
 headway_train_0_segment_2_time_11: 3 y_train_0_node_2_11 + y_train_1_node_2_10
                                 + y_train_1_node_2_11 + y_train_1_node_2_12
                                 <= 3
 headway_train_0_segment_2_time_12: 3 y_train_0_node_2_12 + y_train_1_node_2_11
                                 + y_train_1_node_2_12 + y_train_1_node_2_13
                                 <= 3
 headway_train_0_segment_2_time_13: 3 y_train_0_node_2_13 + y_train_1_node_2_12
                                 + y_train_1_node_2_13 + y_train_1_node_2_14
                                 <= 3
 headway_train_0_segment_2_time_14: 3 y_train_0_node_2_14 + y_train_1_node_2_13
                                 + y_train_1_node_2_14 + y_train_1_node_2_15
                                 <= 3
 headway_train_0_segment_2_time_15: 3 y_train_0_node_2_15 + y_train_1_node_2_14
                                 + y_train_1_node_2_15 + y_train_1_node_2_16
                                 <= 3
 headway_train_0_segment_2_time_16: 3 y_train_0_node_2_16 + y_train_1_node_2_15
                                 + y_train_1_node_2_16 + y_train_1_node_2_17
                                 <= 3
 headway_train_0_segment_2_time_17: 3 y_train_0_node_2_17 + y_train_1_node_2_16
                                 + y_train_1_node_2_17 + y_train_1_node_2_18
                                 <= 3
 headway_train_0_segment_2_time_18: 3 y_train_0_node_2_18 + y_train_1_node_2_17
                                 + y_train_1_node_2_18 + y_train_1_node_2_19
                                 <= 3
 headway_train_0_segment_2_time_19: 3 y_train_0_node_2_19 + y_train_1_node_2_18
                                 + y_train_1_node_2_19 + y_train_1_node_2_20
                                 <= 3
 headway_train_0_segment_2_time_20: 3 y_train_0_node_2_20 + y_train_1_node_2_19
                                 + y_train_1_node_2_20 + y_train_1_node_2_21
                                 <= 3
 headway_train_0_segment_2_time_21: 3 y_train_0_node_2_21 + y_train_1_node_2_20
                                 + y_train_1_node_2_21 + y_train_1_node_2_22
                                 <= 3
 headway_train_0_segment_2_time_22: 3 y_train_0_node_2_22 + y_train_1_node_2_21
                                 + y_train_1_node_2_22 + y_train_1_node_2_23
                                 <= 3
 headway_train_0_segment_2_time_23: 3 y_train_0_node_2_23 + y_train_1_node_2_22
                                 + y_train_1_node_2_23 + y_train_1_node_2_24
                                 <= 3
 headway_train_0_segment_2_time_24: 3 y_train_0_node_2_24 + y_train_1_node_2_23
                                 + y_train_1_node_2_24 + y_train_1_node_2_25
                                 <= 3
 headway_train_0_segment_2_time_25: 3 y_train_0_node_2_25 + y_train_1_node_2_24
                                 + y_train_1_node_2_25 + y_train_1_node_2_26
                                 <= 3
 headway_train_0_segment_2_time_26: 3 y_train_0_node_2_26 + y_train_1_node_2_25
                                 + y_train_1_node_2_26 + y_train_1_node_2_27
                                 <= 3
 headway_train_0_segment_2_time_27: 3 y_train_0_node_2_27 + y_train_1_node_2_26
                                 + y_train_1_node_2_27 + y_train_1_node_2_28
                                 <= 3
 headway_train_0_segment_2_time_28: 3 y_train_0_node_2_28 + y_train_1_node_2_27
                                 + y_train_1_node_2_28 + y_train_1_node_2_29
                                 <= 3
 headway_train_0_segment_3_time_5: 3 y_train_0_node_3_5 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 <= 3
 headway_train_0_segment_3_time_6: 3 y_train_0_node_3_6 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7 <= 3
 headway_train_0_segment_3_time_7: 3 y_train_0_node_3_7 + y_train_1_node_3_6
                                 + y_train_1_node_3_7 + y_train_1_node_3_8 <= 3
 headway_train_0_segment_3_time_8: 3 y_train_0_node_3_8 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9 <= 3
 headway_train_0_segment_3_time_9: 3 y_train_0_node_3_9 + y_train_1_node_3_8
                                 + y_train_1_node_3_9 + y_train_1_node_3_10
                                 <= 3
 headway_train_0_segment_3_time_10: 3 y_train_0_node_3_10 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 <= 3
 headway_train_0_segment_3_time_11: 3 y_train_0_node_3_11 + y_train_1_node_3_10
                                 + y_train_1_node_3_11 + y_train_1_node_3_12
                                 <= 3
 headway_train_0_segment_3_time_12: 3 y_train_0_node_3_12 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 <= 3
 headway_train_0_segment_3_time_13: 3 y_train_0_node_3_13 + y_train_1_node_3_12
                                 + y_train_1_node_3_13 + y_train_1_node_3_14
                                 <= 3
 headway_train_0_segment_3_time_14: 3 y_train_0_node_3_14 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 <= 3
 headway_train_0_segment_3_time_15: 3 y_train_0_node_3_15 + y_train_1_node_3_14
                                 + y_train_1_node_3_15 + y_train_1_node_3_16
                                 <= 3
 headway_train_0_segment_3_time_16: 3 y_train_0_node_3_16 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 <= 3
 headway_train_0_segment_3_time_17: 3 y_train_0_node_3_17 + y_train_1_node_3_16
                                 + y_train_1_node_3_17 + y_train_1_node_3_18
                                 <= 3
 headway_train_0_segment_3_time_18: 3 y_train_0_node_3_18 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 <= 3
 headway_train_0_segment_3_time_19: 3 y_train_0_node_3_19 + y_train_1_node_3_18
                                 + y_train_1_node_3_19 + y_train_1_node_3_20
                                 <= 3
 headway_train_0_segment_3_time_20: 3 y_train_0_node_3_20 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 <= 3
 headway_train_0_segment_3_time_21: 3 y_train_0_node_3_21 + y_train_1_node_3_20
                                 + y_train_1_node_3_21 + y_train_1_node_3_22
                                 <= 3
 headway_train_0_segment_3_time_22: 3 y_train_0_node_3_22 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 <= 3
 headway_train_0_segment_3_time_23: 3 y_train_0_node_3_23 + y_train_1_node_3_22
                                 + y_train_1_node_3_23 + y_train_1_node_3_24
                                 <= 3
 headway_train_0_segment_3_time_24: 3 y_train_0_node_3_24 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 <= 3
 headway_train_0_segment_3_time_25: 3 y_train_0_node_3_25 + y_train_1_node_3_24
                                 + y_train_1_node_3_25 + y_train_1_node_3_26
                                 <= 3
 headway_train_0_segment_3_time_26: 3 y_train_0_node_3_26 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 <= 3
 headway_train_0_segment_3_time_27: 3 y_train_0_node_3_27 + y_train_1_node_3_26
                                 + y_train_1_node_3_27 + y_train_1_node_3_28
                                 <= 3
 headway_train_0_segment_3_time_28: 3 y_train_0_node_3_28 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 3
 headway_train_0_segment_4_time_3: 3 y_train_0_node_4_3 + y_train_1_node_4_2
                                 + y_train_1_node_4_3 + y_train_1_node_4_4 <= 3
 headway_train_0_segment_4_time_4: 3 y_train_0_node_4_4 + y_train_1_node_4_3
                                 + y_train_1_node_4_4 + y_train_1_node_4_5 <= 3
 headway_train_0_segment_4_time_5: 3 y_train_0_node_4_5 + y_train_1_node_4_4
                                 + y_train_1_node_4_5 + y_train_1_node_4_6 <= 3
 headway_train_0_segment_4_time_6: 3 y_train_0_node_4_6 + y_train_1_node_4_5
                                 + y_train_1_node_4_6 + y_train_1_node_4_7 <= 3
 headway_train_0_segment_4_time_7: 3 y_train_0_node_4_7 + y_train_1_node_4_6
                                 + y_train_1_node_4_7 + y_train_1_node_4_8 <= 3
 headway_train_0_segment_4_time_8: 3 y_train_0_node_4_8 + y_train_1_node_4_7
                                 + y_train_1_node_4_8 + y_train_1_node_4_9 <= 3
 headway_train_0_segment_4_time_9: 3 y_train_0_node_4_9 + y_train_1_node_4_8
                                 + y_train_1_node_4_9 + y_train_1_node_4_10
                                 <= 3
 headway_train_0_segment_4_time_10: 3 y_train_0_node_4_10 + y_train_1_node_4_9
                                 + y_train_1_node_4_10 + y_train_1_node_4_11
                                 <= 3
 headway_train_0_segment_4_time_11: 3 y_train_0_node_4_11 + y_train_1_node_4_10
                                 + y_train_1_node_4_11 + y_train_1_node_4_12
                                 <= 3
 headway_train_0_segment_4_time_12: 3 y_train_0_node_4_12 + y_train_1_node_4_11
                                 + y_train_1_node_4_12 + y_train_1_node_4_13
                                 <= 3
 headway_train_0_segment_4_time_13: 3 y_train_0_node_4_13 + y_train_1_node_4_12
                                 + y_train_1_node_4_13 + y_train_1_node_4_14
                                 <= 3
 headway_train_0_segment_4_time_14: 3 y_train_0_node_4_14 + y_train_1_node_4_13
                                 + y_train_1_node_4_14 + y_train_1_node_4_15
                                 <= 3
 headway_train_0_segment_4_time_15: 3 y_train_0_node_4_15 + y_train_1_node_4_14
                                 + y_train_1_node_4_15 + y_train_1_node_4_16
                                 <= 3
 headway_train_0_segment_4_time_16: 3 y_train_0_node_4_16 + y_train_1_node_4_15
                                 + y_train_1_node_4_16 + y_train_1_node_4_17
                                 <= 3
 headway_train_0_segment_4_time_17: 3 y_train_0_node_4_17 + y_train_1_node_4_16
                                 + y_train_1_node_4_17 + y_train_1_node_4_18
                                 <= 3
 headway_train_0_segment_4_time_18: 3 y_train_0_node_4_18 + y_train_1_node_4_17
                                 + y_train_1_node_4_18 + y_train_1_node_4_19
                                 <= 3
 headway_train_0_segment_4_time_19: 3 y_train_0_node_4_19 + y_train_1_node_4_18
                                 + y_train_1_node_4_19 + y_train_1_node_4_20
                                 <= 3
 headway_train_0_segment_4_time_20: 3 y_train_0_node_4_20 + y_train_1_node_4_19
                                 + y_train_1_node_4_20 + y_train_1_node_4_21
                                 <= 3
 headway_train_0_segment_4_time_21: 3 y_train_0_node_4_21 + y_train_1_node_4_20
                                 + y_train_1_node_4_21 + y_train_1_node_4_22
                                 <= 3
 headway_train_0_segment_4_time_22: 3 y_train_0_node_4_22 + y_train_1_node_4_21
                                 + y_train_1_node_4_22 + y_train_1_node_4_23
                                 <= 3
 headway_train_0_segment_4_time_23: 3 y_train_0_node_4_23 + y_train_1_node_4_22
                                 + y_train_1_node_4_23 + y_train_1_node_4_24
                                 <= 3
 headway_train_0_segment_4_time_24: 3 y_train_0_node_4_24 + y_train_1_node_4_23
                                 + y_train_1_node_4_24 + y_train_1_node_4_25
                                 <= 3
 headway_train_0_segment_4_time_25: 3 y_train_0_node_4_25 + y_train_1_node_4_24
                                 + y_train_1_node_4_25 + y_train_1_node_4_26
                                 <= 3
 headway_train_0_segment_4_time_26: 3 y_train_0_node_4_26 + y_train_1_node_4_25
                                 + y_train_1_node_4_26 + y_train_1_node_4_27
                                 <= 3
 headway_train_0_segment_4_time_27: 3 y_train_0_node_4_27 + y_train_1_node_4_26
                                 + y_train_1_node_4_27 + y_train_1_node_4_28
                                 <= 3
 headway_train_0_segment_4_time_28: 3 y_train_0_node_4_28 + y_train_1_node_4_27
                                 + y_train_1_node_4_28 <= 3
 headway_train_0_segment_4_time_29: 3 y_train_0_node_4_29 + y_train_1_node_4_28
                                 <= 3
 headway_train_0_segment_5_time_6: 3 y_train_0_node_5_6 + y_train_1_node_5_5
                                 + y_train_1_node_5_6 + y_train_1_node_5_7 <= 3
 headway_train_0_segment_5_time_7: 3 y_train_0_node_5_7 + y_train_1_node_5_6
                                 + y_train_1_node_5_7 + y_train_1_node_5_8 <= 3
 headway_train_0_segment_5_time_8: 3 y_train_0_node_5_8 + y_train_1_node_5_7
                                 + y_train_1_node_5_8 + y_train_1_node_5_9 <= 3
 headway_train_0_segment_5_time_9: 3 y_train_0_node_5_9 + y_train_1_node_5_8
                                 + y_train_1_node_5_9 + y_train_1_node_5_10
                                 <= 3
 headway_train_0_segment_5_time_10: 3 y_train_0_node_5_10 + y_train_1_node_5_9
                                 + y_train_1_node_5_10 + y_train_1_node_5_11
                                 <= 3
 headway_train_0_segment_5_time_11: 3 y_train_0_node_5_11 + y_train_1_node_5_10
                                 + y_train_1_node_5_11 + y_train_1_node_5_12
                                 <= 3
 headway_train_0_segment_5_time_12: 3 y_train_0_node_5_12 + y_train_1_node_5_11
                                 + y_train_1_node_5_12 + y_train_1_node_5_13
                                 <= 3
 headway_train_0_segment_5_time_13: 3 y_train_0_node_5_13 + y_train_1_node_5_12
                                 + y_train_1_node_5_13 + y_train_1_node_5_14
                                 <= 3
 headway_train_0_segment_5_time_14: 3 y_train_0_node_5_14 + y_train_1_node_5_13
                                 + y_train_1_node_5_14 + y_train_1_node_5_15
                                 <= 3
 headway_train_0_segment_5_time_15: 3 y_train_0_node_5_15 + y_train_1_node_5_14
                                 + y_train_1_node_5_15 + y_train_1_node_5_16
                                 <= 3
 headway_train_0_segment_5_time_16: 3 y_train_0_node_5_16 + y_train_1_node_5_15
                                 + y_train_1_node_5_16 + y_train_1_node_5_17
                                 <= 3
 headway_train_0_segment_5_time_17: 3 y_train_0_node_5_17 + y_train_1_node_5_16
                                 + y_train_1_node_5_17 + y_train_1_node_5_18
                                 <= 3
 headway_train_0_segment_5_time_18: 3 y_train_0_node_5_18 + y_train_1_node_5_17
                                 + y_train_1_node_5_18 + y_train_1_node_5_19
                                 <= 3
 headway_train_0_segment_5_time_19: 3 y_train_0_node_5_19 + y_train_1_node_5_18
                                 + y_train_1_node_5_19 + y_train_1_node_5_20
                                 <= 3
 headway_train_0_segment_5_time_20: 3 y_train_0_node_5_20 + y_train_1_node_5_19
                                 + y_train_1_node_5_20 + y_train_1_node_5_21
                                 <= 3
 headway_train_0_segment_5_time_21: 3 y_train_0_node_5_21 + y_train_1_node_5_20
                                 + y_train_1_node_5_21 + y_train_1_node_5_22
                                 <= 3
 headway_train_0_segment_5_time_22: 3 y_train_0_node_5_22 + y_train_1_node_5_21
                                 + y_train_1_node_5_22 + y_train_1_node_5_23
                                 <= 3
 headway_train_0_segment_5_time_23: 3 y_train_0_node_5_23 + y_train_1_node_5_22
                                 + y_train_1_node_5_23 + y_train_1_node_5_24
                                 <= 3
 headway_train_0_segment_5_time_24: 3 y_train_0_node_5_24 + y_train_1_node_5_23
                                 + y_train_1_node_5_24 + y_train_1_node_5_25
                                 <= 3
 headway_train_0_segment_5_time_25: 3 y_train_0_node_5_25 + y_train_1_node_5_24
                                 + y_train_1_node_5_25 + y_train_1_node_5_26
                                 <= 3
 headway_train_0_segment_5_time_26: 3 y_train_0_node_5_26 + y_train_1_node_5_25
                                 + y_train_1_node_5_26 + y_train_1_node_5_27
                                 <= 3
 headway_train_0_segment_5_time_27: 3 y_train_0_node_5_27 + y_train_1_node_5_26
                                 + y_train_1_node_5_27 <= 3
 headway_train_0_segment_5_time_28: 3 y_train_0_node_5_28 + y_train_1_node_5_27
                                 <= 3
 headway_train_0_segment_5_time_29: 3 y_train_0_node_5_29 <= 3
 headway_train_0_segment_6_time_4: 3 y_train_0_node_6_4 + y_train_1_node_6_3
                                 + y_train_1_node_6_4 + y_train_1_node_6_5 <= 3
 headway_train_0_segment_6_time_5: 3 y_train_0_node_6_5 + y_train_1_node_6_4
                                 + y_train_1_node_6_5 + y_train_1_node_6_6 <= 3
 headway_train_0_segment_6_time_6: 3 y_train_0_node_6_6 + y_train_1_node_6_5
                                 + y_train_1_node_6_6 + y_train_1_node_6_7 <= 3
 headway_train_0_segment_6_time_7: 3 y_train_0_node_6_7 + y_train_1_node_6_6
                                 + y_train_1_node_6_7 + y_train_1_node_6_8 <= 3
 headway_train_0_segment_6_time_8: 3 y_train_0_node_6_8 + y_train_1_node_6_7
                                 + y_train_1_node_6_8 + y_train_1_node_6_9 <= 3
 headway_train_0_segment_6_time_9: 3 y_train_0_node_6_9 + y_train_1_node_6_8
                                 + y_train_1_node_6_9 + y_train_1_node_6_10
                                 <= 3
 headway_train_0_segment_6_time_10: 3 y_train_0_node_6_10 + y_train_1_node_6_9
                                 + y_train_1_node_6_10 + y_train_1_node_6_11
                                 <= 3
 headway_train_0_segment_6_time_11: 3 y_train_0_node_6_11 + y_train_1_node_6_10
                                 + y_train_1_node_6_11 + y_train_1_node_6_12
                                 <= 3
 headway_train_0_segment_6_time_12: 3 y_train_0_node_6_12 + y_train_1_node_6_11
                                 + y_train_1_node_6_12 + y_train_1_node_6_13
                                 <= 3
 headway_train_0_segment_6_time_13: 3 y_train_0_node_6_13 + y_train_1_node_6_12
                                 + y_train_1_node_6_13 + y_train_1_node_6_14
                                 <= 3
 headway_train_0_segment_6_time_14: 3 y_train_0_node_6_14 + y_train_1_node_6_13
                                 + y_train_1_node_6_14 + y_train_1_node_6_15
                                 <= 3
 headway_train_0_segment_6_time_15: 3 y_train_0_node_6_15 + y_train_1_node_6_14
                                 + y_train_1_node_6_15 + y_train_1_node_6_16
                                 <= 3
 headway_train_0_segment_6_time_16: 3 y_train_0_node_6_16 + y_train_1_node_6_15
                                 + y_train_1_node_6_16 + y_train_1_node_6_17
                                 <= 3
 headway_train_0_segment_6_time_17: 3 y_train_0_node_6_17 + y_train_1_node_6_16
                                 + y_train_1_node_6_17 + y_train_1_node_6_18
                                 <= 3
 headway_train_0_segment_6_time_18: 3 y_train_0_node_6_18 + y_train_1_node_6_17
                                 + y_train_1_node_6_18 + y_train_1_node_6_19
                                 <= 3
 headway_train_0_segment_6_time_19: 3 y_train_0_node_6_19 + y_train_1_node_6_18
                                 + y_train_1_node_6_19 + y_train_1_node_6_20
                                 <= 3
 headway_train_0_segment_6_time_20: 3 y_train_0_node_6_20 + y_train_1_node_6_19
                                 + y_train_1_node_6_20 + y_train_1_node_6_21
                                 <= 3
 headway_train_0_segment_6_time_21: 3 y_train_0_node_6_21 + y_train_1_node_6_20
                                 + y_train_1_node_6_21 + y_train_1_node_6_22
                                 <= 3
 headway_train_0_segment_6_time_22: 3 y_train_0_node_6_22 + y_train_1_node_6_21
                                 + y_train_1_node_6_22 + y_train_1_node_6_23
                                 <= 3
 headway_train_0_segment_6_time_23: 3 y_train_0_node_6_23 + y_train_1_node_6_22
                                 + y_train_1_node_6_23 + y_train_1_node_6_24
                                 <= 3
 headway_train_0_segment_6_time_24: 3 y_train_0_node_6_24 + y_train_1_node_6_23
                                 + y_train_1_node_6_24 + y_train_1_node_6_25
                                 <= 3
 headway_train_0_segment_6_time_25: 3 y_train_0_node_6_25 + y_train_1_node_6_24
                                 + y_train_1_node_6_25 + y_train_1_node_6_26
                                 <= 3
 headway_train_0_segment_6_time_26: 3 y_train_0_node_6_26 + y_train_1_node_6_25
                                 + y_train_1_node_6_26 + y_train_1_node_6_27
                                 <= 3
 headway_train_0_segment_6_time_27: 3 y_train_0_node_6_27 + y_train_1_node_6_26
                                 + y_train_1_node_6_27 <= 3
 headway_train_0_segment_6_time_28: 3 y_train_0_node_6_28 + y_train_1_node_6_27
                                 <= 3
 headway_train_0_segment_6_time_29: 3 y_train_0_node_6_29 <= 3
 headway_train_1_segment_0_time_4: y_train_0_node_0_3 + y_train_0_node_0_4
                                 + y_train_0_node_0_5 + 3 y_train_1_node_0_4
                                 <= 3
 headway_train_1_segment_0_time_5: y_train_0_node_0_4 + y_train_0_node_0_5
                                 + y_train_0_node_0_6 + 3 y_train_1_node_0_5
                                 <= 3
 headway_train_1_segment_0_time_6: y_train_0_node_0_5 + y_train_0_node_0_6
                                 + y_train_0_node_0_7 + 3 y_train_1_node_0_6
                                 <= 3
 headway_train_1_segment_0_time_7: y_train_0_node_0_6 + y_train_0_node_0_7
                                 + y_train_0_node_0_8 + 3 y_train_1_node_0_7
                                 <= 3
 headway_train_1_segment_0_time_8: y_train_0_node_0_7 + y_train_0_node_0_8
                                 + y_train_0_node_0_9 + 3 y_train_1_node_0_8
                                 <= 3
 headway_train_1_segment_0_time_9: y_train_0_node_0_8 + y_train_0_node_0_9
                                 + y_train_0_node_0_10 + 3 y_train_1_node_0_9
                                 <= 3
 headway_train_1_segment_0_time_10: y_train_0_node_0_9 + y_train_0_node_0_10
                                 + y_train_0_node_0_11 + 3 y_train_1_node_0_10
                                 <= 3
 headway_train_1_segment_0_time_11: y_train_0_node_0_10 + y_train_0_node_0_11
                                 + y_train_0_node_0_12 + 3 y_train_1_node_0_11
                                 <= 3
 headway_train_1_segment_0_time_12: y_train_0_node_0_11 + y_train_0_node_0_12
                                 + y_train_0_node_0_13 + 3 y_train_1_node_0_12
                                 <= 3
 headway_train_1_segment_0_time_13: y_train_0_node_0_12 + y_train_0_node_0_13
                                 + y_train_0_node_0_14 + 3 y_train_1_node_0_13
                                 <= 3
 headway_train_1_segment_0_time_14: y_train_0_node_0_13 + y_train_0_node_0_14
                                 + y_train_0_node_0_15 + 3 y_train_1_node_0_14
                                 <= 3
 headway_train_1_segment_0_time_15: y_train_0_node_0_14 + y_train_0_node_0_15
                                 + y_train_0_node_0_16 + 3 y_train_1_node_0_15
                                 <= 3
 headway_train_1_segment_0_time_16: y_train_0_node_0_15 + y_train_0_node_0_16
                                 + y_train_0_node_0_17 + 3 y_train_1_node_0_16
                                 <= 3
 headway_train_1_segment_0_time_17: y_train_0_node_0_16 + y_train_0_node_0_17
                                 + y_train_0_node_0_18 + 3 y_train_1_node_0_17
                                 <= 3
 headway_train_1_segment_0_time_18: y_train_0_node_0_17 + y_train_0_node_0_18
                                 + y_train_0_node_0_19 + 3 y_train_1_node_0_18
                                 <= 3
 headway_train_1_segment_0_time_19: y_train_0_node_0_18 + y_train_0_node_0_19
                                 + y_train_0_node_0_20 + 3 y_train_1_node_0_19
                                 <= 3
 headway_train_1_segment_0_time_20: y_train_0_node_0_19 + y_train_0_node_0_20
                                 + y_train_0_node_0_21 + 3 y_train_1_node_0_20
                                 <= 3
 headway_train_1_segment_0_time_21: y_train_0_node_0_20 + y_train_0_node_0_21
                                 + y_train_0_node_0_22 + 3 y_train_1_node_0_21
                                 <= 3
 headway_train_1_segment_0_time_22: y_train_0_node_0_21 + y_train_0_node_0_22
                                 + y_train_0_node_0_23 + 3 y_train_1_node_0_22
                                 <= 3
 headway_train_1_segment_0_time_23: y_train_0_node_0_22 + y_train_0_node_0_23
                                 + y_train_0_node_0_24 + 3 y_train_1_node_0_23
                                 <= 3
 headway_train_1_segment_0_time_24: y_train_0_node_0_23 + y_train_0_node_0_24
                                 + y_train_0_node_0_25 + 3 y_train_1_node_0_24
                                 <= 3
 headway_train_1_segment_0_time_25: y_train_0_node_0_24 + y_train_0_node_0_25
                                 + y_train_0_node_0_26 + 3 y_train_1_node_0_25
                                 <= 3
 headway_train_1_segment_0_time_26: y_train_0_node_0_25 + y_train_0_node_0_26
                                 + y_train_0_node_0_27 + 3 y_train_1_node_0_26
                                 <= 3
 headway_train_1_segment_0_time_27: y_train_0_node_0_26 + y_train_0_node_0_27
                                 + 3 y_train_1_node_0_27 <= 3
 headway_train_1_segment_0_time_28: y_train_0_node_0_27 + 3 y_train_1_node_0_28
                                 <= 3
 headway_train_1_segment_0_time_29: 3 y_train_1_node_0_29 <= 3
 headway_train_1_segment_1_time_6: y_train_0_node_1_5 + y_train_0_node_1_6
                                 + y_train_0_node_1_7 + 3 y_train_1_node_1_6
                                 <= 3
 headway_train_1_segment_1_time_7: y_train_0_node_1_6 + y_train_0_node_1_7
                                 + y_train_0_node_1_8 + 3 y_train_1_node_1_7
                                 <= 3
 headway_train_1_segment_1_time_8: y_train_0_node_1_7 + y_train_0_node_1_8
                                 + y_train_0_node_1_9 + 3 y_train_1_node_1_8
                                 <= 3
 headway_train_1_segment_1_time_9: y_train_0_node_1_8 + y_train_0_node_1_9
                                 + y_train_0_node_1_10 + 3 y_train_1_node_1_9
                                 <= 3
 headway_train_1_segment_1_time_10: y_train_0_node_1_9 + y_train_0_node_1_10
                                 + y_train_0_node_1_11 + 3 y_train_1_node_1_10
                                 <= 3
 headway_train_1_segment_1_time_11: y_train_0_node_1_10 + y_train_0_node_1_11
                                 + y_train_0_node_1_12 + 3 y_train_1_node_1_11
                                 <= 3
 headway_train_1_segment_1_time_12: y_train_0_node_1_11 + y_train_0_node_1_12
                                 + y_train_0_node_1_13 + 3 y_train_1_node_1_12
                                 <= 3
 headway_train_1_segment_1_time_13: y_train_0_node_1_12 + y_train_0_node_1_13
                                 + y_train_0_node_1_14 + 3 y_train_1_node_1_13
                                 <= 3
 headway_train_1_segment_1_time_14: y_train_0_node_1_13 + y_train_0_node_1_14
                                 + y_train_0_node_1_15 + 3 y_train_1_node_1_14
                                 <= 3
 headway_train_1_segment_1_time_15: y_train_0_node_1_14 + y_train_0_node_1_15
                                 + y_train_0_node_1_16 + 3 y_train_1_node_1_15
                                 <= 3
 headway_train_1_segment_1_time_16: y_train_0_node_1_15 + y_train_0_node_1_16
                                 + y_train_0_node_1_17 + 3 y_train_1_node_1_16
                                 <= 3
 headway_train_1_segment_1_time_17: y_train_0_node_1_16 + y_train_0_node_1_17
                                 + y_train_0_node_1_18 + 3 y_train_1_node_1_17
                                 <= 3
 headway_train_1_segment_1_time_18: y_train_0_node_1_17 + y_train_0_node_1_18
                                 + y_train_0_node_1_19 + 3 y_train_1_node_1_18
                                 <= 3
 headway_train_1_segment_1_time_19: y_train_0_node_1_18 + y_train_0_node_1_19
                                 + y_train_0_node_1_20 + 3 y_train_1_node_1_19
                                 <= 3
 headway_train_1_segment_1_time_20: y_train_0_node_1_19 + y_train_0_node_1_20
                                 + y_train_0_node_1_21 + 3 y_train_1_node_1_20
                                 <= 3
 headway_train_1_segment_1_time_21: y_train_0_node_1_20 + y_train_0_node_1_21
                                 + y_train_0_node_1_22 + 3 y_train_1_node_1_21
                                 <= 3
 headway_train_1_segment_1_time_22: y_train_0_node_1_21 + y_train_0_node_1_22
                                 + y_train_0_node_1_23 + 3 y_train_1_node_1_22
                                 <= 3
 headway_train_1_segment_1_time_23: y_train_0_node_1_22 + y_train_0_node_1_23
                                 + y_train_0_node_1_24 + 3 y_train_1_node_1_23
                                 <= 3
 headway_train_1_segment_1_time_24: y_train_0_node_1_23 + y_train_0_node_1_24
                                 + y_train_0_node_1_25 + 3 y_train_1_node_1_24
                                 <= 3
 headway_train_1_segment_1_time_25: y_train_0_node_1_24 + y_train_0_node_1_25
                                 + y_train_0_node_1_26 + 3 y_train_1_node_1_25
                                 <= 3
 headway_train_1_segment_1_time_26: y_train_0_node_1_25 + y_train_0_node_1_26
                                 + y_train_0_node_1_27 + 3 y_train_1_node_1_26
                                 <= 3
 headway_train_1_segment_1_time_27: y_train_0_node_1_26 + y_train_0_node_1_27
                                 + 3 y_train_1_node_1_27 <= 3
 headway_train_1_segment_1_time_28: y_train_0_node_1_27 + 3 y_train_1_node_1_28
                                 <= 3
 headway_train_1_segment_1_time_29: 3 y_train_1_node_1_29 <= 3
 headway_train_1_segment_2_time_3: y_train_0_node_2_2 + y_train_0_node_2_3
                                 + y_train_0_node_2_4 + 3 y_train_1_node_2_3
                                 <= 3
 headway_train_1_segment_2_time_4: y_train_0_node_2_3 + y_train_0_node_2_4
                                 + y_train_0_node_2_5 + 3 y_train_1_node_2_4
                                 <= 3
 headway_train_1_segment_2_time_5: y_train_0_node_2_4 + y_train_0_node_2_5
                                 + y_train_0_node_2_6 + 3 y_train_1_node_2_5
                                 <= 3
 headway_train_1_segment_2_time_6: y_train_0_node_2_5 + y_train_0_node_2_6
                                 + y_train_0_node_2_7 + 3 y_train_1_node_2_6
                                 <= 3
 headway_train_1_segment_2_time_7: y_train_0_node_2_6 + y_train_0_node_2_7
                                 + y_train_0_node_2_8 + 3 y_train_1_node_2_7
                                 <= 3
 headway_train_1_segment_2_time_8: y_train_0_node_2_7 + y_train_0_node_2_8
                                 + y_train_0_node_2_9 + 3 y_train_1_node_2_8
                                 <= 3
 headway_train_1_segment_2_time_9: y_train_0_node_2_8 + y_train_0_node_2_9
                                 + y_train_0_node_2_10 + 3 y_train_1_node_2_9
                                 <= 3
 headway_train_1_segment_2_time_10: y_train_0_node_2_9 + y_train_0_node_2_10
                                 + y_train_0_node_2_11 + 3 y_train_1_node_2_10
                                 <= 3
 headway_train_1_segment_2_time_11: y_train_0_node_2_10 + y_train_0_node_2_11
                                 + y_train_0_node_2_12 + 3 y_train_1_node_2_11
                                 <= 3
 headway_train_1_segment_2_time_12: y_train_0_node_2_11 + y_train_0_node_2_12
                                 + y_train_0_node_2_13 + 3 y_train_1_node_2_12
                                 <= 3
 headway_train_1_segment_2_time_13: y_train_0_node_2_12 + y_train_0_node_2_13
                                 + y_train_0_node_2_14 + 3 y_train_1_node_2_13
                                 <= 3
 headway_train_1_segment_2_time_14: y_train_0_node_2_13 + y_train_0_node_2_14
                                 + y_train_0_node_2_15 + 3 y_train_1_node_2_14
                                 <= 3
 headway_train_1_segment_2_time_15: y_train_0_node_2_14 + y_train_0_node_2_15
                                 + y_train_0_node_2_16 + 3 y_train_1_node_2_15
                                 <= 3
 headway_train_1_segment_2_time_16: y_train_0_node_2_15 + y_train_0_node_2_16
                                 + y_train_0_node_2_17 + 3 y_train_1_node_2_16
                                 <= 3
 headway_train_1_segment_2_time_17: y_train_0_node_2_16 + y_train_0_node_2_17
                                 + y_train_0_node_2_18 + 3 y_train_1_node_2_17
                                 <= 3
 headway_train_1_segment_2_time_18: y_train_0_node_2_17 + y_train_0_node_2_18
                                 + y_train_0_node_2_19 + 3 y_train_1_node_2_18
                                 <= 3
 headway_train_1_segment_2_time_19: y_train_0_node_2_18 + y_train_0_node_2_19
                                 + y_train_0_node_2_20 + 3 y_train_1_node_2_19
                                 <= 3
 headway_train_1_segment_2_time_20: y_train_0_node_2_19 + y_train_0_node_2_20
                                 + y_train_0_node_2_21 + 3 y_train_1_node_2_20
                                 <= 3
 headway_train_1_segment_2_time_21: y_train_0_node_2_20 + y_train_0_node_2_21
                                 + y_train_0_node_2_22 + 3 y_train_1_node_2_21
                                 <= 3
 headway_train_1_segment_2_time_22: y_train_0_node_2_21 + y_train_0_node_2_22
                                 + y_train_0_node_2_23 + 3 y_train_1_node_2_22
                                 <= 3
 headway_train_1_segment_2_time_23: y_train_0_node_2_22 + y_train_0_node_2_23
                                 + y_train_0_node_2_24 + 3 y_train_1_node_2_23
                                 <= 3
 headway_train_1_segment_2_time_24: y_train_0_node_2_23 + y_train_0_node_2_24
                                 + y_train_0_node_2_25 + 3 y_train_1_node_2_24
                                 <= 3
 headway_train_1_segment_2_time_25: y_train_0_node_2_24 + y_train_0_node_2_25
                                 + y_train_0_node_2_26 + 3 y_train_1_node_2_25
                                 <= 3
 headway_train_1_segment_2_time_26: y_train_0_node_2_25 + y_train_0_node_2_26
                                 + y_train_0_node_2_27 + 3 y_train_1_node_2_26
                                 <= 3
 headway_train_1_segment_2_time_27: y_train_0_node_2_26 + y_train_0_node_2_27
                                 + y_train_0_node_2_28 + 3 y_train_1_node_2_27
                                 <= 3
 headway_train_1_segment_2_time_28: y_train_0_node_2_27 + y_train_0_node_2_28
                                 + 3 y_train_1_node_2_28 <= 3
 headway_train_1_segment_2_time_29: y_train_0_node_2_28 + 3 y_train_1_node_2_29
                                 <= 3
 headway_train_1_segment_3_time_5: y_train_0_node_3_5 + y_train_0_node_3_6
                                 + 3 y_train_1_node_3_5 <= 3
 headway_train_1_segment_3_time_6: y_train_0_node_3_5 + y_train_0_node_3_6
                                 + y_train_0_node_3_7 + 3 y_train_1_node_3_6
                                 <= 3
 headway_train_1_segment_3_time_7: y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + 3 y_train_1_node_3_7
                                 <= 3
 headway_train_1_segment_3_time_8: y_train_0_node_3_7 + y_train_0_node_3_8
                                 + y_train_0_node_3_9 + 3 y_train_1_node_3_8
                                 <= 3
 headway_train_1_segment_3_time_9: y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + 3 y_train_1_node_3_9
                                 <= 3
 headway_train_1_segment_3_time_10: y_train_0_node_3_9 + y_train_0_node_3_10
                                 + y_train_0_node_3_11 + 3 y_train_1_node_3_10
                                 <= 3
 headway_train_1_segment_3_time_11: y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + 3 y_train_1_node_3_11
                                 <= 3
 headway_train_1_segment_3_time_12: y_train_0_node_3_11 + y_train_0_node_3_12
                                 + y_train_0_node_3_13 + 3 y_train_1_node_3_12
                                 <= 3
 headway_train_1_segment_3_time_13: y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + 3 y_train_1_node_3_13
                                 <= 3
 headway_train_1_segment_3_time_14: y_train_0_node_3_13 + y_train_0_node_3_14
                                 + y_train_0_node_3_15 + 3 y_train_1_node_3_14
                                 <= 3
 headway_train_1_segment_3_time_15: y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + 3 y_train_1_node_3_15
                                 <= 3
 headway_train_1_segment_3_time_16: y_train_0_node_3_15 + y_train_0_node_3_16
                                 + y_train_0_node_3_17 + 3 y_train_1_node_3_16
                                 <= 3
 headway_train_1_segment_3_time_17: y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + 3 y_train_1_node_3_17
                                 <= 3
 headway_train_1_segment_3_time_18: y_train_0_node_3_17 + y_train_0_node_3_18
                                 + y_train_0_node_3_19 + 3 y_train_1_node_3_18
                                 <= 3
 headway_train_1_segment_3_time_19: y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + 3 y_train_1_node_3_19
                                 <= 3
 headway_train_1_segment_3_time_20: y_train_0_node_3_19 + y_train_0_node_3_20
                                 + y_train_0_node_3_21 + 3 y_train_1_node_3_20
                                 <= 3
 headway_train_1_segment_3_time_21: y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + 3 y_train_1_node_3_21
                                 <= 3
 headway_train_1_segment_3_time_22: y_train_0_node_3_21 + y_train_0_node_3_22
                                 + y_train_0_node_3_23 + 3 y_train_1_node_3_22
                                 <= 3
 headway_train_1_segment_3_time_23: y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + 3 y_train_1_node_3_23
                                 <= 3
 headway_train_1_segment_3_time_24: y_train_0_node_3_23 + y_train_0_node_3_24
                                 + y_train_0_node_3_25 + 3 y_train_1_node_3_24
                                 <= 3
 headway_train_1_segment_3_time_25: y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + 3 y_train_1_node_3_25
                                 <= 3
 headway_train_1_segment_3_time_26: y_train_0_node_3_25 + y_train_0_node_3_26
                                 + y_train_0_node_3_27 + 3 y_train_1_node_3_26
                                 <= 3
 headway_train_1_segment_3_time_27: y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 + 3 y_train_1_node_3_27
                                 <= 3
 headway_train_1_segment_3_time_28: y_train_0_node_3_27 + y_train_0_node_3_28
                                 + 3 y_train_1_node_3_28 <= 3
 headway_train_1_segment_4_time_2: y_train_0_node_4_3 + 3 y_train_1_node_4_2
                                 <= 3
 headway_train_1_segment_4_time_3: y_train_0_node_4_3 + y_train_0_node_4_4
                                 + 3 y_train_1_node_4_3 <= 3
 headway_train_1_segment_4_time_4: y_train_0_node_4_3 + y_train_0_node_4_4
                                 + y_train_0_node_4_5 + 3 y_train_1_node_4_4
                                 <= 3
 headway_train_1_segment_4_time_5: y_train_0_node_4_4 + y_train_0_node_4_5
                                 + y_train_0_node_4_6 + 3 y_train_1_node_4_5
                                 <= 3
 headway_train_1_segment_4_time_6: y_train_0_node_4_5 + y_train_0_node_4_6
                                 + y_train_0_node_4_7 + 3 y_train_1_node_4_6
                                 <= 3
 headway_train_1_segment_4_time_7: y_train_0_node_4_6 + y_train_0_node_4_7
                                 + y_train_0_node_4_8 + 3 y_train_1_node_4_7
                                 <= 3
 headway_train_1_segment_4_time_8: y_train_0_node_4_7 + y_train_0_node_4_8
                                 + y_train_0_node_4_9 + 3 y_train_1_node_4_8
                                 <= 3
 headway_train_1_segment_4_time_9: y_train_0_node_4_8 + y_train_0_node_4_9
                                 + y_train_0_node_4_10 + 3 y_train_1_node_4_9
                                 <= 3
 headway_train_1_segment_4_time_10: y_train_0_node_4_9 + y_train_0_node_4_10
                                 + y_train_0_node_4_11 + 3 y_train_1_node_4_10
                                 <= 3
 headway_train_1_segment_4_time_11: y_train_0_node_4_10 + y_train_0_node_4_11
                                 + y_train_0_node_4_12 + 3 y_train_1_node_4_11
                                 <= 3
 headway_train_1_segment_4_time_12: y_train_0_node_4_11 + y_train_0_node_4_12
                                 + y_train_0_node_4_13 + 3 y_train_1_node_4_12
                                 <= 3
 headway_train_1_segment_4_time_13: y_train_0_node_4_12 + y_train_0_node_4_13
                                 + y_train_0_node_4_14 + 3 y_train_1_node_4_13
                                 <= 3
 headway_train_1_segment_4_time_14: y_train_0_node_4_13 + y_train_0_node_4_14
                                 + y_train_0_node_4_15 + 3 y_train_1_node_4_14
                                 <= 3
 headway_train_1_segment_4_time_15: y_train_0_node_4_14 + y_train_0_node_4_15
                                 + y_train_0_node_4_16 + 3 y_train_1_node_4_15
                                 <= 3
 headway_train_1_segment_4_time_16: y_train_0_node_4_15 + y_train_0_node_4_16
                                 + y_train_0_node_4_17 + 3 y_train_1_node_4_16
                                 <= 3
 headway_train_1_segment_4_time_17: y_train_0_node_4_16 + y_train_0_node_4_17
                                 + y_train_0_node_4_18 + 3 y_train_1_node_4_17
                                 <= 3
 headway_train_1_segment_4_time_18: y_train_0_node_4_17 + y_train_0_node_4_18
                                 + y_train_0_node_4_19 + 3 y_train_1_node_4_18
                                 <= 3
 headway_train_1_segment_4_time_19: y_train_0_node_4_18 + y_train_0_node_4_19
                                 + y_train_0_node_4_20 + 3 y_train_1_node_4_19
                                 <= 3
 headway_train_1_segment_4_time_20: y_train_0_node_4_19 + y_train_0_node_4_20
                                 + y_train_0_node_4_21 + 3 y_train_1_node_4_20
                                 <= 3
 headway_train_1_segment_4_time_21: y_train_0_node_4_20 + y_train_0_node_4_21
                                 + y_train_0_node_4_22 + 3 y_train_1_node_4_21
                                 <= 3
 headway_train_1_segment_4_time_22: y_train_0_node_4_21 + y_train_0_node_4_22
                                 + y_train_0_node_4_23 + 3 y_train_1_node_4_22
                                 <= 3
 headway_train_1_segment_4_time_23: y_train_0_node_4_22 + y_train_0_node_4_23
                                 + y_train_0_node_4_24 + 3 y_train_1_node_4_23
                                 <= 3
 headway_train_1_segment_4_time_24: y_train_0_node_4_23 + y_train_0_node_4_24
                                 + y_train_0_node_4_25 + 3 y_train_1_node_4_24
                                 <= 3
 headway_train_1_segment_4_time_25: y_train_0_node_4_24 + y_train_0_node_4_25
                                 + y_train_0_node_4_26 + 3 y_train_1_node_4_25
                                 <= 3
 headway_train_1_segment_4_time_26: y_train_0_node_4_25 + y_train_0_node_4_26
                                 + y_train_0_node_4_27 + 3 y_train_1_node_4_26
                                 <= 3
 headway_train_1_segment_4_time_27: y_train_0_node_4_26 + y_train_0_node_4_27
                                 + y_train_0_node_4_28 + 3 y_train_1_node_4_27
                                 <= 3
 headway_train_1_segment_4_time_28: y_train_0_node_4_27 + y_train_0_node_4_28
                                 + y_train_0_node_4_29 + 3 y_train_1_node_4_28
                                 <= 3
 headway_train_1_segment_5_time_2: 3 y_train_1_node_5_2 <= 3
 headway_train_1_segment_5_time_3: 3 y_train_1_node_5_3 <= 3
 headway_train_1_segment_5_time_4: 3 y_train_1_node_5_4 <= 3
 headway_train_1_segment_5_time_5: y_train_0_node_5_6 + 3 y_train_1_node_5_5
                                 <= 3
 headway_train_1_segment_5_time_6: y_train_0_node_5_6 + y_train_0_node_5_7
                                 + 3 y_train_1_node_5_6 <= 3
 headway_train_1_segment_5_time_7: y_train_0_node_5_6 + y_train_0_node_5_7
                                 + y_train_0_node_5_8 + 3 y_train_1_node_5_7
                                 <= 3
 headway_train_1_segment_5_time_8: y_train_0_node_5_7 + y_train_0_node_5_8
                                 + y_train_0_node_5_9 + 3 y_train_1_node_5_8
                                 <= 3
 headway_train_1_segment_5_time_9: y_train_0_node_5_8 + y_train_0_node_5_9
                                 + y_train_0_node_5_10 + 3 y_train_1_node_5_9
                                 <= 3
 headway_train_1_segment_5_time_10: y_train_0_node_5_9 + y_train_0_node_5_10
                                 + y_train_0_node_5_11 + 3 y_train_1_node_5_10
                                 <= 3
 headway_train_1_segment_5_time_11: y_train_0_node_5_10 + y_train_0_node_5_11
                                 + y_train_0_node_5_12 + 3 y_train_1_node_5_11
                                 <= 3
 headway_train_1_segment_5_time_12: y_train_0_node_5_11 + y_train_0_node_5_12
                                 + y_train_0_node_5_13 + 3 y_train_1_node_5_12
                                 <= 3
 headway_train_1_segment_5_time_13: y_train_0_node_5_12 + y_train_0_node_5_13
                                 + y_train_0_node_5_14 + 3 y_train_1_node_5_13
                                 <= 3
 headway_train_1_segment_5_time_14: y_train_0_node_5_13 + y_train_0_node_5_14
                                 + y_train_0_node_5_15 + 3 y_train_1_node_5_14
                                 <= 3
 headway_train_1_segment_5_time_15: y_train_0_node_5_14 + y_train_0_node_5_15
                                 + y_train_0_node_5_16 + 3 y_train_1_node_5_15
                                 <= 3
 headway_train_1_segment_5_time_16: y_train_0_node_5_15 + y_train_0_node_5_16
                                 + y_train_0_node_5_17 + 3 y_train_1_node_5_16
                                 <= 3
 headway_train_1_segment_5_time_17: y_train_0_node_5_16 + y_train_0_node_5_17
                                 + y_train_0_node_5_18 + 3 y_train_1_node_5_17
                                 <= 3
 headway_train_1_segment_5_time_18: y_train_0_node_5_17 + y_train_0_node_5_18
                                 + y_train_0_node_5_19 + 3 y_train_1_node_5_18
                                 <= 3
 headway_train_1_segment_5_time_19: y_train_0_node_5_18 + y_train_0_node_5_19
                                 + y_train_0_node_5_20 + 3 y_train_1_node_5_19
                                 <= 3
 headway_train_1_segment_5_time_20: y_train_0_node_5_19 + y_train_0_node_5_20
                                 + y_train_0_node_5_21 + 3 y_train_1_node_5_20
                                 <= 3
 headway_train_1_segment_5_time_21: y_train_0_node_5_20 + y_train_0_node_5_21
                                 + y_train_0_node_5_22 + 3 y_train_1_node_5_21
                                 <= 3
 headway_train_1_segment_5_time_22: y_train_0_node_5_21 + y_train_0_node_5_22
                                 + y_train_0_node_5_23 + 3 y_train_1_node_5_22
                                 <= 3
 headway_train_1_segment_5_time_23: y_train_0_node_5_22 + y_train_0_node_5_23
                                 + y_train_0_node_5_24 + 3 y_train_1_node_5_23
                                 <= 3
 headway_train_1_segment_5_time_24: y_train_0_node_5_23 + y_train_0_node_5_24
                                 + y_train_0_node_5_25 + 3 y_train_1_node_5_24
                                 <= 3
 headway_train_1_segment_5_time_25: y_train_0_node_5_24 + y_train_0_node_5_25
                                 + y_train_0_node_5_26 + 3 y_train_1_node_5_25
                                 <= 3
 headway_train_1_segment_5_time_26: y_train_0_node_5_25 + y_train_0_node_5_26
                                 + y_train_0_node_5_27 + 3 y_train_1_node_5_26
                                 <= 3
 headway_train_1_segment_5_time_27: y_train_0_node_5_26 + y_train_0_node_5_27
                                 + y_train_0_node_5_28 + 3 y_train_1_node_5_27
                                 <= 3
 headway_train_1_segment_6_time_1: 3 y_train_1_node_6_1 <= 3
 headway_train_1_segment_6_time_2: 3 y_train_1_node_6_2 <= 3
 headway_train_1_segment_6_time_3: y_train_0_node_6_4 + 3 y_train_1_node_6_3
                                 <= 3
 headway_train_1_segment_6_time_4: y_train_0_node_6_4 + y_train_0_node_6_5
                                 + 3 y_train_1_node_6_4 <= 3
 headway_train_1_segment_6_time_5: y_train_0_node_6_4 + y_train_0_node_6_5
                                 + y_train_0_node_6_6 + 3 y_train_1_node_6_5
                                 <= 3
 headway_train_1_segment_6_time_6: y_train_0_node_6_5 + y_train_0_node_6_6
                                 + y_train_0_node_6_7 + 3 y_train_1_node_6_6
                                 <= 3
 headway_train_1_segment_6_time_7: y_train_0_node_6_6 + y_train_0_node_6_7
                                 + y_train_0_node_6_8 + 3 y_train_1_node_6_7
                                 <= 3
 headway_train_1_segment_6_time_8: y_train_0_node_6_7 + y_train_0_node_6_8
                                 + y_train_0_node_6_9 + 3 y_train_1_node_6_8
                                 <= 3
 headway_train_1_segment_6_time_9: y_train_0_node_6_8 + y_train_0_node_6_9
                                 + y_train_0_node_6_10 + 3 y_train_1_node_6_9
                                 <= 3
 headway_train_1_segment_6_time_10: y_train_0_node_6_9 + y_train_0_node_6_10
                                 + y_train_0_node_6_11 + 3 y_train_1_node_6_10
                                 <= 3
 headway_train_1_segment_6_time_11: y_train_0_node_6_10 + y_train_0_node_6_11
                                 + y_train_0_node_6_12 + 3 y_train_1_node_6_11
                                 <= 3
 headway_train_1_segment_6_time_12: y_train_0_node_6_11 + y_train_0_node_6_12
                                 + y_train_0_node_6_13 + 3 y_train_1_node_6_12
                                 <= 3
 headway_train_1_segment_6_time_13: y_train_0_node_6_12 + y_train_0_node_6_13
                                 + y_train_0_node_6_14 + 3 y_train_1_node_6_13
                                 <= 3
 headway_train_1_segment_6_time_14: y_train_0_node_6_13 + y_train_0_node_6_14
                                 + y_train_0_node_6_15 + 3 y_train_1_node_6_14
                                 <= 3
 headway_train_1_segment_6_time_15: y_train_0_node_6_14 + y_train_0_node_6_15
                                 + y_train_0_node_6_16 + 3 y_train_1_node_6_15
                                 <= 3
 headway_train_1_segment_6_time_16: y_train_0_node_6_15 + y_train_0_node_6_16
                                 + y_train_0_node_6_17 + 3 y_train_1_node_6_16
                                 <= 3
 headway_train_1_segment_6_time_17: y_train_0_node_6_16 + y_train_0_node_6_17
                                 + y_train_0_node_6_18 + 3 y_train_1_node_6_17
                                 <= 3
 headway_train_1_segment_6_time_18: y_train_0_node_6_17 + y_train_0_node_6_18
                                 + y_train_0_node_6_19 + 3 y_train_1_node_6_18
                                 <= 3
 headway_train_1_segment_6_time_19: y_train_0_node_6_18 + y_train_0_node_6_19
                                 + y_train_0_node_6_20 + 3 y_train_1_node_6_19
                                 <= 3
 headway_train_1_segment_6_time_20: y_train_0_node_6_19 + y_train_0_node_6_20
                                 + y_train_0_node_6_21 + 3 y_train_1_node_6_20
                                 <= 3
 headway_train_1_segment_6_time_21: y_train_0_node_6_20 + y_train_0_node_6_21
                                 + y_train_0_node_6_22 + 3 y_train_1_node_6_21
                                 <= 3
 headway_train_1_segment_6_time_22: y_train_0_node_6_21 + y_train_0_node_6_22
                                 + y_train_0_node_6_23 + 3 y_train_1_node_6_22
                                 <= 3
 headway_train_1_segment_6_time_23: y_train_0_node_6_22 + y_train_0_node_6_23
                                 + y_train_0_node_6_24 + 3 y_train_1_node_6_23
                                 <= 3
 headway_train_1_segment_6_time_24: y_train_0_node_6_23 + y_train_0_node_6_24
                                 + y_train_0_node_6_25 + 3 y_train_1_node_6_24
                                 <= 3
 headway_train_1_segment_6_time_25: y_train_0_node_6_24 + y_train_0_node_6_25
                                 + y_train_0_node_6_26 + 3 y_train_1_node_6_25
                                 <= 3
 headway_train_1_segment_6_time_26: y_train_0_node_6_25 + y_train_0_node_6_26
                                 + y_train_0_node_6_27 + 3 y_train_1_node_6_26
                                 <= 3
 headway_train_1_segment_6_time_27: y_train_0_node_6_26 + y_train_0_node_6_27
                                 + y_train_0_node_6_28 + 3 y_train_1_node_6_27
                                 <= 3
 can_take_siding_train_0_segment_3_time_5: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_4
                                 - y_train_1_node_0_5 - y_train_1_node_0_6
                                 - y_train_1_node_2_4 - y_train_1_node_2_5
                                 - y_train_1_node_2_6 - y_train_1_node_4_4
                                 - y_train_1_node_4_5 - y_train_1_node_4_6
                                 - y_train_1_node_6_4 - y_train_1_node_6_5
                                 - y_train_1_node_6_6 <= 0
 can_take_siding_train_0_segment_3_time_6: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_5
                                 - y_train_1_node_0_6 - y_train_1_node_0_7
                                 - y_train_1_node_2_5 - y_train_1_node_2_6
                                 - y_train_1_node_2_7 - y_train_1_node_4_5
                                 - y_train_1_node_4_6 - y_train_1_node_4_7
                                 - y_train_1_node_6_5 - y_train_1_node_6_6
                                 - y_train_1_node_6_7 <= 0
 can_take_siding_train_0_segment_3_time_7: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_6
                                 - y_train_1_node_0_7 - y_train_1_node_0_8
                                 - y_train_1_node_2_6 - y_train_1_node_2_7
                                 - y_train_1_node_2_8 - y_train_1_node_4_6
                                 - y_train_1_node_4_7 - y_train_1_node_4_8
                                 - y_train_1_node_6_6 - y_train_1_node_6_7
                                 - y_train_1_node_6_8 <= 0
 can_take_siding_train_0_segment_3_time_8: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_7
                                 - y_train_1_node_0_8 - y_train_1_node_0_9
                                 - y_train_1_node_2_7 - y_train_1_node_2_8
                                 - y_train_1_node_2_9 - y_train_1_node_4_7
                                 - y_train_1_node_4_8 - y_train_1_node_4_9
                                 - y_train_1_node_6_7 - y_train_1_node_6_8
                                 - y_train_1_node_6_9 <= 0
 can_take_siding_train_0_segment_3_time_9: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_8
                                 - y_train_1_node_0_9 - y_train_1_node_0_10
                                 - y_train_1_node_2_8 - y_train_1_node_2_9
                                 - y_train_1_node_2_10 - y_train_1_node_4_8
                                 - y_train_1_node_4_9 - y_train_1_node_4_10
                                 - y_train_1_node_6_8 - y_train_1_node_6_9
                                 - y_train_1_node_6_10 <= 0
 can_take_siding_train_0_segment_3_time_10: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_9
                                 - y_train_1_node_0_10 - y_train_1_node_0_11
                                 - y_train_1_node_2_9 - y_train_1_node_2_10
                                 - y_train_1_node_2_11 - y_train_1_node_4_9
                                 - y_train_1_node_4_10 - y_train_1_node_4_11
                                 - y_train_1_node_6_9 - y_train_1_node_6_10
                                 - y_train_1_node_6_11 <= 0
 can_take_siding_train_0_segment_3_time_11: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_10
                                 - y_train_1_node_0_11 - y_train_1_node_0_12
                                 - y_train_1_node_2_10 - y_train_1_node_2_11
                                 - y_train_1_node_2_12 - y_train_1_node_4_10
                                 - y_train_1_node_4_11 - y_train_1_node_4_12
                                 - y_train_1_node_6_10 - y_train_1_node_6_11
                                 - y_train_1_node_6_12 <= 0
 can_take_siding_train_0_segment_3_time_12: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_11
                                 - y_train_1_node_0_12 - y_train_1_node_0_13
                                 - y_train_1_node_2_11 - y_train_1_node_2_12
                                 - y_train_1_node_2_13 - y_train_1_node_4_11
                                 - y_train_1_node_4_12 - y_train_1_node_4_13
                                 - y_train_1_node_6_11 - y_train_1_node_6_12
                                 - y_train_1_node_6_13 <= 0
 can_take_siding_train_0_segment_3_time_13: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_12
                                 - y_train_1_node_0_13 - y_train_1_node_0_14
                                 - y_train_1_node_2_12 - y_train_1_node_2_13
                                 - y_train_1_node_2_14 - y_train_1_node_4_12
                                 - y_train_1_node_4_13 - y_train_1_node_4_14
                                 - y_train_1_node_6_12 - y_train_1_node_6_13
                                 - y_train_1_node_6_14 <= 0
 can_take_siding_train_0_segment_3_time_14: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_13
                                 - y_train_1_node_0_14 - y_train_1_node_0_15
                                 - y_train_1_node_2_13 - y_train_1_node_2_14
                                 - y_train_1_node_2_15 - y_train_1_node_4_13
                                 - y_train_1_node_4_14 - y_train_1_node_4_15
                                 - y_train_1_node_6_13 - y_train_1_node_6_14
                                 - y_train_1_node_6_15 <= 0
 can_take_siding_train_0_segment_3_time_15: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_14
                                 - y_train_1_node_0_15 - y_train_1_node_0_16
                                 - y_train_1_node_2_14 - y_train_1_node_2_15
                                 - y_train_1_node_2_16 - y_train_1_node_4_14
                                 - y_train_1_node_4_15 - y_train_1_node_4_16
                                 - y_train_1_node_6_14 - y_train_1_node_6_15
                                 - y_train_1_node_6_16 <= 0
 can_take_siding_train_0_segment_3_time_16: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_15
                                 - y_train_1_node_0_16 - y_train_1_node_0_17
                                 - y_train_1_node_2_15 - y_train_1_node_2_16
                                 - y_train_1_node_2_17 - y_train_1_node_4_15
                                 - y_train_1_node_4_16 - y_train_1_node_4_17
                                 - y_train_1_node_6_15 - y_train_1_node_6_16
                                 - y_train_1_node_6_17 <= 0
 can_take_siding_train_0_segment_3_time_17: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_16
                                 - y_train_1_node_0_17 - y_train_1_node_0_18
                                 - y_train_1_node_2_16 - y_train_1_node_2_17
                                 - y_train_1_node_2_18 - y_train_1_node_4_16
                                 - y_train_1_node_4_17 - y_train_1_node_4_18
                                 - y_train_1_node_6_16 - y_train_1_node_6_17
                                 - y_train_1_node_6_18 <= 0
 can_take_siding_train_0_segment_3_time_18: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_17
                                 - y_train_1_node_0_18 - y_train_1_node_0_19
                                 - y_train_1_node_2_17 - y_train_1_node_2_18
                                 - y_train_1_node_2_19 - y_train_1_node_4_17
                                 - y_train_1_node_4_18 - y_train_1_node_4_19
                                 - y_train_1_node_6_17 - y_train_1_node_6_18
                                 - y_train_1_node_6_19 <= 0
 can_take_siding_train_0_segment_3_time_19: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_18
                                 - y_train_1_node_0_19 - y_train_1_node_0_20
                                 - y_train_1_node_2_18 - y_train_1_node_2_19
                                 - y_train_1_node_2_20 - y_train_1_node_4_18
                                 - y_train_1_node_4_19 - y_train_1_node_4_20
                                 - y_train_1_node_6_18 - y_train_1_node_6_19
                                 - y_train_1_node_6_20 <= 0
 can_take_siding_train_0_segment_3_time_20: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_19
                                 - y_train_1_node_0_20 - y_train_1_node_0_21
                                 - y_train_1_node_2_19 - y_train_1_node_2_20
                                 - y_train_1_node_2_21 - y_train_1_node_4_19
                                 - y_train_1_node_4_20 - y_train_1_node_4_21
                                 - y_train_1_node_6_19 - y_train_1_node_6_20
                                 - y_train_1_node_6_21 <= 0
 can_take_siding_train_0_segment_3_time_21: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_20
                                 - y_train_1_node_0_21 - y_train_1_node_0_22
                                 - y_train_1_node_2_20 - y_train_1_node_2_21
                                 - y_train_1_node_2_22 - y_train_1_node_4_20
                                 - y_train_1_node_4_21 - y_train_1_node_4_22
                                 - y_train_1_node_6_20 - y_train_1_node_6_21
                                 - y_train_1_node_6_22 <= 0
 can_take_siding_train_0_segment_3_time_22: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_21
                                 - y_train_1_node_0_22 - y_train_1_node_0_23
                                 - y_train_1_node_2_21 - y_train_1_node_2_22
                                 - y_train_1_node_2_23 - y_train_1_node_4_21
                                 - y_train_1_node_4_22 - y_train_1_node_4_23
                                 - y_train_1_node_6_21 - y_train_1_node_6_22
                                 - y_train_1_node_6_23 <= 0
 can_take_siding_train_0_segment_3_time_23: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_22
                                 - y_train_1_node_0_23 - y_train_1_node_0_24
                                 - y_train_1_node_2_22 - y_train_1_node_2_23
                                 - y_train_1_node_2_24 - y_train_1_node_4_22
                                 - y_train_1_node_4_23 - y_train_1_node_4_24
                                 - y_train_1_node_6_22 - y_train_1_node_6_23
                                 - y_train_1_node_6_24 <= 0
 can_take_siding_train_0_segment_3_time_24: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_23
                                 - y_train_1_node_0_24 - y_train_1_node_0_25
                                 - y_train_1_node_2_23 - y_train_1_node_2_24
                                 - y_train_1_node_2_25 - y_train_1_node_4_23
                                 - y_train_1_node_4_24 - y_train_1_node_4_25
                                 - y_train_1_node_6_23 - y_train_1_node_6_24
                                 - y_train_1_node_6_25 <= 0
 can_take_siding_train_0_segment_3_time_25: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_24
                                 - y_train_1_node_0_25 - y_train_1_node_0_26
                                 - y_train_1_node_2_24 - y_train_1_node_2_25
                                 - y_train_1_node_2_26 - y_train_1_node_4_24
                                 - y_train_1_node_4_25 - y_train_1_node_4_26
                                 - y_train_1_node_6_24 - y_train_1_node_6_25
                                 - y_train_1_node_6_26 <= 0
 can_take_siding_train_0_segment_3_time_26: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_25
                                 - y_train_1_node_0_26 - y_train_1_node_0_27
                                 - y_train_1_node_2_25 - y_train_1_node_2_26
                                 - y_train_1_node_2_27 - y_train_1_node_4_25
                                 - y_train_1_node_4_26 - y_train_1_node_4_27
                                 - y_train_1_node_6_25 - y_train_1_node_6_26
                                 - y_train_1_node_6_27 <= 0
 can_take_siding_train_0_segment_3_time_27: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_26
                                 - y_train_1_node_0_27 - y_train_1_node_0_28
                                 - y_train_1_node_2_26 - y_train_1_node_2_27
                                 - y_train_1_node_2_28 - y_train_1_node_4_26
                                 - y_train_1_node_4_27 - y_train_1_node_4_28
                                 - y_train_1_node_6_26 - y_train_1_node_6_27
                                 <= 0
 can_take_siding_train_0_segment_3_time_28: y_train_0_node_3_5
                                 + y_train_0_node_3_6 + y_train_0_node_3_7
                                 + y_train_0_node_3_8 + y_train_0_node_3_9
                                 + y_train_0_node_3_10 + y_train_0_node_3_11
                                 + y_train_0_node_3_12 + y_train_0_node_3_13
                                 + y_train_0_node_3_14 + y_train_0_node_3_15
                                 + y_train_0_node_3_16 + y_train_0_node_3_17
                                 + y_train_0_node_3_18 + y_train_0_node_3_19
                                 + y_train_0_node_3_20 + y_train_0_node_3_21
                                 + y_train_0_node_3_22 + y_train_0_node_3_23
                                 + y_train_0_node_3_24 + y_train_0_node_3_25
                                 + y_train_0_node_3_26 + y_train_0_node_3_27
                                 + y_train_0_node_3_28 - y_train_1_node_0_27
                                 - y_train_1_node_0_28 - y_train_1_node_0_29
                                 - y_train_1_node_2_27 - y_train_1_node_2_28
                                 - y_train_1_node_2_29 - y_train_1_node_4_27
                                 - y_train_1_node_4_28 - y_train_1_node_6_27
                                 <= 0
 can_take_siding_train_1_segment_3_time_5: - y_train_0_node_0_4
                                 - y_train_0_node_0_5 - y_train_0_node_0_6
                                 - y_train_0_node_2_4 - y_train_0_node_2_5
                                 - y_train_0_node_2_6 - y_train_0_node_4_4
                                 - y_train_0_node_4_5 - y_train_0_node_4_6
                                 - y_train_0_node_6_4 - y_train_0_node_6_5
                                 - y_train_0_node_6_6 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_6: - y_train_0_node_0_5
                                 - y_train_0_node_0_6 - y_train_0_node_0_7
                                 - y_train_0_node_2_5 - y_train_0_node_2_6
                                 - y_train_0_node_2_7 - y_train_0_node_4_5
                                 - y_train_0_node_4_6 - y_train_0_node_4_7
                                 - y_train_0_node_6_5 - y_train_0_node_6_6
                                 - y_train_0_node_6_7 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_7: - y_train_0_node_0_6
                                 - y_train_0_node_0_7 - y_train_0_node_0_8
                                 - y_train_0_node_2_6 - y_train_0_node_2_7
                                 - y_train_0_node_2_8 - y_train_0_node_4_6
                                 - y_train_0_node_4_7 - y_train_0_node_4_8
                                 - y_train_0_node_6_6 - y_train_0_node_6_7
                                 - y_train_0_node_6_8 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_8: - y_train_0_node_0_7
                                 - y_train_0_node_0_8 - y_train_0_node_0_9
                                 - y_train_0_node_2_7 - y_train_0_node_2_8
                                 - y_train_0_node_2_9 - y_train_0_node_4_7
                                 - y_train_0_node_4_8 - y_train_0_node_4_9
                                 - y_train_0_node_6_7 - y_train_0_node_6_8
                                 - y_train_0_node_6_9 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_9: - y_train_0_node_0_8
                                 - y_train_0_node_0_9 - y_train_0_node_0_10
                                 - y_train_0_node_2_8 - y_train_0_node_2_9
                                 - y_train_0_node_2_10 - y_train_0_node_4_8
                                 - y_train_0_node_4_9 - y_train_0_node_4_10
                                 - y_train_0_node_6_8 - y_train_0_node_6_9
                                 - y_train_0_node_6_10 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_10: - y_train_0_node_0_9
                                 - y_train_0_node_0_10 - y_train_0_node_0_11
                                 - y_train_0_node_2_9 - y_train_0_node_2_10
                                 - y_train_0_node_2_11 - y_train_0_node_4_9
                                 - y_train_0_node_4_10 - y_train_0_node_4_11
                                 - y_train_0_node_6_9 - y_train_0_node_6_10
                                 - y_train_0_node_6_11 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_11: - y_train_0_node_0_10
                                 - y_train_0_node_0_11 - y_train_0_node_0_12
                                 - y_train_0_node_2_10 - y_train_0_node_2_11
                                 - y_train_0_node_2_12 - y_train_0_node_4_10
                                 - y_train_0_node_4_11 - y_train_0_node_4_12
                                 - y_train_0_node_6_10 - y_train_0_node_6_11
                                 - y_train_0_node_6_12 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_12: - y_train_0_node_0_11
                                 - y_train_0_node_0_12 - y_train_0_node_0_13
                                 - y_train_0_node_2_11 - y_train_0_node_2_12
                                 - y_train_0_node_2_13 - y_train_0_node_4_11
                                 - y_train_0_node_4_12 - y_train_0_node_4_13
                                 - y_train_0_node_6_11 - y_train_0_node_6_12
                                 - y_train_0_node_6_13 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_13: - y_train_0_node_0_12
                                 - y_train_0_node_0_13 - y_train_0_node_0_14
                                 - y_train_0_node_2_12 - y_train_0_node_2_13
                                 - y_train_0_node_2_14 - y_train_0_node_4_12
                                 - y_train_0_node_4_13 - y_train_0_node_4_14
                                 - y_train_0_node_6_12 - y_train_0_node_6_13
                                 - y_train_0_node_6_14 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_14: - y_train_0_node_0_13
                                 - y_train_0_node_0_14 - y_train_0_node_0_15
                                 - y_train_0_node_2_13 - y_train_0_node_2_14
                                 - y_train_0_node_2_15 - y_train_0_node_4_13
                                 - y_train_0_node_4_14 - y_train_0_node_4_15
                                 - y_train_0_node_6_13 - y_train_0_node_6_14
                                 - y_train_0_node_6_15 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_15: - y_train_0_node_0_14
                                 - y_train_0_node_0_15 - y_train_0_node_0_16
                                 - y_train_0_node_2_14 - y_train_0_node_2_15
                                 - y_train_0_node_2_16 - y_train_0_node_4_14
                                 - y_train_0_node_4_15 - y_train_0_node_4_16
                                 - y_train_0_node_6_14 - y_train_0_node_6_15
                                 - y_train_0_node_6_16 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_16: - y_train_0_node_0_15
                                 - y_train_0_node_0_16 - y_train_0_node_0_17
                                 - y_train_0_node_2_15 - y_train_0_node_2_16
                                 - y_train_0_node_2_17 - y_train_0_node_4_15
                                 - y_train_0_node_4_16 - y_train_0_node_4_17
                                 - y_train_0_node_6_15 - y_train_0_node_6_16
                                 - y_train_0_node_6_17 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_17: - y_train_0_node_0_16
                                 - y_train_0_node_0_17 - y_train_0_node_0_18
                                 - y_train_0_node_2_16 - y_train_0_node_2_17
                                 - y_train_0_node_2_18 - y_train_0_node_4_16
                                 - y_train_0_node_4_17 - y_train_0_node_4_18
                                 - y_train_0_node_6_16 - y_train_0_node_6_17
                                 - y_train_0_node_6_18 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_18: - y_train_0_node_0_17
                                 - y_train_0_node_0_18 - y_train_0_node_0_19
                                 - y_train_0_node_2_17 - y_train_0_node_2_18
                                 - y_train_0_node_2_19 - y_train_0_node_4_17
                                 - y_train_0_node_4_18 - y_train_0_node_4_19
                                 - y_train_0_node_6_17 - y_train_0_node_6_18
                                 - y_train_0_node_6_19 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_19: - y_train_0_node_0_18
                                 - y_train_0_node_0_19 - y_train_0_node_0_20
                                 - y_train_0_node_2_18 - y_train_0_node_2_19
                                 - y_train_0_node_2_20 - y_train_0_node_4_18
                                 - y_train_0_node_4_19 - y_train_0_node_4_20
                                 - y_train_0_node_6_18 - y_train_0_node_6_19
                                 - y_train_0_node_6_20 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_20: - y_train_0_node_0_19
                                 - y_train_0_node_0_20 - y_train_0_node_0_21
                                 - y_train_0_node_2_19 - y_train_0_node_2_20
                                 - y_train_0_node_2_21 - y_train_0_node_4_19
                                 - y_train_0_node_4_20 - y_train_0_node_4_21
                                 - y_train_0_node_6_19 - y_train_0_node_6_20
                                 - y_train_0_node_6_21 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_21: - y_train_0_node_0_20
                                 - y_train_0_node_0_21 - y_train_0_node_0_22
                                 - y_train_0_node_2_20 - y_train_0_node_2_21
                                 - y_train_0_node_2_22 - y_train_0_node_4_20
                                 - y_train_0_node_4_21 - y_train_0_node_4_22
                                 - y_train_0_node_6_20 - y_train_0_node_6_21
                                 - y_train_0_node_6_22 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_22: - y_train_0_node_0_21
                                 - y_train_0_node_0_22 - y_train_0_node_0_23
                                 - y_train_0_node_2_21 - y_train_0_node_2_22
                                 - y_train_0_node_2_23 - y_train_0_node_4_21
                                 - y_train_0_node_4_22 - y_train_0_node_4_23
                                 - y_train_0_node_6_21 - y_train_0_node_6_22
                                 - y_train_0_node_6_23 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_23: - y_train_0_node_0_22
                                 - y_train_0_node_0_23 - y_train_0_node_0_24
                                 - y_train_0_node_2_22 - y_train_0_node_2_23
                                 - y_train_0_node_2_24 - y_train_0_node_4_22
                                 - y_train_0_node_4_23 - y_train_0_node_4_24
                                 - y_train_0_node_6_22 - y_train_0_node_6_23
                                 - y_train_0_node_6_24 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_24: - y_train_0_node_0_23
                                 - y_train_0_node_0_24 - y_train_0_node_0_25
                                 - y_train_0_node_2_23 - y_train_0_node_2_24
                                 - y_train_0_node_2_25 - y_train_0_node_4_23
                                 - y_train_0_node_4_24 - y_train_0_node_4_25
                                 - y_train_0_node_6_23 - y_train_0_node_6_24
                                 - y_train_0_node_6_25 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_25: - y_train_0_node_0_24
                                 - y_train_0_node_0_25 - y_train_0_node_0_26
                                 - y_train_0_node_2_24 - y_train_0_node_2_25
                                 - y_train_0_node_2_26 - y_train_0_node_4_24
                                 - y_train_0_node_4_25 - y_train_0_node_4_26
                                 - y_train_0_node_6_24 - y_train_0_node_6_25
                                 - y_train_0_node_6_26 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_26: - y_train_0_node_0_25
                                 - y_train_0_node_0_26 - y_train_0_node_0_27
                                 - y_train_0_node_2_25 - y_train_0_node_2_26
                                 - y_train_0_node_2_27 - y_train_0_node_4_25
                                 - y_train_0_node_4_26 - y_train_0_node_4_27
                                 - y_train_0_node_6_25 - y_train_0_node_6_26
                                 - y_train_0_node_6_27 + y_train_1_node_3_5
                                 + y_train_1_node_3_6 + y_train_1_node_3_7
                                 + y_train_1_node_3_8 + y_train_1_node_3_9
                                 + y_train_1_node_3_10 + y_train_1_node_3_11
                                 + y_train_1_node_3_12 + y_train_1_node_3_13
                                 + y_train_1_node_3_14 + y_train_1_node_3_15
                                 + y_train_1_node_3_16 + y_train_1_node_3_17
                                 + y_train_1_node_3_18 + y_train_1_node_3_19
                                 + y_train_1_node_3_20 + y_train_1_node_3_21
                                 + y_train_1_node_3_22 + y_train_1_node_3_23
                                 + y_train_1_node_3_24 + y_train_1_node_3_25
                                 + y_train_1_node_3_26 + y_train_1_node_3_27
                                 + y_train_1_node_3_28 <= 0
 can_take_siding_train_1_segment_3_time_27: - y_train_0_node_0_26
                                 - y_train_0_node_0_27 - y_train_0_node_2_26
                                 - y_train_0_node_2_27 - y_train_0_node_2_28
                                 - y_train_0_node_4_26 - y_train_0_node_4_27
                                 - y_train_0_node_4_28 - y_train_0_node_6_26
                                 - y_train_0_node_6_27 - y_train_0_node_6_28
                                 + y_train_1_node_3_5 + y_train_1_node_3_6
                                 + y_train_1_node_3_7 + y_train_1_node_3_8
                                 + y_train_1_node_3_9 + y_train_1_node_3_10
                                 + y_train_1_node_3_11 + y_train_1_node_3_12
                                 + y_train_1_node_3_13 + y_train_1_node_3_14
                                 + y_train_1_node_3_15 + y_train_1_node_3_16
                                 + y_train_1_node_3_17 + y_train_1_node_3_18
                                 + y_train_1_node_3_19 + y_train_1_node_3_20
                                 + y_train_1_node_3_21 + y_train_1_node_3_22
                                 + y_train_1_node_3_23 + y_train_1_node_3_24
                                 + y_train_1_node_3_25 + y_train_1_node_3_26
                                 + y_train_1_node_3_27 + y_train_1_node_3_28
                                 <= 0
 can_take_siding_train_1_segment_3_time_28: - y_train_0_node_0_27
                                 - y_train_0_node_2_27 - y_train_0_node_2_28
                                 - y_train_0_node_4_27 - y_train_0_node_4_28
                                 - y_train_0_node_4_29 - y_train_0_node_6_27
                                 - y_train_0_node_6_28 - y_train_0_node_6_29
                                 + y_train_1_node_3_5 + y_train_1_node_3_6
                                 + y_train_1_node_3_7 + y_train_1_node_3_8
                                 + y_train_1_node_3_9 + y_train_1_node_3_10
                                 + y_train_1_node_3_11 + y_train_1_node_3_12
                                 + y_train_1_node_3_13 + y_train_1_node_3_14
                                 + y_train_1_node_3_15 + y_train_1_node_3_16
                                 + y_train_1_node_3_17 + y_train_1_node_3_18
                                 + y_train_1_node_3_19 + y_train_1_node_3_20
                                 + y_train_1_node_3_21 + y_train_1_node_3_22
                                 + y_train_1_node_3_23 + y_train_1_node_3_24
                                 + y_train_1_node_3_25 + y_train_1_node_3_26
                                 + y_train_1_node_3_27 + y_train_1_node_3_28
                                 <= 0
Bounds
 0 <= z_train_0_segment_0 <= 1
 0 <= z_train_0_segment_1 <= 1
 0 <= z_train_0_segment_2 <= 1
 0 <= z_train_0_segment_3 <= 1
 0 <= z_train_0_segment_4 <= 1
 0 <= z_train_0_segment_5 <= 1
 0 <= z_train_0_segment_6 <= 1
 0 <= z_train_1_segment_0 <= 1
 0 <= z_train_1_segment_1 <= 1
 0 <= z_train_1_segment_2 <= 1
 0 <= z_train_1_segment_3 <= 1
 0 <= z_train_1_segment_4 <= 1
 0 <= z_train_1_segment_5 <= 1
 0 <= z_train_1_segment_6 <= 1
 0 <= theta_train_0_segment_0 <= 2
 0 <= theta_train_0_segment_1 <= 2
 0 <= theta_train_0_segment_2 <= 2
 0 <= theta_train_0_segment_3 <= 2
 0 <= theta_train_0_segment_4 <= 2
 0 <= theta_train_0_segment_5 <= 2
 0 <= theta_train_0_segment_6 <= 2
 0 <= theta_train_1_segment_0 <= 2
 0 <= theta_train_1_segment_1 <= 2
 0 <= theta_train_1_segment_2 <= 2
 0 <= theta_train_1_segment_3 <= 2
 0 <= theta_train_1_segment_4 <= 2
 0 <= theta_train_1_segment_5 <= 2
 0 <= theta_train_1_segment_6 <= 2
-2 <= d_train_0 <= 2
-2 <= d_train_1 <= 2
 0 <= _e_train_0_segment_1 <= 2
      x32 = 0
 0 <= x_train_0_from_sigma_to_0_1 <= 1
 0 <= x_train_0_from_sigma_to_0_2 <= 1
 0 <= x_train_0_from_sigma_to_0_3 <= 1
 0 <= x_train_0_from_sigma_to_0_4 <= 1
 0 <= x_train_0_from_sigma_to_0_5 <= 1
 0 <= x_train_0_from_sigma_to_0_6 <= 1
 0 <= x_train_0_from_sigma_to_0_7 <= 1
 0 <= x_train_0_from_sigma_to_0_8 <= 1
 0 <= x_train_0_from_sigma_to_0_9 <= 1
 0 <= x_train_0_from_sigma_to_0_10 <= 1
 0 <= x_train_0_from_sigma_to_0_11 <= 1
 0 <= x_train_0_from_sigma_to_0_12 <= 1
 0 <= x_train_0_from_sigma_to_0_13 <= 1
 0 <= x_train_0_from_sigma_to_0_14 <= 1
 0 <= x_train_0_from_sigma_to_0_15 <= 1
 0 <= x_train_0_from_sigma_to_0_16 <= 1
 0 <= x_train_0_from_sigma_to_0_17 <= 1
 0 <= x_train_0_from_sigma_to_0_18 <= 1
 0 <= x_train_0_from_sigma_to_0_19 <= 1
 0 <= x_train_0_from_sigma_to_0_20 <= 1
 0 <= x_train_0_from_sigma_to_0_21 <= 1
 0 <= x_train_0_from_sigma_to_0_22 <= 1
 0 <= x_train_0_from_sigma_to_0_23 <= 1
 0 <= x_train_0_from_sigma_to_0_24 <= 1
 0 <= x_train_0_from_sigma_to_0_25 <= 1
 0 <= x_train_0_from_sigma_to_0_26 <= 1
 0 <= x_train_0_from_sigma_to_0_27 <= 1
 0 <= x_train_1_from_sigma_to_6_1 <= 1
 0 <= x_train_1_from_sigma_to_6_2 <= 1
 0 <= x_train_1_from_sigma_to_6_3 <= 1
 0 <= x_train_1_from_sigma_to_6_4 <= 1
 0 <= x_train_1_from_sigma_to_6_5 <= 1
 0 <= x_train_1_from_sigma_to_6_6 <= 1
 0 <= x_train_1_from_sigma_to_6_7 <= 1
 0 <= x_train_1_from_sigma_to_6_8 <= 1
 0 <= x_train_1_from_sigma_to_6_9 <= 1
 0 <= x_train_1_from_sigma_to_6_10 <= 1
 0 <= x_train_1_from_sigma_to_6_11 <= 1
 0 <= x_train_1_from_sigma_to_6_12 <= 1
 0 <= x_train_1_from_sigma_to_6_13 <= 1
 0 <= x_train_1_from_sigma_to_6_14 <= 1
 0 <= x_train_1_from_sigma_to_6_15 <= 1
 0 <= x_train_1_from_sigma_to_6_16 <= 1
 0 <= x_train_1_from_sigma_to_6_17 <= 1
 0 <= x_train_1_from_sigma_to_6_18 <= 1
 0 <= x_train_1_from_sigma_to_6_19 <= 1
 0 <= x_train_1_from_sigma_to_6_20 <= 1
 0 <= x_train_1_from_sigma_to_6_21 <= 1
 0 <= x_train_1_from_sigma_to_6_22 <= 1
 0 <= x_train_1_from_sigma_to_6_23 <= 1
 0 <= x_train_1_from_sigma_to_6_24 <= 1
 0 <= x_train_1_from_sigma_to_6_25 <= 1
 0 <= x_train_1_from_sigma_to_6_26 <= 1
 0 <= x_train_1_from_sigma_to_6_27 <= 1
 0 <= x_train_0_from_6_4_to_tau <= 1
 0 <= x_train_0_from_6_5_to_tau <= 1
 0 <= x_train_0_from_6_6_to_tau <= 1
 0 <= x_train_0_from_6_7_to_tau <= 1
 0 <= x_train_0_from_6_8_to_tau <= 1
 0 <= x_train_0_from_6_9_to_tau <= 1
 0 <= x_train_0_from_6_10_to_tau <= 1
 0 <= x_train_0_from_6_11_to_tau <= 1
 0 <= x_train_0_from_6_12_to_tau <= 1
 0 <= x_train_0_from_6_13_to_tau <= 1
 0 <= x_train_0_from_6_14_to_tau <= 1
 0 <= x_train_0_from_6_15_to_tau <= 1
 0 <= x_train_0_from_6_16_to_tau <= 1
 0 <= x_train_0_from_6_17_to_tau <= 1
 0 <= x_train_0_from_6_18_to_tau <= 1
 0 <= x_train_0_from_6_19_to_tau <= 1
 0 <= x_train_0_from_6_20_to_tau <= 1
 0 <= x_train_0_from_6_21_to_tau <= 1
 0 <= x_train_0_from_6_22_to_tau <= 1
 0 <= x_train_0_from_6_23_to_tau <= 1
 0 <= x_train_0_from_6_24_to_tau <= 1
 0 <= x_train_0_from_6_25_to_tau <= 1
 0 <= x_train_0_from_6_26_to_tau <= 1
 0 <= x_train_0_from_6_27_to_tau <= 1
 0 <= x_train_0_from_6_28_to_tau <= 1
 0 <= x_train_0_from_6_29_to_tau <= 1
 0 <= x_train_0_from_6_30_to_tau <= 1
 0 <= x_train_1_from_0_4_to_tau <= 1
 0 <= x_train_1_from_0_5_to_tau <= 1
 0 <= x_train_1_from_0_6_to_tau <= 1
 0 <= x_train_1_from_0_7_to_tau <= 1
 0 <= x_train_1_from_0_8_to_tau <= 1
 0 <= x_train_1_from_0_9_to_tau <= 1
 0 <= x_train_1_from_0_10_to_tau <= 1
 0 <= x_train_1_from_0_11_to_tau <= 1
 0 <= x_train_1_from_0_12_to_tau <= 1
 0 <= x_train_1_from_0_13_to_tau <= 1
 0 <= x_train_1_from_0_14_to_tau <= 1
 0 <= x_train_1_from_0_15_to_tau <= 1
 0 <= x_train_1_from_0_16_to_tau <= 1
 0 <= x_train_1_from_0_17_to_tau <= 1
 0 <= x_train_1_from_0_18_to_tau <= 1
 0 <= x_train_1_from_0_19_to_tau <= 1
 0 <= x_train_1_from_0_20_to_tau <= 1
 0 <= x_train_1_from_0_21_to_tau <= 1
 0 <= x_train_1_from_0_22_to_tau <= 1
 0 <= x_train_1_from_0_23_to_tau <= 1
 0 <= x_train_1_from_0_24_to_tau <= 1
 0 <= x_train_1_from_0_25_to_tau <= 1
 0 <= x_train_1_from_0_26_to_tau <= 1
 0 <= x_train_1_from_0_27_to_tau <= 1
 0 <= x_train_1_from_0_28_to_tau <= 1
 0 <= x_train_1_from_0_29_to_tau <= 1
 0 <= x_train_1_from_0_30_to_tau <= 1
 0 <= y_train_0_node_0_1 <= 1
 0 <= x_train_0_from_0_1_to_0_2 <= 1
 0 <= y_train_0_node_0_2 <= 1
 0 <= x_train_0_from_0_2_to_0_3 <= 1
 0 <= y_train_0_node_0_3 <= 1
 0 <= x_train_0_from_0_3_to_0_4 <= 1
 0 <= y_train_0_node_0_4 <= 1
 0 <= x_train_0_from_0_4_to_0_5 <= 1
 0 <= y_train_0_node_0_5 <= 1
 0 <= x_train_0_from_0_5_to_0_6 <= 1
 0 <= y_train_0_node_0_6 <= 1
 0 <= x_train_0_from_0_6_to_0_7 <= 1
 0 <= y_train_0_node_0_7 <= 1
 0 <= x_train_0_from_0_7_to_0_8 <= 1
 0 <= y_train_0_node_0_8 <= 1
 0 <= x_train_0_from_0_8_to_0_9 <= 1
 0 <= y_train_0_node_0_9 <= 1
 0 <= x_train_0_from_0_9_to_0_10 <= 1
 0 <= y_train_0_node_0_10 <= 1
 0 <= x_train_0_from_0_10_to_0_11 <= 1
 0 <= y_train_0_node_0_11 <= 1
 0 <= x_train_0_from_0_11_to_0_12 <= 1
 0 <= y_train_0_node_0_12 <= 1
 0 <= x_train_0_from_0_12_to_0_13 <= 1
 0 <= y_train_0_node_0_13 <= 1
 0 <= x_train_0_from_0_13_to_0_14 <= 1
 0 <= y_train_0_node_0_14 <= 1
 0 <= x_train_0_from_0_14_to_0_15 <= 1
 0 <= y_train_0_node_0_15 <= 1
 0 <= x_train_0_from_0_15_to_0_16 <= 1
 0 <= y_train_0_node_0_16 <= 1
 0 <= x_train_0_from_0_16_to_0_17 <= 1
 0 <= y_train_0_node_0_17 <= 1
 0 <= x_train_0_from_0_17_to_0_18 <= 1
 0 <= y_train_0_node_0_18 <= 1
 0 <= x_train_0_from_0_18_to_0_19 <= 1
 0 <= y_train_0_node_0_19 <= 1
 0 <= x_train_0_from_0_19_to_0_20 <= 1
 0 <= y_train_0_node_0_20 <= 1
 0 <= x_train_0_from_0_20_to_0_21 <= 1
 0 <= y_train_0_node_0_21 <= 1
 0 <= x_train_0_from_0_21_to_0_22 <= 1
 0 <= y_train_0_node_0_22 <= 1
 0 <= x_train_0_from_0_22_to_0_23 <= 1
 0 <= y_train_0_node_0_23 <= 1
 0 <= x_train_0_from_0_23_to_0_24 <= 1
 0 <= y_train_0_node_0_24 <= 1
 0 <= x_train_0_from_0_24_to_0_25 <= 1
 0 <= y_train_0_node_0_25 <= 1
 0 <= x_train_0_from_0_25_to_0_26 <= 1
 0 <= y_train_0_node_0_26 <= 1
 0 <= x_train_0_from_0_26_to_0_27 <= 1
 0 <= y_train_0_node_0_27 <= 1
 0 <= x_train_0_from_0_1_to_1_2 <= 1
 0 <= y_train_0_node_1_2 <= 1
 0 <= x_train_0_from_0_2_to_1_3 <= 1
 0 <= x_train_0_from_1_2_to_1_3 <= 1
 0 <= y_train_0_node_1_3 <= 1
 0 <= x_train_0_from_0_3_to_1_4 <= 1
 0 <= x_train_0_from_1_3_to_1_4 <= 1
 0 <= y_train_0_node_1_4 <= 1
 0 <= x_train_0_from_0_4_to_1_5 <= 1
 0 <= x_train_0_from_1_4_to_1_5 <= 1
 0 <= y_train_0_node_1_5 <= 1
 0 <= x_train_0_from_0_5_to_1_6 <= 1
 0 <= x_train_0_from_1_5_to_1_6 <= 1
 0 <= y_train_0_node_1_6 <= 1
 0 <= x_train_0_from_0_6_to_1_7 <= 1
 0 <= x_train_0_from_1_6_to_1_7 <= 1
 0 <= y_train_0_node_1_7 <= 1
 0 <= x_train_0_from_0_7_to_1_8 <= 1
 0 <= x_train_0_from_1_7_to_1_8 <= 1
 0 <= y_train_0_node_1_8 <= 1
 0 <= x_train_0_from_0_8_to_1_9 <= 1
 0 <= x_train_0_from_1_8_to_1_9 <= 1
 0 <= y_train_0_node_1_9 <= 1
 0 <= x_train_0_from_0_9_to_1_10 <= 1
 0 <= x_train_0_from_1_9_to_1_10 <= 1
 0 <= y_train_0_node_1_10 <= 1
 0 <= x_train_0_from_0_10_to_1_11 <= 1
 0 <= x_train_0_from_1_10_to_1_11 <= 1
 0 <= y_train_0_node_1_11 <= 1
 0 <= x_train_0_from_0_11_to_1_12 <= 1
 0 <= x_train_0_from_1_11_to_1_12 <= 1
 0 <= y_train_0_node_1_12 <= 1
 0 <= x_train_0_from_0_12_to_1_13 <= 1
 0 <= x_train_0_from_1_12_to_1_13 <= 1
 0 <= y_train_0_node_1_13 <= 1
 0 <= x_train_0_from_0_13_to_1_14 <= 1
 0 <= x_train_0_from_1_13_to_1_14 <= 1
 0 <= y_train_0_node_1_14 <= 1
 0 <= x_train_0_from_0_14_to_1_15 <= 1
 0 <= x_train_0_from_1_14_to_1_15 <= 1
 0 <= y_train_0_node_1_15 <= 1
 0 <= x_train_0_from_0_15_to_1_16 <= 1
 0 <= x_train_0_from_1_15_to_1_16 <= 1
 0 <= y_train_0_node_1_16 <= 1
 0 <= x_train_0_from_0_16_to_1_17 <= 1
 0 <= x_train_0_from_1_16_to_1_17 <= 1
 0 <= y_train_0_node_1_17 <= 1
 0 <= x_train_0_from_0_17_to_1_18 <= 1
 0 <= x_train_0_from_1_17_to_1_18 <= 1
 0 <= y_train_0_node_1_18 <= 1
 0 <= x_train_0_from_0_18_to_1_19 <= 1
 0 <= x_train_0_from_1_18_to_1_19 <= 1
 0 <= y_train_0_node_1_19 <= 1
 0 <= x_train_0_from_0_19_to_1_20 <= 1
 0 <= x_train_0_from_1_19_to_1_20 <= 1
 0 <= y_train_0_node_1_20 <= 1
 0 <= x_train_0_from_0_20_to_1_21 <= 1
 0 <= x_train_0_from_1_20_to_1_21 <= 1
 0 <= y_train_0_node_1_21 <= 1
 0 <= x_train_0_from_0_21_to_1_22 <= 1
 0 <= x_train_0_from_1_21_to_1_22 <= 1
 0 <= y_train_0_node_1_22 <= 1
 0 <= x_train_0_from_0_22_to_1_23 <= 1
 0 <= x_train_0_from_1_22_to_1_23 <= 1
 0 <= y_train_0_node_1_23 <= 1
 0 <= x_train_0_from_0_23_to_1_24 <= 1
 0 <= x_train_0_from_1_23_to_1_24 <= 1
 0 <= y_train_0_node_1_24 <= 1
 0 <= x_train_0_from_0_24_to_1_25 <= 1
 0 <= x_train_0_from_1_24_to_1_25 <= 1
 0 <= y_train_0_node_1_25 <= 1
 0 <= x_train_0_from_0_25_to_1_26 <= 1
 0 <= x_train_0_from_1_25_to_1_26 <= 1
 0 <= y_train_0_node_1_26 <= 1
 0 <= x_train_0_from_0_26_to_1_27 <= 1
 0 <= x_train_0_from_1_26_to_1_27 <= 1
 0 <= y_train_0_node_1_27 <= 1
 0 <= x_train_0_from_0_1_to_2_2 <= 1
 0 <= y_train_0_node_2_2 <= 1
 0 <= x_train_0_from_0_2_to_2_3 <= 1
 0 <= x_train_0_from_2_2_to_2_3 <= 1
 0 <= y_train_0_node_2_3 <= 1
 0 <= x_train_0_from_0_3_to_2_4 <= 1
 0 <= x_train_0_from_2_3_to_2_4 <= 1
 0 <= y_train_0_node_2_4 <= 1
 0 <= x_train_0_from_0_4_to_2_5 <= 1
 0 <= x_train_0_from_2_4_to_2_5 <= 1
 0 <= y_train_0_node_2_5 <= 1
 0 <= x_train_0_from_0_5_to_2_6 <= 1
 0 <= x_train_0_from_2_5_to_2_6 <= 1
 0 <= y_train_0_node_2_6 <= 1
 0 <= x_train_0_from_0_6_to_2_7 <= 1
 0 <= x_train_0_from_2_6_to_2_7 <= 1
 0 <= y_train_0_node_2_7 <= 1
 0 <= x_train_0_from_0_7_to_2_8 <= 1
 0 <= x_train_0_from_2_7_to_2_8 <= 1
 0 <= y_train_0_node_2_8 <= 1
 0 <= x_train_0_from_0_8_to_2_9 <= 1
 0 <= x_train_0_from_2_8_to_2_9 <= 1
 0 <= y_train_0_node_2_9 <= 1
 0 <= x_train_0_from_0_9_to_2_10 <= 1
 0 <= x_train_0_from_2_9_to_2_10 <= 1
 0 <= y_train_0_node_2_10 <= 1
 0 <= x_train_0_from_0_10_to_2_11 <= 1
 0 <= x_train_0_from_2_10_to_2_11 <= 1
 0 <= y_train_0_node_2_11 <= 1
 0 <= x_train_0_from_0_11_to_2_12 <= 1
 0 <= x_train_0_from_2_11_to_2_12 <= 1
 0 <= y_train_0_node_2_12 <= 1
 0 <= x_train_0_from_0_12_to_2_13 <= 1
 0 <= x_train_0_from_2_12_to_2_13 <= 1
 0 <= y_train_0_node_2_13 <= 1
 0 <= x_train_0_from_0_13_to_2_14 <= 1
 0 <= x_train_0_from_2_13_to_2_14 <= 1
 0 <= y_train_0_node_2_14 <= 1
 0 <= x_train_0_from_0_14_to_2_15 <= 1
 0 <= x_train_0_from_2_14_to_2_15 <= 1
 0 <= y_train_0_node_2_15 <= 1
 0 <= x_train_0_from_0_15_to_2_16 <= 1
 0 <= x_train_0_from_2_15_to_2_16 <= 1
 0 <= y_train_0_node_2_16 <= 1
 0 <= x_train_0_from_0_16_to_2_17 <= 1
 0 <= x_train_0_from_2_16_to_2_17 <= 1
 0 <= y_train_0_node_2_17 <= 1
 0 <= x_train_0_from_0_17_to_2_18 <= 1
 0 <= x_train_0_from_2_17_to_2_18 <= 1
 0 <= y_train_0_node_2_18 <= 1
 0 <= x_train_0_from_0_18_to_2_19 <= 1
 0 <= x_train_0_from_2_18_to_2_19 <= 1
 0 <= y_train_0_node_2_19 <= 1
 0 <= x_train_0_from_0_19_to_2_20 <= 1
 0 <= x_train_0_from_2_19_to_2_20 <= 1
 0 <= y_train_0_node_2_20 <= 1
 0 <= x_train_0_from_0_20_to_2_21 <= 1
 0 <= x_train_0_from_2_20_to_2_21 <= 1
 0 <= y_train_0_node_2_21 <= 1
 0 <= x_train_0_from_0_21_to_2_22 <= 1
 0 <= x_train_0_from_2_21_to_2_22 <= 1
 0 <= y_train_0_node_2_22 <= 1
 0 <= x_train_0_from_0_22_to_2_23 <= 1
 0 <= x_train_0_from_2_22_to_2_23 <= 1
 0 <= y_train_0_node_2_23 <= 1
 0 <= x_train_0_from_0_23_to_2_24 <= 1
 0 <= x_train_0_from_2_23_to_2_24 <= 1
 0 <= y_train_0_node_2_24 <= 1
 0 <= x_train_0_from_0_24_to_2_25 <= 1
 0 <= x_train_0_from_2_24_to_2_25 <= 1
 0 <= y_train_0_node_2_25 <= 1
 0 <= x_train_0_from_0_25_to_2_26 <= 1
 0 <= x_train_0_from_2_25_to_2_26 <= 1
 0 <= y_train_0_node_2_26 <= 1
 0 <= x_train_0_from_0_26_to_2_27 <= 1
 0 <= x_train_0_from_2_26_to_2_27 <= 1
 0 <= y_train_0_node_2_27 <= 1
 0 <= x_train_0_from_0_27_to_2_28 <= 1
 0 <= x_train_0_from_2_27_to_2_28 <= 1
 0 <= y_train_0_node_2_28 <= 1
 0 <= x_train_0_from_1_4_to_3_5 <= 1
 0 <= y_train_0_node_3_5 <= 1
 0 <= x_train_0_from_1_5_to_3_6 <= 1
 0 <= x_train_0_from_3_5_to_3_6 <= 1
 0 <= y_train_0_node_3_6 <= 1
 0 <= x_train_0_from_1_6_to_3_7 <= 1
 0 <= x_train_0_from_3_6_to_3_7 <= 1
 0 <= y_train_0_node_3_7 <= 1
 0 <= x_train_0_from_1_7_to_3_8 <= 1
 0 <= x_train_0_from_3_7_to_3_8 <= 1
 0 <= y_train_0_node_3_8 <= 1
 0 <= x_train_0_from_1_8_to_3_9 <= 1
 0 <= x_train_0_from_3_8_to_3_9 <= 1
 0 <= y_train_0_node_3_9 <= 1
 0 <= x_train_0_from_1_9_to_3_10 <= 1
 0 <= x_train_0_from_3_9_to_3_10 <= 1
 0 <= y_train_0_node_3_10 <= 1
 0 <= x_train_0_from_1_10_to_3_11 <= 1
 0 <= x_train_0_from_3_10_to_3_11 <= 1
 0 <= y_train_0_node_3_11 <= 1
 0 <= x_train_0_from_1_11_to_3_12 <= 1
 0 <= x_train_0_from_3_11_to_3_12 <= 1
 0 <= y_train_0_node_3_12 <= 1
 0 <= x_train_0_from_1_12_to_3_13 <= 1
 0 <= x_train_0_from_3_12_to_3_13 <= 1
 0 <= y_train_0_node_3_13 <= 1
 0 <= x_train_0_from_1_13_to_3_14 <= 1
 0 <= x_train_0_from_3_13_to_3_14 <= 1
 0 <= y_train_0_node_3_14 <= 1
 0 <= x_train_0_from_1_14_to_3_15 <= 1
 0 <= x_train_0_from_3_14_to_3_15 <= 1
 0 <= y_train_0_node_3_15 <= 1
 0 <= x_train_0_from_1_15_to_3_16 <= 1
 0 <= x_train_0_from_3_15_to_3_16 <= 1
 0 <= y_train_0_node_3_16 <= 1
 0 <= x_train_0_from_1_16_to_3_17 <= 1
 0 <= x_train_0_from_3_16_to_3_17 <= 1
 0 <= y_train_0_node_3_17 <= 1
 0 <= x_train_0_from_1_17_to_3_18 <= 1
 0 <= x_train_0_from_3_17_to_3_18 <= 1
 0 <= y_train_0_node_3_18 <= 1
 0 <= x_train_0_from_1_18_to_3_19 <= 1
 0 <= x_train_0_from_3_18_to_3_19 <= 1
 0 <= y_train_0_node_3_19 <= 1
 0 <= x_train_0_from_1_19_to_3_20 <= 1
 0 <= x_train_0_from_3_19_to_3_20 <= 1
 0 <= y_train_0_node_3_20 <= 1
 0 <= x_train_0_from_1_20_to_3_21 <= 1
 0 <= x_train_0_from_3_20_to_3_21 <= 1
 0 <= y_train_0_node_3_21 <= 1
 0 <= x_train_0_from_1_21_to_3_22 <= 1
 0 <= x_train_0_from_3_21_to_3_22 <= 1
 0 <= y_train_0_node_3_22 <= 1
 0 <= x_train_0_from_1_22_to_3_23 <= 1
 0 <= x_train_0_from_3_22_to_3_23 <= 1
 0 <= y_train_0_node_3_23 <= 1
 0 <= x_train_0_from_1_23_to_3_24 <= 1
 0 <= x_train_0_from_3_23_to_3_24 <= 1
 0 <= y_train_0_node_3_24 <= 1
 0 <= x_train_0_from_1_24_to_3_25 <= 1
 0 <= x_train_0_from_3_24_to_3_25 <= 1
 0 <= y_train_0_node_3_25 <= 1
 0 <= x_train_0_from_1_25_to_3_26 <= 1
 0 <= x_train_0_from_3_25_to_3_26 <= 1
 0 <= y_train_0_node_3_26 <= 1
 0 <= x_train_0_from_1_26_to_3_27 <= 1
 0 <= x_train_0_from_3_26_to_3_27 <= 1
 0 <= y_train_0_node_3_27 <= 1
 0 <= x_train_0_from_1_27_to_3_28 <= 1
 0 <= x_train_0_from_3_27_to_3_28 <= 1
 0 <= y_train_0_node_3_28 <= 1
 0 <= x_train_0_from_2_2_to_4_3 <= 1
 0 <= y_train_0_node_4_3 <= 1
 0 <= x_train_0_from_2_3_to_4_4 <= 1
 0 <= x_train_0_from_4_3_to_4_4 <= 1
 0 <= y_train_0_node_4_4 <= 1
 0 <= x_train_0_from_2_4_to_4_5 <= 1
 0 <= x_train_0_from_4_4_to_4_5 <= 1
 0 <= y_train_0_node_4_5 <= 1
 0 <= x_train_0_from_2_5_to_4_6 <= 1
 0 <= x_train_0_from_4_5_to_4_6 <= 1
 0 <= y_train_0_node_4_6 <= 1
 0 <= x_train_0_from_2_6_to_4_7 <= 1
 0 <= x_train_0_from_4_6_to_4_7 <= 1
 0 <= y_train_0_node_4_7 <= 1
 0 <= x_train_0_from_2_7_to_4_8 <= 1
 0 <= x_train_0_from_4_7_to_4_8 <= 1
 0 <= y_train_0_node_4_8 <= 1
 0 <= x_train_0_from_2_8_to_4_9 <= 1
 0 <= x_train_0_from_4_8_to_4_9 <= 1
 0 <= y_train_0_node_4_9 <= 1
 0 <= x_train_0_from_2_9_to_4_10 <= 1
 0 <= x_train_0_from_4_9_to_4_10 <= 1
 0 <= y_train_0_node_4_10 <= 1
 0 <= x_train_0_from_2_10_to_4_11 <= 1
 0 <= x_train_0_from_4_10_to_4_11 <= 1
 0 <= y_train_0_node_4_11 <= 1
 0 <= x_train_0_from_2_11_to_4_12 <= 1
 0 <= x_train_0_from_4_11_to_4_12 <= 1
 0 <= y_train_0_node_4_12 <= 1
 0 <= x_train_0_from_2_12_to_4_13 <= 1
 0 <= x_train_0_from_4_12_to_4_13 <= 1
 0 <= y_train_0_node_4_13 <= 1
 0 <= x_train_0_from_2_13_to_4_14 <= 1
 0 <= x_train_0_from_4_13_to_4_14 <= 1
 0 <= y_train_0_node_4_14 <= 1
 0 <= x_train_0_from_2_14_to_4_15 <= 1
 0 <= x_train_0_from_4_14_to_4_15 <= 1
 0 <= y_train_0_node_4_15 <= 1
 0 <= x_train_0_from_2_15_to_4_16 <= 1
 0 <= x_train_0_from_4_15_to_4_16 <= 1
 0 <= y_train_0_node_4_16 <= 1
 0 <= x_train_0_from_2_16_to_4_17 <= 1
 0 <= x_train_0_from_4_16_to_4_17 <= 1
 0 <= y_train_0_node_4_17 <= 1
 0 <= x_train_0_from_2_17_to_4_18 <= 1
 0 <= x_train_0_from_4_17_to_4_18 <= 1
 0 <= y_train_0_node_4_18 <= 1
 0 <= x_train_0_from_2_18_to_4_19 <= 1
 0 <= x_train_0_from_4_18_to_4_19 <= 1
 0 <= y_train_0_node_4_19 <= 1
 0 <= x_train_0_from_2_19_to_4_20 <= 1
 0 <= x_train_0_from_4_19_to_4_20 <= 1
 0 <= y_train_0_node_4_20 <= 1
 0 <= x_train_0_from_2_20_to_4_21 <= 1
 0 <= x_train_0_from_4_20_to_4_21 <= 1
 0 <= y_train_0_node_4_21 <= 1
 0 <= x_train_0_from_2_21_to_4_22 <= 1
 0 <= x_train_0_from_4_21_to_4_22 <= 1
 0 <= y_train_0_node_4_22 <= 1
 0 <= x_train_0_from_2_22_to_4_23 <= 1
 0 <= x_train_0_from_4_22_to_4_23 <= 1
 0 <= y_train_0_node_4_23 <= 1
 0 <= x_train_0_from_2_23_to_4_24 <= 1
 0 <= x_train_0_from_4_23_to_4_24 <= 1
 0 <= y_train_0_node_4_24 <= 1
 0 <= x_train_0_from_2_24_to_4_25 <= 1
 0 <= x_train_0_from_4_24_to_4_25 <= 1
 0 <= y_train_0_node_4_25 <= 1
 0 <= x_train_0_from_2_25_to_4_26 <= 1
 0 <= x_train_0_from_4_25_to_4_26 <= 1
 0 <= y_train_0_node_4_26 <= 1
 0 <= x_train_0_from_2_26_to_4_27 <= 1
 0 <= x_train_0_from_4_26_to_4_27 <= 1
 0 <= y_train_0_node_4_27 <= 1
 0 <= x_train_0_from_2_27_to_4_28 <= 1
 0 <= x_train_0_from_4_27_to_4_28 <= 1
 0 <= y_train_0_node_4_28 <= 1
 0 <= x_train_0_from_2_28_to_4_29 <= 1
 0 <= x_train_0_from_4_28_to_4_29 <= 1
 0 <= y_train_0_node_4_29 <= 1
 0 <= x_train_0_from_3_5_to_5_6 <= 1
 0 <= y_train_0_node_5_6 <= 1
 0 <= x_train_0_from_3_6_to_5_7 <= 1
 0 <= x_train_0_from_5_6_to_5_7 <= 1
 0 <= y_train_0_node_5_7 <= 1
 0 <= x_train_0_from_3_7_to_5_8 <= 1
 0 <= x_train_0_from_5_7_to_5_8 <= 1
 0 <= y_train_0_node_5_8 <= 1
 0 <= x_train_0_from_3_8_to_5_9 <= 1
 0 <= x_train_0_from_5_8_to_5_9 <= 1
 0 <= y_train_0_node_5_9 <= 1
 0 <= x_train_0_from_3_9_to_5_10 <= 1
 0 <= x_train_0_from_5_9_to_5_10 <= 1
 0 <= y_train_0_node_5_10 <= 1
 0 <= x_train_0_from_3_10_to_5_11 <= 1
 0 <= x_train_0_from_5_10_to_5_11 <= 1
 0 <= y_train_0_node_5_11 <= 1
 0 <= x_train_0_from_3_11_to_5_12 <= 1
 0 <= x_train_0_from_5_11_to_5_12 <= 1
 0 <= y_train_0_node_5_12 <= 1
 0 <= x_train_0_from_3_12_to_5_13 <= 1
 0 <= x_train_0_from_5_12_to_5_13 <= 1
 0 <= y_train_0_node_5_13 <= 1
 0 <= x_train_0_from_3_13_to_5_14 <= 1
 0 <= x_train_0_from_5_13_to_5_14 <= 1
 0 <= y_train_0_node_5_14 <= 1
 0 <= x_train_0_from_3_14_to_5_15 <= 1
 0 <= x_train_0_from_5_14_to_5_15 <= 1
 0 <= y_train_0_node_5_15 <= 1
 0 <= x_train_0_from_3_15_to_5_16 <= 1
 0 <= x_train_0_from_5_15_to_5_16 <= 1
 0 <= y_train_0_node_5_16 <= 1
 0 <= x_train_0_from_3_16_to_5_17 <= 1
 0 <= x_train_0_from_5_16_to_5_17 <= 1
 0 <= y_train_0_node_5_17 <= 1
 0 <= x_train_0_from_3_17_to_5_18 <= 1
 0 <= x_train_0_from_5_17_to_5_18 <= 1
 0 <= y_train_0_node_5_18 <= 1
 0 <= x_train_0_from_3_18_to_5_19 <= 1
 0 <= x_train_0_from_5_18_to_5_19 <= 1
 0 <= y_train_0_node_5_19 <= 1
 0 <= x_train_0_from_3_19_to_5_20 <= 1
 0 <= x_train_0_from_5_19_to_5_20 <= 1
 0 <= y_train_0_node_5_20 <= 1
 0 <= x_train_0_from_3_20_to_5_21 <= 1
 0 <= x_train_0_from_5_20_to_5_21 <= 1
 0 <= y_train_0_node_5_21 <= 1
 0 <= x_train_0_from_3_21_to_5_22 <= 1
 0 <= x_train_0_from_5_21_to_5_22 <= 1
 0 <= y_train_0_node_5_22 <= 1
 0 <= x_train_0_from_3_22_to_5_23 <= 1
 0 <= x_train_0_from_5_22_to_5_23 <= 1
 0 <= y_train_0_node_5_23 <= 1
 0 <= x_train_0_from_3_23_to_5_24 <= 1
 0 <= x_train_0_from_5_23_to_5_24 <= 1
 0 <= y_train_0_node_5_24 <= 1
 0 <= x_train_0_from_3_24_to_5_25 <= 1
 0 <= x_train_0_from_5_24_to_5_25 <= 1
 0 <= y_train_0_node_5_25 <= 1
 0 <= x_train_0_from_3_25_to_5_26 <= 1
 0 <= x_train_0_from_5_25_to_5_26 <= 1
 0 <= y_train_0_node_5_26 <= 1
 0 <= x_train_0_from_3_26_to_5_27 <= 1
 0 <= x_train_0_from_5_26_to_5_27 <= 1
 0 <= y_train_0_node_5_27 <= 1
 0 <= x_train_0_from_3_27_to_5_28 <= 1
 0 <= x_train_0_from_5_27_to_5_28 <= 1
 0 <= y_train_0_node_5_28 <= 1
 0 <= x_train_0_from_3_28_to_5_29 <= 1
 0 <= x_train_0_from_5_28_to_5_29 <= 1
 0 <= y_train_0_node_5_29 <= 1
 0 <= x_train_0_from_4_3_to_6_4 <= 1
 0 <= y_train_0_node_6_4 <= 1
 0 <= x_train_0_from_4_4_to_6_5 <= 1
 0 <= x_train_0_from_6_4_to_6_5 <= 1
 0 <= y_train_0_node_6_5 <= 1
 0 <= x_train_0_from_4_5_to_6_6 <= 1
 0 <= x_train_0_from_6_5_to_6_6 <= 1
 0 <= y_train_0_node_6_6 <= 1
 0 <= x_train_0_from_4_6_to_6_7 <= 1
 0 <= x_train_0_from_5_6_to_6_7 <= 1
 0 <= x_train_0_from_6_6_to_6_7 <= 1
 0 <= y_train_0_node_6_7 <= 1
 0 <= x_train_0_from_4_7_to_6_8 <= 1
 0 <= x_train_0_from_5_7_to_6_8 <= 1
 0 <= x_train_0_from_6_7_to_6_8 <= 1
 0 <= y_train_0_node_6_8 <= 1
 0 <= x_train_0_from_4_8_to_6_9 <= 1
 0 <= x_train_0_from_5_8_to_6_9 <= 1
 0 <= x_train_0_from_6_8_to_6_9 <= 1
 0 <= y_train_0_node_6_9 <= 1
 0 <= x_train_0_from_4_9_to_6_10 <= 1
 0 <= x_train_0_from_5_9_to_6_10 <= 1
 0 <= x_train_0_from_6_9_to_6_10 <= 1
 0 <= y_train_0_node_6_10 <= 1
 0 <= x_train_0_from_4_10_to_6_11 <= 1
 0 <= x_train_0_from_5_10_to_6_11 <= 1
 0 <= x_train_0_from_6_10_to_6_11 <= 1
 0 <= y_train_0_node_6_11 <= 1
 0 <= x_train_0_from_4_11_to_6_12 <= 1
 0 <= x_train_0_from_5_11_to_6_12 <= 1
 0 <= x_train_0_from_6_11_to_6_12 <= 1
 0 <= y_train_0_node_6_12 <= 1
 0 <= x_train_0_from_4_12_to_6_13 <= 1
 0 <= x_train_0_from_5_12_to_6_13 <= 1
 0 <= x_train_0_from_6_12_to_6_13 <= 1
 0 <= y_train_0_node_6_13 <= 1
 0 <= x_train_0_from_4_13_to_6_14 <= 1
 0 <= x_train_0_from_5_13_to_6_14 <= 1
 0 <= x_train_0_from_6_13_to_6_14 <= 1
 0 <= y_train_0_node_6_14 <= 1
 0 <= x_train_0_from_4_14_to_6_15 <= 1
 0 <= x_train_0_from_5_14_to_6_15 <= 1
 0 <= x_train_0_from_6_14_to_6_15 <= 1
 0 <= y_train_0_node_6_15 <= 1
 0 <= x_train_0_from_4_15_to_6_16 <= 1
 0 <= x_train_0_from_5_15_to_6_16 <= 1
 0 <= x_train_0_from_6_15_to_6_16 <= 1
 0 <= y_train_0_node_6_16 <= 1
 0 <= x_train_0_from_4_16_to_6_17 <= 1
 0 <= x_train_0_from_5_16_to_6_17 <= 1
 0 <= x_train_0_from_6_16_to_6_17 <= 1
 0 <= y_train_0_node_6_17 <= 1
 0 <= x_train_0_from_4_17_to_6_18 <= 1
 0 <= x_train_0_from_5_17_to_6_18 <= 1
 0 <= x_train_0_from_6_17_to_6_18 <= 1
 0 <= y_train_0_node_6_18 <= 1
 0 <= x_train_0_from_4_18_to_6_19 <= 1
 0 <= x_train_0_from_5_18_to_6_19 <= 1
 0 <= x_train_0_from_6_18_to_6_19 <= 1
 0 <= y_train_0_node_6_19 <= 1
 0 <= x_train_0_from_4_19_to_6_20 <= 1
 0 <= x_train_0_from_5_19_to_6_20 <= 1
 0 <= x_train_0_from_6_19_to_6_20 <= 1
 0 <= y_train_0_node_6_20 <= 1
 0 <= x_train_0_from_4_20_to_6_21 <= 1
 0 <= x_train_0_from_5_20_to_6_21 <= 1
 0 <= x_train_0_from_6_20_to_6_21 <= 1
 0 <= y_train_0_node_6_21 <= 1
 0 <= x_train_0_from_4_21_to_6_22 <= 1
 0 <= x_train_0_from_5_21_to_6_22 <= 1
 0 <= x_train_0_from_6_21_to_6_22 <= 1
 0 <= y_train_0_node_6_22 <= 1
 0 <= x_train_0_from_4_22_to_6_23 <= 1
 0 <= x_train_0_from_5_22_to_6_23 <= 1
 0 <= x_train_0_from_6_22_to_6_23 <= 1
 0 <= y_train_0_node_6_23 <= 1
 0 <= x_train_0_from_4_23_to_6_24 <= 1
 0 <= x_train_0_from_5_23_to_6_24 <= 1
 0 <= x_train_0_from_6_23_to_6_24 <= 1
 0 <= y_train_0_node_6_24 <= 1
 0 <= x_train_0_from_4_24_to_6_25 <= 1
 0 <= x_train_0_from_5_24_to_6_25 <= 1
 0 <= x_train_0_from_6_24_to_6_25 <= 1
 0 <= y_train_0_node_6_25 <= 1
 0 <= x_train_0_from_4_25_to_6_26 <= 1
 0 <= x_train_0_from_5_25_to_6_26 <= 1
 0 <= x_train_0_from_6_25_to_6_26 <= 1
 0 <= y_train_0_node_6_26 <= 1
 0 <= x_train_0_from_4_26_to_6_27 <= 1
 0 <= x_train_0_from_5_26_to_6_27 <= 1
 0 <= x_train_0_from_6_26_to_6_27 <= 1
 0 <= y_train_0_node_6_27 <= 1
 0 <= x_train_0_from_4_27_to_6_28 <= 1
 0 <= x_train_0_from_5_27_to_6_28 <= 1
 0 <= x_train_0_from_6_27_to_6_28 <= 1
 0 <= y_train_0_node_6_28 <= 1
 0 <= x_train_0_from_4_28_to_6_29 <= 1
 0 <= x_train_0_from_5_28_to_6_29 <= 1
 0 <= x_train_0_from_6_28_to_6_29 <= 1
 0 <= y_train_0_node_6_29 <= 1
 0 <= x_train_0_from_4_29_to_6_30 <= 1
 0 <= x_train_0_from_5_29_to_6_30 <= 1
 0 <= x_train_0_from_6_29_to_6_30 <= 1
 0 <= y_train_0_node_6_30 <= 1
 0 <= x_train_1_from_2_3_to_0_4 <= 1
 0 <= y_train_1_node_0_4 <= 1
 0 <= x_train_1_from_0_4_to_0_5 <= 1
 0 <= x_train_1_from_2_4_to_0_5 <= 1
 0 <= y_train_1_node_0_5 <= 1
 0 <= x_train_1_from_0_5_to_0_6 <= 1
 0 <= x_train_1_from_2_5_to_0_6 <= 1
 0 <= y_train_1_node_0_6 <= 1
 0 <= x_train_1_from_0_6_to_0_7 <= 1
 0 <= x_train_1_from_1_6_to_0_7 <= 1
 0 <= x_train_1_from_2_6_to_0_7 <= 1
 0 <= y_train_1_node_0_7 <= 1
 0 <= x_train_1_from_0_7_to_0_8 <= 1
 0 <= x_train_1_from_1_7_to_0_8 <= 1
 0 <= x_train_1_from_2_7_to_0_8 <= 1
 0 <= y_train_1_node_0_8 <= 1
 0 <= x_train_1_from_0_8_to_0_9 <= 1
 0 <= x_train_1_from_1_8_to_0_9 <= 1
 0 <= x_train_1_from_2_8_to_0_9 <= 1
 0 <= y_train_1_node_0_9 <= 1
 0 <= x_train_1_from_0_9_to_0_10 <= 1
 0 <= x_train_1_from_1_9_to_0_10 <= 1
 0 <= x_train_1_from_2_9_to_0_10 <= 1
 0 <= y_train_1_node_0_10 <= 1
 0 <= x_train_1_from_0_10_to_0_11 <= 1
 0 <= x_train_1_from_1_10_to_0_11 <= 1
 0 <= x_train_1_from_2_10_to_0_11 <= 1
 0 <= y_train_1_node_0_11 <= 1
 0 <= x_train_1_from_0_11_to_0_12 <= 1
 0 <= x_train_1_from_1_11_to_0_12 <= 1
 0 <= x_train_1_from_2_11_to_0_12 <= 1
 0 <= y_train_1_node_0_12 <= 1
 0 <= x_train_1_from_0_12_to_0_13 <= 1
 0 <= x_train_1_from_1_12_to_0_13 <= 1
 0 <= x_train_1_from_2_12_to_0_13 <= 1
 0 <= y_train_1_node_0_13 <= 1
 0 <= x_train_1_from_0_13_to_0_14 <= 1
 0 <= x_train_1_from_1_13_to_0_14 <= 1
 0 <= x_train_1_from_2_13_to_0_14 <= 1
 0 <= y_train_1_node_0_14 <= 1
 0 <= x_train_1_from_0_14_to_0_15 <= 1
 0 <= x_train_1_from_1_14_to_0_15 <= 1
 0 <= x_train_1_from_2_14_to_0_15 <= 1
 0 <= y_train_1_node_0_15 <= 1
 0 <= x_train_1_from_0_15_to_0_16 <= 1
 0 <= x_train_1_from_1_15_to_0_16 <= 1
 0 <= x_train_1_from_2_15_to_0_16 <= 1
 0 <= y_train_1_node_0_16 <= 1
 0 <= x_train_1_from_0_16_to_0_17 <= 1
 0 <= x_train_1_from_1_16_to_0_17 <= 1
 0 <= x_train_1_from_2_16_to_0_17 <= 1
 0 <= y_train_1_node_0_17 <= 1
 0 <= x_train_1_from_0_17_to_0_18 <= 1
 0 <= x_train_1_from_1_17_to_0_18 <= 1
 0 <= x_train_1_from_2_17_to_0_18 <= 1
 0 <= y_train_1_node_0_18 <= 1
 0 <= x_train_1_from_0_18_to_0_19 <= 1
 0 <= x_train_1_from_1_18_to_0_19 <= 1
 0 <= x_train_1_from_2_18_to_0_19 <= 1
 0 <= y_train_1_node_0_19 <= 1
 0 <= x_train_1_from_0_19_to_0_20 <= 1
 0 <= x_train_1_from_1_19_to_0_20 <= 1
 0 <= x_train_1_from_2_19_to_0_20 <= 1
 0 <= y_train_1_node_0_20 <= 1
 0 <= x_train_1_from_0_20_to_0_21 <= 1
 0 <= x_train_1_from_1_20_to_0_21 <= 1
 0 <= x_train_1_from_2_20_to_0_21 <= 1
 0 <= y_train_1_node_0_21 <= 1
 0 <= x_train_1_from_0_21_to_0_22 <= 1
 0 <= x_train_1_from_1_21_to_0_22 <= 1
 0 <= x_train_1_from_2_21_to_0_22 <= 1
 0 <= y_train_1_node_0_22 <= 1
 0 <= x_train_1_from_0_22_to_0_23 <= 1
 0 <= x_train_1_from_1_22_to_0_23 <= 1
 0 <= x_train_1_from_2_22_to_0_23 <= 1
 0 <= y_train_1_node_0_23 <= 1
 0 <= x_train_1_from_0_23_to_0_24 <= 1
 0 <= x_train_1_from_1_23_to_0_24 <= 1
 0 <= x_train_1_from_2_23_to_0_24 <= 1
 0 <= y_train_1_node_0_24 <= 1
 0 <= x_train_1_from_0_24_to_0_25 <= 1
 0 <= x_train_1_from_1_24_to_0_25 <= 1
 0 <= x_train_1_from_2_24_to_0_25 <= 1
 0 <= y_train_1_node_0_25 <= 1
 0 <= x_train_1_from_0_25_to_0_26 <= 1
 0 <= x_train_1_from_1_25_to_0_26 <= 1
 0 <= x_train_1_from_2_25_to_0_26 <= 1
 0 <= y_train_1_node_0_26 <= 1
 0 <= x_train_1_from_0_26_to_0_27 <= 1
 0 <= x_train_1_from_1_26_to_0_27 <= 1
 0 <= x_train_1_from_2_26_to_0_27 <= 1
 0 <= y_train_1_node_0_27 <= 1
 0 <= x_train_1_from_0_27_to_0_28 <= 1
 0 <= x_train_1_from_1_27_to_0_28 <= 1
 0 <= x_train_1_from_2_27_to_0_28 <= 1
 0 <= y_train_1_node_0_28 <= 1
 0 <= x_train_1_from_0_28_to_0_29 <= 1
 0 <= x_train_1_from_1_28_to_0_29 <= 1
 0 <= x_train_1_from_2_28_to_0_29 <= 1
 0 <= y_train_1_node_0_29 <= 1
 0 <= x_train_1_from_0_29_to_0_30 <= 1
 0 <= x_train_1_from_1_29_to_0_30 <= 1
 0 <= x_train_1_from_2_29_to_0_30 <= 1
 0 <= y_train_1_node_0_30 <= 1
 0 <= x_train_1_from_3_5_to_1_6 <= 1
 0 <= y_train_1_node_1_6 <= 1
 0 <= x_train_1_from_1_6_to_1_7 <= 1
 0 <= x_train_1_from_3_6_to_1_7 <= 1
 0 <= y_train_1_node_1_7 <= 1
 0 <= x_train_1_from_1_7_to_1_8 <= 1
 0 <= x_train_1_from_3_7_to_1_8 <= 1
 0 <= y_train_1_node_1_8 <= 1
 0 <= x_train_1_from_1_8_to_1_9 <= 1
 0 <= x_train_1_from_3_8_to_1_9 <= 1
 0 <= y_train_1_node_1_9 <= 1
 0 <= x_train_1_from_1_9_to_1_10 <= 1
 0 <= x_train_1_from_3_9_to_1_10 <= 1
 0 <= y_train_1_node_1_10 <= 1
 0 <= x_train_1_from_1_10_to_1_11 <= 1
 0 <= x_train_1_from_3_10_to_1_11 <= 1
 0 <= y_train_1_node_1_11 <= 1
 0 <= x_train_1_from_1_11_to_1_12 <= 1
 0 <= x_train_1_from_3_11_to_1_12 <= 1
 0 <= y_train_1_node_1_12 <= 1
 0 <= x_train_1_from_1_12_to_1_13 <= 1
 0 <= x_train_1_from_3_12_to_1_13 <= 1
 0 <= y_train_1_node_1_13 <= 1
 0 <= x_train_1_from_1_13_to_1_14 <= 1
 0 <= x_train_1_from_3_13_to_1_14 <= 1
 0 <= y_train_1_node_1_14 <= 1
 0 <= x_train_1_from_1_14_to_1_15 <= 1
 0 <= x_train_1_from_3_14_to_1_15 <= 1
 0 <= y_train_1_node_1_15 <= 1
 0 <= x_train_1_from_1_15_to_1_16 <= 1
 0 <= x_train_1_from_3_15_to_1_16 <= 1
 0 <= y_train_1_node_1_16 <= 1
 0 <= x_train_1_from_1_16_to_1_17 <= 1
 0 <= x_train_1_from_3_16_to_1_17 <= 1
 0 <= y_train_1_node_1_17 <= 1
 0 <= x_train_1_from_1_17_to_1_18 <= 1
 0 <= x_train_1_from_3_17_to_1_18 <= 1
 0 <= y_train_1_node_1_18 <= 1
 0 <= x_train_1_from_1_18_to_1_19 <= 1
 0 <= x_train_1_from_3_18_to_1_19 <= 1
 0 <= y_train_1_node_1_19 <= 1
 0 <= x_train_1_from_1_19_to_1_20 <= 1
 0 <= x_train_1_from_3_19_to_1_20 <= 1
 0 <= y_train_1_node_1_20 <= 1
 0 <= x_train_1_from_1_20_to_1_21 <= 1
 0 <= x_train_1_from_3_20_to_1_21 <= 1
 0 <= y_train_1_node_1_21 <= 1
 0 <= x_train_1_from_1_21_to_1_22 <= 1
 0 <= x_train_1_from_3_21_to_1_22 <= 1
 0 <= y_train_1_node_1_22 <= 1
 0 <= x_train_1_from_1_22_to_1_23 <= 1
 0 <= x_train_1_from_3_22_to_1_23 <= 1
 0 <= y_train_1_node_1_23 <= 1
 0 <= x_train_1_from_1_23_to_1_24 <= 1
 0 <= x_train_1_from_3_23_to_1_24 <= 1
 0 <= y_train_1_node_1_24 <= 1
 0 <= x_train_1_from_1_24_to_1_25 <= 1
 0 <= x_train_1_from_3_24_to_1_25 <= 1
 0 <= y_train_1_node_1_25 <= 1
 0 <= x_train_1_from_1_25_to_1_26 <= 1
 0 <= x_train_1_from_3_25_to_1_26 <= 1
 0 <= y_train_1_node_1_26 <= 1
 0 <= x_train_1_from_1_26_to_1_27 <= 1
 0 <= x_train_1_from_3_26_to_1_27 <= 1
 0 <= y_train_1_node_1_27 <= 1
 0 <= x_train_1_from_1_27_to_1_28 <= 1
 0 <= x_train_1_from_3_27_to_1_28 <= 1
 0 <= y_train_1_node_1_28 <= 1
 0 <= x_train_1_from_1_28_to_1_29 <= 1
 0 <= x_train_1_from_3_28_to_1_29 <= 1
 0 <= y_train_1_node_1_29 <= 1
 0 <= x_train_1_from_4_2_to_2_3 <= 1
 0 <= y_train_1_node_2_3 <= 1
 0 <= x_train_1_from_2_3_to_2_4 <= 1
 0 <= x_train_1_from_4_3_to_2_4 <= 1
 0 <= y_train_1_node_2_4 <= 1
 0 <= x_train_1_from_2_4_to_2_5 <= 1
 0 <= x_train_1_from_4_4_to_2_5 <= 1
 0 <= y_train_1_node_2_5 <= 1
 0 <= x_train_1_from_2_5_to_2_6 <= 1
 0 <= x_train_1_from_4_5_to_2_6 <= 1
 0 <= y_train_1_node_2_6 <= 1
 0 <= x_train_1_from_2_6_to_2_7 <= 1
 0 <= x_train_1_from_4_6_to_2_7 <= 1
 0 <= y_train_1_node_2_7 <= 1
 0 <= x_train_1_from_2_7_to_2_8 <= 1
 0 <= x_train_1_from_4_7_to_2_8 <= 1
 0 <= y_train_1_node_2_8 <= 1
 0 <= x_train_1_from_2_8_to_2_9 <= 1
 0 <= x_train_1_from_4_8_to_2_9 <= 1
 0 <= y_train_1_node_2_9 <= 1
 0 <= x_train_1_from_2_9_to_2_10 <= 1
 0 <= x_train_1_from_4_9_to_2_10 <= 1
 0 <= y_train_1_node_2_10 <= 1
 0 <= x_train_1_from_2_10_to_2_11 <= 1
 0 <= x_train_1_from_4_10_to_2_11 <= 1
 0 <= y_train_1_node_2_11 <= 1
 0 <= x_train_1_from_2_11_to_2_12 <= 1
 0 <= x_train_1_from_4_11_to_2_12 <= 1
 0 <= y_train_1_node_2_12 <= 1
 0 <= x_train_1_from_2_12_to_2_13 <= 1
 0 <= x_train_1_from_4_12_to_2_13 <= 1
 0 <= y_train_1_node_2_13 <= 1
 0 <= x_train_1_from_2_13_to_2_14 <= 1
 0 <= x_train_1_from_4_13_to_2_14 <= 1
 0 <= y_train_1_node_2_14 <= 1
 0 <= x_train_1_from_2_14_to_2_15 <= 1
 0 <= x_train_1_from_4_14_to_2_15 <= 1
 0 <= y_train_1_node_2_15 <= 1
 0 <= x_train_1_from_2_15_to_2_16 <= 1
 0 <= x_train_1_from_4_15_to_2_16 <= 1
 0 <= y_train_1_node_2_16 <= 1
 0 <= x_train_1_from_2_16_to_2_17 <= 1
 0 <= x_train_1_from_4_16_to_2_17 <= 1
 0 <= y_train_1_node_2_17 <= 1
 0 <= x_train_1_from_2_17_to_2_18 <= 1
 0 <= x_train_1_from_4_17_to_2_18 <= 1
 0 <= y_train_1_node_2_18 <= 1
 0 <= x_train_1_from_2_18_to_2_19 <= 1
 0 <= x_train_1_from_4_18_to_2_19 <= 1
 0 <= y_train_1_node_2_19 <= 1
 0 <= x_train_1_from_2_19_to_2_20 <= 1
 0 <= x_train_1_from_4_19_to_2_20 <= 1
 0 <= y_train_1_node_2_20 <= 1
 0 <= x_train_1_from_2_20_to_2_21 <= 1
 0 <= x_train_1_from_4_20_to_2_21 <= 1
 0 <= y_train_1_node_2_21 <= 1
 0 <= x_train_1_from_2_21_to_2_22 <= 1
 0 <= x_train_1_from_4_21_to_2_22 <= 1
 0 <= y_train_1_node_2_22 <= 1
 0 <= x_train_1_from_2_22_to_2_23 <= 1
 0 <= x_train_1_from_4_22_to_2_23 <= 1
 0 <= y_train_1_node_2_23 <= 1
 0 <= x_train_1_from_2_23_to_2_24 <= 1
 0 <= x_train_1_from_4_23_to_2_24 <= 1
 0 <= y_train_1_node_2_24 <= 1
 0 <= x_train_1_from_2_24_to_2_25 <= 1
 0 <= x_train_1_from_4_24_to_2_25 <= 1
 0 <= y_train_1_node_2_25 <= 1
 0 <= x_train_1_from_2_25_to_2_26 <= 1
 0 <= x_train_1_from_4_25_to_2_26 <= 1
 0 <= y_train_1_node_2_26 <= 1
 0 <= x_train_1_from_2_26_to_2_27 <= 1
 0 <= x_train_1_from_4_26_to_2_27 <= 1
 0 <= y_train_1_node_2_27 <= 1
 0 <= x_train_1_from_2_27_to_2_28 <= 1
 0 <= x_train_1_from_4_27_to_2_28 <= 1
 0 <= y_train_1_node_2_28 <= 1
 0 <= x_train_1_from_2_28_to_2_29 <= 1
 0 <= x_train_1_from_4_28_to_2_29 <= 1
 0 <= y_train_1_node_2_29 <= 1
 0 <= x_train_1_from_5_4_to_3_5 <= 1
 0 <= y_train_1_node_3_5 <= 1
 0 <= x_train_1_from_3_5_to_3_6 <= 1
 0 <= x_train_1_from_5_5_to_3_6 <= 1
 0 <= y_train_1_node_3_6 <= 1
 0 <= x_train_1_from_3_6_to_3_7 <= 1
 0 <= x_train_1_from_5_6_to_3_7 <= 1
 0 <= y_train_1_node_3_7 <= 1
 0 <= x_train_1_from_3_7_to_3_8 <= 1
 0 <= x_train_1_from_5_7_to_3_8 <= 1
 0 <= y_train_1_node_3_8 <= 1
 0 <= x_train_1_from_3_8_to_3_9 <= 1
 0 <= x_train_1_from_5_8_to_3_9 <= 1
 0 <= y_train_1_node_3_9 <= 1
 0 <= x_train_1_from_3_9_to_3_10 <= 1
 0 <= x_train_1_from_5_9_to_3_10 <= 1
 0 <= y_train_1_node_3_10 <= 1
 0 <= x_train_1_from_3_10_to_3_11 <= 1
 0 <= x_train_1_from_5_10_to_3_11 <= 1
 0 <= y_train_1_node_3_11 <= 1
 0 <= x_train_1_from_3_11_to_3_12 <= 1
 0 <= x_train_1_from_5_11_to_3_12 <= 1
 0 <= y_train_1_node_3_12 <= 1
 0 <= x_train_1_from_3_12_to_3_13 <= 1
 0 <= x_train_1_from_5_12_to_3_13 <= 1
 0 <= y_train_1_node_3_13 <= 1
 0 <= x_train_1_from_3_13_to_3_14 <= 1
 0 <= x_train_1_from_5_13_to_3_14 <= 1
 0 <= y_train_1_node_3_14 <= 1
 0 <= x_train_1_from_3_14_to_3_15 <= 1
 0 <= x_train_1_from_5_14_to_3_15 <= 1
 0 <= y_train_1_node_3_15 <= 1
 0 <= x_train_1_from_3_15_to_3_16 <= 1
 0 <= x_train_1_from_5_15_to_3_16 <= 1
 0 <= y_train_1_node_3_16 <= 1
 0 <= x_train_1_from_3_16_to_3_17 <= 1
 0 <= x_train_1_from_5_16_to_3_17 <= 1
 0 <= y_train_1_node_3_17 <= 1
 0 <= x_train_1_from_3_17_to_3_18 <= 1
 0 <= x_train_1_from_5_17_to_3_18 <= 1
 0 <= y_train_1_node_3_18 <= 1
 0 <= x_train_1_from_3_18_to_3_19 <= 1
 0 <= x_train_1_from_5_18_to_3_19 <= 1
 0 <= y_train_1_node_3_19 <= 1
 0 <= x_train_1_from_3_19_to_3_20 <= 1
 0 <= x_train_1_from_5_19_to_3_20 <= 1
 0 <= y_train_1_node_3_20 <= 1
 0 <= x_train_1_from_3_20_to_3_21 <= 1
 0 <= x_train_1_from_5_20_to_3_21 <= 1
 0 <= y_train_1_node_3_21 <= 1
 0 <= x_train_1_from_3_21_to_3_22 <= 1
 0 <= x_train_1_from_5_21_to_3_22 <= 1
 0 <= y_train_1_node_3_22 <= 1
 0 <= x_train_1_from_3_22_to_3_23 <= 1
 0 <= x_train_1_from_5_22_to_3_23 <= 1
 0 <= y_train_1_node_3_23 <= 1
 0 <= x_train_1_from_3_23_to_3_24 <= 1
 0 <= x_train_1_from_5_23_to_3_24 <= 1
 0 <= y_train_1_node_3_24 <= 1
 0 <= x_train_1_from_3_24_to_3_25 <= 1
 0 <= x_train_1_from_5_24_to_3_25 <= 1
 0 <= y_train_1_node_3_25 <= 1
 0 <= x_train_1_from_3_25_to_3_26 <= 1
 0 <= x_train_1_from_5_25_to_3_26 <= 1
 0 <= y_train_1_node_3_26 <= 1
 0 <= x_train_1_from_3_26_to_3_27 <= 1
 0 <= x_train_1_from_5_26_to_3_27 <= 1
 0 <= y_train_1_node_3_27 <= 1
 0 <= x_train_1_from_3_27_to_3_28 <= 1
 0 <= x_train_1_from_5_27_to_3_28 <= 1
 0 <= y_train_1_node_3_28 <= 1
 0 <= x_train_1_from_6_1_to_4_2 <= 1
 0 <= y_train_1_node_4_2 <= 1
 0 <= x_train_1_from_4_2_to_4_3 <= 1
 0 <= x_train_1_from_6_2_to_4_3 <= 1
 0 <= y_train_1_node_4_3 <= 1
 0 <= x_train_1_from_4_3_to_4_4 <= 1
 0 <= x_train_1_from_6_3_to_4_4 <= 1
 0 <= y_train_1_node_4_4 <= 1
 0 <= x_train_1_from_4_4_to_4_5 <= 1
 0 <= x_train_1_from_6_4_to_4_5 <= 1
 0 <= y_train_1_node_4_5 <= 1
 0 <= x_train_1_from_4_5_to_4_6 <= 1
 0 <= x_train_1_from_6_5_to_4_6 <= 1
 0 <= y_train_1_node_4_6 <= 1
 0 <= x_train_1_from_4_6_to_4_7 <= 1
 0 <= x_train_1_from_6_6_to_4_7 <= 1
 0 <= y_train_1_node_4_7 <= 1
 0 <= x_train_1_from_4_7_to_4_8 <= 1
 0 <= x_train_1_from_6_7_to_4_8 <= 1
 0 <= y_train_1_node_4_8 <= 1
 0 <= x_train_1_from_4_8_to_4_9 <= 1
 0 <= x_train_1_from_6_8_to_4_9 <= 1
 0 <= y_train_1_node_4_9 <= 1
 0 <= x_train_1_from_4_9_to_4_10 <= 1
 0 <= x_train_1_from_6_9_to_4_10 <= 1
 0 <= y_train_1_node_4_10 <= 1
 0 <= x_train_1_from_4_10_to_4_11 <= 1
 0 <= x_train_1_from_6_10_to_4_11 <= 1
 0 <= y_train_1_node_4_11 <= 1
 0 <= x_train_1_from_4_11_to_4_12 <= 1
 0 <= x_train_1_from_6_11_to_4_12 <= 1
 0 <= y_train_1_node_4_12 <= 1
 0 <= x_train_1_from_4_12_to_4_13 <= 1
 0 <= x_train_1_from_6_12_to_4_13 <= 1
 0 <= y_train_1_node_4_13 <= 1
 0 <= x_train_1_from_4_13_to_4_14 <= 1
 0 <= x_train_1_from_6_13_to_4_14 <= 1
 0 <= y_train_1_node_4_14 <= 1
 0 <= x_train_1_from_4_14_to_4_15 <= 1
 0 <= x_train_1_from_6_14_to_4_15 <= 1
 0 <= y_train_1_node_4_15 <= 1
 0 <= x_train_1_from_4_15_to_4_16 <= 1
 0 <= x_train_1_from_6_15_to_4_16 <= 1
 0 <= y_train_1_node_4_16 <= 1
 0 <= x_train_1_from_4_16_to_4_17 <= 1
 0 <= x_train_1_from_6_16_to_4_17 <= 1
 0 <= y_train_1_node_4_17 <= 1
 0 <= x_train_1_from_4_17_to_4_18 <= 1
 0 <= x_train_1_from_6_17_to_4_18 <= 1
 0 <= y_train_1_node_4_18 <= 1
 0 <= x_train_1_from_4_18_to_4_19 <= 1
 0 <= x_train_1_from_6_18_to_4_19 <= 1
 0 <= y_train_1_node_4_19 <= 1
 0 <= x_train_1_from_4_19_to_4_20 <= 1
 0 <= x_train_1_from_6_19_to_4_20 <= 1
 0 <= y_train_1_node_4_20 <= 1
 0 <= x_train_1_from_4_20_to_4_21 <= 1
 0 <= x_train_1_from_6_20_to_4_21 <= 1
 0 <= y_train_1_node_4_21 <= 1
 0 <= x_train_1_from_4_21_to_4_22 <= 1
 0 <= x_train_1_from_6_21_to_4_22 <= 1
 0 <= y_train_1_node_4_22 <= 1
 0 <= x_train_1_from_4_22_to_4_23 <= 1
 0 <= x_train_1_from_6_22_to_4_23 <= 1
 0 <= y_train_1_node_4_23 <= 1
 0 <= x_train_1_from_4_23_to_4_24 <= 1
 0 <= x_train_1_from_6_23_to_4_24 <= 1
 0 <= y_train_1_node_4_24 <= 1
 0 <= x_train_1_from_4_24_to_4_25 <= 1
 0 <= x_train_1_from_6_24_to_4_25 <= 1
 0 <= y_train_1_node_4_25 <= 1
 0 <= x_train_1_from_4_25_to_4_26 <= 1
 0 <= x_train_1_from_6_25_to_4_26 <= 1
 0 <= y_train_1_node_4_26 <= 1
 0 <= x_train_1_from_4_26_to_4_27 <= 1
 0 <= x_train_1_from_6_26_to_4_27 <= 1
 0 <= y_train_1_node_4_27 <= 1
 0 <= x_train_1_from_4_27_to_4_28 <= 1
 0 <= x_train_1_from_6_27_to_4_28 <= 1
 0 <= y_train_1_node_4_28 <= 1
 0 <= x_train_1_from_6_1_to_5_2 <= 1
 0 <= y_train_1_node_5_2 <= 1
 0 <= x_train_1_from_5_2_to_5_3 <= 1
 0 <= x_train_1_from_6_2_to_5_3 <= 1
 0 <= y_train_1_node_5_3 <= 1
 0 <= x_train_1_from_5_3_to_5_4 <= 1
 0 <= x_train_1_from_6_3_to_5_4 <= 1
 0 <= y_train_1_node_5_4 <= 1
 0 <= x_train_1_from_5_4_to_5_5 <= 1
 0 <= x_train_1_from_6_4_to_5_5 <= 1
 0 <= y_train_1_node_5_5 <= 1
 0 <= x_train_1_from_5_5_to_5_6 <= 1
 0 <= x_train_1_from_6_5_to_5_6 <= 1
 0 <= y_train_1_node_5_6 <= 1
 0 <= x_train_1_from_5_6_to_5_7 <= 1
 0 <= x_train_1_from_6_6_to_5_7 <= 1
 0 <= y_train_1_node_5_7 <= 1
 0 <= x_train_1_from_5_7_to_5_8 <= 1
 0 <= x_train_1_from_6_7_to_5_8 <= 1
 0 <= y_train_1_node_5_8 <= 1
 0 <= x_train_1_from_5_8_to_5_9 <= 1
 0 <= x_train_1_from_6_8_to_5_9 <= 1
 0 <= y_train_1_node_5_9 <= 1
 0 <= x_train_1_from_5_9_to_5_10 <= 1
 0 <= x_train_1_from_6_9_to_5_10 <= 1
 0 <= y_train_1_node_5_10 <= 1
 0 <= x_train_1_from_5_10_to_5_11 <= 1
 0 <= x_train_1_from_6_10_to_5_11 <= 1
 0 <= y_train_1_node_5_11 <= 1
 0 <= x_train_1_from_5_11_to_5_12 <= 1
 0 <= x_train_1_from_6_11_to_5_12 <= 1
 0 <= y_train_1_node_5_12 <= 1
 0 <= x_train_1_from_5_12_to_5_13 <= 1
 0 <= x_train_1_from_6_12_to_5_13 <= 1
 0 <= y_train_1_node_5_13 <= 1
 0 <= x_train_1_from_5_13_to_5_14 <= 1
 0 <= x_train_1_from_6_13_to_5_14 <= 1
 0 <= y_train_1_node_5_14 <= 1
 0 <= x_train_1_from_5_14_to_5_15 <= 1
 0 <= x_train_1_from_6_14_to_5_15 <= 1
 0 <= y_train_1_node_5_15 <= 1
 0 <= x_train_1_from_5_15_to_5_16 <= 1
 0 <= x_train_1_from_6_15_to_5_16 <= 1
 0 <= y_train_1_node_5_16 <= 1
 0 <= x_train_1_from_5_16_to_5_17 <= 1
 0 <= x_train_1_from_6_16_to_5_17 <= 1
 0 <= y_train_1_node_5_17 <= 1
 0 <= x_train_1_from_5_17_to_5_18 <= 1
 0 <= x_train_1_from_6_17_to_5_18 <= 1
 0 <= y_train_1_node_5_18 <= 1
 0 <= x_train_1_from_5_18_to_5_19 <= 1
 0 <= x_train_1_from_6_18_to_5_19 <= 1
 0 <= y_train_1_node_5_19 <= 1
 0 <= x_train_1_from_5_19_to_5_20 <= 1
 0 <= x_train_1_from_6_19_to_5_20 <= 1
 0 <= y_train_1_node_5_20 <= 1
 0 <= x_train_1_from_5_20_to_5_21 <= 1
 0 <= x_train_1_from_6_20_to_5_21 <= 1
 0 <= y_train_1_node_5_21 <= 1
 0 <= x_train_1_from_5_21_to_5_22 <= 1
 0 <= x_train_1_from_6_21_to_5_22 <= 1
 0 <= y_train_1_node_5_22 <= 1
 0 <= x_train_1_from_5_22_to_5_23 <= 1
 0 <= x_train_1_from_6_22_to_5_23 <= 1
 0 <= y_train_1_node_5_23 <= 1
 0 <= x_train_1_from_5_23_to_5_24 <= 1
 0 <= x_train_1_from_6_23_to_5_24 <= 1
 0 <= y_train_1_node_5_24 <= 1
 0 <= x_train_1_from_5_24_to_5_25 <= 1
 0 <= x_train_1_from_6_24_to_5_25 <= 1
 0 <= y_train_1_node_5_25 <= 1
 0 <= x_train_1_from_5_25_to_5_26 <= 1
 0 <= x_train_1_from_6_25_to_5_26 <= 1
 0 <= y_train_1_node_5_26 <= 1
 0 <= x_train_1_from_5_26_to_5_27 <= 1
 0 <= x_train_1_from_6_26_to_5_27 <= 1
 0 <= y_train_1_node_5_27 <= 1
 0 <= y_train_1_node_6_1 <= 1
 0 <= x_train_1_from_6_1_to_6_2 <= 1
 0 <= y_train_1_node_6_2 <= 1
 0 <= x_train_1_from_6_2_to_6_3 <= 1
 0 <= y_train_1_node_6_3 <= 1
 0 <= x_train_1_from_6_3_to_6_4 <= 1
 0 <= y_train_1_node_6_4 <= 1
 0 <= x_train_1_from_6_4_to_6_5 <= 1
 0 <= y_train_1_node_6_5 <= 1
 0 <= x_train_1_from_6_5_to_6_6 <= 1
 0 <= y_train_1_node_6_6 <= 1
 0 <= x_train_1_from_6_6_to_6_7 <= 1
 0 <= y_train_1_node_6_7 <= 1
 0 <= x_train_1_from_6_7_to_6_8 <= 1
 0 <= y_train_1_node_6_8 <= 1
 0 <= x_train_1_from_6_8_to_6_9 <= 1
 0 <= y_train_1_node_6_9 <= 1
 0 <= x_train_1_from_6_9_to_6_10 <= 1
 0 <= y_train_1_node_6_10 <= 1
 0 <= x_train_1_from_6_10_to_6_11 <= 1
 0 <= y_train_1_node_6_11 <= 1
 0 <= x_train_1_from_6_11_to_6_12 <= 1
 0 <= y_train_1_node_6_12 <= 1
 0 <= x_train_1_from_6_12_to_6_13 <= 1
 0 <= y_train_1_node_6_13 <= 1
 0 <= x_train_1_from_6_13_to_6_14 <= 1
 0 <= y_train_1_node_6_14 <= 1
 0 <= x_train_1_from_6_14_to_6_15 <= 1
 0 <= y_train_1_node_6_15 <= 1
 0 <= x_train_1_from_6_15_to_6_16 <= 1
 0 <= y_train_1_node_6_16 <= 1
 0 <= x_train_1_from_6_16_to_6_17 <= 1
 0 <= y_train_1_node_6_17 <= 1
 0 <= x_train_1_from_6_17_to_6_18 <= 1
 0 <= y_train_1_node_6_18 <= 1
 0 <= x_train_1_from_6_18_to_6_19 <= 1
 0 <= y_train_1_node_6_19 <= 1
 0 <= x_train_1_from_6_19_to_6_20 <= 1
 0 <= y_train_1_node_6_20 <= 1
 0 <= x_train_1_from_6_20_to_6_21 <= 1
 0 <= y_train_1_node_6_21 <= 1
 0 <= x_train_1_from_6_21_to_6_22 <= 1
 0 <= y_train_1_node_6_22 <= 1
 0 <= x_train_1_from_6_22_to_6_23 <= 1
 0 <= y_train_1_node_6_23 <= 1
 0 <= x_train_1_from_6_23_to_6_24 <= 1
 0 <= y_train_1_node_6_24 <= 1
 0 <= x_train_1_from_6_24_to_6_25 <= 1
 0 <= y_train_1_node_6_25 <= 1
 0 <= x_train_1_from_6_25_to_6_26 <= 1
 0 <= y_train_1_node_6_26 <= 1
 0 <= x_train_1_from_6_26_to_6_27 <= 1
 0 <= y_train_1_node_6_27 <= 1
Binaries
 z_train_0_segment_0  z_train_0_segment_1  z_train_0_segment_2 
 z_train_0_segment_3  z_train_0_segment_4  z_train_0_segment_5 
 z_train_0_segment_6  z_train_1_segment_0  z_train_1_segment_1 
 z_train_1_segment_2  z_train_1_segment_3  z_train_1_segment_4 
 z_train_1_segment_5  z_train_1_segment_6  x_train_0_from_sigma_to_0_1 
 x_train_0_from_sigma_to_0_2  x_train_0_from_sigma_to_0_3 
 x_train_0_from_sigma_to_0_4  x_train_0_from_sigma_to_0_5 
 x_train_0_from_sigma_to_0_6  x_train_0_from_sigma_to_0_7 
 x_train_0_from_sigma_to_0_8  x_train_0_from_sigma_to_0_9 
 x_train_0_from_sigma_to_0_10  x_train_0_from_sigma_to_0_11 
 x_train_0_from_sigma_to_0_12  x_train_0_from_sigma_to_0_13 
 x_train_0_from_sigma_to_0_14  x_train_0_from_sigma_to_0_15 
 x_train_0_from_sigma_to_0_16  x_train_0_from_sigma_to_0_17 
 x_train_0_from_sigma_to_0_18  x_train_0_from_sigma_to_0_19 
 x_train_0_from_sigma_to_0_20  x_train_0_from_sigma_to_0_21 
 x_train_0_from_sigma_to_0_22  x_train_0_from_sigma_to_0_23 
 x_train_0_from_sigma_to_0_24  x_train_0_from_sigma_to_0_25 
 x_train_0_from_sigma_to_0_26  x_train_0_from_sigma_to_0_27 
 x_train_1_from_sigma_to_6_1  x_train_1_from_sigma_to_6_2 
 x_train_1_from_sigma_to_6_3  x_train_1_from_sigma_to_6_4 
 x_train_1_from_sigma_to_6_5  x_train_1_from_sigma_to_6_6 
 x_train_1_from_sigma_to_6_7  x_train_1_from_sigma_to_6_8 
 x_train_1_from_sigma_to_6_9  x_train_1_from_sigma_to_6_10 
 x_train_1_from_sigma_to_6_11  x_train_1_from_sigma_to_6_12 
 x_train_1_from_sigma_to_6_13  x_train_1_from_sigma_to_6_14 
 x_train_1_from_sigma_to_6_15  x_train_1_from_sigma_to_6_16 
 x_train_1_from_sigma_to_6_17  x_train_1_from_sigma_to_6_18 
 x_train_1_from_sigma_to_6_19  x_train_1_from_sigma_to_6_20 
 x_train_1_from_sigma_to_6_21  x_train_1_from_sigma_to_6_22 
 x_train_1_from_sigma_to_6_23  x_train_1_from_sigma_to_6_24 
 x_train_1_from_sigma_to_6_25  x_train_1_from_sigma_to_6_26 
 x_train_1_from_sigma_to_6_27  x_train_0_from_6_4_to_tau 
 x_train_0_from_6_5_to_tau  x_train_0_from_6_6_to_tau 
 x_train_0_from_6_7_to_tau  x_train_0_from_6_8_to_tau 
 x_train_0_from_6_9_to_tau  x_train_0_from_6_10_to_tau 
 x_train_0_from_6_11_to_tau  x_train_0_from_6_12_to_tau 
 x_train_0_from_6_13_to_tau  x_train_0_from_6_14_to_tau 
 x_train_0_from_6_15_to_tau  x_train_0_from_6_16_to_tau 
 x_train_0_from_6_17_to_tau  x_train_0_from_6_18_to_tau 
 x_train_0_from_6_19_to_tau  x_train_0_from_6_20_to_tau 
 x_train_0_from_6_21_to_tau  x_train_0_from_6_22_to_tau 
 x_train_0_from_6_23_to_tau  x_train_0_from_6_24_to_tau 
 x_train_0_from_6_25_to_tau  x_train_0_from_6_26_to_tau 
 x_train_0_from_6_27_to_tau  x_train_0_from_6_28_to_tau 
 x_train_0_from_6_29_to_tau  x_train_0_from_6_30_to_tau 
 x_train_1_from_0_4_to_tau  x_train_1_from_0_5_to_tau 
 x_train_1_from_0_6_to_tau  x_train_1_from_0_7_to_tau 
 x_train_1_from_0_8_to_tau  x_train_1_from_0_9_to_tau 
 x_train_1_from_0_10_to_tau  x_train_1_from_0_11_to_tau 
 x_train_1_from_0_12_to_tau  x_train_1_from_0_13_to_tau 
 x_train_1_from_0_14_to_tau  x_train_1_from_0_15_to_tau 
 x_train_1_from_0_16_to_tau  x_train_1_from_0_17_to_tau 
 x_train_1_from_0_18_to_tau  x_train_1_from_0_19_to_tau 
 x_train_1_from_0_20_to_tau  x_train_1_from_0_21_to_tau 
 x_train_1_from_0_22_to_tau  x_train_1_from_0_23_to_tau 
 x_train_1_from_0_24_to_tau  x_train_1_from_0_25_to_tau 
 x_train_1_from_0_26_to_tau  x_train_1_from_0_27_to_tau 
 x_train_1_from_0_28_to_tau  x_train_1_from_0_29_to_tau 
 x_train_1_from_0_30_to_tau  y_train_0_node_0_1  x_train_0_from_0_1_to_0_2 
 y_train_0_node_0_2  x_train_0_from_0_2_to_0_3  y_train_0_node_0_3 
 x_train_0_from_0_3_to_0_4  y_train_0_node_0_4  x_train_0_from_0_4_to_0_5 
 y_train_0_node_0_5  x_train_0_from_0_5_to_0_6  y_train_0_node_0_6 
 x_train_0_from_0_6_to_0_7  y_train_0_node_0_7  x_train_0_from_0_7_to_0_8 
 y_train_0_node_0_8  x_train_0_from_0_8_to_0_9  y_train_0_node_0_9 
 x_train_0_from_0_9_to_0_10  y_train_0_node_0_10  x_train_0_from_0_10_to_0_11 
 y_train_0_node_0_11  x_train_0_from_0_11_to_0_12  y_train_0_node_0_12 
 x_train_0_from_0_12_to_0_13  y_train_0_node_0_13  x_train_0_from_0_13_to_0_14 
 y_train_0_node_0_14  x_train_0_from_0_14_to_0_15  y_train_0_node_0_15 
 x_train_0_from_0_15_to_0_16  y_train_0_node_0_16  x_train_0_from_0_16_to_0_17 
 y_train_0_node_0_17  x_train_0_from_0_17_to_0_18  y_train_0_node_0_18 
 x_train_0_from_0_18_to_0_19  y_train_0_node_0_19  x_train_0_from_0_19_to_0_20 
 y_train_0_node_0_20  x_train_0_from_0_20_to_0_21  y_train_0_node_0_21 
 x_train_0_from_0_21_to_0_22  y_train_0_node_0_22  x_train_0_from_0_22_to_0_23 
 y_train_0_node_0_23  x_train_0_from_0_23_to_0_24  y_train_0_node_0_24 
 x_train_0_from_0_24_to_0_25  y_train_0_node_0_25  x_train_0_from_0_25_to_0_26 
 y_train_0_node_0_26  x_train_0_from_0_26_to_0_27  y_train_0_node_0_27 
 x_train_0_from_0_1_to_1_2  y_train_0_node_1_2  x_train_0_from_0_2_to_1_3 
 x_train_0_from_1_2_to_1_3  y_train_0_node_1_3  x_train_0_from_0_3_to_1_4 
 x_train_0_from_1_3_to_1_4  y_train_0_node_1_4  x_train_0_from_0_4_to_1_5 
 x_train_0_from_1_4_to_1_5  y_train_0_node_1_5  x_train_0_from_0_5_to_1_6 
 x_train_0_from_1_5_to_1_6  y_train_0_node_1_6  x_train_0_from_0_6_to_1_7 
 x_train_0_from_1_6_to_1_7  y_train_0_node_1_7  x_train_0_from_0_7_to_1_8 
 x_train_0_from_1_7_to_1_8  y_train_0_node_1_8  x_train_0_from_0_8_to_1_9 
 x_train_0_from_1_8_to_1_9  y_train_0_node_1_9  x_train_0_from_0_9_to_1_10 
 x_train_0_from_1_9_to_1_10  y_train_0_node_1_10  x_train_0_from_0_10_to_1_11 
 x_train_0_from_1_10_to_1_11  y_train_0_node_1_11  x_train_0_from_0_11_to_1_12 
 x_train_0_from_1_11_to_1_12  y_train_0_node_1_12  x_train_0_from_0_12_to_1_13 
 x_train_0_from_1_12_to_1_13  y_train_0_node_1_13  x_train_0_from_0_13_to_1_14 
 x_train_0_from_1_13_to_1_14  y_train_0_node_1_14  x_train_0_from_0_14_to_1_15 
 x_train_0_from_1_14_to_1_15  y_train_0_node_1_15  x_train_0_from_0_15_to_1_16 
 x_train_0_from_1_15_to_1_16  y_train_0_node_1_16  x_train_0_from_0_16_to_1_17 
 x_train_0_from_1_16_to_1_17  y_train_0_node_1_17  x_train_0_from_0_17_to_1_18 
 x_train_0_from_1_17_to_1_18  y_train_0_node_1_18  x_train_0_from_0_18_to_1_19 
 x_train_0_from_1_18_to_1_19  y_train_0_node_1_19  x_train_0_from_0_19_to_1_20 
 x_train_0_from_1_19_to_1_20  y_train_0_node_1_20  x_train_0_from_0_20_to_1_21 
 x_train_0_from_1_20_to_1_21  y_train_0_node_1_21  x_train_0_from_0_21_to_1_22 
 x_train_0_from_1_21_to_1_22  y_train_0_node_1_22  x_train_0_from_0_22_to_1_23 
 x_train_0_from_1_22_to_1_23  y_train_0_node_1_23  x_train_0_from_0_23_to_1_24 
 x_train_0_from_1_23_to_1_24  y_train_0_node_1_24  x_train_0_from_0_24_to_1_25 
 x_train_0_from_1_24_to_1_25  y_train_0_node_1_25  x_train_0_from_0_25_to_1_26 
 x_train_0_from_1_25_to_1_26  y_train_0_node_1_26  x_train_0_from_0_26_to_1_27 
 x_train_0_from_1_26_to_1_27  y_train_0_node_1_27  x_train_0_from_0_1_to_2_2 
 y_train_0_node_2_2  x_train_0_from_0_2_to_2_3  x_train_0_from_2_2_to_2_3 
 y_train_0_node_2_3  x_train_0_from_0_3_to_2_4  x_train_0_from_2_3_to_2_4 
 y_train_0_node_2_4  x_train_0_from_0_4_to_2_5  x_train_0_from_2_4_to_2_5 
 y_train_0_node_2_5  x_train_0_from_0_5_to_2_6  x_train_0_from_2_5_to_2_6 
 y_train_0_node_2_6  x_train_0_from_0_6_to_2_7  x_train_0_from_2_6_to_2_7 
 y_train_0_node_2_7  x_train_0_from_0_7_to_2_8  x_train_0_from_2_7_to_2_8 
 y_train_0_node_2_8  x_train_0_from_0_8_to_2_9  x_train_0_from_2_8_to_2_9 
 y_train_0_node_2_9  x_train_0_from_0_9_to_2_10  x_train_0_from_2_9_to_2_10 
 y_train_0_node_2_10  x_train_0_from_0_10_to_2_11  x_train_0_from_2_10_to_2_11 
 y_train_0_node_2_11  x_train_0_from_0_11_to_2_12  x_train_0_from_2_11_to_2_12 
 y_train_0_node_2_12  x_train_0_from_0_12_to_2_13  x_train_0_from_2_12_to_2_13 
 y_train_0_node_2_13  x_train_0_from_0_13_to_2_14  x_train_0_from_2_13_to_2_14 
 y_train_0_node_2_14  x_train_0_from_0_14_to_2_15  x_train_0_from_2_14_to_2_15 
 y_train_0_node_2_15  x_train_0_from_0_15_to_2_16  x_train_0_from_2_15_to_2_16 
 y_train_0_node_2_16  x_train_0_from_0_16_to_2_17  x_train_0_from_2_16_to_2_17 
 y_train_0_node_2_17  x_train_0_from_0_17_to_2_18  x_train_0_from_2_17_to_2_18 
 y_train_0_node_2_18  x_train_0_from_0_18_to_2_19  x_train_0_from_2_18_to_2_19 
 y_train_0_node_2_19  x_train_0_from_0_19_to_2_20  x_train_0_from_2_19_to_2_20 
 y_train_0_node_2_20  x_train_0_from_0_20_to_2_21  x_train_0_from_2_20_to_2_21 
 y_train_0_node_2_21  x_train_0_from_0_21_to_2_22  x_train_0_from_2_21_to_2_22 
 y_train_0_node_2_22  x_train_0_from_0_22_to_2_23  x_train_0_from_2_22_to_2_23 
 y_train_0_node_2_23  x_train_0_from_0_23_to_2_24  x_train_0_from_2_23_to_2_24 
 y_train_0_node_2_24  x_train_0_from_0_24_to_2_25  x_train_0_from_2_24_to_2_25 
 y_train_0_node_2_25  x_train_0_from_0_25_to_2_26  x_train_0_from_2_25_to_2_26 
 y_train_0_node_2_26  x_train_0_from_0_26_to_2_27  x_train_0_from_2_26_to_2_27 
 y_train_0_node_2_27  x_train_0_from_0_27_to_2_28  x_train_0_from_2_27_to_2_28 
 y_train_0_node_2_28  x_train_0_from_1_4_to_3_5  y_train_0_node_3_5 
 x_train_0_from_1_5_to_3_6  x_train_0_from_3_5_to_3_6  y_train_0_node_3_6 
 x_train_0_from_1_6_to_3_7  x_train_0_from_3_6_to_3_7  y_train_0_node_3_7 
 x_train_0_from_1_7_to_3_8  x_train_0_from_3_7_to_3_8  y_train_0_node_3_8 
 x_train_0_from_1_8_to_3_9  x_train_0_from_3_8_to_3_9  y_train_0_node_3_9 
 x_train_0_from_1_9_to_3_10  x_train_0_from_3_9_to_3_10  y_train_0_node_3_10 
 x_train_0_from_1_10_to_3_11  x_train_0_from_3_10_to_3_11  y_train_0_node_3_11 
 x_train_0_from_1_11_to_3_12  x_train_0_from_3_11_to_3_12  y_train_0_node_3_12 
 x_train_0_from_1_12_to_3_13  x_train_0_from_3_12_to_3_13  y_train_0_node_3_13 
 x_train_0_from_1_13_to_3_14  x_train_0_from_3_13_to_3_14  y_train_0_node_3_14 
 x_train_0_from_1_14_to_3_15  x_train_0_from_3_14_to_3_15  y_train_0_node_3_15 
 x_train_0_from_1_15_to_3_16  x_train_0_from_3_15_to_3_16  y_train_0_node_3_16 
 x_train_0_from_1_16_to_3_17  x_train_0_from_3_16_to_3_17  y_train_0_node_3_17 
 x_train_0_from_1_17_to_3_18  x_train_0_from_3_17_to_3_18  y_train_0_node_3_18 
 x_train_0_from_1_18_to_3_19  x_train_0_from_3_18_to_3_19  y_train_0_node_3_19 
 x_train_0_from_1_19_to_3_20  x_train_0_from_3_19_to_3_20  y_train_0_node_3_20 
 x_train_0_from_1_20_to_3_21  x_train_0_from_3_20_to_3_21  y_train_0_node_3_21 
 x_train_0_from_1_21_to_3_22  x_train_0_from_3_21_to_3_22  y_train_0_node_3_22 
 x_train_0_from_1_22_to_3_23  x_train_0_from_3_22_to_3_23  y_train_0_node_3_23 
 x_train_0_from_1_23_to_3_24  x_train_0_from_3_23_to_3_24  y_train_0_node_3_24 
 x_train_0_from_1_24_to_3_25  x_train_0_from_3_24_to_3_25  y_train_0_node_3_25 
 x_train_0_from_1_25_to_3_26  x_train_0_from_3_25_to_3_26  y_train_0_node_3_26 
 x_train_0_from_1_26_to_3_27  x_train_0_from_3_26_to_3_27  y_train_0_node_3_27 
 x_train_0_from_1_27_to_3_28  x_train_0_from_3_27_to_3_28  y_train_0_node_3_28 
 x_train_0_from_2_2_to_4_3  y_train_0_node_4_3  x_train_0_from_2_3_to_4_4 
 x_train_0_from_4_3_to_4_4  y_train_0_node_4_4  x_train_0_from_2_4_to_4_5 
 x_train_0_from_4_4_to_4_5  y_train_0_node_4_5  x_train_0_from_2_5_to_4_6 
 x_train_0_from_4_5_to_4_6  y_train_0_node_4_6  x_train_0_from_2_6_to_4_7 
 x_train_0_from_4_6_to_4_7  y_train_0_node_4_7  x_train_0_from_2_7_to_4_8 
 x_train_0_from_4_7_to_4_8  y_train_0_node_4_8  x_train_0_from_2_8_to_4_9 
 x_train_0_from_4_8_to_4_9  y_train_0_node_4_9  x_train_0_from_2_9_to_4_10 
 x_train_0_from_4_9_to_4_10  y_train_0_node_4_10  x_train_0_from_2_10_to_4_11 
 x_train_0_from_4_10_to_4_11  y_train_0_node_4_11  x_train_0_from_2_11_to_4_12 
 x_train_0_from_4_11_to_4_12  y_train_0_node_4_12  x_train_0_from_2_12_to_4_13 
 x_train_0_from_4_12_to_4_13  y_train_0_node_4_13  x_train_0_from_2_13_to_4_14 
 x_train_0_from_4_13_to_4_14  y_train_0_node_4_14  x_train_0_from_2_14_to_4_15 
 x_train_0_from_4_14_to_4_15  y_train_0_node_4_15  x_train_0_from_2_15_to_4_16 
 x_train_0_from_4_15_to_4_16  y_train_0_node_4_16  x_train_0_from_2_16_to_4_17 
 x_train_0_from_4_16_to_4_17  y_train_0_node_4_17  x_train_0_from_2_17_to_4_18 
 x_train_0_from_4_17_to_4_18  y_train_0_node_4_18  x_train_0_from_2_18_to_4_19 
 x_train_0_from_4_18_to_4_19  y_train_0_node_4_19  x_train_0_from_2_19_to_4_20 
 x_train_0_from_4_19_to_4_20  y_train_0_node_4_20  x_train_0_from_2_20_to_4_21 
 x_train_0_from_4_20_to_4_21  y_train_0_node_4_21  x_train_0_from_2_21_to_4_22 
 x_train_0_from_4_21_to_4_22  y_train_0_node_4_22  x_train_0_from_2_22_to_4_23 
 x_train_0_from_4_22_to_4_23  y_train_0_node_4_23  x_train_0_from_2_23_to_4_24 
 x_train_0_from_4_23_to_4_24  y_train_0_node_4_24  x_train_0_from_2_24_to_4_25 
 x_train_0_from_4_24_to_4_25  y_train_0_node_4_25  x_train_0_from_2_25_to_4_26 
 x_train_0_from_4_25_to_4_26  y_train_0_node_4_26  x_train_0_from_2_26_to_4_27 
 x_train_0_from_4_26_to_4_27  y_train_0_node_4_27  x_train_0_from_2_27_to_4_28 
 x_train_0_from_4_27_to_4_28  y_train_0_node_4_28  x_train_0_from_2_28_to_4_29 
 x_train_0_from_4_28_to_4_29  y_train_0_node_4_29  x_train_0_from_3_5_to_5_6 
 y_train_0_node_5_6  x_train_0_from_3_6_to_5_7  x_train_0_from_5_6_to_5_7 
 y_train_0_node_5_7  x_train_0_from_3_7_to_5_8  x_train_0_from_5_7_to_5_8 
 y_train_0_node_5_8  x_train_0_from_3_8_to_5_9  x_train_0_from_5_8_to_5_9 
 y_train_0_node_5_9  x_train_0_from_3_9_to_5_10  x_train_0_from_5_9_to_5_10 
 y_train_0_node_5_10  x_train_0_from_3_10_to_5_11  x_train_0_from_5_10_to_5_11 
 y_train_0_node_5_11  x_train_0_from_3_11_to_5_12  x_train_0_from_5_11_to_5_12 
 y_train_0_node_5_12  x_train_0_from_3_12_to_5_13  x_train_0_from_5_12_to_5_13 
 y_train_0_node_5_13  x_train_0_from_3_13_to_5_14  x_train_0_from_5_13_to_5_14 
 y_train_0_node_5_14  x_train_0_from_3_14_to_5_15  x_train_0_from_5_14_to_5_15 
 y_train_0_node_5_15  x_train_0_from_3_15_to_5_16  x_train_0_from_5_15_to_5_16 
 y_train_0_node_5_16  x_train_0_from_3_16_to_5_17  x_train_0_from_5_16_to_5_17 
 y_train_0_node_5_17  x_train_0_from_3_17_to_5_18  x_train_0_from_5_17_to_5_18 
 y_train_0_node_5_18  x_train_0_from_3_18_to_5_19  x_train_0_from_5_18_to_5_19 
 y_train_0_node_5_19  x_train_0_from_3_19_to_5_20  x_train_0_from_5_19_to_5_20 
 y_train_0_node_5_20  x_train_0_from_3_20_to_5_21  x_train_0_from_5_20_to_5_21 
 y_train_0_node_5_21  x_train_0_from_3_21_to_5_22  x_train_0_from_5_21_to_5_22 
 y_train_0_node_5_22  x_train_0_from_3_22_to_5_23  x_train_0_from_5_22_to_5_23 
 y_train_0_node_5_23  x_train_0_from_3_23_to_5_24  x_train_0_from_5_23_to_5_24 
 y_train_0_node_5_24  x_train_0_from_3_24_to_5_25  x_train_0_from_5_24_to_5_25 
 y_train_0_node_5_25  x_train_0_from_3_25_to_5_26  x_train_0_from_5_25_to_5_26 
 y_train_0_node_5_26  x_train_0_from_3_26_to_5_27  x_train_0_from_5_26_to_5_27 
 y_train_0_node_5_27  x_train_0_from_3_27_to_5_28  x_train_0_from_5_27_to_5_28 
 y_train_0_node_5_28  x_train_0_from_3_28_to_5_29  x_train_0_from_5_28_to_5_29 
 y_train_0_node_5_29  x_train_0_from_4_3_to_6_4  y_train_0_node_6_4 
 x_train_0_from_4_4_to_6_5  x_train_0_from_6_4_to_6_5  y_train_0_node_6_5 
 x_train_0_from_4_5_to_6_6  x_train_0_from_6_5_to_6_6  y_train_0_node_6_6 
 x_train_0_from_4_6_to_6_7  x_train_0_from_5_6_to_6_7 
 x_train_0_from_6_6_to_6_7  y_train_0_node_6_7  x_train_0_from_4_7_to_6_8 
 x_train_0_from_5_7_to_6_8  x_train_0_from_6_7_to_6_8  y_train_0_node_6_8 
 x_train_0_from_4_8_to_6_9  x_train_0_from_5_8_to_6_9 
 x_train_0_from_6_8_to_6_9  y_train_0_node_6_9  x_train_0_from_4_9_to_6_10 
 x_train_0_from_5_9_to_6_10  x_train_0_from_6_9_to_6_10  y_train_0_node_6_10 
 x_train_0_from_4_10_to_6_11  x_train_0_from_5_10_to_6_11 
 x_train_0_from_6_10_to_6_11  y_train_0_node_6_11  x_train_0_from_4_11_to_6_12 
 x_train_0_from_5_11_to_6_12  x_train_0_from_6_11_to_6_12  y_train_0_node_6_12 
 x_train_0_from_4_12_to_6_13  x_train_0_from_5_12_to_6_13 
 x_train_0_from_6_12_to_6_13  y_train_0_node_6_13  x_train_0_from_4_13_to_6_14 
 x_train_0_from_5_13_to_6_14  x_train_0_from_6_13_to_6_14  y_train_0_node_6_14 
 x_train_0_from_4_14_to_6_15  x_train_0_from_5_14_to_6_15 
 x_train_0_from_6_14_to_6_15  y_train_0_node_6_15  x_train_0_from_4_15_to_6_16 
 x_train_0_from_5_15_to_6_16  x_train_0_from_6_15_to_6_16  y_train_0_node_6_16 
 x_train_0_from_4_16_to_6_17  x_train_0_from_5_16_to_6_17 
 x_train_0_from_6_16_to_6_17  y_train_0_node_6_17  x_train_0_from_4_17_to_6_18 
 x_train_0_from_5_17_to_6_18  x_train_0_from_6_17_to_6_18  y_train_0_node_6_18 
 x_train_0_from_4_18_to_6_19  x_train_0_from_5_18_to_6_19 
 x_train_0_from_6_18_to_6_19  y_train_0_node_6_19  x_train_0_from_4_19_to_6_20 
 x_train_0_from_5_19_to_6_20  x_train_0_from_6_19_to_6_20  y_train_0_node_6_20 
 x_train_0_from_4_20_to_6_21  x_train_0_from_5_20_to_6_21 
 x_train_0_from_6_20_to_6_21  y_train_0_node_6_21  x_train_0_from_4_21_to_6_22 
 x_train_0_from_5_21_to_6_22  x_train_0_from_6_21_to_6_22  y_train_0_node_6_22 
 x_train_0_from_4_22_to_6_23  x_train_0_from_5_22_to_6_23 
 x_train_0_from_6_22_to_6_23  y_train_0_node_6_23  x_train_0_from_4_23_to_6_24 
 x_train_0_from_5_23_to_6_24  x_train_0_from_6_23_to_6_24  y_train_0_node_6_24 
 x_train_0_from_4_24_to_6_25  x_train_0_from_5_24_to_6_25 
 x_train_0_from_6_24_to_6_25  y_train_0_node_6_25  x_train_0_from_4_25_to_6_26 
 x_train_0_from_5_25_to_6_26  x_train_0_from_6_25_to_6_26  y_train_0_node_6_26 
 x_train_0_from_4_26_to_6_27  x_train_0_from_5_26_to_6_27 
 x_train_0_from_6_26_to_6_27  y_train_0_node_6_27  x_train_0_from_4_27_to_6_28 
 x_train_0_from_5_27_to_6_28  x_train_0_from_6_27_to_6_28  y_train_0_node_6_28 
 x_train_0_from_4_28_to_6_29  x_train_0_from_5_28_to_6_29 
 x_train_0_from_6_28_to_6_29  y_train_0_node_6_29  x_train_0_from_4_29_to_6_30 
 x_train_0_from_5_29_to_6_30  x_train_0_from_6_29_to_6_30  y_train_0_node_6_30 
 x_train_1_from_2_3_to_0_4  y_train_1_node_0_4  x_train_1_from_0_4_to_0_5 
 x_train_1_from_2_4_to_0_5  y_train_1_node_0_5  x_train_1_from_0_5_to_0_6 
 x_train_1_from_2_5_to_0_6  y_train_1_node_0_6  x_train_1_from_0_6_to_0_7 
 x_train_1_from_1_6_to_0_7  x_train_1_from_2_6_to_0_7  y_train_1_node_0_7 
 x_train_1_from_0_7_to_0_8  x_train_1_from_1_7_to_0_8 
 x_train_1_from_2_7_to_0_8  y_train_1_node_0_8  x_train_1_from_0_8_to_0_9 
 x_train_1_from_1_8_to_0_9  x_train_1_from_2_8_to_0_9  y_train_1_node_0_9 
 x_train_1_from_0_9_to_0_10  x_train_1_from_1_9_to_0_10 
 x_train_1_from_2_9_to_0_10  y_train_1_node_0_10  x_train_1_from_0_10_to_0_11 
 x_train_1_from_1_10_to_0_11  x_train_1_from_2_10_to_0_11  y_train_1_node_0_11 
 x_train_1_from_0_11_to_0_12  x_train_1_from_1_11_to_0_12 
 x_train_1_from_2_11_to_0_12  y_train_1_node_0_12  x_train_1_from_0_12_to_0_13 
 x_train_1_from_1_12_to_0_13  x_train_1_from_2_12_to_0_13  y_train_1_node_0_13 
 x_train_1_from_0_13_to_0_14  x_train_1_from_1_13_to_0_14 
 x_train_1_from_2_13_to_0_14  y_train_1_node_0_14  x_train_1_from_0_14_to_0_15 
 x_train_1_from_1_14_to_0_15  x_train_1_from_2_14_to_0_15  y_train_1_node_0_15 
 x_train_1_from_0_15_to_0_16  x_train_1_from_1_15_to_0_16 
 x_train_1_from_2_15_to_0_16  y_train_1_node_0_16  x_train_1_from_0_16_to_0_17 
 x_train_1_from_1_16_to_0_17  x_train_1_from_2_16_to_0_17  y_train_1_node_0_17 
 x_train_1_from_0_17_to_0_18  x_train_1_from_1_17_to_0_18 
 x_train_1_from_2_17_to_0_18  y_train_1_node_0_18  x_train_1_from_0_18_to_0_19 
 x_train_1_from_1_18_to_0_19  x_train_1_from_2_18_to_0_19  y_train_1_node_0_19 
 x_train_1_from_0_19_to_0_20  x_train_1_from_1_19_to_0_20 
 x_train_1_from_2_19_to_0_20  y_train_1_node_0_20  x_train_1_from_0_20_to_0_21 
 x_train_1_from_1_20_to_0_21  x_train_1_from_2_20_to_0_21  y_train_1_node_0_21 
 x_train_1_from_0_21_to_0_22  x_train_1_from_1_21_to_0_22 
 x_train_1_from_2_21_to_0_22  y_train_1_node_0_22  x_train_1_from_0_22_to_0_23 
 x_train_1_from_1_22_to_0_23  x_train_1_from_2_22_to_0_23  y_train_1_node_0_23 
 x_train_1_from_0_23_to_0_24  x_train_1_from_1_23_to_0_24 
 x_train_1_from_2_23_to_0_24  y_train_1_node_0_24  x_train_1_from_0_24_to_0_25 
 x_train_1_from_1_24_to_0_25  x_train_1_from_2_24_to_0_25  y_train_1_node_0_25 
 x_train_1_from_0_25_to_0_26  x_train_1_from_1_25_to_0_26 
 x_train_1_from_2_25_to_0_26  y_train_1_node_0_26  x_train_1_from_0_26_to_0_27 
 x_train_1_from_1_26_to_0_27  x_train_1_from_2_26_to_0_27  y_train_1_node_0_27 
 x_train_1_from_0_27_to_0_28  x_train_1_from_1_27_to_0_28 
 x_train_1_from_2_27_to_0_28  y_train_1_node_0_28  x_train_1_from_0_28_to_0_29 
 x_train_1_from_1_28_to_0_29  x_train_1_from_2_28_to_0_29  y_train_1_node_0_29 
 x_train_1_from_0_29_to_0_30  x_train_1_from_1_29_to_0_30 
 x_train_1_from_2_29_to_0_30  y_train_1_node_0_30  x_train_1_from_3_5_to_1_6 
 y_train_1_node_1_6  x_train_1_from_1_6_to_1_7  x_train_1_from_3_6_to_1_7 
 y_train_1_node_1_7  x_train_1_from_1_7_to_1_8  x_train_1_from_3_7_to_1_8 
 y_train_1_node_1_8  x_train_1_from_1_8_to_1_9  x_train_1_from_3_8_to_1_9 
 y_train_1_node_1_9  x_train_1_from_1_9_to_1_10  x_train_1_from_3_9_to_1_10 
 y_train_1_node_1_10  x_train_1_from_1_10_to_1_11  x_train_1_from_3_10_to_1_11 
 y_train_1_node_1_11  x_train_1_from_1_11_to_1_12  x_train_1_from_3_11_to_1_12 
 y_train_1_node_1_12  x_train_1_from_1_12_to_1_13  x_train_1_from_3_12_to_1_13 
 y_train_1_node_1_13  x_train_1_from_1_13_to_1_14  x_train_1_from_3_13_to_1_14 
 y_train_1_node_1_14  x_train_1_from_1_14_to_1_15  x_train_1_from_3_14_to_1_15 
 y_train_1_node_1_15  x_train_1_from_1_15_to_1_16  x_train_1_from_3_15_to_1_16 
 y_train_1_node_1_16  x_train_1_from_1_16_to_1_17  x_train_1_from_3_16_to_1_17 
 y_train_1_node_1_17  x_train_1_from_1_17_to_1_18  x_train_1_from_3_17_to_1_18 
 y_train_1_node_1_18  x_train_1_from_1_18_to_1_19  x_train_1_from_3_18_to_1_19 
 y_train_1_node_1_19  x_train_1_from_1_19_to_1_20  x_train_1_from_3_19_to_1_20 
 y_train_1_node_1_20  x_train_1_from_1_20_to_1_21  x_train_1_from_3_20_to_1_21 
 y_train_1_node_1_21  x_train_1_from_1_21_to_1_22  x_train_1_from_3_21_to_1_22 
 y_train_1_node_1_22  x_train_1_from_1_22_to_1_23  x_train_1_from_3_22_to_1_23 
 y_train_1_node_1_23  x_train_1_from_1_23_to_1_24  x_train_1_from_3_23_to_1_24 
 y_train_1_node_1_24  x_train_1_from_1_24_to_1_25  x_train_1_from_3_24_to_1_25 
 y_train_1_node_1_25  x_train_1_from_1_25_to_1_26  x_train_1_from_3_25_to_1_26 
 y_train_1_node_1_26  x_train_1_from_1_26_to_1_27  x_train_1_from_3_26_to_1_27 
 y_train_1_node_1_27  x_train_1_from_1_27_to_1_28  x_train_1_from_3_27_to_1_28 
 y_train_1_node_1_28  x_train_1_from_1_28_to_1_29  x_train_1_from_3_28_to_1_29 
 y_train_1_node_1_29  x_train_1_from_4_2_to_2_3  y_train_1_node_2_3 
 x_train_1_from_2_3_to_2_4  x_train_1_from_4_3_to_2_4  y_train_1_node_2_4 
 x_train_1_from_2_4_to_2_5  x_train_1_from_4_4_to_2_5  y_train_1_node_2_5 
 x_train_1_from_2_5_to_2_6  x_train_1_from_4_5_to_2_6  y_train_1_node_2_6 
 x_train_1_from_2_6_to_2_7  x_train_1_from_4_6_to_2_7  y_train_1_node_2_7 
 x_train_1_from_2_7_to_2_8  x_train_1_from_4_7_to_2_8  y_train_1_node_2_8 
 x_train_1_from_2_8_to_2_9  x_train_1_from_4_8_to_2_9  y_train_1_node_2_9 
 x_train_1_from_2_9_to_2_10  x_train_1_from_4_9_to_2_10  y_train_1_node_2_10 
 x_train_1_from_2_10_to_2_11  x_train_1_from_4_10_to_2_11  y_train_1_node_2_11 
 x_train_1_from_2_11_to_2_12  x_train_1_from_4_11_to_2_12  y_train_1_node_2_12 
 x_train_1_from_2_12_to_2_13  x_train_1_from_4_12_to_2_13  y_train_1_node_2_13 
 x_train_1_from_2_13_to_2_14  x_train_1_from_4_13_to_2_14  y_train_1_node_2_14 
 x_train_1_from_2_14_to_2_15  x_train_1_from_4_14_to_2_15  y_train_1_node_2_15 
 x_train_1_from_2_15_to_2_16  x_train_1_from_4_15_to_2_16  y_train_1_node_2_16 
 x_train_1_from_2_16_to_2_17  x_train_1_from_4_16_to_2_17  y_train_1_node_2_17 
 x_train_1_from_2_17_to_2_18  x_train_1_from_4_17_to_2_18  y_train_1_node_2_18 
 x_train_1_from_2_18_to_2_19  x_train_1_from_4_18_to_2_19  y_train_1_node_2_19 
 x_train_1_from_2_19_to_2_20  x_train_1_from_4_19_to_2_20  y_train_1_node_2_20 
 x_train_1_from_2_20_to_2_21  x_train_1_from_4_20_to_2_21  y_train_1_node_2_21 
 x_train_1_from_2_21_to_2_22  x_train_1_from_4_21_to_2_22  y_train_1_node_2_22 
 x_train_1_from_2_22_to_2_23  x_train_1_from_4_22_to_2_23  y_train_1_node_2_23 
 x_train_1_from_2_23_to_2_24  x_train_1_from_4_23_to_2_24  y_train_1_node_2_24 
 x_train_1_from_2_24_to_2_25  x_train_1_from_4_24_to_2_25  y_train_1_node_2_25 
 x_train_1_from_2_25_to_2_26  x_train_1_from_4_25_to_2_26  y_train_1_node_2_26 
 x_train_1_from_2_26_to_2_27  x_train_1_from_4_26_to_2_27  y_train_1_node_2_27 
 x_train_1_from_2_27_to_2_28  x_train_1_from_4_27_to_2_28  y_train_1_node_2_28 
 x_train_1_from_2_28_to_2_29  x_train_1_from_4_28_to_2_29  y_train_1_node_2_29 
 x_train_1_from_5_4_to_3_5  y_train_1_node_3_5  x_train_1_from_3_5_to_3_6 
 x_train_1_from_5_5_to_3_6  y_train_1_node_3_6  x_train_1_from_3_6_to_3_7 
 x_train_1_from_5_6_to_3_7  y_train_1_node_3_7  x_train_1_from_3_7_to_3_8 
 x_train_1_from_5_7_to_3_8  y_train_1_node_3_8  x_train_1_from_3_8_to_3_9 
 x_train_1_from_5_8_to_3_9  y_train_1_node_3_9  x_train_1_from_3_9_to_3_10 
 x_train_1_from_5_9_to_3_10  y_train_1_node_3_10  x_train_1_from_3_10_to_3_11 
 x_train_1_from_5_10_to_3_11  y_train_1_node_3_11  x_train_1_from_3_11_to_3_12 
 x_train_1_from_5_11_to_3_12  y_train_1_node_3_12  x_train_1_from_3_12_to_3_13 
 x_train_1_from_5_12_to_3_13  y_train_1_node_3_13  x_train_1_from_3_13_to_3_14 
 x_train_1_from_5_13_to_3_14  y_train_1_node_3_14  x_train_1_from_3_14_to_3_15 
 x_train_1_from_5_14_to_3_15  y_train_1_node_3_15  x_train_1_from_3_15_to_3_16 
 x_train_1_from_5_15_to_3_16  y_train_1_node_3_16  x_train_1_from_3_16_to_3_17 
 x_train_1_from_5_16_to_3_17  y_train_1_node_3_17  x_train_1_from_3_17_to_3_18 
 x_train_1_from_5_17_to_3_18  y_train_1_node_3_18  x_train_1_from_3_18_to_3_19 
 x_train_1_from_5_18_to_3_19  y_train_1_node_3_19  x_train_1_from_3_19_to_3_20 
 x_train_1_from_5_19_to_3_20  y_train_1_node_3_20  x_train_1_from_3_20_to_3_21 
 x_train_1_from_5_20_to_3_21  y_train_1_node_3_21  x_train_1_from_3_21_to_3_22 
 x_train_1_from_5_21_to_3_22  y_train_1_node_3_22  x_train_1_from_3_22_to_3_23 
 x_train_1_from_5_22_to_3_23  y_train_1_node_3_23  x_train_1_from_3_23_to_3_24 
 x_train_1_from_5_23_to_3_24  y_train_1_node_3_24  x_train_1_from_3_24_to_3_25 
 x_train_1_from_5_24_to_3_25  y_train_1_node_3_25  x_train_1_from_3_25_to_3_26 
 x_train_1_from_5_25_to_3_26  y_train_1_node_3_26  x_train_1_from_3_26_to_3_27 
 x_train_1_from_5_26_to_3_27  y_train_1_node_3_27  x_train_1_from_3_27_to_3_28 
 x_train_1_from_5_27_to_3_28  y_train_1_node_3_28  x_train_1_from_6_1_to_4_2 
 y_train_1_node_4_2  x_train_1_from_4_2_to_4_3  x_train_1_from_6_2_to_4_3 
 y_train_1_node_4_3  x_train_1_from_4_3_to_4_4  x_train_1_from_6_3_to_4_4 
 y_train_1_node_4_4  x_train_1_from_4_4_to_4_5  x_train_1_from_6_4_to_4_5 
 y_train_1_node_4_5  x_train_1_from_4_5_to_4_6  x_train_1_from_6_5_to_4_6 
 y_train_1_node_4_6  x_train_1_from_4_6_to_4_7  x_train_1_from_6_6_to_4_7 
 y_train_1_node_4_7  x_train_1_from_4_7_to_4_8  x_train_1_from_6_7_to_4_8 
 y_train_1_node_4_8  x_train_1_from_4_8_to_4_9  x_train_1_from_6_8_to_4_9 
 y_train_1_node_4_9  x_train_1_from_4_9_to_4_10  x_train_1_from_6_9_to_4_10 
 y_train_1_node_4_10  x_train_1_from_4_10_to_4_11  x_train_1_from_6_10_to_4_11 
 y_train_1_node_4_11  x_train_1_from_4_11_to_4_12  x_train_1_from_6_11_to_4_12 
 y_train_1_node_4_12  x_train_1_from_4_12_to_4_13  x_train_1_from_6_12_to_4_13 
 y_train_1_node_4_13  x_train_1_from_4_13_to_4_14  x_train_1_from_6_13_to_4_14 
 y_train_1_node_4_14  x_train_1_from_4_14_to_4_15  x_train_1_from_6_14_to_4_15 
 y_train_1_node_4_15  x_train_1_from_4_15_to_4_16  x_train_1_from_6_15_to_4_16 
 y_train_1_node_4_16  x_train_1_from_4_16_to_4_17  x_train_1_from_6_16_to_4_17 
 y_train_1_node_4_17  x_train_1_from_4_17_to_4_18  x_train_1_from_6_17_to_4_18 
 y_train_1_node_4_18  x_train_1_from_4_18_to_4_19  x_train_1_from_6_18_to_4_19 
 y_train_1_node_4_19  x_train_1_from_4_19_to_4_20  x_train_1_from_6_19_to_4_20 
 y_train_1_node_4_20  x_train_1_from_4_20_to_4_21  x_train_1_from_6_20_to_4_21 
 y_train_1_node_4_21  x_train_1_from_4_21_to_4_22  x_train_1_from_6_21_to_4_22 
 y_train_1_node_4_22  x_train_1_from_4_22_to_4_23  x_train_1_from_6_22_to_4_23 
 y_train_1_node_4_23  x_train_1_from_4_23_to_4_24  x_train_1_from_6_23_to_4_24 
 y_train_1_node_4_24  x_train_1_from_4_24_to_4_25  x_train_1_from_6_24_to_4_25 
 y_train_1_node_4_25  x_train_1_from_4_25_to_4_26  x_train_1_from_6_25_to_4_26 
 y_train_1_node_4_26  x_train_1_from_4_26_to_4_27  x_train_1_from_6_26_to_4_27 
 y_train_1_node_4_27  x_train_1_from_4_27_to_4_28  x_train_1_from_6_27_to_4_28 
 y_train_1_node_4_28  x_train_1_from_6_1_to_5_2  y_train_1_node_5_2 
 x_train_1_from_5_2_to_5_3  x_train_1_from_6_2_to_5_3  y_train_1_node_5_3 
 x_train_1_from_5_3_to_5_4  x_train_1_from_6_3_to_5_4  y_train_1_node_5_4 
 x_train_1_from_5_4_to_5_5  x_train_1_from_6_4_to_5_5  y_train_1_node_5_5 
 x_train_1_from_5_5_to_5_6  x_train_1_from_6_5_to_5_6  y_train_1_node_5_6 
 x_train_1_from_5_6_to_5_7  x_train_1_from_6_6_to_5_7  y_train_1_node_5_7 
 x_train_1_from_5_7_to_5_8  x_train_1_from_6_7_to_5_8  y_train_1_node_5_8 
 x_train_1_from_5_8_to_5_9  x_train_1_from_6_8_to_5_9  y_train_1_node_5_9 
 x_train_1_from_5_9_to_5_10  x_train_1_from_6_9_to_5_10  y_train_1_node_5_10 
 x_train_1_from_5_10_to_5_11  x_train_1_from_6_10_to_5_11  y_train_1_node_5_11 
 x_train_1_from_5_11_to_5_12  x_train_1_from_6_11_to_5_12  y_train_1_node_5_12 
 x_train_1_from_5_12_to_5_13  x_train_1_from_6_12_to_5_13  y_train_1_node_5_13 
 x_train_1_from_5_13_to_5_14  x_train_1_from_6_13_to_5_14  y_train_1_node_5_14 
 x_train_1_from_5_14_to_5_15  x_train_1_from_6_14_to_5_15  y_train_1_node_5_15 
 x_train_1_from_5_15_to_5_16  x_train_1_from_6_15_to_5_16  y_train_1_node_5_16 
 x_train_1_from_5_16_to_5_17  x_train_1_from_6_16_to_5_17  y_train_1_node_5_17 
 x_train_1_from_5_17_to_5_18  x_train_1_from_6_17_to_5_18  y_train_1_node_5_18 
 x_train_1_from_5_18_to_5_19  x_train_1_from_6_18_to_5_19  y_train_1_node_5_19 
 x_train_1_from_5_19_to_5_20  x_train_1_from_6_19_to_5_20  y_train_1_node_5_20 
 x_train_1_from_5_20_to_5_21  x_train_1_from_6_20_to_5_21  y_train_1_node_5_21 
 x_train_1_from_5_21_to_5_22  x_train_1_from_6_21_to_5_22  y_train_1_node_5_22 
 x_train_1_from_5_22_to_5_23  x_train_1_from_6_22_to_5_23  y_train_1_node_5_23 
 x_train_1_from_5_23_to_5_24  x_train_1_from_6_23_to_5_24  y_train_1_node_5_24 
 x_train_1_from_5_24_to_5_25  x_train_1_from_6_24_to_5_25  y_train_1_node_5_25 
 x_train_1_from_5_25_to_5_26  x_train_1_from_6_25_to_5_26  y_train_1_node_5_26 
 x_train_1_from_5_26_to_5_27  x_train_1_from_6_26_to_5_27  y_train_1_node_5_27 
 y_train_1_node_6_1  x_train_1_from_6_1_to_6_2  y_train_1_node_6_2 
 x_train_1_from_6_2_to_6_3  y_train_1_node_6_3  x_train_1_from_6_3_to_6_4 
 y_train_1_node_6_4  x_train_1_from_6_4_to_6_5  y_train_1_node_6_5 
 x_train_1_from_6_5_to_6_6  y_train_1_node_6_6  x_train_1_from_6_6_to_6_7 
 y_train_1_node_6_7  x_train_1_from_6_7_to_6_8  y_train_1_node_6_8 
 x_train_1_from_6_8_to_6_9  y_train_1_node_6_9  x_train_1_from_6_9_to_6_10 
 y_train_1_node_6_10  x_train_1_from_6_10_to_6_11  y_train_1_node_6_11 
 x_train_1_from_6_11_to_6_12  y_train_1_node_6_12  x_train_1_from_6_12_to_6_13 
 y_train_1_node_6_13  x_train_1_from_6_13_to_6_14  y_train_1_node_6_14 
 x_train_1_from_6_14_to_6_15  y_train_1_node_6_15  x_train_1_from_6_15_to_6_16 
 y_train_1_node_6_16  x_train_1_from_6_16_to_6_17  y_train_1_node_6_17 
 x_train_1_from_6_17_to_6_18  y_train_1_node_6_18  x_train_1_from_6_18_to_6_19 
 y_train_1_node_6_19  x_train_1_from_6_19_to_6_20  y_train_1_node_6_20 
 x_train_1_from_6_20_to_6_21  y_train_1_node_6_21  x_train_1_from_6_21_to_6_22 
 y_train_1_node_6_22  x_train_1_from_6_22_to_6_23  y_train_1_node_6_23 
 x_train_1_from_6_23_to_6_24  y_train_1_node_6_24  x_train_1_from_6_24_to_6_25 
 y_train_1_node_6_25  x_train_1_from_6_25_to_6_26  y_train_1_node_6_26 
 x_train_1_from_6_26_to_6_27  y_train_1_node_6_27 
Generals
 theta_train_0_segment_0  theta_train_0_segment_1  theta_train_0_segment_2 
 theta_train_0_segment_3  theta_train_0_segment_4  theta_train_0_segment_5 
 theta_train_0_segment_6  theta_train_1_segment_0  theta_train_1_segment_1 
 theta_train_1_segment_2  theta_train_1_segment_3  theta_train_1_segment_4 
 theta_train_1_segment_5  theta_train_1_segment_6  d_train_0  d_train_1 
 _e_train_0_segment_1 
End
