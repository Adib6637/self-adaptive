\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  drone_used_0 + drone_used_1 + drone_used_2 + drone_used_3 + drone_used_4
   + 0 sensor_fps_2_0 + 0 sensor_pix_2_0 + 0 sensor_pix_x_inv_0
   + drone_energy_consumption_0 + 0 sensor_fps_2_1 + 0 sensor_pix_2_1
   + 0 sensor_pix_x_inv_1 + drone_energy_consumption_1 + 0 sensor_fps_2_2
   + 0 sensor_pix_2_2 + 0 sensor_pix_x_inv_2 + drone_energy_consumption_2
   + 0 sensor_fps_2_3 + 0 sensor_pix_2_3 + 0 sensor_pix_x_inv_3
   + drone_energy_consumption_3 + 0 sensor_fps_2_4 + 0 sensor_pix_2_4
   + 0 sensor_pix_x_inv_4 + drone_energy_consumption_4
Subject To
 one_identity: one + [ ] = 1
 drone_used_total_limit: drone_used_0 + drone_used_1 + drone_used_2
   + drone_used_3 + drone_used_4 + [ ] <= 5
 drone_v_true_identity_0: - 12 drone_v_0 + drone_v_true_0 + [ ] = 0
 sensor_fps_pix_identity_0: sensor_fps_pix_0 + [
   - sensor_fps_0 * sensor_pix_0 ] = 0
 sensor_fps_true_identity_0: - 129 sensor_fps_0 + sensor_fps_true_0 + [ ]
   = 15
 coverage_fps_lower_bound_0: - 0.595876796297105 drone_v_0
   + sensor_fps_true_0 + [ ] >= 0
 sensor_pix_xy_selector_total_1_0: sensor_pix_selector_0_0
   + sensor_pix_selector_0_1 + sensor_pix_selector_0_2 + [ ] = 1
 sensor_pix_x_value_restriction_0: sensor_pix_x_0
   - 640 sensor_pix_selector_0_0 - 1280 sensor_pix_selector_0_1
   - 1600 sensor_pix_selector_0_2 + [ ] = 0
 sensor_pix_y_value_restriction_0: sensor_pix_y_0
   - 480 sensor_pix_selector_0_0 - 1120 sensor_pix_selector_0_1
   - 1400 sensor_pix_selector_0_2 + [ ] = 0
 drone_pix_value_restriction_0: sensor_pix_0
   - 0.0382857142857143 sensor_pix_selector_0_0
   - 0.4405714285714286 sensor_pix_selector_0_1
   - 0.7285714285714285 sensor_pix_selector_0_2 + [ ] = 0
 drone_set_fps_selector_total_1_0: drone_set_fps_selector_0_0
   + drone_set_fps_selector_0_1 + drone_set_fps_selector_0_2 + [ ] = 1
 drone_set_fps_value_restriction_0: sensor_fps_0
   - 0.1162790697674419 drone_set_fps_selector_0_0
   - 0.3488372093023256 drone_set_fps_selector_0_1
   - 0.5813953488372093 drone_set_fps_selector_0_2 + [ ] = 0
 covered_area_total_t0_identity_0: covered_area_total_t0_0 + [
   - covered_area_x_t0_0 * covered_area_y_t0_0 ] = 0
 covered_area_x_t0_y_t0_identity_0: [ - sensor_pix_x_0 * sensor_pix_y_inv_0
   + covered_area_x_t0_0 * covered_area_y_t0_inv_0 ] = 0
 number_of_place_covered_identity_0: number_of_place_covered_0 + [
   - covered_area_total_t0_inv_0 * covered_area_total_0 ] = 0
 covered_area_x_t0_identity_0: - 167.819926235456 drone_h_0
   + covered_area_x_t0_0 + [ ] = 0
 covered_distance_identity_0: covered_area_x_t0_0 + covered_distance_0 + [
   - covered_area_x_t0_0 * number_of_place_covered_0 ] = 0
 operation_time_identity_0: operation_time_0 + [
   - drone_v_true_inv_0 * covered_distance_0 ] = 0
 actuator_power_positive_0: 1.076926165335457 drone_v_0
   - 0.6565098249938612 drone_v_2_0 + 0.306118091548782 drone_v_3_0
   + 0.00247314618404392 drone_h_0 + [ ] >= 0.2096125814661343
 drone_pa_consumption_identity_0: - 861.5409322683654 drone_v_0
   + 525.2078599950889 drone_v_2_0 - 244.8944732390256 drone_v_3_0
   - 1.978516947235133 drone_h_0 + drone_pa_consumption_0 + [ ]
   = 182.3099348270926
 drone_ps_consumption_identity_0: - 6.267389498042036 sensor_fps_0
   - 34.40582216259665 sensor_pix_0 + drone_ps_consumption_0 + [ ]
   = -0.8703970046105134
 drone_energy_consumption_identity_0: drone_energy_consumption_0 + [
   - operation_time_0 * drone_pa_consumption_0
   - operation_time_0 * drone_ps_consumption_0 ] = 0
 covered_area_true_identity_0: covered_area_true_0 + [
   - drone_used_0 * covered_area_total_0 ] = 0
 covered_area_total_limit_0: covered_area_total_0 + [ ] <= 1e+06
 covered_area_total_positive_0: [ drone_used_0 * covered_area_total_0 ]
   >= 0
 operation_time_req_identity_0: - 1200 drone_used_0 - operation_time_0
   + operation_time_req_0 + [ ] = 0
 charging_cycles_identity_0: - 5.5555555555555556e-04 operation_time_0
   + charging_cycles_0 + [ ] = -1
 drone_v_true_identity_1: - 12 drone_v_1 + drone_v_true_1 + [ ] = 0
 sensor_fps_pix_identity_1: sensor_fps_pix_1 + [
   - sensor_fps_1 * sensor_pix_1 ] = 0
 sensor_fps_true_identity_1: - 129 sensor_fps_1 + sensor_fps_true_1 + [ ]
   = 15
 coverage_fps_lower_bound_1: - 0.595876796297105 drone_v_1
   + sensor_fps_true_1 + [ ] >= 0
 sensor_pix_xy_selector_total_1_1: sensor_pix_selector_1_0
   + sensor_pix_selector_1_1 + sensor_pix_selector_1_2 + [ ] = 1
 sensor_pix_x_value_restriction_1: sensor_pix_x_1
   - 640 sensor_pix_selector_1_0 - 1280 sensor_pix_selector_1_1
   - 1600 sensor_pix_selector_1_2 + [ ] = 0
 sensor_pix_y_value_restriction_1: sensor_pix_y_1
   - 480 sensor_pix_selector_1_0 - 1120 sensor_pix_selector_1_1
   - 1400 sensor_pix_selector_1_2 + [ ] = 0
 drone_pix_value_restriction_1: sensor_pix_1
   - 0.0382857142857143 sensor_pix_selector_1_0
   - 0.4405714285714286 sensor_pix_selector_1_1
   - 0.7285714285714285 sensor_pix_selector_1_2 + [ ] = 0
 drone_set_fps_selector_total_1_1: drone_set_fps_selector_1_0
   + drone_set_fps_selector_1_1 + drone_set_fps_selector_1_2 + [ ] = 1
 drone_set_fps_value_restriction_1: sensor_fps_1
   - 0.1162790697674419 drone_set_fps_selector_1_0
   - 0.3488372093023256 drone_set_fps_selector_1_1
   - 0.5813953488372093 drone_set_fps_selector_1_2 + [ ] = 0
 covered_area_total_t0_identity_1: covered_area_total_t0_1 + [
   - covered_area_x_t0_1 * covered_area_y_t0_1 ] = 0
 covered_area_x_t0_y_t0_identity_1: [ - sensor_pix_x_1 * sensor_pix_y_inv_1
   + covered_area_x_t0_1 * covered_area_y_t0_inv_1 ] = 0
 number_of_place_covered_identity_1: number_of_place_covered_1 + [
   - covered_area_total_t0_inv_1 * covered_area_total_1 ] = 0
 covered_area_x_t0_identity_1: - 167.819926235456 drone_h_1
   + covered_area_x_t0_1 + [ ] = 0
 covered_distance_identity_1: covered_area_x_t0_1 + covered_distance_1 + [
   - covered_area_x_t0_1 * number_of_place_covered_1 ] = 0
 operation_time_identity_1: operation_time_1 + [
   - drone_v_true_inv_1 * covered_distance_1 ] = 0
 actuator_power_positive_1: 1.076926165335457 drone_v_1
   - 0.6565098249938612 drone_v_2_1 + 0.306118091548782 drone_v_3_1
   + 0.00247314618404392 drone_h_1 + [ ] >= 0.2096125814661343
 drone_pa_consumption_identity_1: - 861.5409322683654 drone_v_1
   + 525.2078599950889 drone_v_2_1 - 244.8944732390256 drone_v_3_1
   - 1.978516947235133 drone_h_1 + drone_pa_consumption_1 + [ ]
   = 182.3099348270926
 drone_ps_consumption_identity_1: - 6.267389498042036 sensor_fps_1
   - 34.40582216259665 sensor_pix_1 + drone_ps_consumption_1 + [ ]
   = -0.8703970046105134
 drone_energy_consumption_identity_1: drone_energy_consumption_1 + [
   - operation_time_1 * drone_pa_consumption_1
   - operation_time_1 * drone_ps_consumption_1 ] = 0
 covered_area_true_identity_1: covered_area_true_1 + [
   - drone_used_1 * covered_area_total_1 ] = 0
 covered_area_total_limit_1: covered_area_total_1 + [ ] <= 1e+06
 covered_area_total_positive_1: [ drone_used_1 * covered_area_total_1 ]
   >= 0
 operation_time_req_identity_1: - 1200 drone_used_1 - operation_time_1
   + operation_time_req_1 + [ ] = 0
 charging_cycles_identity_1: - 5.5555555555555556e-04 operation_time_1
   + charging_cycles_1 + [ ] = -1
 drone_v_true_identity_2: - 12 drone_v_2 + drone_v_true_2 + [ ] = 0
 sensor_fps_pix_identity_2: sensor_fps_pix_2 + [
   - sensor_fps_2 * sensor_pix_2 ] = 0
 sensor_fps_true_identity_2: - 129 sensor_fps_2 + sensor_fps_true_2 + [ ]
   = 15
 coverage_fps_lower_bound_2: - 0.595876796297105 drone_v_2
   + sensor_fps_true_2 + [ ] >= 0
 sensor_pix_xy_selector_total_1_2: sensor_pix_selector_2_0
   + sensor_pix_selector_2_1 + sensor_pix_selector_2_2 + [ ] = 1
 sensor_pix_x_value_restriction_2: sensor_pix_x_2
   - 640 sensor_pix_selector_2_0 - 1280 sensor_pix_selector_2_1
   - 1600 sensor_pix_selector_2_2 + [ ] = 0
 sensor_pix_y_value_restriction_2: sensor_pix_y_2
   - 480 sensor_pix_selector_2_0 - 1120 sensor_pix_selector_2_1
   - 1400 sensor_pix_selector_2_2 + [ ] = 0
 drone_pix_value_restriction_2: sensor_pix_2
   - 0.0382857142857143 sensor_pix_selector_2_0
   - 0.4405714285714286 sensor_pix_selector_2_1
   - 0.7285714285714285 sensor_pix_selector_2_2 + [ ] = 0
 drone_set_fps_selector_total_1_2: drone_set_fps_selector_2_0
   + drone_set_fps_selector_2_1 + drone_set_fps_selector_2_2 + [ ] = 1
 drone_set_fps_value_restriction_2: sensor_fps_2
   - 0.1162790697674419 drone_set_fps_selector_2_0
   - 0.3488372093023256 drone_set_fps_selector_2_1
   - 0.5813953488372093 drone_set_fps_selector_2_2 + [ ] = 0
 covered_area_total_t0_identity_2: covered_area_total_t0_2 + [
   - covered_area_x_t0_2 * covered_area_y_t0_2 ] = 0
 covered_area_x_t0_y_t0_identity_2: [ - sensor_pix_x_2 * sensor_pix_y_inv_2
   + covered_area_x_t0_2 * covered_area_y_t0_inv_2 ] = 0
 number_of_place_covered_identity_2: number_of_place_covered_2 + [
   - covered_area_total_t0_inv_2 * covered_area_total_2 ] = 0
 covered_area_x_t0_identity_2: - 167.819926235456 drone_h_2
   + covered_area_x_t0_2 + [ ] = 0
 covered_distance_identity_2: covered_area_x_t0_2 + covered_distance_2 + [
   - covered_area_x_t0_2 * number_of_place_covered_2 ] = 0
 operation_time_identity_2: operation_time_2 + [
   - drone_v_true_inv_2 * covered_distance_2 ] = 0
 actuator_power_positive_2: 1.076926165335457 drone_v_2
   - 0.6565098249938612 drone_v_2_2 + 0.306118091548782 drone_v_3_2
   + 0.00247314618404392 drone_h_2 + [ ] >= 0.2096125814661343
 drone_pa_consumption_identity_2: - 861.5409322683654 drone_v_2
   + 525.2078599950889 drone_v_2_2 - 244.8944732390256 drone_v_3_2
   - 1.978516947235133 drone_h_2 + drone_pa_consumption_2 + [ ]
   = 182.3099348270926
 drone_ps_consumption_identity_2: - 6.267389498042036 sensor_fps_2
   - 34.40582216259665 sensor_pix_2 + drone_ps_consumption_2 + [ ]
   = -0.8703970046105134
 drone_energy_consumption_identity_2: drone_energy_consumption_2 + [
   - operation_time_2 * drone_pa_consumption_2
   - operation_time_2 * drone_ps_consumption_2 ] = 0
 covered_area_true_identity_2: covered_area_true_2 + [
   - drone_used_2 * covered_area_total_2 ] = 0
 covered_area_total_limit_2: covered_area_total_2 + [ ] <= 1e+06
 covered_area_total_positive_2: [ drone_used_2 * covered_area_total_2 ]
   >= 0
 operation_time_req_identity_2: - 1200 drone_used_2 - operation_time_2
   + operation_time_req_2 + [ ] = 0
 charging_cycles_identity_2: - 5.5555555555555556e-04 operation_time_2
   + charging_cycles_2 + [ ] = -1
 drone_v_true_identity_3: - 12 drone_v_3 + drone_v_true_3 + [ ] = 0
 sensor_fps_pix_identity_3: sensor_fps_pix_3 + [
   - sensor_fps_3 * sensor_pix_3 ] = 0
 sensor_fps_true_identity_3: - 129 sensor_fps_3 + sensor_fps_true_3 + [ ]
   = 15
 coverage_fps_lower_bound_3: - 0.595876796297105 drone_v_3
   + sensor_fps_true_3 + [ ] >= 0
 sensor_pix_xy_selector_total_1_3: sensor_pix_selector_3_0
   + sensor_pix_selector_3_1 + sensor_pix_selector_3_2 + [ ] = 1
 sensor_pix_x_value_restriction_3: sensor_pix_x_3
   - 640 sensor_pix_selector_3_0 - 1280 sensor_pix_selector_3_1
   - 1600 sensor_pix_selector_3_2 + [ ] = 0
 sensor_pix_y_value_restriction_3: sensor_pix_y_3
   - 480 sensor_pix_selector_3_0 - 1120 sensor_pix_selector_3_1
   - 1400 sensor_pix_selector_3_2 + [ ] = 0
 drone_pix_value_restriction_3: sensor_pix_3
   - 0.0382857142857143 sensor_pix_selector_3_0
   - 0.4405714285714286 sensor_pix_selector_3_1
   - 0.7285714285714285 sensor_pix_selector_3_2 + [ ] = 0
 drone_set_fps_selector_total_1_3: drone_set_fps_selector_3_0
   + drone_set_fps_selector_3_1 + drone_set_fps_selector_3_2 + [ ] = 1
 drone_set_fps_value_restriction_3: sensor_fps_3
   - 0.1162790697674419 drone_set_fps_selector_3_0
   - 0.3488372093023256 drone_set_fps_selector_3_1
   - 0.5813953488372093 drone_set_fps_selector_3_2 + [ ] = 0
 covered_area_total_t0_identity_3: covered_area_total_t0_3 + [
   - covered_area_x_t0_3 * covered_area_y_t0_3 ] = 0
 covered_area_x_t0_y_t0_identity_3: [ - sensor_pix_x_3 * sensor_pix_y_inv_3
   + covered_area_x_t0_3 * covered_area_y_t0_inv_3 ] = 0
 number_of_place_covered_identity_3: number_of_place_covered_3 + [
   - covered_area_total_t0_inv_3 * covered_area_total_3 ] = 0
 covered_area_x_t0_identity_3: - 167.819926235456 drone_h_3
   + covered_area_x_t0_3 + [ ] = 0
 covered_distance_identity_3: covered_area_x_t0_3 + covered_distance_3 + [
   - covered_area_x_t0_3 * number_of_place_covered_3 ] = 0
 operation_time_identity_3: operation_time_3 + [
   - drone_v_true_inv_3 * covered_distance_3 ] = 0
 actuator_power_positive_3: 1.076926165335457 drone_v_3
   - 0.6565098249938612 drone_v_2_3 + 0.306118091548782 drone_v_3_3
   + 0.00247314618404392 drone_h_3 + [ ] >= 0.2096125814661343
 drone_pa_consumption_identity_3: - 861.5409322683654 drone_v_3
   + 525.2078599950889 drone_v_2_3 - 244.8944732390256 drone_v_3_3
   - 1.978516947235133 drone_h_3 + drone_pa_consumption_3 + [ ]
   = 182.3099348270926
 drone_ps_consumption_identity_3: - 6.267389498042036 sensor_fps_3
   - 34.40582216259665 sensor_pix_3 + drone_ps_consumption_3 + [ ]
   = -0.8703970046105134
 drone_energy_consumption_identity_3: drone_energy_consumption_3 + [
   - operation_time_3 * drone_pa_consumption_3
   - operation_time_3 * drone_ps_consumption_3 ] = 0
 covered_area_true_identity_3: covered_area_true_3 + [
   - drone_used_3 * covered_area_total_3 ] = 0
 covered_area_total_limit_3: covered_area_total_3 + [ ] <= 1e+06
 covered_area_total_positive_3: [ drone_used_3 * covered_area_total_3 ]
   >= 0
 operation_time_req_identity_3: - 1200 drone_used_3 - operation_time_3
   + operation_time_req_3 + [ ] = 0
 charging_cycles_identity_3: - 5.5555555555555556e-04 operation_time_3
   + charging_cycles_3 + [ ] = -1
 drone_v_true_identity_4: - 12 drone_v_4 + drone_v_true_4 + [ ] = 0
 sensor_fps_pix_identity_4: sensor_fps_pix_4 + [
   - sensor_fps_4 * sensor_pix_4 ] = 0
 sensor_fps_true_identity_4: - 129 sensor_fps_4 + sensor_fps_true_4 + [ ]
   = 15
 coverage_fps_lower_bound_4: - 0.595876796297105 drone_v_4
   + sensor_fps_true_4 + [ ] >= 0
 sensor_pix_xy_selector_total_1_4: sensor_pix_selector_4_0
   + sensor_pix_selector_4_1 + sensor_pix_selector_4_2 + [ ] = 1
 sensor_pix_x_value_restriction_4: sensor_pix_x_4
   - 640 sensor_pix_selector_4_0 - 1280 sensor_pix_selector_4_1
   - 1600 sensor_pix_selector_4_2 + [ ] = 0
 sensor_pix_y_value_restriction_4: sensor_pix_y_4
   - 480 sensor_pix_selector_4_0 - 1120 sensor_pix_selector_4_1
   - 1400 sensor_pix_selector_4_2 + [ ] = 0
 drone_pix_value_restriction_4: sensor_pix_4
   - 0.0382857142857143 sensor_pix_selector_4_0
   - 0.4405714285714286 sensor_pix_selector_4_1
   - 0.7285714285714285 sensor_pix_selector_4_2 + [ ] = 0
 drone_set_fps_selector_total_1_4: drone_set_fps_selector_4_0
   + drone_set_fps_selector_4_1 + drone_set_fps_selector_4_2 + [ ] = 1
 drone_set_fps_value_restriction_4: sensor_fps_4
   - 0.1162790697674419 drone_set_fps_selector_4_0
   - 0.3488372093023256 drone_set_fps_selector_4_1
   - 0.5813953488372093 drone_set_fps_selector_4_2 + [ ] = 0
 covered_area_total_t0_identity_4: covered_area_total_t0_4 + [
   - covered_area_x_t0_4 * covered_area_y_t0_4 ] = 0
 covered_area_x_t0_y_t0_identity_4: [ - sensor_pix_x_4 * sensor_pix_y_inv_4
   + covered_area_x_t0_4 * covered_area_y_t0_inv_4 ] = 0
 number_of_place_covered_identity_4: number_of_place_covered_4 + [
   - covered_area_total_t0_inv_4 * covered_area_total_4 ] = 0
 covered_area_x_t0_identity_4: - 167.819926235456 drone_h_4
   + covered_area_x_t0_4 + [ ] = 0
 covered_distance_identity_4: covered_area_x_t0_4 + covered_distance_4 + [
   - covered_area_x_t0_4 * number_of_place_covered_4 ] = 0
 operation_time_identity_4: operation_time_4 + [
   - drone_v_true_inv_4 * covered_distance_4 ] = 0
 actuator_power_positive_4: 1.076926165335457 drone_v_4
   - 0.6565098249938612 drone_v_2_4 + 0.306118091548782 drone_v_3_4
   + 0.00247314618404392 drone_h_4 + [ ] >= 0.2096125814661343
 drone_pa_consumption_identity_4: - 861.5409322683654 drone_v_4
   + 525.2078599950889 drone_v_2_4 - 244.8944732390256 drone_v_3_4
   - 1.978516947235133 drone_h_4 + drone_pa_consumption_4 + [ ]
   = 182.3099348270926
 drone_ps_consumption_identity_4: - 6.267389498042036 sensor_fps_4
   - 34.40582216259665 sensor_pix_4 + drone_ps_consumption_4 + [ ]
   = -0.8703970046105134
 drone_energy_consumption_identity_4: drone_energy_consumption_4 + [
   - operation_time_4 * drone_pa_consumption_4
   - operation_time_4 * drone_ps_consumption_4 ] = 0
 covered_area_true_identity_4: covered_area_true_4 + [
   - drone_used_4 * covered_area_total_4 ] = 0
 covered_area_total_limit_4: covered_area_total_4 + [ ] <= 1e+06
 covered_area_total_positive_4: [ drone_used_4 * covered_area_total_4 ]
   >= 0
 operation_time_req_identity_4: - 1200 drone_used_4 - operation_time_4
   + operation_time_req_4 + [ ] = 0
 charging_cycles_identity_4: - 5.5555555555555556e-04 operation_time_4
   + charging_cycles_4 + [ ] = -1
 covered_area_total_sum_constraint: covered_area_true_0
   + covered_area_true_1 + covered_area_true_2 + covered_area_true_3
   + covered_area_true_4 + [ ] = 1e+06
Bounds
 0.9 <= one <= 1
 drone_v_0 <= 1
 drone_v_2_0 <= 1
 drone_v_3_0 <= 1
 4 <= drone_v_true_0 <= 12
 drone_v_true_inv_0 >= 0.0833333333333333
 drone_h_0 <= 1
 sensor_fps_0 <= 1
 sensor_fps_2_0 <= 1
 15 <= sensor_fps_true_0 <= 144
 sensor_pix_0 <= 1
 sensor_pix_2_0 <= 1
 640 <= sensor_pix_x_0 <= 1600
 0.000625 <= sensor_pix_x_inv_0 <= 0.0015625
 480 <= sensor_pix_y_0 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_0 <= 0.00208333333333333
 sensor_fps_pix_0 <= 1
 1e-08 <= covered_area_x_t0_0 <= 167.819926235456
 covered_area_y_t0_0 <= 167.819926235456
 0.00595876796297105 <= covered_area_y_t0_inv_0 <= 1e+08
 1e-16 <= covered_area_total_t0_0 <= 28163.52764167389
 3.55069e-05 <= covered_area_total_t0_inv_0 <= 1e+16
 1e-16 <= covered_area_total_0 <= 1e+06
 covered_distance_0 >= 1e-08
 drone_ps_consumption_0 <= 10
 covered_area_true_0 <= 1e+06
 charging_cycles_0 <= 20
 drone_v_1 <= 1
 drone_v_2_1 <= 1
 drone_v_3_1 <= 1
 4 <= drone_v_true_1 <= 12
 drone_v_true_inv_1 >= 0.0833333333333333
 drone_h_1 <= 1
 sensor_fps_1 <= 1
 sensor_fps_2_1 <= 1
 15 <= sensor_fps_true_1 <= 144
 sensor_pix_1 <= 1
 sensor_pix_2_1 <= 1
 640 <= sensor_pix_x_1 <= 1600
 0.000625 <= sensor_pix_x_inv_1 <= 0.0015625
 480 <= sensor_pix_y_1 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_1 <= 0.00208333333333333
 sensor_fps_pix_1 <= 1
 1e-08 <= covered_area_x_t0_1 <= 167.819926235456
 covered_area_y_t0_1 <= 167.819926235456
 0.00595876796297105 <= covered_area_y_t0_inv_1 <= 1e+08
 1e-16 <= covered_area_total_t0_1 <= 28163.52764167389
 3.55069e-05 <= covered_area_total_t0_inv_1 <= 1e+16
 1e-16 <= covered_area_total_1 <= 1e+06
 covered_distance_1 >= 1e-08
 drone_ps_consumption_1 <= 10
 covered_area_true_1 <= 1e+06
 charging_cycles_1 <= 20
 drone_v_2 <= 1
 drone_v_2_2 <= 1
 drone_v_3_2 <= 1
 4 <= drone_v_true_2 <= 12
 drone_v_true_inv_2 >= 0.0833333333333333
 drone_h_2 <= 1
 sensor_fps_2 <= 1
 sensor_fps_2_2 <= 1
 15 <= sensor_fps_true_2 <= 144
 sensor_pix_2 <= 1
 sensor_pix_2_2 <= 1
 640 <= sensor_pix_x_2 <= 1600
 0.000625 <= sensor_pix_x_inv_2 <= 0.0015625
 480 <= sensor_pix_y_2 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_2 <= 0.00208333333333333
 sensor_fps_pix_2 <= 1
 1e-08 <= covered_area_x_t0_2 <= 167.819926235456
 covered_area_y_t0_2 <= 167.819926235456
 0.00595876796297105 <= covered_area_y_t0_inv_2 <= 1e+08
 1e-16 <= covered_area_total_t0_2 <= 28163.52764167389
 3.55069e-05 <= covered_area_total_t0_inv_2 <= 1e+16
 1e-16 <= covered_area_total_2 <= 1e+06
 covered_distance_2 >= 1e-08
 drone_ps_consumption_2 <= 10
 covered_area_true_2 <= 1e+06
 charging_cycles_2 <= 20
 drone_v_3 <= 1
 drone_v_2_3 <= 1
 drone_v_3_3 <= 1
 4 <= drone_v_true_3 <= 12
 drone_v_true_inv_3 >= 0.0833333333333333
 drone_h_3 <= 1
 sensor_fps_3 <= 1
 sensor_fps_2_3 <= 1
 15 <= sensor_fps_true_3 <= 144
 sensor_pix_3 <= 1
 sensor_pix_2_3 <= 1
 640 <= sensor_pix_x_3 <= 1600
 0.000625 <= sensor_pix_x_inv_3 <= 0.0015625
 480 <= sensor_pix_y_3 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_3 <= 0.00208333333333333
 sensor_fps_pix_3 <= 1
 1e-08 <= covered_area_x_t0_3 <= 167.819926235456
 covered_area_y_t0_3 <= 167.819926235456
 0.00595876796297105 <= covered_area_y_t0_inv_3 <= 1e+08
 1e-16 <= covered_area_total_t0_3 <= 28163.52764167389
 3.55069e-05 <= covered_area_total_t0_inv_3 <= 1e+16
 1e-16 <= covered_area_total_3 <= 1e+06
 covered_distance_3 >= 1e-08
 drone_ps_consumption_3 <= 10
 covered_area_true_3 <= 1e+06
 charging_cycles_3 <= 20
 drone_v_4 <= 1
 drone_v_2_4 <= 1
 drone_v_3_4 <= 1
 4 <= drone_v_true_4 <= 12
 drone_v_true_inv_4 >= 0.0833333333333333
 drone_h_4 <= 1
 sensor_fps_4 <= 1
 sensor_fps_2_4 <= 1
 15 <= sensor_fps_true_4 <= 144
 sensor_pix_4 <= 1
 sensor_pix_2_4 <= 1
 640 <= sensor_pix_x_4 <= 1600
 0.000625 <= sensor_pix_x_inv_4 <= 0.0015625
 480 <= sensor_pix_y_4 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_4 <= 0.00208333333333333
 sensor_fps_pix_4 <= 1
 1e-08 <= covered_area_x_t0_4 <= 167.819926235456
 covered_area_y_t0_4 <= 167.819926235456
 0.00595876796297105 <= covered_area_y_t0_inv_4 <= 1e+08
 1e-16 <= covered_area_total_t0_4 <= 28163.52764167389
 3.55069e-05 <= covered_area_total_t0_inv_4 <= 1e+16
 1e-16 <= covered_area_total_4 <= 1e+06
 covered_distance_4 >= 1e-08
 drone_ps_consumption_4 <= 10
 covered_area_true_4 <= 1e+06
 charging_cycles_4 <= 20
Binaries
 drone_used_0 drone_used_1 drone_used_2 drone_used_3 drone_used_4
 sensor_pix_selector_0_0 sensor_pix_selector_0_1 sensor_pix_selector_0_2
 drone_set_fps_selector_0_0 drone_set_fps_selector_0_1
 drone_set_fps_selector_0_2 sensor_pix_selector_1_0 sensor_pix_selector_1_1
 sensor_pix_selector_1_2 drone_set_fps_selector_1_0
 drone_set_fps_selector_1_1 drone_set_fps_selector_1_2
 sensor_pix_selector_2_0 sensor_pix_selector_2_1 sensor_pix_selector_2_2
 drone_set_fps_selector_2_0 drone_set_fps_selector_2_1
 drone_set_fps_selector_2_2 sensor_pix_selector_3_0 sensor_pix_selector_3_1
 sensor_pix_selector_3_2 drone_set_fps_selector_3_0
 drone_set_fps_selector_3_1 drone_set_fps_selector_3_2
 sensor_pix_selector_4_0 sensor_pix_selector_4_1 sensor_pix_selector_4_2
 drone_set_fps_selector_4_0 drone_set_fps_selector_4_1
 drone_set_fps_selector_4_2
Generals
 sensor_fps_true_0 sensor_pix_x_0 sensor_pix_y_0 charging_cycles_0
 sensor_fps_true_1 sensor_pix_x_1 sensor_pix_y_1 charging_cycles_1
 sensor_fps_true_2 sensor_pix_x_2 sensor_pix_y_2 charging_cycles_2
 sensor_fps_true_3 sensor_pix_x_3 sensor_pix_y_3 charging_cycles_3
 sensor_fps_true_4 sensor_pix_x_4 sensor_pix_y_4 charging_cycles_4
Semi-continuous
 drone_v_true_0 drone_v_true_inv_0 covered_area_x_t0_0
 covered_area_y_t0_inv_0 covered_area_total_t0_0
 covered_area_total_t0_inv_0 covered_area_total_0 number_of_place_covered_0
 covered_distance_0 operation_time_0 drone_pa_consumption_0
 drone_ps_consumption_0 drone_energy_consumption_0 covered_area_true_0
 operation_time_req_0 charging_cycles_0 drone_v_true_1 drone_v_true_inv_1
 covered_area_x_t0_1 covered_area_y_t0_inv_1 covered_area_total_t0_1
 covered_area_total_t0_inv_1 covered_area_total_1 number_of_place_covered_1
 covered_distance_1 operation_time_1 drone_pa_consumption_1
 drone_ps_consumption_1 drone_energy_consumption_1 covered_area_true_1
 operation_time_req_1 charging_cycles_1 drone_v_true_2 drone_v_true_inv_2
 covered_area_x_t0_2 covered_area_y_t0_inv_2 covered_area_total_t0_2
 covered_area_total_t0_inv_2 covered_area_total_2 number_of_place_covered_2
 covered_distance_2 operation_time_2 drone_pa_consumption_2
 drone_ps_consumption_2 drone_energy_consumption_2 covered_area_true_2
 operation_time_req_2 charging_cycles_2 drone_v_true_3 drone_v_true_inv_3
 covered_area_x_t0_3 covered_area_y_t0_inv_3 covered_area_total_t0_3
 covered_area_total_t0_inv_3 covered_area_total_3 number_of_place_covered_3
 covered_distance_3 operation_time_3 drone_pa_consumption_3
 drone_ps_consumption_3 drone_energy_consumption_3 covered_area_true_3
 operation_time_req_3 charging_cycles_3 drone_v_true_4 drone_v_true_inv_4
 covered_area_x_t0_4 covered_area_y_t0_inv_4 covered_area_total_t0_4
 covered_area_total_t0_inv_4 covered_area_total_4 number_of_place_covered_4
 covered_distance_4 operation_time_4 drone_pa_consumption_4
 drone_ps_consumption_4 drone_energy_consumption_4 covered_area_true_4
 operation_time_req_4 charging_cycles_4
General Constraints
 drone_v_2_identity_0: drone_v_2_0 = POW ( drone_v_0 ^ 2 )
 drone_v_3_identity_0: drone_v_3_0 = POW ( drone_v_0 ^ 3 )
 drone_v_true_inv_identity_0: drone_v_true_inv_0 =
   POW ( drone_v_true_0 ^ -1 )
 sensor_fps_2_identity_0: sensor_fps_2_0 = POW ( sensor_fps_0 ^ 2 )
 sensor_pix_2_identity_0: sensor_pix_2_0 = POW ( sensor_pix_0 ^ 2 )
 sensor_pix_x_inv_identity_0: sensor_pix_x_inv_0 =
   POW ( sensor_pix_x_0 ^ -1 )
 sensor_pix_y_inv_identity_0: sensor_pix_y_inv_0 =
   POW ( sensor_pix_y_0 ^ -1 )
 covered_area_y_t0_inv_identity_0: covered_area_y_t0_inv_0 =
   POW ( covered_area_y_t0_0 ^ -1 )
 covered_area_total_t0_inv_identity_0: covered_area_total_t0_inv_0 =
   POW ( covered_area_total_t0_0 ^ -1 )
 drone_v_2_identity_1: drone_v_2_1 = POW ( drone_v_1 ^ 2 )
 drone_v_3_identity_1: drone_v_3_1 = POW ( drone_v_1 ^ 3 )
 drone_v_true_inv_identity_1: drone_v_true_inv_1 =
   POW ( drone_v_true_1 ^ -1 )
 sensor_fps_2_identity_1: sensor_fps_2_1 = POW ( sensor_fps_1 ^ 2 )
 sensor_pix_2_identity_1: sensor_pix_2_1 = POW ( sensor_pix_1 ^ 2 )
 sensor_pix_x_inv_identity_1: sensor_pix_x_inv_1 =
   POW ( sensor_pix_x_1 ^ -1 )
 sensor_pix_y_inv_identity_1: sensor_pix_y_inv_1 =
   POW ( sensor_pix_y_1 ^ -1 )
 covered_area_y_t0_inv_identity_1: covered_area_y_t0_inv_1 =
   POW ( covered_area_y_t0_1 ^ -1 )
 covered_area_total_t0_inv_identity_1: covered_area_total_t0_inv_1 =
   POW ( covered_area_total_t0_1 ^ -1 )
 drone_v_2_identity_2: drone_v_2_2 = POW ( drone_v_2 ^ 2 )
 drone_v_3_identity_2: drone_v_3_2 = POW ( drone_v_2 ^ 3 )
 drone_v_true_inv_identity_2: drone_v_true_inv_2 =
   POW ( drone_v_true_2 ^ -1 )
 sensor_fps_2_identity_2: sensor_fps_2_2 = POW ( sensor_fps_2 ^ 2 )
 sensor_pix_2_identity_2: sensor_pix_2_2 = POW ( sensor_pix_2 ^ 2 )
 sensor_pix_x_inv_identity_2: sensor_pix_x_inv_2 =
   POW ( sensor_pix_x_2 ^ -1 )
 sensor_pix_y_inv_identity_2: sensor_pix_y_inv_2 =
   POW ( sensor_pix_y_2 ^ -1 )
 covered_area_y_t0_inv_identity_2: covered_area_y_t0_inv_2 =
   POW ( covered_area_y_t0_2 ^ -1 )
 covered_area_total_t0_inv_identity_2: covered_area_total_t0_inv_2 =
   POW ( covered_area_total_t0_2 ^ -1 )
 drone_v_2_identity_3: drone_v_2_3 = POW ( drone_v_3 ^ 2 )
 drone_v_3_identity_3: drone_v_3_3 = POW ( drone_v_3 ^ 3 )
 drone_v_true_inv_identity_3: drone_v_true_inv_3 =
   POW ( drone_v_true_3 ^ -1 )
 sensor_fps_2_identity_3: sensor_fps_2_3 = POW ( sensor_fps_3 ^ 2 )
 sensor_pix_2_identity_3: sensor_pix_2_3 = POW ( sensor_pix_3 ^ 2 )
 sensor_pix_x_inv_identity_3: sensor_pix_x_inv_3 =
   POW ( sensor_pix_x_3 ^ -1 )
 sensor_pix_y_inv_identity_3: sensor_pix_y_inv_3 =
   POW ( sensor_pix_y_3 ^ -1 )
 covered_area_y_t0_inv_identity_3: covered_area_y_t0_inv_3 =
   POW ( covered_area_y_t0_3 ^ -1 )
 covered_area_total_t0_inv_identity_3: covered_area_total_t0_inv_3 =
   POW ( covered_area_total_t0_3 ^ -1 )
 drone_v_2_identity_4: drone_v_2_4 = POW ( drone_v_4 ^ 2 )
 drone_v_3_identity_4: drone_v_3_4 = POW ( drone_v_4 ^ 3 )
 drone_v_true_inv_identity_4: drone_v_true_inv_4 =
   POW ( drone_v_true_4 ^ -1 )
 sensor_fps_2_identity_4: sensor_fps_2_4 = POW ( sensor_fps_4 ^ 2 )
 sensor_pix_2_identity_4: sensor_pix_2_4 = POW ( sensor_pix_4 ^ 2 )
 sensor_pix_x_inv_identity_4: sensor_pix_x_inv_4 =
   POW ( sensor_pix_x_4 ^ -1 )
 sensor_pix_y_inv_identity_4: sensor_pix_y_inv_4 =
   POW ( sensor_pix_y_4 ^ -1 )
 covered_area_y_t0_inv_identity_4: covered_area_y_t0_inv_4 =
   POW ( covered_area_y_t0_4 ^ -1 )
 covered_area_total_t0_inv_identity_4: covered_area_total_t0_inv_4 =
   POW ( covered_area_total_t0_4 ^ -1 )
End
