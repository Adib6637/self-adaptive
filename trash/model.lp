\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0xbfd34a126752a5d2
Minimize
  drone_used_0 + drone_used_1 + drone_used_2 + drone_used_3 + drone_used_4
   + drone_energy_consumption_0 + drone_energy_consumption_1
   + drone_energy_consumption_2 + drone_energy_consumption_3
   + drone_energy_consumption_4
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
 covered_area_y_t0_y_t0_inv_identity_0: [
   sensor_pix_y_inv_0 * covered_area_y_t0_0 ] <= 0.01
 covered_area_x_t0_identity_0: - 167.819926235456 drone_h_0
   + covered_area_x_t0_0 + [ ] = 0
 covered_distance_identity_0: covered_area_x_t0_0 + covered_distance_0 + [
   - covered_area_x_t0_0 * number_of_place_covered_0 ] = 0
 operation_time_identity_0: operation_time_0 + [
   - drone_v_true_inv_0 * covered_distance_0 ] = 0
 actuator_power_positive_0: 1.080807827885279 drone_v_0
   - 0.6589299118717831 drone_v_2_0 + 0.3072465322030534 drone_v_3_0
   + 0.00255271204255636 drone_h_0 + [ ] >= 0.00124191198834134
 sensor_power_positive_0: 0.9289169752078502 sensor_fps_0
   + 0.00562203952865926 sensor_fps_2_0 + 0.6202206846179508 sensor_pix_0
   + 0.0628410939088008 sensor_pix_2_0
   + 0.0407911874142603 sensor_fps_pix_0 + [ ] >= 0
 drone_pa_consumption_identity_0: - 86.46462623082229 drone_v_0
   + 52.71439294974265 drone_v_2_0 - 24.57972257624427 drone_v_3_0
   - 0.2042169634045086 drone_h_0 + drone_pa_consumption_0 + [ ]
   = -0.099352959067307
 drone_ps_consumption_identity_0: - 5.573501851247102 sensor_fps_0
   - 0.0337322371719556 sensor_fps_2_0 - 3.721324107707705 sensor_pix_0
   - 0.3770465634528045 sensor_pix_2_0
   - 0.2447471244855617 sensor_fps_pix_0 + drone_ps_consumption_0 + [ ]
   = 5
 drone_energy_consumption_identity_0: drone_energy_consumption_0 + [
   - operation_time_0 * drone_pa_consumption_0
   - operation_time_0 * drone_ps_consumption_0 ] = 0
 covered_area_true_identity_0: covered_area_true_0 + [
   - drone_used_0 * covered_area_total_0 ] = 0
 covered_area_total_limit_0: covered_area_total_0 + [ ] <= 1e+06
 qc26: - 5.5555555555555556e-04 operation_time_0 + charging_cycles_0 + [ ]
   = -1
 operation_time_req_identity_0: - operation_time_0 + operation_time_req_0
   - 1200 charging_cycles_0 + [ ] = 0
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
 covered_area_y_t0_y_t0_inv_identity_1: [
   sensor_pix_y_inv_1 * covered_area_y_t0_1 ] <= 0.01
 covered_area_x_t0_identity_1: - 167.819926235456 drone_h_1
   + covered_area_x_t0_1 + [ ] = 0
 covered_distance_identity_1: covered_area_x_t0_1 + covered_distance_1 + [
   - covered_area_x_t0_1 * number_of_place_covered_1 ] = 0
 operation_time_identity_1: operation_time_1 + [
   - drone_v_true_inv_1 * covered_distance_1 ] = 0
 actuator_power_positive_1: 1.080807827885279 drone_v_1
   - 0.6589299118717831 drone_v_2_1 + 0.3072465322030534 drone_v_3_1
   + 0.00255271204255636 drone_h_1 + [ ] >= 0.00124191198834134
 sensor_power_positive_1: 0.9289169752078502 sensor_fps_1
   + 0.00562203952865926 sensor_fps_2_1 + 0.6202206846179508 sensor_pix_1
   + 0.0628410939088008 sensor_pix_2_1
   + 0.0407911874142603 sensor_fps_pix_1 + [ ] >= 0
 drone_pa_consumption_identity_1: - 86.46462623082229 drone_v_1
   + 52.71439294974265 drone_v_2_1 - 24.57972257624427 drone_v_3_1
   - 0.2042169634045086 drone_h_1 + drone_pa_consumption_1 + [ ]
   = -0.099352959067307
 drone_ps_consumption_identity_1: - 5.573501851247102 sensor_fps_1
   - 0.0337322371719556 sensor_fps_2_1 - 3.721324107707705 sensor_pix_1
   - 0.3770465634528045 sensor_pix_2_1
   - 0.2447471244855617 sensor_fps_pix_1 + drone_ps_consumption_1 + [ ]
   = 5
 drone_energy_consumption_identity_1: drone_energy_consumption_1 + [
   - operation_time_1 * drone_pa_consumption_1
   - operation_time_1 * drone_ps_consumption_1 ] = 0
 covered_area_true_identity_1: covered_area_true_1 + [
   - drone_used_1 * covered_area_total_1 ] = 0
 covered_area_total_limit_1: covered_area_total_1 + [ ] <= 1e+06
 qc52: - 5.5555555555555556e-04 operation_time_1 + charging_cycles_1 + [ ]
   = -1
 operation_time_req_identity_1: - operation_time_1 + operation_time_req_1
   - 1200 charging_cycles_1 + [ ] = 0
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
 covered_area_y_t0_y_t0_inv_identity_2: [
   sensor_pix_y_inv_2 * covered_area_y_t0_2 ] <= 0.01
 covered_area_x_t0_identity_2: - 167.819926235456 drone_h_2
   + covered_area_x_t0_2 + [ ] = 0
 covered_distance_identity_2: covered_area_x_t0_2 + covered_distance_2 + [
   - covered_area_x_t0_2 * number_of_place_covered_2 ] = 0
 operation_time_identity_2: operation_time_2 + [
   - drone_v_true_inv_2 * covered_distance_2 ] = 0
 actuator_power_positive_2: 1.080807827885279 drone_v_2
   - 0.6589299118717831 drone_v_2_2 + 0.3072465322030534 drone_v_3_2
   + 0.00255271204255636 drone_h_2 + [ ] >= 0.00124191198834134
 sensor_power_positive_2: 0.9289169752078502 sensor_fps_2
   + 0.00562203952865926 sensor_fps_2_2 + 0.6202206846179508 sensor_pix_2
   + 0.0628410939088008 sensor_pix_2_2
   + 0.0407911874142603 sensor_fps_pix_2 + [ ] >= 0
 drone_pa_consumption_identity_2: - 86.46462623082229 drone_v_2
   + 52.71439294974265 drone_v_2_2 - 24.57972257624427 drone_v_3_2
   - 0.2042169634045086 drone_h_2 + drone_pa_consumption_2 + [ ]
   = -0.099352959067307
 drone_ps_consumption_identity_2: - 5.573501851247102 sensor_fps_2
   - 0.0337322371719556 sensor_fps_2_2 - 3.721324107707705 sensor_pix_2
   - 0.3770465634528045 sensor_pix_2_2
   - 0.2447471244855617 sensor_fps_pix_2 + drone_ps_consumption_2 + [ ]
   = 5
 drone_energy_consumption_identity_2: drone_energy_consumption_2 + [
   - operation_time_2 * drone_pa_consumption_2
   - operation_time_2 * drone_ps_consumption_2 ] = 0
 covered_area_true_identity_2: covered_area_true_2 + [
   - drone_used_2 * covered_area_total_2 ] = 0
 covered_area_total_limit_2: covered_area_total_2 + [ ] <= 1e+06
 qc78: - 5.5555555555555556e-04 operation_time_2 + charging_cycles_2 + [ ]
   = -1
 operation_time_req_identity_2: - operation_time_2 + operation_time_req_2
   - 1200 charging_cycles_2 + [ ] = 0
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
 covered_area_y_t0_y_t0_inv_identity_3: [
   sensor_pix_y_inv_3 * covered_area_y_t0_3 ] <= 0.01
 covered_area_x_t0_identity_3: - 167.819926235456 drone_h_3
   + covered_area_x_t0_3 + [ ] = 0
 covered_distance_identity_3: covered_area_x_t0_3 + covered_distance_3 + [
   - covered_area_x_t0_3 * number_of_place_covered_3 ] = 0
 operation_time_identity_3: operation_time_3 + [
   - drone_v_true_inv_3 * covered_distance_3 ] = 0
 actuator_power_positive_3: 1.080807827885279 drone_v_3
   - 0.6589299118717831 drone_v_2_3 + 0.3072465322030534 drone_v_3_3
   + 0.00255271204255636 drone_h_3 + [ ] >= 0.00124191198834134
 sensor_power_positive_3: 0.9289169752078502 sensor_fps_3
   + 0.00562203952865926 sensor_fps_2_3 + 0.6202206846179508 sensor_pix_3
   + 0.0628410939088008 sensor_pix_2_3
   + 0.0407911874142603 sensor_fps_pix_3 + [ ] >= 0
 drone_pa_consumption_identity_3: - 86.46462623082229 drone_v_3
   + 52.71439294974265 drone_v_2_3 - 24.57972257624427 drone_v_3_3
   - 0.2042169634045086 drone_h_3 + drone_pa_consumption_3 + [ ]
   = -0.099352959067307
 drone_ps_consumption_identity_3: - 5.573501851247102 sensor_fps_3
   - 0.0337322371719556 sensor_fps_2_3 - 3.721324107707705 sensor_pix_3
   - 0.3770465634528045 sensor_pix_2_3
   - 0.2447471244855617 sensor_fps_pix_3 + drone_ps_consumption_3 + [ ]
   = 5
 drone_energy_consumption_identity_3: drone_energy_consumption_3 + [
   - operation_time_3 * drone_pa_consumption_3
   - operation_time_3 * drone_ps_consumption_3 ] = 0
 covered_area_true_identity_3: covered_area_true_3 + [
   - drone_used_3 * covered_area_total_3 ] = 0
 covered_area_total_limit_3: covered_area_total_3 + [ ] <= 1e+06
 qc104: - 5.5555555555555556e-04 operation_time_3 + charging_cycles_3 + [ ]
   = -1
 operation_time_req_identity_3: - operation_time_3 + operation_time_req_3
   - 1200 charging_cycles_3 + [ ] = 0
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
 covered_area_y_t0_y_t0_inv_identity_4: [
   sensor_pix_y_inv_4 * covered_area_y_t0_4 ] <= 0.01
 covered_area_x_t0_identity_4: - 167.819926235456 drone_h_4
   + covered_area_x_t0_4 + [ ] = 0
 covered_distance_identity_4: covered_area_x_t0_4 + covered_distance_4 + [
   - covered_area_x_t0_4 * number_of_place_covered_4 ] = 0
 operation_time_identity_4: operation_time_4 + [
   - drone_v_true_inv_4 * covered_distance_4 ] = 0
 actuator_power_positive_4: 1.080807827885279 drone_v_4
   - 0.6589299118717831 drone_v_2_4 + 0.3072465322030534 drone_v_3_4
   + 0.00255271204255636 drone_h_4 + [ ] >= 0.00124191198834134
 sensor_power_positive_4: 0.9289169752078502 sensor_fps_4
   + 0.00562203952865926 sensor_fps_2_4 + 0.6202206846179508 sensor_pix_4
   + 0.0628410939088008 sensor_pix_2_4
   + 0.0407911874142603 sensor_fps_pix_4 + [ ] >= 0
 drone_pa_consumption_identity_4: - 86.46462623082229 drone_v_4
   + 52.71439294974265 drone_v_2_4 - 24.57972257624427 drone_v_3_4
   - 0.2042169634045086 drone_h_4 + drone_pa_consumption_4 + [ ]
   = -0.099352959067307
 drone_ps_consumption_identity_4: - 5.573501851247102 sensor_fps_4
   - 0.0337322371719556 sensor_fps_2_4 - 3.721324107707705 sensor_pix_4
   - 0.3770465634528045 sensor_pix_2_4
   - 0.2447471244855617 sensor_fps_pix_4 + drone_ps_consumption_4 + [ ]
   = 5
 drone_energy_consumption_identity_4: drone_energy_consumption_4 + [
   - operation_time_4 * drone_pa_consumption_4
   - operation_time_4 * drone_ps_consumption_4 ] = 0
 covered_area_true_identity_4: covered_area_true_4 + [
   - drone_used_4 * covered_area_total_4 ] = 0
 covered_area_total_limit_4: covered_area_total_4 + [ ] <= 1e+06
 qc130: - 5.5555555555555556e-04 operation_time_4 + charging_cycles_4 + [ ]
   = -1
 operation_time_req_identity_4: - operation_time_4 + operation_time_req_4
   - 1200 charging_cycles_4 + [ ] = 0
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
 480 <= sensor_pix_y_0 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_0 <= 0.00208333333333333
 sensor_fps_pix_0 <= 1
 covered_area_x_t0_0 <= 167.819926235456
 covered_area_y_t0_0 <= 167.819926235456
 covered_area_y_t0_inv_0 >= 0.00595876796297105
 covered_area_total_t0_0 <= 28163.52764167389
 covered_area_total_t0_inv_0 >= 3.55069e-05
 covered_area_total_0 <= 1e+06
 number_of_place_covered_0 <= 2.8163527641673893e+10
 covered_distance_0 <= 4.7264011311881182e+12
 operation_time_0 >= 60
 drone_pa_consumption_0 <= 80
 5 <= drone_ps_consumption_0 <= 6
 drone_energy_consumption_0 <= 1e+07
 covered_area_true_0 <= 1e+06
 operation_time_req_0 >= 60
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
 480 <= sensor_pix_y_1 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_1 <= 0.00208333333333333
 sensor_fps_pix_1 <= 1
 covered_area_x_t0_1 <= 167.819926235456
 covered_area_y_t0_1 <= 167.819926235456
 covered_area_y_t0_inv_1 >= 0.00595876796297105
 covered_area_total_t0_1 <= 28163.52764167389
 covered_area_total_t0_inv_1 >= 3.55069e-05
 covered_area_total_1 <= 1e+06
 number_of_place_covered_1 <= 2.8163527641673893e+10
 covered_distance_1 <= 4.7264011311881182e+12
 operation_time_1 >= 60
 drone_pa_consumption_1 <= 80
 5 <= drone_ps_consumption_1 <= 6
 drone_energy_consumption_1 <= 1e+07
 covered_area_true_1 <= 1e+06
 operation_time_req_1 >= 60
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
 480 <= sensor_pix_y_2 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_2 <= 0.00208333333333333
 sensor_fps_pix_2 <= 1
 covered_area_x_t0_2 <= 167.819926235456
 covered_area_y_t0_2 <= 167.819926235456
 covered_area_y_t0_inv_2 >= 0.00595876796297105
 covered_area_total_t0_2 <= 28163.52764167389
 covered_area_total_t0_inv_2 >= 3.55069e-05
 covered_area_total_2 <= 1e+06
 number_of_place_covered_2 <= 2.8163527641673893e+10
 covered_distance_2 <= 4.7264011311881182e+12
 operation_time_2 >= 60
 drone_pa_consumption_2 <= 80
 5 <= drone_ps_consumption_2 <= 6
 drone_energy_consumption_2 <= 1e+07
 covered_area_true_2 <= 1e+06
 operation_time_req_2 >= 60
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
 480 <= sensor_pix_y_3 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_3 <= 0.00208333333333333
 sensor_fps_pix_3 <= 1
 covered_area_x_t0_3 <= 167.819926235456
 covered_area_y_t0_3 <= 167.819926235456
 covered_area_y_t0_inv_3 >= 0.00595876796297105
 covered_area_total_t0_3 <= 28163.52764167389
 covered_area_total_t0_inv_3 >= 3.55069e-05
 covered_area_total_3 <= 1e+06
 number_of_place_covered_3 <= 2.8163527641673893e+10
 covered_distance_3 <= 4.7264011311881182e+12
 operation_time_3 >= 60
 drone_pa_consumption_3 <= 80
 5 <= drone_ps_consumption_3 <= 6
 drone_energy_consumption_3 <= 1e+07
 covered_area_true_3 <= 1e+06
 operation_time_req_3 >= 60
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
 480 <= sensor_pix_y_4 <= 1400
 7.1428571428571429e-04 <= sensor_pix_y_inv_4 <= 0.00208333333333333
 sensor_fps_pix_4 <= 1
 covered_area_x_t0_4 <= 167.819926235456
 covered_area_y_t0_4 <= 167.819926235456
 covered_area_y_t0_inv_4 >= 0.00595876796297105
 covered_area_total_t0_4 <= 28163.52764167389
 covered_area_total_t0_inv_4 >= 3.55069e-05
 covered_area_total_4 <= 1e+06
 number_of_place_covered_4 <= 2.8163527641673893e+10
 covered_distance_4 <= 4.7264011311881182e+12
 operation_time_4 >= 60
 drone_pa_consumption_4 <= 80
 5 <= drone_ps_consumption_4 <= 6
 drone_energy_consumption_4 <= 1e+07
 covered_area_true_4 <= 1e+06
 operation_time_req_4 >= 60
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
 covered_area_y_t0_inv_identity_4: covered_area_y_t0_inv_4 =
   POW ( covered_area_y_t0_4 ^ -1 )
 covered_area_total_t0_inv_identity_4: covered_area_total_t0_inv_4 =
   POW ( covered_area_total_t0_4 ^ -1 )
End
