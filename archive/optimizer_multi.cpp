#include "optimizer.h"
#include "gurobi_c++.h"
#include "parameter.h"
#include <cmath>
#include <iostream>
#include <vector>


void Optimizer::optimize() { 

  if (model_parameter[1].read() == 0){
      return;
  }
  if (counter == observed_data[19].read()) {
      return; // No new data to process
  }else {
      counter = observed_data[19].read(); // Update counter to the latest data
  }

  //################################################################################## Constants ##################################################################################
  
  int num_drones = 2;//NUMBER_DRONE_MAX;
  // drone pixel set
  std::vector<double> drone_set_pix = DRONE_SET_PIX;
  std::vector<double> drone_set_pix_x = DRONE_SET_PIX_X;
  std::vector<double> drone_set_pix_y = DRONE_SET_PIX_Y;
  std::vector<double> drone_set_pix_normalized;
  for (int i = 0; i < drone_set_pix.size(); ++i) {
      drone_set_pix_normalized.push_back((drone_set_pix[i] - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN)); // Normalize pixel values
  }
  // fps set
  std::vector<double> drone_set_fps = DRONE_SET_FPS;
  std::vector<double> drone_set_fps_normalized;
  for (int i = 0; i < drone_set_fps.size(); ++i) {
      drone_set_fps_normalized.push_back((drone_set_fps[i] - FPS_MIN) / (FPS_MAX - FPS_MIN)); // Normalize fps values
  }

  // weather prediction
  double wp_v_wind = (weather_prediction[0].read()-WIND_SPEED_MIN)/WIND_SPEED_MAX;         
  double wp_theta_wind = weather_prediction[1].read();   

  // model_param for drone typr (for now only conside one type)
  double pa_eta = model_parameter[0].read();  
  double pa_delta = model_parameter[1].read(); 
  double pa_alpha = model_parameter[2].read(); 
  double pa_beta = model_parameter[3].read(); 

  double ps_a = model_parameter[4].read();
  double ps_b = model_parameter[5].read();
  double ps_c = model_parameter[6].read();
  double ps_d = model_parameter[7].read();
  double ps_e = model_parameter[8].read();
  double ps_f = model_parameter[9].read();

  // observed data (set of done mass. for now consider each drone will have the same payload mass)
  double drone_payload_mass = 0;

  // folding actuator equation
  double pa_phi = std::sin(wp_theta_wind);
  double pa_theta = 1/(pa_eta + 1e-8); // 1/eta
  double pa_C_0 = pa_theta * (((GRAVITY-(LINEAR_ACCELERATION_Z_MAX*(-1)))/((LINEAR_ACCELERATION_Z_MIN*(-1))-(LINEAR_ACCELERATION_Z_MAX*(-1)))) * ((DRONE_MASS + drone_payload_mass)/MASS_MAX)) * pa_phi + pa_theta*pa_alpha*std::pow(pa_phi,3) + pa_delta;
  double pa_C_1 = pa_theta * (((GRAVITY-(LINEAR_ACCELERATION_Z_MAX*(-1)))/((LINEAR_ACCELERATION_Z_MIN*(-1))-(LINEAR_ACCELERATION_Z_MAX*(-1)))) * ((DRONE_MASS + drone_payload_mass)/MASS_MAX)) + 3*pa_theta*pa_alpha*std::pow(pa_phi,2);
  double pa_C_2 = 3 * pa_theta * pa_alpha * pa_phi;
  double pa_C_3 = pa_theta * pa_alpha;
  double pa_C_4 = pa_theta * pa_beta * ((DRONE_MASS + drone_payload_mass)/MASS_MAX);


  ////################################################################################# Gurobi environment ################################################################################
   try{

    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_NonConvex, 2);
    GRBVar one = model.addVar(0.9, 1.0, 0.0, GRB_CONTINUOUS, "one");
    model.addQConstr(one == 1, "one_identity");

  //###############################################################################  drone dependent vatiable  #############################################################################
    // Store variables for each drone
    std::vector<GRBVar> 
    drone_v(num_drones), 
    drone_v_2(num_drones), 
    drone_v_3(num_drones), 
    drone_v_inv(num_drones), 
    drone_v_true(num_drones), 
    drone_v_true_inv(num_drones), 
    drone_h(num_drones), 
    drone_h_true(num_drones), 
    sensor_fps(num_drones), 
    sensor_fps_2(num_drones), 
    sensor_fps_true(num_drones), 
    sensor_pix(num_drones), 
    sensor_pix_2(num_drones), 
    sensor_pix_x(num_drones), 
    sensor_pix_x_inv(num_drones), 
    sensor_pix_y(num_drones), 
    sensor_pix_y_inv(num_drones), 
    sensor_fps_pix(num_drones),

    covered_area_x_t0(num_drones), 
    covered_area_y_t0(num_drones), 
    covered_area_y_t0_inv(num_drones), 
    covered_area_total_t0(num_drones), 
    covered_area_total_t0_inv(num_drones), 
    covered_area_total(num_drones),

    number_of_place_covered(num_drones), 
    distance_between_place(num_drones), 
    covered_area_total_inv(num_drones), 
    covered_distance(num_drones), 
    operation_time(num_drones),

    drone_energy_consumption(num_drones), // energy consumption for each drone
    drone_pa_consumption(num_drones), // actuator power consumption for each drone
    drone_ps_consumption(num_drones), // sensor power consumption for each drone

    covered_area_true(num_drones); // sum of total covered area for each drone

    // pixel x and y selector
    std::vector<GRBVar> sensor_pix_selector(num_drones * drone_set_pix.size());  // binary vars [0-2] dorne 1, [3-5] drone 2, [6-8] drone 3, etc.
    std::vector<GRBLinExpr> sensor_pix_selector_total;
    std::vector<GRBLinExpr> sensor_pix_x_value;
    std::vector<GRBLinExpr> sensor_pix_y_value;
    std::vector<GRBLinExpr> drone_set_pix_value;
    for (int i = 0; i < num_drones; ++i) {
        sensor_pix_selector_total.push_back(0);
        sensor_pix_x_value.push_back(0);
        sensor_pix_y_value.push_back(0);
        drone_set_pix_value.push_back(0);
    }
    // fps selector
    std::vector<GRBVar> drone_set_fps_selector(num_drones * drone_set_fps.size());  // binary vars, // [0-2] dorne 1, [3-5] drone 2, [6-8] drone 3, etc.
    std::vector<GRBLinExpr> drone_set_fps_selector_total; // binary vars for total fps selector, [0] for drone 1, [1] for drone 2, etc.
    std::vector<GRBLinExpr> drone_set_fps_value; // fps value for each drone
    for (int i = 0; i < num_drones; ++i) {
        drone_set_fps_selector_total.push_back(0);
        drone_set_fps_value.push_back(0);
    }
    
    // power and energy expressions
    std::vector<GRBQuadExpr> pa_exprs(num_drones); // actuator power expressions for each drone
    std::vector<GRBQuadExpr> ps_exprs(num_drones); // sensor power expressions for each drone

    std::vector<GRBVar> drone_used(num_drones); // binary variable to indicate if the drone is used
    GRBLinExpr drone_used_total = 0;
    for (int drone = 0; drone < num_drones; ++drone) {
        drone_used[drone] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_used_" + std::to_string(drone));
        drone_used_total += drone_used[drone];
    }
    model.addQConstr(drone_used_total <= num_drones, "drone_used_total_limit"); // limit the number of drones used to the number of drones available
    //model.addQConstr(drone_used_total == 2, "drone_used_total_limit"); // limit the number of drones used to the number of drones available

    for (int drone = 0; drone < num_drones; drone++) {
      // drone basic variables
      drone_v[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_v_" + std::to_string(drone));
      drone_v_2[drone] = model.addVar(0.0, 10000, 0.0, GRB_CONTINUOUS, "drone_v_2_" + std::to_string(drone));
      drone_v_3[drone] = model.addVar(0.0, 1000000, 0.0, GRB_CONTINUOUS, "drone_v_3_" + std::to_string(drone));
      drone_v_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "v_inv_" + std::to_string(drone));
      drone_v_true[drone] = model.addVar(0.0, 18.0, 0.0, GRB_CONTINUOUS, "drone_v_true_" + std::to_string(drone)); // true velocity of the drone
      drone_v_true_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "drone_v_true_inv_" + std::to_string(drone)); // inverse of true velocity of the drone
      drone_h[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_h_" + std::to_string(drone));
      drone_h_true[drone] = model.addVar(0.0, 100.0, 0.0, GRB_CONTINUOUS, "drone_h_true_" + std::to_string(drone)); // true height of the drone
      sensor_fps[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_" + std::to_string(drone));
      sensor_fps_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_2_" + std::to_string(drone));
      sensor_fps_true[drone] = model.addVar(15.0, 144.0, 0.0, GRB_CONTINUOUS, "sensor_fps_true_" + std::to_string(drone)); // true fps
      sensor_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_" + std::to_string(drone));
      sensor_pix_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_2_" + std::to_string(drone));
      sensor_pix_x[drone] = model.addVar(PIX_X_MIN, PIX_X_MAX, 0.0, GRB_CONTINUOUS, "sensor_pix_x_" + std::to_string(drone));
      sensor_pix_x_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sensor_pix_x_inv_" + std::to_string(drone));
      sensor_pix_y[drone] = model.addVar(PIX_Y_MIN, PIX_Y_MAX, 0.0, GRB_CONTINUOUS, "sensor_pix_y_" + std::to_string(drone));
      sensor_pix_y_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sensor_pix_y_inv_" + std::to_string(drone));
      sensor_fps_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_pix_" + std::to_string(drone));

      model.addGenConstrPow(drone_v[drone], drone_v_2[drone], 2.0, "drone_v_2_identity_" + std::to_string(drone));
      model.addGenConstrPow(drone_v[drone], drone_v_3[drone], 3.0, "drone_v_3_identity_" + std::to_string(drone));
      model.addGenConstrPow(drone_v[drone], drone_v_inv[drone], -1.0, "drone_v_inv_identity_" + std::to_string(drone));
      model.addQConstr(drone_v_true[drone] == drone_v[drone] * ( SPEED_MAX -  SPEED_MIN ) + SPEED_MIN, "drone_v_true_identity_" + std::to_string(drone)); // true velocity of the drone is equal to the normalized velocity multiplied by the maximum speed plus the minimum speed
      model.addGenConstrPow(drone_v_true[drone], drone_v_true_inv[drone], -1.0, "drone_v_true_inv_identity_" + std::to_string(drone));
      model.addQConstr(drone_h_true[drone] == drone_h[drone] * ( ALTITUDE_MAX - ALTITUDE_MIN ) + ALTITUDE_MIN, "drone_h_true_identity_" + std::to_string(drone)); // true height of the drone is equal to the normalized height multiplied by the maximum height plus the minimum heigh
      model.addQConstr(sensor_fps_pix[drone] == sensor_fps[drone] * sensor_pix[drone], "sensor_fps_pix_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_fps[drone], sensor_fps_2[drone], 2.0, "sensor_fps_2_identity_" + std::to_string(drone));
      model.addQConstr(sensor_fps_true[drone] == sensor_fps[drone] * ( FPS_MAX - FPS_MIN) + FPS_MIN, "sensor_fps_true_identity_" + std::to_string(drone)); // fps == sensor_fps
      model.addQConstr(sensor_fps_true[drone] >= drone_v[drone]*CONST_2_TAN_CAMERA_THETA_INV, "coverage_fps_lower_bound_" + std::to_string(drone)); // fps >= v * 2 * tan(camera_theta)^-1
      model.addGenConstrPow(sensor_pix[drone], sensor_pix_2[drone], 2.0, "sensor_pix_2_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_pix_x[drone], sensor_pix_x_inv[drone], -1.0, "sensor_pix_x_inv_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_pix_y[drone], sensor_pix_y_inv[drone], -1.0, "sensor_pix_y_inv_identity_" + std::to_string(drone));
      

      // pixel x and y selector
      for (int i = 0; i < drone_set_pix.size(); ++i) { // binary variable to choose only one cfg
          sensor_pix_selector[drone * drone_set_pix.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "sensor_pix_selector_" + std::to_string(drone) + "_" + std::to_string(i));
      }
      for (int i = 0; i < drone_set_pix.size(); ++i) {
          sensor_pix_selector_total[drone] += sensor_pix_selector[drone * drone_set_pix.size() + i];
      }
      for (int i = 0; i < drone_set_pix_x.size(); ++i) {
          sensor_pix_x_value[drone] += drone_set_pix_x[i] * sensor_pix_selector[drone * drone_set_pix.size() + i]; 
          sensor_pix_y_value[drone] += drone_set_pix_y[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          drone_set_pix_value[drone] += drone_set_pix_normalized[i] * sensor_pix_selector[drone * drone_set_pix.size() + i]; 
      }
      model.addQConstr(sensor_pix_selector_total[drone] == 1, "sensor_pix_xy_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_pix_x[drone] == sensor_pix_x_value[drone], "sensor_pix_x_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix_y[drone] == sensor_pix_y_value[drone], "sensor_pix_y_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix[drone] == drone_set_pix_value[drone], "drone_pix_value_restriction_" + std::to_string(drone));
      // fps selector
      for (int i = 0; i < drone_set_fps.size(); ++i) {
        drone_set_fps_selector[drone * drone_set_fps.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_set_fps_selector_" + std::to_string(drone) + "_" + std::to_string(i));
      }
      for (int i = 0; i < drone_set_fps.size(); ++i) {
        drone_set_fps_selector_total[drone] += drone_set_fps_selector[drone * drone_set_fps.size() + i];
      }
      for (int i = 0; i < drone_set_fps.size(); ++i) {
        drone_set_fps_value[drone] += drone_set_fps_normalized[i] * drone_set_fps_selector[drone * drone_set_fps.size() + i];
      }
      model.addQConstr(drone_set_fps_selector_total[drone] == 1, "drone_set_fps_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_fps[drone] == drone_set_fps_value[drone], "drone_set_fps_value_restriction_" + std::to_string(drone));

      // covered area variables
      covered_area_x_t0[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_x_t0_" + std::to_string(drone));
      covered_area_y_t0[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_y_t0_" + std::to_string(drone));
      covered_area_y_t0_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_y_t0_inv_" + std::to_string(drone));
      covered_area_total_t0[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_total_t0_" + std::to_string(drone));
      covered_area_total_t0_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_total_t0_inv_" + std::to_string(drone));
      covered_area_total[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_total_" + std::to_string(drone));
      model.addGenConstrPow(covered_area_y_t0[drone], covered_area_y_t0_inv[drone], -1.0, "covered_area_y_t0_inv_identity_" + std::to_string(drone)); // inverse of area covered by the drone in y direction
      model.addGenConstrPow(covered_area_total_t0[drone], covered_area_total_t0_inv[drone], -1.0, "covered_area_total_t0_inv_identity_" + std::to_string(drone)); // inverse of total area covered by the drone at t0      
      model.addQConstr(covered_area_total_t0[drone] == covered_area_x_t0[drone] * covered_area_y_t0[drone], "covered_area_total_t0_identity_" + std::to_string(drone)); // total area covered by the drone at t0
      model.addQConstr(covered_area_x_t0[drone]* covered_area_y_t0_inv[drone] == sensor_pix_x[drone] * sensor_pix_y_inv[drone], "covered_area_x_t0_y_t0_identity_" + std::to_string(drone)); // area covered in x direction divided by area covered in y direction is equal to pixel x resolution divided by pixel y resolution
          
      number_of_place_covered[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "number_of_place_covered_" + std::to_string(drone)); // number of place covered by the drone
      distance_between_place[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "distance_between_place_" + std::to_string(drone)); // distance between place covered by the drone
      covered_area_total_inv[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_area_total_inv_" + std::to_string(drone)); // inverse of total area covered by the drone
      covered_distance[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "covered_distance_" + std::to_string(drone)); // distance covered by the drone
      operation_time[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "operation_time_" + std::to_string(drone)); // operation time of the drone   
      
      // what if the covered_area_total is 0, what happened to covered_area_total_inv
      model.addQConstr(number_of_place_covered[drone] == covered_area_total[drone] * covered_area_total_t0_inv[drone], "number_of_place_covered_identity_" + std::to_string(drone)); // number of place covered by the drone is equal to total area covered by the drone multiplied by inverse of total area covered by the drone
      model.addQConstr(distance_between_place[drone] == covered_area_x_t0[drone], "distance_between_place_identity_" + std::to_string(drone)); // distance between place covered by the drone is equal to area covered in x direction
      model.addGenConstrPow(covered_area_total[drone], covered_area_total_inv[drone], -1.0, "covered_area_total_inv_identity_" + std::to_string(drone)); // inverse of total area covered by the drone

      model.addQConstr(covered_area_y_t0[drone] * sensor_pix_y_inv[drone] <= 0.01, "covered_area_y_t0_y_t0_inv_identity_" + std::to_string(drone)); // area covered in y direction multiplied by inverse of area covered in y direction is less than or equal to 0.01>
      model.addQConstr(covered_area_x_t0[drone] == CONST_2_TAN_CAMERA_THETA * drone_h_true[drone], "covered_area_x_t0_identity_" + std::to_string(drone)); // area covered in x direction is equal to 2 * tan(camera_theta) * (drone_h * altitude_max + altitude_min)
      
      model.addQConstr(covered_distance[drone] == (distance_between_place[drone] * number_of_place_covered[drone]) - distance_between_place[drone], "covered_distance_identity_" + std::to_string(drone)); // distance covered by the drone is equal to distance between place covered by the drone multiplied by number of place covered by the drone minus distance between place covered by the drone
      model.addQConstr(operation_time[drone] == covered_distance[drone] * drone_v_true_inv[drone], "operation_time_identity_" + std::to_string(drone)); // operation time of the drone is equal to distance covered by the drone multiplied by inverse of true velocity of the drone
      // actuator power expressions
      pa_exprs[drone] = GRBQuadExpr();
      pa_exprs[drone] += pa_C_0;
      pa_exprs[drone] += pa_C_1 * drone_v[drone];
      pa_exprs[drone] += pa_C_2 * drone_v_2[drone];
      pa_exprs[drone] += pa_C_3 * drone_v_3[drone];
      pa_exprs[drone] += pa_C_4 * drone_h[drone];
      // sensor power expressions
      ps_exprs[drone] = GRBQuadExpr();
      ps_exprs[drone] += ps_a*sensor_fps[drone];
      ps_exprs[drone] += ps_b*sensor_pix[drone];
      ps_exprs[drone] += ps_c*sensor_fps[drone]*sensor_pix[drone];
      ps_exprs[drone] += ps_d*sensor_fps_2[drone];
      ps_exprs[drone] += ps_e*sensor_pix_2[drone];

      // power true
      drone_pa_consumption[drone] = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "drone_pa_consumption_" + std::to_string(drone)); // actuator power consumption for each drone
      drone_ps_consumption[drone] = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "drone_ps_consumption_" + std::to_string(drone)); // sensor power consumption for each drone
      drone_energy_consumption[drone] = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "drone_energy_consumption_" + std::to_string(drone)); // energy consumption for each drone
      model.addQConstr(drone_pa_consumption[drone] == pa_exprs[drone] * POWER_ACTUATOR_MAX + POWER_ACTUATOR_MIN, "drone_pa_consumption_identity_" + std::to_string(drone)); // actuator power consumption is equal to the actuator power expression multiplied by the maximum actuator power plus the minimum actuator power
      model.addQConstr(drone_ps_consumption[drone] == ps_exprs[drone] * POWER_SENSOR_MAX + POWER_SENSOR_MIN, "drone_ps_consumption_identity_" + std::to_string(drone)); // sensor power consumption is equal to the sensor power expression multiplied by the maximum sensor power plus the minimum sensor power
      model.addQConstr(drone_energy_consumption[drone] == operation_time[drone] * (drone_pa_consumption[drone] + drone_ps_consumption[drone]), "drone_energy_consumption_identity_" + std::to_string(drone)); // energy consumption is equal to power multiplied by operation time
      
      covered_area_true[drone] = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "covered_area_true_" + std::to_string(drone)); // total area covered by the drone
      model.addQConstr(covered_area_true[drone] == covered_area_total[drone] * drone_used[drone], "covered_area_true_identity_" + std::to_string(drone)); // total area covered by the drone is equal to total area covered by the drone multiplied by binary variable indicating if the drone is used
      model.addQConstr(covered_area_true[drone] <= FIELD_AREA, "covered_area_total_limit_" + std::to_string(drone)); // total area covered by the drone is less than or equal to field area
      model.addQConstr(covered_area_true[drone] >= 0.0, "covered_area_total_positive_" + std::to_string(drone)); // total area covered by the drone must be positive

    
      //add power and energy constraints
      model.addQConstr(pa_exprs[drone] >= 0.0, "actuator_power_positive_" + std::to_string(drone)); // actuator power must be positive
      model.addQConstr(ps_exprs[drone] >= 0.0, "sensor_power_positive_" + std::to_string(drone)); // sensor power must be positive


    }
    // sum of covered_area_total = FIELD_AREA
    GRBLinExpr covered_area_total_sum = 0.0; 
    for (int drone = 0; drone < num_drones; drone++) {
      covered_area_total_sum += covered_area_true[drone];
    }
    model.addQConstr(covered_area_total_sum == FIELD_AREA, "covered_area_total_sum_constraint");
    
    // sum of actuator power expressions
    GRBQuadExpr pa_exprs_sum = GRBQuadExpr();
    for (int drone = 0; drone < num_drones; drone++) {
      pa_exprs_sum += pa_exprs[drone];
    }

    // sum of sensor power expressions
    GRBQuadExpr ps_exprs_sum = GRBQuadExpr();
    for (int drone = 0; drone < num_drones; drone++) {
      ps_exprs_sum += ps_exprs[drone];
    }

    // sum of power expressions
    GRBQuadExpr p_exprs_sum = GRBQuadExpr();
    for (int drone = 0; drone < num_drones; drone++) {
      p_exprs_sum += drone_pa_consumption[drone] + drone_ps_consumption[drone];
    }

    // sum of energy expressions
    GRBQuadExpr E_exprs_sum = GRBQuadExpr();
    for (int drone = 0; drone < num_drones; drone++) {
      E_exprs_sum += drone_energy_consumption[drone];
    }

    // multiobjectieve optimization
    GRBQuadExpr objective_expr = GRBQuadExpr();
    objective_expr = E_exprs_sum + 10*drone_used_total;

    // objective optimization
    model.setObjective(objective_expr , GRB_MINIMIZE);//GRB_MAXIMIZE);//
    model.optimize();


    int model_status = model.get(GRB_IntAttr_Status);
    if (model_status == GRB_OPTIMAL) {
      std::cout << "wind speed: " << weather_prediction[0].read() << std::endl;
      std::cout << "wind angle: " << weather_prediction[1].read() << std::endl;
      double total_pa = 0.0, total_ps = 0.0, total_power = 0.0, total_energy = 0.0, total_covered_area = 0.0;
      for (int drone = 0; drone < num_drones; drone++) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Drone " << drone << " used: " << drone_used_total.getValue() << std::endl;
        std::cout << "Drone " << drone << " results:" << std::endl;
        std::cout << "  is used: " << drone_used[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal v: " << drone_v[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal v_true: " << drone_v_true[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal h: " << drone_h[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal h_true: " << drone_h_true[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  CONST_2_TAN_CAMERA_THETA*h: " << CONST_2_TAN_CAMERA_THETA * (drone_h[drone].get(GRB_DoubleAttr_X)*( ALTITUDE_MAX - ALTITUDE_MIN) + ALTITUDE_MIN) << std::endl;
        std::cout << "  CONST_2_TAN_CAMERA_THETA: " << CONST_2_TAN_CAMERA_THETA << std::endl;
        std::cout << "  Optimal fps: " << sensor_fps[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  sensor_fps_true: " << sensor_fps_true[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal pix: " << sensor_pix[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal sensor pix x: " << sensor_pix_x[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Optimal sensor pix y: " << sensor_pix_y[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_x_t0: " << covered_area_x_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_y_t0: " << covered_area_y_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total_t0: " << covered_area_total_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total: " << covered_area_total[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  number_of_place_covered: " << number_of_place_covered[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  distance_between_place: " << distance_between_place[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_distance: " << covered_distance[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  operation_time: " << operation_time[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Actuator Power: " << drone_pa_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Sensor Power: " << drone_ps_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Power: " << drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  Energy: " << drone_energy_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_y_t0_inv: " << covered_area_y_t0_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total_t0_inv: " << covered_area_total_t0_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total_inv: " << covered_area_total_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_true: " << covered_area_true[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total_inv: " << covered_area_total_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_y_t0_inv: " << covered_area_y_t0_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  covered_area_total_t0_inv: " << covered_area_total_t0_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  sensor_pix_x_inv: " << sensor_pix_x_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "  sensor_pix_y_inv: " << sensor_pix_y_inv[drone].get(GRB_DoubleAttr_X) << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        total_pa += drone_pa_consumption[drone].get(GRB_DoubleAttr_X);
        total_ps += drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
        total_power += drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
        total_energy += drone_energy_consumption[drone].get(GRB_DoubleAttr_X);
        total_covered_area += covered_area_true[drone].get(GRB_DoubleAttr_X);
      }
    std::cout << "Total Actuator Power: " << total_pa << std::endl;
    std::cout << "Total Sensor Power: " << total_ps << std::endl;
    std::cout << "Total Power: " << total_power << std::endl;
    std::cout << "Total Energy: " << total_energy << std::endl;
    std::cout << "Total covered area: " << total_covered_area << std::endl;
    std::cout << "Optimization was successful." << std::endl;
    } else {
      std::cout << "Optimization was not successful." << std::endl;
      std::cout << "Model status: " << model_status << std::endl;
      if (model_status == GRB_INFEASIBLE) {
        std::cout << "Model is infeasible. Please check constraints and input data." << std::endl;
      } else if (model_status == GRB_UNBOUNDED) {
        std::cout << "Model is unbounded. Please check constraints and input data." << std::endl;
      }
    }
  } catch (GRBException &e) {
    std::cout << "Gurobi exception caught in fifth block: " << e.getMessage() << " code: " << e.getErrorCode() << std::endl;
    return;
  } catch (std::exception &e) {
    std::cout << "Standard exception caught in fifth block: " << e.what() << std::endl;
    return;
  } catch (...) {
    std::cout << "Unknown exception caught in fifth block." << std::endl;
    return;
  }
  return;
} 

//sc_in<double> model_parameter[10];  [eta, delta, alpha, beta][sigma, omega, epsilon]
//sc_in<double> constraints_value[10];
//sc_in<double> weather_prediction[10];
//sc_out<double> cfg[10];
//sc_out<double> power_consumption[10];
//sc_out<double> operation_time[10];
//sc_core::sc_vector<sc_in<double>> observed_data; //drone mass, payload mass, altitute, wind speed, wind angle, speed of drone,power actuator, umber of pixel, fps, power sensor