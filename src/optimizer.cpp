#include "optimizer.h"
#include "gurobi_c++.h"
#include "parameter.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>

int success = 0;

void Optimizer::optimize() { 
  if (!OPTIMIZER_ON)return;

  if (model_parameter[1].read() == 0){
      return;
  }
  if (counter == observed_data[19].read()) {
      return; // No new data to process
  }else {
      counter = observed_data[19].read(); // Update counter to the latest data
  }
  try{

  // debugging
  //#define DEBUG
  #ifdef DEBUG
  if (success > 42) return;
  std::cout << "########################################################################################################################################################################" << std::endl;
  std::cout << "Optimizer started with counter: " << counter << std::endl;
  std::cout << "Weather prediction: " << weather_prediction[0].read() << ", " << weather_prediction[1].read() << std::endl;
  std::cout << "Model parameters: ";
  std::cout << std::setprecision(32); // Force full precision
  for (int i = 0; i < 7; ++i) {
      std::cout << model_parameter[i].read() << ", ";
  }
  std::cout << std::setprecision(6); // Reset to default precision if needed
  std::cout << std::endl;
  std::cout << "########################################################################################################################################################################" << std::endl;
  #endif // DEBUG
  // end debugging

  //####################################################################################### Constants ###################################################################################
  
    int num_drones = 5;//NUMBER_DRONE_MAX;
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
    
    #define FIX 

    #ifndef FIX
    double pa_eta = model_parameter[0].read();  
    double pa_delta = model_parameter[1].read(); 
    double pa_alpha = model_parameter[2].read(); 
    double pa_beta = model_parameter[3].read(); 

    double ps_a = model_parameter[4].read();
    double ps_b = model_parameter[5].read();
    double ps_c = model_parameter[6].read();
    #else
    double fix_model_parameter[] = {0.31862743382724822982510204383289, 0.2127602615282839781940538159688, 0.097328390658043009708855208828027, 0.0021011339810415512012464755997598, 1.1463925428785151083843629749026, 10.212336874294889454972690145951, -1.88007013107074638647020492499}; 
    double pa_eta = fix_model_parameter[0]; //model_parameter[0].read();  
    double pa_delta = fix_model_parameter[1]; //model_parameter[1].read(); 
    double pa_alpha = fix_model_parameter[2]; //model_parameter[2].read(); 
    double pa_beta = fix_model_parameter[3]; //model_parameter[3].read(); 

    double ps_a = fix_model_parameter[4];//model_parameter[4].read();
    double ps_b = fix_model_parameter[5];//model_parameter[5].read();
    double ps_c = fix_model_parameter[6]; //model_parameter[6].read();
    #endif

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


  //################################################################################# Gurobi environment ################################################################################
  

    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_NonConvex, 2);
    GRBVar one = model.addVar(0.9, 1.0, 0.0, GRB_CONTINUOUS, "one");
    model.addQConstr(one == 1, "one_identity");

  //###############################################################################  drone dependent variable  #############################################################################
    // Store variables for each drone
    std::vector<GRBVar> 
    drone_v(num_drones), 
    drone_v_2(num_drones), 
    drone_v_3(num_drones), 
    drone_v_true(num_drones), 
    drone_v_true_inv(num_drones), 
    drone_h(num_drones), 
    //drone_h_true(num_drones), 
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

    operation_time_req(num_drones),  // total oeperation time including charging time
    charging_cycles(num_drones), // number of charging cycles for each drone

    covered_area_x_t0(num_drones), 
    covered_area_y_t0(num_drones), 
    covered_area_y_t0_inv(num_drones), 
    covered_area_total_t0(num_drones), 
    covered_area_total_t0_inv(num_drones), 
    covered_area_total(num_drones),

    number_of_place_covered(num_drones), 
    covered_distance(num_drones), 
    operation_time(num_drones),

    drone_energy_consumption(num_drones), // energy consumption for each drone
    drone_pa_consumption(num_drones), // actuator power consumption for each drone
    drone_ps_consumption(num_drones), // sensor power consumption for each drone

    covered_area_true(num_drones);

    // pixel x and y selector (SOS1 instead of binaries)
    std::vector<GRBVar> sensor_pix_selector(num_drones * drone_set_pix.size());
    std::vector<GRBLinExpr> sensor_pix_selector_total(num_drones);
    std::vector<GRBLinExpr> sensor_pix_x_value(num_drones);
    std::vector<GRBLinExpr> sensor_pix_y_value(num_drones);
    std::vector<GRBLinExpr> drone_set_pix_value(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        sensor_pix_selector_total[i] = 0;
        sensor_pix_x_value[i] = 0;
        sensor_pix_y_value[i] = 0;
        drone_set_pix_value[i] = 0;
    }
    // fps selector (SOS1 instead of binaries)
    std::vector<GRBVar> drone_set_fps_selector(num_drones * drone_set_fps.size());
    std::vector<GRBLinExpr> drone_set_fps_selector_total(num_drones);
    std::vector<GRBLinExpr> drone_set_fps_value(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        drone_set_fps_selector_total[i] = 0;
        drone_set_fps_value[i] = 0;
    }
    
    // power and energy expressions
    std::vector<GRBQuadExpr> pa_exprs(num_drones); // actuator power expressions for each drone
    std::vector<GRBQuadExpr> ps_exprs(num_drones); // sensor power expressions for each drone

    std::vector<GRBVar> drone_is_used(num_drones); // binary variable to indicate if the drone is used
    GRBLinExpr drone_used_total = 0;
    for (int drone = 0; drone < num_drones; ++drone) {
        drone_is_used[drone] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_used_" + std::to_string(drone));
        drone_used_total += drone_is_used[drone];
    }
    model.addQConstr(drone_used_total <= num_drones, "drone_used_total_limit"); // limit the number of drones used to the number of drones available
    //model.addQConstr(drone_used_total == 1, "drone_used_total_limit"); // limit the number of drones used to the number of drones available

    for (int drone = 0; drone < num_drones; drone++) {
      // drone basic variables
      drone_v[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_v_" + std::to_string(drone));
      drone_v_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_v_2_" + std::to_string(drone));
      drone_v_3[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_v_3_" + std::to_string(drone));
      drone_v_true[drone] = model.addVar(4.0, SPEED_MAX, 0.0, GRB_SEMICONT, "drone_v_true_" + std::to_string(drone)); // true velocity of the drone
      drone_v_true_inv[drone] = model.addVar(1/SPEED_MAX, 1/SPEED_MIN, 0.0, GRB_SEMICONT, "drone_v_true_inv_" + std::to_string(drone)); // inverse of true velocity of the drone
      drone_h[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_h_" + std::to_string(drone));
      //[drone] = model.addVar(ALTITUDE_MIN, ALTITUDE_MAX, 0.0, GRB_CONTINUOUS, "drone_h_true_" + std::to_string(drone)); // true height of the drone
      sensor_fps[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_" + std::to_string(drone));
      sensor_fps_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_2_" + std::to_string(drone));
      sensor_fps_true[drone] = model.addVar(FPS_MIN, FPS_MAX, 0.0, GRB_INTEGER, "sensor_fps_true_" + std::to_string(drone)); // true fps
      sensor_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_" + std::to_string(drone));
      sensor_pix_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_2_" + std::to_string(drone));
      sensor_pix_x[drone] = model.addVar(PIX_X_MIN, PIX_X_MAX, 0.0, GRB_INTEGER, "sensor_pix_x_" + std::to_string(drone));
      sensor_pix_x_inv[drone] = model.addVar(1/PIX_X_MAX, 1/PIX_X_MIN, 0.0, GRB_CONTINUOUS, "sensor_pix_x_inv_" + std::to_string(drone));
      sensor_pix_y[drone] = model.addVar(PIX_Y_MIN, PIX_Y_MAX, 0.0, GRB_INTEGER, "sensor_pix_y_" + std::to_string(drone));
      sensor_pix_y_inv[drone] = model.addVar(1/PIX_Y_MAX, 1/PIX_Y_MIN, 0.0, GRB_CONTINUOUS, "sensor_pix_y_inv_" + std::to_string(drone));
      sensor_fps_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_pix_" + std::to_string(drone));

      model.addGenConstrPow(drone_v[drone], drone_v_2[drone], 2.0, "drone_v_2_identity_" + std::to_string(drone));
      model.addGenConstrPow(drone_v[drone], drone_v_3[drone], 3.0, "drone_v_3_identity_" + std::to_string(drone));
      model.addQConstr(drone_v_true[drone] == drone_v[drone] * ( SPEED_MAX -  SPEED_MIN ) + SPEED_MIN, "drone_v_true_identity_" + std::to_string(drone)); // true velocity of the drone is equal to the normalized velocity multiplied by the maximum speed plus the minimum speed
      model.addGenConstrPow(drone_v_true[drone], drone_v_true_inv[drone], -1.0, "drone_v_true_inv_identity_" + std::to_string(drone));
      model.addQConstr(sensor_fps_pix[drone] == sensor_fps[drone] * sensor_pix[drone], "sensor_fps_pix_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_fps[drone], sensor_fps_2[drone], 2.0, "sensor_fps_2_identity_" + std::to_string(drone));
      model.addQConstr(sensor_fps_true[drone] == sensor_fps[drone] * ( FPS_MAX - FPS_MIN) + FPS_MIN, "sensor_fps_true_identity_" + std::to_string(drone)); // fps == sensor_fps
      model.addQConstr(sensor_fps_true[drone] >= drone_v[drone]*CONST_2_TAN_CAMERA_THETA_INV, "coverage_fps_lower_bound_" + std::to_string(drone)); // fps >= v * 2 * tan(camera_theta)^-1
      model.addGenConstrPow(sensor_pix[drone], sensor_pix_2[drone], 2.0, "sensor_pix_2_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_pix_x[drone], sensor_pix_x_inv[drone], -1.0, "sensor_pix_x_inv_identity_" + std::to_string(drone));
      model.addGenConstrPow(sensor_pix_y[drone], sensor_pix_y_inv[drone], -1.0, "sensor_pix_y_inv_identity_" + std::to_string(drone));

      // pixel x and y selector 
      for (int i = 0; i < drone_set_pix.size(); i++) {
          //int i = feasible_pix_indices[idx];
          sensor_pix_selector[drone * drone_set_pix.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "sensor_pix_selector_" + std::to_string(drone) + "_" + std::to_string(i));
          sensor_pix_selector_total[drone] += sensor_pix_selector[drone * drone_set_pix.size() + i];
          sensor_pix_x_value[drone] += drone_set_pix_x[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          sensor_pix_y_value[drone] += drone_set_pix_y[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          drone_set_pix_value[drone] += drone_set_pix_normalized[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
      }
      model.addQConstr(sensor_pix_selector_total[drone] == 1, "sensor_pix_xy_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_pix_x[drone] == sensor_pix_x_value[drone], "sensor_pix_x_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix_y[drone] == sensor_pix_y_value[drone], "sensor_pix_y_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix[drone] == drone_set_pix_value[drone], "drone_pix_value_restriction_" + std::to_string(drone));
      // fps selector (SOS1)
      for (int i = 0; i < drone_set_fps.size();i++) {
          //int i = feasible_fps_indices[idx];
          drone_set_fps_selector[drone * drone_set_fps.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_set_fps_selector_" + std::to_string(drone) + "_" + std::to_string(i));
          drone_set_fps_selector_total[drone] += drone_set_fps_selector[drone * drone_set_fps.size() + i];
          drone_set_fps_value[drone] += drone_set_fps_normalized[i] * drone_set_fps_selector[drone * drone_set_fps.size() + i];
      }
      model.addQConstr(drone_set_fps_selector_total[drone] == 1, "drone_set_fps_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_fps[drone] == drone_set_fps_value[drone], "drone_set_fps_value_restriction_" + std::to_string(drone));

      // covered area variables
      covered_area_x_t0[drone] = model.addVar(COVERED_AREA_X_MIN, COVERED_AREA_X_MAX, 0.0, GRB_SEMICONT, "covered_area_x_t0_" + std::to_string(drone));
      covered_area_y_t0[drone] = model.addVar(0.0, COVERED_AREA_X_MAX, 0.0, GRB_CONTINUOUS, "covered_area_y_t0_" + std::to_string(drone));
      covered_area_y_t0_inv[drone] = model.addVar(1/COVERED_AREA_X_MAX, 1/COVERED_AREA_X_MIN, 0.0, GRB_SEMICONT, "covered_area_y_t0_inv_" + std::to_string(drone));
      covered_area_total_t0[drone] = model.addVar(COVERED_AREA_TOTAL_MIN, COVERED_AREA_TOTAL_MAX, 0.0, GRB_SEMICONT, "covered_area_total_t0_" + std::to_string(drone));
      covered_area_total_t0_inv[drone] = model.addVar(1/COVERED_AREA_TOTAL_MAX, 1/COVERED_AREA_TOTAL_MIN, 0.0, GRB_SEMICONT, "covered_area_total_t0_inv_" + std::to_string(drone));
      covered_area_total[drone] = model.addVar(COVERED_AREA_TOTAL_MIN, FIELD_AREA, 0.0, GRB_SEMICONT, "covered_area_total_" + std::to_string(drone));
      model.addGenConstrPow(covered_area_y_t0[drone], covered_area_y_t0_inv[drone], -1.0, "covered_area_y_t0_inv_identity_" + std::to_string(drone)); // inverse of area covered by the drone in y direction
      model.addGenConstrPow(covered_area_total_t0[drone], covered_area_total_t0_inv[drone], -1.0, "covered_area_total_t0_inv_identity_" + std::to_string(drone)); // inverse of total area covered by the drone at t0      
      model.addQConstr(covered_area_total_t0[drone] == covered_area_x_t0[drone] * covered_area_y_t0[drone], "covered_area_total_t0_identity_" + std::to_string(drone)); // total area covered by the drone at t0
      model.addQConstr(covered_area_x_t0[drone]* covered_area_y_t0_inv[drone] == sensor_pix_x[drone] * sensor_pix_y_inv[drone], "covered_area_x_t0_y_t0_identity_" + std::to_string(drone)); // area covered in x direction divided by area covered in y direction is equal to pixel x resolution divided by pixel y resolution
          
      number_of_place_covered[drone] = model.addVar(0.0, NUMBER_PLACE_COVERED_MAX, 0.0, GRB_SEMICONT, "number_of_place_covered_" + std::to_string(drone)); // number of place covered by the drone
      covered_distance[drone] = model.addVar(COVERED_DISTANCE_MIN, COVERED_DISTANCE_MAX, 0.0, GRB_SEMICONT, "covered_distance_" + std::to_string(drone)); // distance covered by the drone
      operation_time[drone] = model.addVar(OPERATION_TIME_MIN, OPERATION_TIME_MAX, 0.0, GRB_SEMICONT, "operation_time_" + std::to_string(drone)); // operation time of the drone   
      
      model.addQConstr(number_of_place_covered[drone] == covered_area_total[drone] * covered_area_total_t0_inv[drone], "number_of_place_covered_identity_" + std::to_string(drone)); // number of place covered by the drone is equal to total area covered by the drone multiplied by inverse of total area covered by the drone
  //    model.addQConstr(covered_area_y_t0[drone] * sensor_pix_y_inv[drone] <= 0.01, "resolution__identity_" + std::to_string(drone)); // area covered in y direction multiplied by inverse of area covered in y direction is less than or equal to 0.01>
      //model.addQConstr(covered_area_x_t0[drone] == CONST_2_TAN_CAMERA_THETA * drone_h_true[drone], "covered_area_x_t0_identity_" + std::to_string(drone)); // area covered in x direction is equal to 2 * tan(camera_theta) * (drone_h * altitude_max + altitude_min)
      model.addQConstr(covered_area_x_t0[drone] == CONST_2_TAN_CAMERA_THETA * ((drone_h[drone] * ( ALTITUDE_MAX - ALTITUDE_MIN )) + ALTITUDE_MIN), "covered_area_x_t0_identity_" + std::to_string(drone)); // area covered in x direction is equal to 2 * tan(camera_theta) * (drone_h * altitude_max + altitude_min)
      model.addQConstr(covered_distance[drone] == (covered_area_x_t0[drone] * number_of_place_covered[drone]) - covered_area_x_t0[drone], "covered_distance_identity_" + std::to_string(drone)); // distance covered by the drone is equal to distance between place covered by the drone multiplied by number of place covered by the drone minus distance between place covered by the drone
      model.addQConstr(operation_time[drone] == covered_distance[drone] * drone_v_true_inv[drone], "operation_time_identity_" + std::to_string(drone)); // operation time of the drone is equal to distance covered by the drone multiplied by inverse of true velocity of the drone
      
      // actuator power expressions
      pa_exprs[drone] = GRBQuadExpr();
      pa_exprs[drone] += pa_C_0;
      pa_exprs[drone] += pa_C_1 * drone_v[drone];
      pa_exprs[drone] += pa_C_2 * drone_v_2[drone];
      pa_exprs[drone] += pa_C_3 * drone_v_3[drone];
      pa_exprs[drone] += pa_C_4 * drone_h[drone];
      model.addQConstr(pa_exprs[drone] >= 0.0, "actuator_power_positive_" + std::to_string(drone)); // actuator power must be positive

      // sensor power expressions
      
      ps_exprs[drone] = GRBQuadExpr();
      ps_exprs[drone] += ps_a*sensor_fps[drone];
      ps_exprs[drone] += ps_b*sensor_pix[drone];
      ps_exprs[drone] += ps_c;
      
      //*sensor_fps_pix[drone];
      //ps_exprs[drone] += ps_d*sensor_fps_2[drone];
      //ps_exprs[drone] += ps_e*sensor_pix_2[drone];
      //model.addQConstr(ps_exprs[drone] >= 0.0, "sensor_power_positive_" + std::to_string(drone)); // sensor power must be positive

      // power true
      drone_pa_consumption[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_SEMICONT, "drone_pa_consumption_" + std::to_string(drone)); // actuator power consumption for each drone
      drone_ps_consumption[drone] = model.addVar(0.0, 10.0, 0.0, GRB_SEMICONT, "drone_ps_consumption_" + std::to_string(drone)); // sensor power consumption for each drone
      drone_energy_consumption[drone] = model.addVar(0.0, num_drones*(POWER_ACTUATOR_MAX + 200)*OPERATION_TIME_MAX , 0.0, GRB_SEMICONT, "drone_energy_consumption_" + std::to_string(drone)); // energy consumption for each drone
      model.addQConstr(drone_pa_consumption[drone] == (pa_exprs[drone] * POWER_ACTUATOR_MAX) + POWER_ACTUATOR_MIN, "drone_pa_consumption_identity_" + std::to_string(drone)); // actuator power consumption is equal to the actuator power expression multiplied by the maximum actuator power plus the minimum actuator power
      model.addQConstr(drone_ps_consumption[drone] == (ps_exprs[drone] * POWER_SENSOR_MAX) + POWER_SENSOR_MIN, "drone_ps_consumption_identity_" + std::to_string(drone)); // sensor power consumption is equal to the sensor power expression multiplied by the maximum sensor power plus the minimum sensor power
      model.addQConstr(drone_energy_consumption[drone] == operation_time[drone] * (drone_pa_consumption[drone] + drone_ps_consumption[drone]), "drone_energy_consumption_identity_" + std::to_string(drone)); // energy consumption is equal to power multiplied by operation time
      //model.addQConstr(drone_energy_consumption[drone] == operation_time[drone] * drone_pa_consumption[drone], "drone_energy_consumption_identity_" + std::to_string(drone)); // energy consumption is equal to power multiplied by operation time
      
      covered_area_true[drone] = model.addVar(0.0, FIELD_AREA, 0.0, GRB_SEMICONT, "covered_area_true_" + std::to_string(drone)); // total area covered by the drone
      model.addQConstr(covered_area_true[drone] == covered_area_total[drone] * drone_is_used[drone], "covered_area_true_identity_" + std::to_string(drone)); // total area covered by the drone is equal to total area covered by the drone multiplied by binary variable indicating if the drone is used
      model.addQConstr(covered_area_total[drone] <= FIELD_AREA, "covered_area_total_limit_" + std::to_string(drone)); // total area covered by the drone is less than or equal to field area
      model.addQConstr(covered_area_total[drone] * drone_is_used[drone] >= 0.0, "covered_area_total_positive_" + std::to_string(drone)); // total area covered by the drone must be positive
      
      //model.addGenConstrIndicator(drone_is_used[drone], 0, covered_area_true[drone] <= 1.0 , "addGenConstrIndicator_drone_is_used");

      // number of charging cycles is operation time divided by maximum operation time
      charging_cycles[drone] = model.addVar(0.0, 20, 0.0, GRB_SEMIINT, "charging_cycles_" + std::to_string(drone)); // number of charging cycles for each drone
      model.addQConstr(charging_cycles[drone] == (operation_time[drone] - OPERATION_MAX_PER_CHARGING)*OPERATION_MAX_PER_CHARGING_INV, "charging_cycles_identity_" + std::to_string(drone)); // number of charging cycles is equal to operation time divided by maximum operation time
      operation_time_req[drone] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_SEMICONT, "operation_time_req_" + std::to_string(drone)); // total operation time including charging time
      model.addQConstr(operation_time_req[drone] == operation_time[drone] + ((drone_is_used[drone] * CHARGING_TIME))*charging_cycles[drone], "operation_time_req_identity_" + std::to_string(drone)); // total operation time including charging time is equal to operation time plus charging time if the drone is used
      
    }
    // sum of covered_area_total = FIELD_AREA
    GRBLinExpr covered_area_total_sum = 0.0; 
    for (int drone = 0; drone < num_drones; drone++) {
      covered_area_total_sum += covered_area_true[drone];
    }
    model.addQConstr(covered_area_total_sum == FIELD_AREA, "covered_area_total_sum_constraint");

    // oeration time
    // operation_time_total should be the maximum operation_time_req among all drones
    GRBVar operation_time_total = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "operation_time_total");
    model.addGenConstrMax(operation_time_total, operation_time_req.data(), num_drones, -GRB_INFINITY ,"operation_time_total_max");

    
    // sum of sensor power expressions
    //GRBQuadExpr power_sensor_total = GRBQuadExpr();
    //GRBQuadExpr power_actuator_total = GRBQuadExpr();
    //GRBQuadExpr total_power = GRBQuadExpr();
    //for (int drone = 0; drone < num_drones; drone++) {  
    //  power_actuator_total += 1*ps_exprs[drone];
    //  power_sensor_total += pa_exprs[drone];
    //  total_power += 1*drone_pa_consumption[drone] + 1*drone_ps_consumption[drone];
    //}
 
    // sum of energy expressions as objective
    GRBQuadExpr total_energy_consumed = GRBQuadExpr();
    for (int drone = 0; drone < num_drones; drone++) {
      total_energy_consumed += drone_energy_consumption[drone];
    }
    GRBQuadExpr objective_expr = GRBQuadExpr();
    objective_expr = 1*total_energy_consumed + 
                     100*drone_used_total + 
                     100*operation_time_total;


  //####################################################################################  optimization  ##################################################################################
  
    // objective optimization
    model.setObjective(objective_expr , GRB_MINIMIZE);//GRB_MAXIMIZE);//
    model.set(GRB_DoubleParam_MIPGap, 0.01); // 5% optimality gap
    //model.set(GRB_DoubleParam_TimeLimit, 60); // 60 seconds time limit
    //model.set(GRB_IntParam_Threads, 4); // Use 4 threads
    //model.set(GRB_IntParam_Presolve, 2); // Aggressive presolve
    //model.set(GRB_DoubleParam_Heuristics, 0.9); // Increase heuristic effort
    

    //warm start
    /*
    for (int drone = 0; drone < num_drones; ++drone) {
      drone_v[drone].set(GRB_DoubleAttr_Start, 0.5); // normalized speed
      drone_h[drone].set(GRB_DoubleAttr_Start, 0.1); // normalized height
      sensor_fps[drone].set(GRB_DoubleAttr_Start, 0.11); // normalized fps
      sensor_pix[drone].set(GRB_DoubleAttr_Start, 0.035); // normalized pixel
      drone_is_used[drone].set(GRB_DoubleAttr_Start, 1.0); // all drones used
          // For selectors, set the first config to 1, others to 0
      for (int i = 0; i < drone_set_pix.size(); ++i) {
          double val = (i == 0) ? 1.0 : 0.0;
          sensor_pix_selector[drone * drone_set_pix.size() + i].set(GRB_DoubleAttr_Start, val);
      }
      for (int i = 0; i < drone_set_fps.size(); ++i) {
          double val = (i == 0) ? 1.0 : 0.0;
          drone_set_fps_selector[drone * drone_set_fps.size() + i].set(GRB_DoubleAttr_Start, val);
      }
    }*/
    //model.set(GRB_IntParam_Cuts, 2);  //Generate more cuts
    //model.set(GRB_IntParam_CutPasses, 20); //Limit the number of cut passes
    //model.set(GRB_DoubleParam_Heuristics, 1.0); //Spend 50% of time on heuristics
    //model.set(GRB_IntParam_MIPFocus, 1); //Focus on finding feasible solutions
    //model.set(GRB_IntParam_RINS, 10); //Enable RINS heuristic with a frequency of 10 nodes

    model.optimize();
  //####################################################################################  handle result ##################################################################################
  
    // Parameter tuning
    //model.tune(); // Run Gurobi's automatic parameter tuning tool
    // Optionally, after tuning, you can set the best found parameters:
    // model.getTuneResult(0); // Load the best parameter set found
    // model.optimize(); // Re-optimize with tuned parameters

    int model_status = model.get(GRB_IntAttr_Status);
    if (model_status == GRB_OPTIMAL) {
      success += 1;
      std::cout << "wind speed: " << weather_prediction[0].read() << std::endl;
      std::cout << "wind angle: " << weather_prediction[1].read() << std::endl;
      double total_pa = 0.0, total_ps = 0.0, total_power = 0.0, total_energy = 0.0, total_covered_area = 0.0;

      // --- CSV output block ---
      std::ofstream csv("../log/log_optimization_results.csv", std::ios::app);
      if (csv.tellp() == 0) {
        csv << "counter,drone,used,v,v_true,h,fps,pix,pix_x,pix_y,covered_area_x_t0,covered_area_y_t0,covered_area_total_t0,covered_area_total,covered_area_true,number_of_place_covered,covered_distance,operation_time,pa_consumption,ps_consumption,power,energy,charging_cycles,operation_time_req\n";
      }
      // --- end CSV header ---

      for (int drone = 0; drone < num_drones; drone++) {
        if(drone_is_used[drone].get(GRB_DoubleAttr_X)){
          std::cout << "----------------------------------------" << std::endl;
          std::cout << "Drone " << drone + 1  << " used: " << drone_used_total.getValue() << std::endl;
          std::cout << "Drone " << drone + 1 << " results:" << std::endl;
          std::cout << "  is used: " << drone_is_used[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal v: " << drone_v[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal v_true: " << drone_v_true[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal h: " << drone_h[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal h_true: " << drone_h[drone].get(GRB_DoubleAttr_X) * ( ALTITUDE_MAX - ALTITUDE_MIN ) + ALTITUDE_MIN << std::endl;
          std::cout << "  Optimal fps: " << sensor_fps[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  sensor_fps_true: " << sensor_fps[drone].get(GRB_DoubleAttr_X)* ( FPS_MAX - FPS_MIN) + FPS_MIN << std::endl;
          std::cout << "  Optimal pix: " << sensor_pix[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal sensor pix x: " << sensor_pix_x[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Optimal sensor pix y: " << sensor_pix_y[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_area_x_t0: " << covered_area_x_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_area_y_t0: " << covered_area_y_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_area_total_t0: " << covered_area_total_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_area_total: " << covered_area_total[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_area_true: " << covered_area_true[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  number_of_place_covered: " << number_of_place_covered[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  distance_between_place: " << covered_area_total_t0[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  covered_distance: " << covered_distance[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  operation_time: " << operation_time[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Actuator Power: " << drone_pa_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Sensor Power: " << drone_ps_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Power: " << drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  Energy: " << drone_energy_consumption[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  charging_cycles: " << charging_cycles[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "  operation_time_req: " << operation_time_req[drone].get(GRB_DoubleAttr_X) << std::endl;
          std::cout << "----------------------------------------" << std::endl;
          total_pa += drone_pa_consumption[drone].get(GRB_DoubleAttr_X);
          total_ps += drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
          total_power += drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
          total_energy += drone_energy_consumption[drone].get(GRB_DoubleAttr_X);
          total_covered_area += covered_area_true[drone].get(GRB_DoubleAttr_X);

          // --- Write to CSV ---
          csv << counter << "," << drone << ","
              << drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_v[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_v_true[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_h[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << sensor_fps[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << sensor_pix[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << sensor_pix_x[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << sensor_pix_y[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_area_x_t0[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_area_y_t0[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_area_total_t0[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_area_total[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_area_true[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << number_of_place_covered[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << covered_distance[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << operation_time[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_pa_consumption[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_ps_consumption[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_pa_consumption[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_energy_consumption[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << charging_cycles[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << operation_time_req[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X)
              << "\n";
          // --- end CSV row ---
        }
      }
      csv.close();
      // --- end CSV output block ---

    std::cout << "Total Actuator Power: " << total_pa << std::endl;
    std::cout << "Total Sensor Power: " << total_ps << std::endl;
    std::cout << "Total Power: " << total_power << std::endl;
    std::cout << "Total Energy: " << total_energy << std::endl;
    std::cout << "Total covered area: " << total_covered_area << std::endl;
    std::cout << "Optimization was successful." << std::endl;
    std::cout << "Success: " << success << std::endl;
    } else {
      //return; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      std::cout << "Optimization was not successful." << std::endl;
      std::cout << "Model status: " << model_status << std::endl;
      if (model_status == GRB_INFEASIBLE) {
        std::cout << "Model is infeasible. Please check constraints and input data." << std::endl;
      } else if (model_status == GRB_UNBOUNDED) {
        std::cout << "Model is unbounded. Please check constraints and input data." << std::endl;
      }
    }
    // Export model for diagnostics
    //model.write("model.lp"); // Export model to LP file for inspection
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