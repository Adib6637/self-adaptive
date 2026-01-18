#include "optimizer.h"
#include "gurobi_c++.h"
#include "parameter.h"
#include "callback.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>

int success = 0;
const double SCALE = 1000.0;
const double SCALE_INV = 1/SCALE;
const double t_1_60 = 0.016667;//1/60; 
const double t_1_1hour = 0.000278;
// model parameters
double model_param_precision = 100000.0; 
double coeff_precision = 100000.0; 

// Store previous observed data
static std::vector<double> previous_observed_data;


void Optimizer::optimize() { 
  
  static int counter = 0;
  // Store current observed data for future use
  if (previous_observed_data.empty()) {
    previous_observed_data.resize(20, 0.0); // Assuming 20 elements in observed_data
  }
  
  // Check optimization conditions using both current and previous data
  if (!OPTIMIZER_ON || counter == observed_data[19].read()|| observed_data[19].read() < 75 //3 
  ||  observed_data[19].read() > 200
#ifdef FIX_MODEL_PARAMETER
  || model_parameter[0].read() <= 0
#endif
  )return;
  counter = observed_data[19].read(); 

  // Check if optimization is needed
  if (!reasoning()) return; 

  // drone pixel set
  std::vector<double> drone_set_pix = DRONE_SET_PIX;
  std::vector<double> drone_set_pix_x = DRONE_SET_PIX_X;
  std::vector<double> drone_set_pix_y = DRONE_SET_PIX_Y;
  std::vector<double> drone_set_pix_normalized;
  for (int i = 0; i < drone_set_pix.size(); i++) {
      drone_set_pix_normalized.push_back((drone_set_pix[i] - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN)); // Normalize pixel values
  }

  //energy storage
  std::vector<double> drone_energy_capacity = DRONE_ENERGY_CAPACITY;
  std::vector<double> drone_energy_capacity_used = DRONE_ENERGY_CAPACITY_USED;
  static std::vector<double> drone_in_use = DRONE_IN_USE;

  // fps set
  std::vector<double> drone_set_fps = DRONE_SET_FPS;
  std::vector<double> drone_set_fps_normalized;
  for (int i = 0; i < drone_set_fps.size(); i++) {
      drone_set_fps_normalized.push_back((drone_set_fps[i] - FPS_MIN) / (FPS_MAX - FPS_MIN)); // Normalize fps values
  }

  try{
  // ####################################################################################### Constants ###################################################################################
    // weather prediction
    double wp_v_wind = (weather_prediction[0].read()-WIND_SPEED_MIN)/WIND_SPEED_MAX;         
    double wp_theta_wind = weather_prediction[1].read();   
    
#ifdef FIX_MODEL_PARAMETER
    double fix_model_parameter[] = {0.3186, 0.2127, 0.0973, 0.0021, 1.1464, 10.2123, -1.8801}; 
    double pa_alpha = (double)(round((int)(fix_model_parameter[0] * model_param_precision))) / model_param_precision;
    double pa_beta = (double)(round((int)(fix_model_parameter[1] * model_param_precision))) / model_param_precision;
    double pa_eta = (double)(round((int)(fix_model_parameter[2] * model_param_precision))) / model_param_precision;
    double pa_delta = (double)(round((int)(fix_model_parameter[3] * model_param_precision))) / model_param_precision;

    double ps_a = (double)(round((int)(fix_model_parameter[4] * model_param_precision))) / model_param_precision;
    double ps_b = (double)(round((int)(fix_model_parameter[5] * model_param_precision))) / model_param_precision;
    double ps_c = (double)(round((int)(fix_model_parameter[6] * model_param_precision))) / model_param_precision;
#else
    double pa_eta = (double)(round((int)(model_parameter[0].read() * model_param_precision))) / model_param_precision;
    double pa_delta = (double)(round((int)(model_parameter[1].read() * model_param_precision))) / model_param_precision;
    double pa_alpha = (double)(round((int)(model_parameter[2].read() * model_param_precision))) / model_param_precision;
    double pa_beta = (double)(round((int)(model_parameter[3].read() * model_param_precision))) / model_param_precision;

    double ps_a = (double)(round((int)(model_parameter[4].read() * model_param_precision))) / model_param_precision;
    double ps_b = (double)(round((int)(model_parameter[5].read() * model_param_precision))) / model_param_precision;
    double ps_c = (double)(round((int)(model_parameter[6].read() * model_param_precision))) / model_param_precision;
#endif //FIX_MODEL_PARAMETER


    // folding actuator equation
    double linear_acceleration_z_normalized = previous_observed_data[0];
    double linear_acceleration_z = linear_acceleration_z_normalized*(LINEAR_ACCELERATION_Z_MAX - LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN;
    double payload_mass_normalized = previous_observed_data[1];
    double payload_mass = payload_mass_normalized*(PAYLOAD_MAX - PAYLOAD_MIN) + PAYLOAD_MIN; 
    double drone_mass = DRONE_MASS;
    double total_mass = drone_mass + payload_mass;

    double pa_phi = std::abs(std::sin(wp_theta_wind * M_PI / 180.0));
    double pa_theta = -1/(pa_eta + 1e-5); // 1/eta
    double pa_C_0_ = 1*pa_theta * linear_acceleration_z * total_mass * pa_phi + pa_theta*pa_alpha*std::pow(pa_phi,3) + pa_delta;
    double pa_C_1_ = 1*SCALE_INV*(pa_theta * (linear_acceleration_z* total_mass) + 3*pa_theta*pa_alpha*std::pow(pa_phi,2));
    double pa_C_2_ = 1*SCALE_INV*SCALE_INV*(3 * pa_theta * pa_alpha * pa_phi);
    double pa_C_3_ = 1*SCALE_INV*SCALE_INV*SCALE_INV*(pa_theta * pa_alpha);
    double pa_C_4_ = 1*SCALE_INV*(pa_theta * pa_beta * total_mass);

    double pa_C_0 = 0;
    double pa_C_1 = 0;
    double pa_C_2 = 0;
    double pa_C_3 = 0;
    double pa_C_4 = 0;

    if(true){
      pa_C_0 = (double)(round((int)(pa_C_0_* coeff_precision))) / coeff_precision;
      pa_C_1 = (double)(round((int)(pa_C_1_* coeff_precision))) / coeff_precision;
      pa_C_2 = (double)(round((int)(pa_C_2_* coeff_precision))) / coeff_precision;
      pa_C_3 = (double)(round((int)(pa_C_3_* coeff_precision))) / coeff_precision;
      pa_C_4 = (double)(round((int)(pa_C_4_* coeff_precision))) / coeff_precision;
    }else{
      pa_C_0 = pa_C_0_;
      pa_C_1 = pa_C_1_;
      pa_C_2 = pa_C_2_;
      pa_C_3 = pa_C_3_;
      pa_C_4 = pa_C_4_;
    }

    if (false){
      PRINT_LOG(25, "#########################", counter)
      PRINT_LOG(25, "linear_acceleration_z:", linear_acceleration_z)
      PRINT_LOG(25, "pa_eta:", pa_eta)
      PRINT_LOG(25, "pa_delta:", pa_delta)  
      PRINT_LOG(25, "pa_alpha:", pa_alpha)
      PRINT_LOG(25, "pa_beta:", pa_beta)
      PRINT_LOG(25, "pa_phi:", pa_phi)
      PRINT_LOG(25, "pa_theta:", pa_theta)
      PRINT_LOG(25, "pa_C_0:", pa_C_0)
      PRINT_LOG(25, "pa_C_1:", pa_C_1)
      PRINT_LOG(25, "pa_C_2:", pa_C_2)
      PRINT_LOG(25, "pa_C_3:", pa_C_3)
      PRINT_LOG(25, "pa_C_4:", pa_C_4)
      PRINT_LOG(25, "ps_a:", ps_a)
      PRINT_LOG(25, "ps_b:", ps_b)
      PRINT_LOG(25, "ps_c:", ps_c)
      PRINT_LOG(25, "#########################", 0)
    }
    
  // ################################################################################# Gurobi environment ################################################################################
  
    GRBEnv env = GRBEnv(true);
#ifdef PRINT_GUROBI_OUTPUT_FLAG
    env.set(GRB_IntParam_OutputFlag, 1);
#else
    env.set(GRB_IntParam_OutputFlag, 0); // Disable Gurobi output
#endif
    env.start();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_NonConvex, -1);

  // ###############################################################################  drone dependent variable  #############################################################################
    // Store variables for each drone
    std::vector<GRBVar> 
    // speed
    drone_v(NUMBER_DRONE_MAX), 
    drone_v_2(NUMBER_DRONE_MAX), 
    drone_v_3(NUMBER_DRONE_MAX), 

    // height
    drone_h(NUMBER_DRONE_MAX), 

    // fps
    sensor_fps(NUMBER_DRONE_MAX), 
    sensor_fps_2(NUMBER_DRONE_MAX), 
    sensor_fps_true(NUMBER_DRONE_MAX), 

    // pixel
    sensor_pix(NUMBER_DRONE_MAX), 
    sensor_pix_true(NUMBER_DRONE_MAX), 
    sensor_pix_2(NUMBER_DRONE_MAX), 
    sensor_pix_x(NUMBER_DRONE_MAX), 
    sensor_pix_y(NUMBER_DRONE_MAX), 
    
    // fps pixel
    sensor_fps_pix(NUMBER_DRONE_MAX),

    // operation time
    operation_time(NUMBER_DRONE_MAX),
    operation_time_req(NUMBER_DRONE_MAX),  // total oeperation time including charging time

    //charging cycles
    charging_cycles(NUMBER_DRONE_MAX), // number of charging cycles for each drone

    // covered area
    covered_area_x_t0(NUMBER_DRONE_MAX), 
    covered_area_y_t0(NUMBER_DRONE_MAX), 
    covered_area_total_t0(NUMBER_DRONE_MAX), 
    covered_area_total(NUMBER_DRONE_MAX),
    covered_area_true(NUMBER_DRONE_MAX),
    number_of_place_covered(NUMBER_DRONE_MAX), 

    // distance
    covered_distance(NUMBER_DRONE_MAX), 

    // energy consumption
    drone_energy_consumption(NUMBER_DRONE_MAX), // energy consumption for each drone
    drone_pa_consumption(NUMBER_DRONE_MAX), // actuator power consumption for each drone
    drone_ps_consumption(NUMBER_DRONE_MAX); // sensor power consumption for each drone

    // pixel x and y selector 
    std::vector<GRBVar> sensor_pix_selector(NUMBER_DRONE_MAX * drone_set_pix.size());
    std::vector<GRBLinExpr> sensor_pix_selector_total(NUMBER_DRONE_MAX);
    std::vector<GRBLinExpr> sensor_pix_x_value(NUMBER_DRONE_MAX);
    std::vector<GRBLinExpr> sensor_pix_y_value(NUMBER_DRONE_MAX);
    std::vector<GRBLinExpr> drone_set_pix_value(NUMBER_DRONE_MAX);
    std::vector<GRBLinExpr> drone_set_pix_true_value(NUMBER_DRONE_MAX);
    for (int i = 0; i < NUMBER_DRONE_MAX; ++i) {
        sensor_pix_selector_total[i] = 0;
        sensor_pix_x_value[i] = 0;
        sensor_pix_y_value[i] = 0;
        drone_set_pix_value[i] = 0;
        drone_set_pix_true_value[i] = 0;
    }

    // fps selector
    std::vector<GRBVar> drone_set_fps_selector(NUMBER_DRONE_MAX * drone_set_fps.size());
    std::vector<GRBLinExpr> drone_set_fps_selector_total(NUMBER_DRONE_MAX);
    std::vector<GRBLinExpr> drone_set_fps_value(NUMBER_DRONE_MAX);
    for (int i = 0; i < NUMBER_DRONE_MAX; ++i) {
        drone_set_fps_selector_total[i] = 0;
        drone_set_fps_value[i] = 0;
    }
    
    // power and energy expressions
    std::vector<GRBQuadExpr> pa_exprs(NUMBER_DRONE_MAX); // actuator power expressions for each drone
    std::vector<GRBQuadExpr> ps_exprs(NUMBER_DRONE_MAX); // sensor power expressions for each drone

    // drones used
    std::vector<GRBVar> drone_is_used(NUMBER_DRONE_MAX); // binary variable to indicate if the drone is used
    GRBLinExpr drone_used_total = 0;
    for (int drone = 0; drone < NUMBER_DRONE_MAX; ++drone) {
        drone_is_used[drone] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_used_" + std::to_string(drone));
        drone_used_total += drone_is_used[drone];
    }

  // ######################################################################################  Constraint  ####################################################################################

  #if DYNAMIC_DRONE
    model.addQConstr(drone_used_total <= NUMBER_DRONE_MAX, "drone_used_total_limit"); // limit the number of drones used to the number of drones available          case_x_1
  #else
    model.addQConstr(drone_used_total == NUMBER_DRONE_MAX, "drone_used_total_limit"); // limit the number of drones used to the number of drones available          case_x_2 case_x_3
  #endif

    for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
      // drone basic variables
      // fps
      sensor_fps[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_" + std::to_string(drone));
      sensor_fps_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_2_" + std::to_string(drone));
      sensor_fps_true[drone] = model.addVar(FPS_MIN, FPS_MAX, 0.0, GRB_INTEGER, "sensor_fps_true_" + std::to_string(drone)); 
      model.addGenConstrPow(sensor_fps[drone], sensor_fps_2[drone], 2.0, "sensor_fps_2_identity_" + std::to_string(drone));
      model.addQConstr(sensor_fps_true[drone] == (sensor_fps[drone] * ( FPS_MAX - FPS_MIN)) + FPS_MIN, "sensor_fps_true_identity_" + std::to_string(drone)); 

      // speed
      drone_v[drone] = model.addVar(SPEED_MIN*SCALE, SPEED_MAX*SCALE, 0.0, GRB_INTEGER, "drone_v_ " + std::to_string(drone));
      drone_v_2[drone] = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "drone_v_2_" + std::to_string(drone));
      drone_v_3[drone] = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "drone_v_3_" + std::to_string(drone));
      model.addGenConstrPow(drone_v[drone], drone_v_2[drone], 2.0, "drone_v_2_identity_" + std::to_string(drone));
      model.addGenConstrPow(drone_v[drone], drone_v_3[drone], 3.0, "drone_v_3_identity_" + std::to_string(drone));
      model.addQConstr(drone_v[drone] <= (MAX_V_CAPTURING*SCALE), "coverage_fps_lower_bound_" + std::to_string(drone)); 
      model.addQConstr(drone_v[drone] >= (MIN_V_CAPTURING*SCALE), "coverage_fps_lower_bound_" + std::to_string(drone)); 
      //model.addQConstr(drone_v[drone] <= sensor_fps_true[drone]*OVERLAP_FACTOR, "coverage_fps_lower_bound_" + std::to_string(drone)); 

      // height
      drone_h[drone] = model.addVar(ALTITUDE_MIN*SCALE, ALTITUDE_MAX*SCALE, 0.0, GRB_INTEGER, "drone_h_" + std::to_string(drone));
      model.addQConstr(drone_h[drone] >= (5.5*SCALE) , "drone_h_identity_" + std::to_string(drone)); 
      model.addQConstr(drone_h[drone] <= (6.5*SCALE) , "drone_h_identity_" + std::to_string(drone)); 

      // pixel
      sensor_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_" + std::to_string(drone));
      sensor_pix_true[drone] = model.addVar(0.0, 2240000, 0.0, GRB_CONTINUOUS, "sensor_pix_true_" + std::to_string(drone));
      sensor_pix_2[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix_2_" + std::to_string(drone));
      sensor_pix_x[drone] = model.addVar(PIX_X_MIN, PIX_X_MAX, 0.0, GRB_INTEGER, "sensor_pix_x_" + std::to_string(drone));
      sensor_pix_y[drone] = model.addVar(PIX_Y_MIN, PIX_Y_MAX, 0.0, GRB_INTEGER, "sensor_pix_y_" + std::to_string(drone));
      model.addGenConstrPow(sensor_pix[drone], sensor_pix_2[drone], 2.0, "sensor_pix_2_identity_" + std::to_string(drone));
      
      // fps pixel
      sensor_fps_pix[drone] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps_pix_" + std::to_string(drone));
      model.addQConstr(sensor_fps_pix[drone] == sensor_fps[drone] * sensor_pix[drone], "sensor_fps_pix_identity_" + std::to_string(drone));

      // pixel selector 
      for (int i = 0; i < drone_set_pix.size(); i++) {
          sensor_pix_selector[drone * drone_set_pix.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "sensor_pix_selector_" + std::to_string(drone) + "_" + std::to_string(i));
          sensor_pix_selector_total[drone] += sensor_pix_selector[drone * drone_set_pix.size() + i];
          sensor_pix_x_value[drone] += drone_set_pix_x[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          sensor_pix_y_value[drone] += drone_set_pix_y[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          drone_set_pix_value[drone] += drone_set_pix_normalized[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
          drone_set_pix_true_value[drone] += drone_set_pix[i] * sensor_pix_selector[drone * drone_set_pix.size() + i];
      }
      model.addConstr(sensor_pix_selector_total[drone] == 1, "sensor_pix_xy_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_pix_x[drone] == sensor_pix_x_value[drone], "sensor_pix_x_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix_y[drone] == sensor_pix_y_value[drone], "sensor_pix_y_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix[drone] == drone_set_pix_value[drone], "drone_pix_value_restriction_" + std::to_string(drone));
      model.addQConstr(sensor_pix_true[drone] == drone_set_pix_true_value[drone], "drone_pix_true_value_restriction_" + std::to_string(drone));

      // fps selector 
      for (int i = 0; i < drone_set_fps.size();i++) {
          drone_set_fps_selector[drone * drone_set_fps.size() + i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_set_fps_selector_" + std::to_string(drone) + "_" + std::to_string(i));
          drone_set_fps_selector_total[drone] += drone_set_fps_selector[drone * drone_set_fps.size() + i];
          drone_set_fps_value[drone] += drone_set_fps_normalized[i] * drone_set_fps_selector[drone * drone_set_fps.size() + i];
      }
      model.addConstr(drone_set_fps_selector_total[drone] == 1, "drone_set_fps_selector_total_1_" + std::to_string(drone));
      model.addQConstr(sensor_fps[drone] == drone_set_fps_value[drone], "drone_set_fps_value_restriction_" + std::to_string(drone));

      // covered area 
      covered_area_x_t0[drone] = model.addVar(COVERED_AREA_X_MIN, COVERED_AREA_X_MAX, 0.0, GRB_CONTINUOUS, "covered_area_x_t0_" + std::to_string(drone));
      covered_area_y_t0[drone] = model.addVar(COVERED_AREA_X_MIN, COVERED_AREA_X_MAX, 0.0, GRB_CONTINUOUS, "covered_area_y_t0_" + std::to_string(drone));
      covered_area_total_t0[drone] = model.addVar(COVERED_AREA_TOTAL_MIN, COVERED_AREA_TOTAL_MAX, 0.0, GRB_CONTINUOUS, "covered_area_total_t0_" + std::to_string(drone));
      covered_area_total[drone] = model.addVar(COVERED_AREA_TOTAL_MIN, FIELD_AREA, 0.0, GRB_CONTINUOUS, "covered_area_total_" + std::to_string(drone));
      covered_area_true[drone] = model.addVar(0, FIELD_AREA, 0.0, GRB_INTEGER, "covered_area_true_" + std::to_string(drone)); // total area covered by the drone
      model.addQConstr(covered_area_total_t0[drone] == covered_area_x_t0[drone] * covered_area_y_t0[drone], "covered_area_total_t0_identity_" + std::to_string(drone)); // total area covered by the drone at t0
      model.addQConstr(covered_area_x_t0[drone]* sensor_pix_y[drone] == sensor_pix_x[drone]* covered_area_y_t0[drone] , "covered_area_x_t0_y_t0_identity_" + std::to_string(drone)); 
      model.addQConstr(covered_area_x_t0[drone]*SCALE == 2.0*TAN_CAMERA_THETA * drone_h[drone], "covered_area_x_t0_identity_" + std::to_string(drone)); 
      model.addQConstr(covered_area_true[drone] == covered_area_total[drone] * drone_is_used[drone], "covered_area_true_identity_" + std::to_string(drone)); 
      model.addQConstr(covered_area_total[drone] <= FIELD_AREA, "covered_area_total_limit_" + std::to_string(drone)); // total area covered by the drone is less than or equal to field area
      model.addQConstr(covered_area_total[drone] * drone_is_used[drone] >= 0.0, "covered_area_total_positive_" + std::to_string(drone)); // total area covered by the drone must be positive

      // number of place covered          
      number_of_place_covered[drone] = model.addVar(0.0, NUMBER_PLACE_COVERED_MAX, 0.0, GRB_CONTINUOUS, "number_of_place_covered_" + std::to_string(drone)); // number of place covered by the drone
      model.addQConstr(number_of_place_covered[drone]* covered_area_total_t0[drone] == covered_area_total[drone] , "number_of_place_covered_identity_" + std::to_string(drone)); 

      // distance covered
      covered_distance[drone] = model.addVar(COVERED_DISTANCE_MIN, COVERED_DISTANCE_MAX, 0.0, GRB_CONTINUOUS, "covered_distance_" + std::to_string(drone)); // distance covered by the drone
      model.addQConstr(covered_distance[drone] == (covered_area_x_t0[drone] * number_of_place_covered[drone]) - covered_area_x_t0[drone], "covered_distance_identity_" + std::to_string(drone)); 

      // operation time
      operation_time[drone] = model.addVar(OPERATION_TIME_MIN, OPERATION_TIME_MAX, 0.0, GRB_CONTINUOUS, "operation_time_" + std::to_string(drone)); // operation time of the drone   
      model.addQConstr(operation_time[drone]*drone_v[drone]>= covered_distance[drone]*SCALE , "operation_time_identity_" + std::to_string(drone)); 
      model.addQConstr(operation_time[drone]>= 5*60 , "operation_time_min_" + std::to_string(drone)); 

      // resolution
      model.addQConstr(covered_area_total_t0[drone] <= RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX*sensor_pix_true[drone], "resolution_MAX_identity_" + std::to_string(drone)); 
      model.addQConstr(covered_area_total_t0[drone] >= RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN*sensor_pix_true[drone], "resolution_MIN_identity_" + std::to_string(drone)); 
      
      // actuator power 
      pa_exprs[drone] = GRBQuadExpr();
      pa_exprs[drone] += pa_C_0;
      pa_exprs[drone] += pa_C_1 * (drone_v[drone]+wp_v_wind);
      pa_exprs[drone] += pa_C_2 * (drone_v_2[drone]+wp_v_wind);
      pa_exprs[drone] += pa_C_3 * (drone_v_3[drone]+wp_v_wind);
      pa_exprs[drone] += pa_C_4 * drone_h[drone]*H_REF_INV;
      model.addQConstr(pa_exprs[drone] >= 0.0, "actuator_power_positive_" + std::to_string(drone));
      drone_pa_consumption[drone] = model.addVar(0, 1500, 0.0, GRB_SEMICONT, "drone_pa_consumption_" + std::to_string(drone)); // actuator power consumption for each drone
      model.addQConstr(drone_pa_consumption[drone] == pa_exprs[drone], "drone_pa_consumption_identity_" + std::to_string(drone)); 

      // sensor power 
      ps_exprs[drone] = GRBQuadExpr();
      ps_exprs[drone] += ps_a*sensor_fps[drone];
      ps_exprs[drone] += ps_b*sensor_pix[drone];
      ps_exprs[drone] += ps_c;
      model.addQConstr(ps_exprs[drone] >= 0.0, "sensor_power_positive_" + std::to_string(drone));
      drone_ps_consumption[drone] = model.addVar(0, 50, 0.0, GRB_SEMICONT, "drone_ps_consumption_" + std::to_string(drone)); // sensor power consumption for each drone
      model.addQConstr(drone_ps_consumption[drone] == (ps_exprs[drone] * (POWER_SENSOR_MAX-POWER_SENSOR_MIN)) + POWER_SENSOR_MIN, "drone_ps_consumption_identity_" + std::to_string(drone)); 
      
      // energy
      drone_energy_consumption[drone] = model.addVar(300, 10000 , 0.0, GRB_SEMICONT, "drone_energy_consumption_" + std::to_string(drone)); // energy consumption for each drone
      model.addQConstr(drone_energy_consumption[drone] == (operation_time[drone]*t_1_60) * (drone_pa_consumption[drone] + drone_ps_consumption[drone]), "drone_energy_consumption_identity_" + std::to_string(drone)); 
      
      // number of charging cycles 
      charging_cycles[drone] = model.addVar(1.0, MAX_CHARGING_CYCLE, 0.0, GRB_INTEGER, "charging_cycles_" + std::to_string(drone)); 
      model.addQConstr(charging_cycles[drone]*drone_energy_capacity[drone]-drone_energy_capacity_used[drone] >= (drone_energy_consumption[drone]*drone_is_used[drone]), "charging_cycles_identity_" + std::to_string(drone));

      // total operation time including charging time
      operation_time_req[drone] = model.addVar(5*60, 10000, 0.0, GRB_SEMICONT, "operation_time_req_" + std::to_string(drone)); 
      model.addQConstr(operation_time_req[drone] == operation_time[drone]*drone_is_used[drone] + ((drone_is_used[drone] * CHARGING_TIME))*(charging_cycles[drone]-1), "operation_time_req_identity_" + std::to_string(drone)); 
      
    }
  // ################################################################################## drone usage history ################################################################################
    // paroritize the drone in use
    std::vector<GRBVar> bitwise_result_bin(NUMBER_DRONE_MAX);
    GRBLinExpr drone_is_used_sum = 0, drone_in_use_sum = 0, bitwise_result_sum = 0;

    for (int i = 0; i < NUMBER_DRONE_MAX; ++i) {
      // Create r_bin[i] = n_bin[i] & o_bin[i]
      bitwise_result_bin[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "bitwise_result_bin_" + std::to_string(i));

      model.addConstr(bitwise_result_bin[i] <= drone_is_used[i], "r_le_n_" + std::to_string(i));
      model.addConstr(bitwise_result_bin[i] <= drone_in_use[i], "r_le_o_" + std::to_string(i));
      model.addConstr(bitwise_result_bin[i] >= drone_is_used[i] + drone_in_use[i] - 1, "r_ge_sum_" + std::to_string(i));

      drone_is_used_sum += drone_is_used[i];
      drone_in_use_sum += drone_in_use[i];
      bitwise_result_sum += bitwise_result_bin[i];
    }
    
    // Boolean control variables. expr_valid = cond1 OR cond2
    GRBVar expr_valid = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "expr_valid");
    GRBVar le1   = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "le1");
    GRBVar eq1   = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "eq1");
    GRBVar cond1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "cond1");

    GRBVar gt1   = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "gt1");
    GRBVar eq2   = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "eq2");
    GRBVar cond2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "cond2");

    // le1: n <= o 
    model.addGenConstrIndicator(le1, true, drone_is_used_sum - drone_in_use_sum <= 0, "n_le_o");
    // gt1: n > o 
    model.addGenConstrIndicator(gt1, true, drone_is_used_sum - drone_in_use_sum >= 1, "n_gt_o");
    // eq1: n == r 
    model.addGenConstrIndicator(eq1, true, drone_is_used_sum - bitwise_result_sum == 0, "n_eq_r");
    // eq2: o == r  
    model.addGenConstrIndicator(eq2, true, drone_in_use_sum - bitwise_result_sum == 0, "o_eq_r");

    // cond1 = le1 AND eq1
    model.addConstr(cond1 <= le1);
    model.addConstr(cond1 <= eq1);
    model.addConstr(cond1 >= le1 + eq1 - 1);

    // cond2 = gt1 AND eq2
    model.addConstr(cond2 <= gt1);
    model.addConstr(cond2 <= eq2);
    model.addConstr(cond2 >= gt1 + eq2 - 1);

    model.addConstr(expr_valid >= cond1);
    model.addConstr(expr_valid >= cond2);
    model.addConstr(expr_valid <= cond1 + cond2);

    model.addConstr(expr_valid == 1, "force_condition_true"); 
  // ####################################################################################### Objective #####################################################################################
    // sum of covered_area_total
    GRBLinExpr covered_area_total_sum = 0.0; 
    for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
      covered_area_total_sum += covered_area_true[drone];
    }
    model.addQConstr(covered_area_total_sum >= FIELD_AREA, "covered_area_total_sum_constraint");

    // operation time total
    GRBVar operation_time_total = model.addVar(5*60, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "operation_time_total");
    model.addGenConstrMax(operation_time_total, operation_time_req.data(), NUMBER_DRONE_MAX, -GRB_INFINITY ,"operation_time_total_max");

    // sum of charging cycle
    GRBQuadExpr charging_cycle_total = GRBQuadExpr();
    for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
      charging_cycle_total += (charging_cycles[drone]-1)*drone_is_used[drone]; 
    }
 
    // sum of energy
    GRBQuadExpr total_energy_consumed = GRBQuadExpr();
    for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
      total_energy_consumed += drone_energy_consumption[drone]*drone_is_used[drone]; // energy consumption is only considered if the drone is used
    }

    // objective
    GRBQuadExpr objective_expr = GRBQuadExpr();
    objective_expr = OBJECTIVE_ENERGY_WEIGHT*total_energy_consumed + 
                     OBJECTIVE_DRONE_WEIGHT*drone_used_total + 
                     OBJECTIVE_TIME_WEIGHT*operation_time_total +
                     OBJECTIVE_CHARGING_CYCLE_WEIGHT*charging_cycle_total;

  //####################################################################################  optimization  ##################################################################################
  
    // objective optimization
    model.setObjective(objective_expr , GRB_MINIMIZE);

    // optimization parameters
    SET_GUROBI_SOLVER_PARAMS(model) 

    // print info
#ifdef OPTIMIZATION_TIMING_LOG
    std::ofstream logfile("./log/log_runtime_tmp.log", std::ios::app);
    if (!logfile.is_open()) {
      std::cout << "Cannot open log_runtime_tmp.log for callback message" << std::endl;
      return;
    }
    print_callback cb_ = print_callback(&logfile);
    model.setCallback(&cb_);
#endif

    model.reset();
    model.set(GRB_IntParam_Seed, 1);
    model.optimize();

#ifdef OPTIMIZATION_TIMING_LOG
    logfile.close();
#endif
  //####################################################################################### update ######################################################################################
    int model_status = model.get(GRB_IntAttr_Status);
    if(model_status == GRB_OPTIMAL) {
      for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
        if(drone_is_used[drone].get(GRB_DoubleAttr_X)){
          drone_in_use[drone] = true;
        }
      }
    }
  //#################################################################################### handle result ##################################################################################

    
    if (model_status == GRB_OPTIMAL) {
      success += 1;
      double total_pa = 0.0, total_ps = 0.0, total_power = 0.0, total_energy = 0.0, total_covered_area = 0.0;
      double last_runtime = -1.0; // Read the last runtime value from log_runtime_tmp.log

#ifdef OPTIMIZATION_TIMING_LOG
      // last_runtime contains the last value from the log file from call back funciton (or -1.0 if not found)
      std::ifstream runtime_log("./log/log_runtime_tmp.log");
      std::string line;
      while (std::getline(runtime_log, line)) {
        if (!line.empty()) {
      try {
        last_runtime = std::stod(line);
      } catch (...) {
        // Ignore conversion errors
      }
        }
      }
#endif // OPTIMIZATION_TIMING_LOG

#ifdef PRINT_OPTIMIZATION_RESULTS
      std::cout << "========================================" << std::endl;
      PRINT_LOG(25, "Result", "")
      PRINT_LOG(25, "Total drone used", drone_used_total.getValue())
      PRINT_LOG(25, "wind speed:", weather_prediction[0].read())
      PRINT_LOG(25, "wind angle:", weather_prediction[1].read())

      // --- CSV output block ---
#ifdef OPTIMIZATION_RESULT_LOG
      std::ofstream csv("./log/log_optimization_results.csv", std::ios::app);
      if (csv.tellp() == 0) {
        csv << "counter,drone,used,v,v_true,h,fps,pix,pix_x,pix_y,covered_area_x_t0,covered_area_y_t0,covered_area_total_t0,covered_area_total,covered_area_true,number_of_place_covered,covered_distance,operation_time,pa_consumption,ps_consumption,power,energy,charging_cycles,operation_time_req\n";
      }
#endif // OPTIMIZATION_RESULT_LOG

      for (int drone = 0; drone < NUMBER_DRONE_MAX; drone++) {
        if(drone_is_used[drone].get(GRB_DoubleAttr_X)){

          std::cout << "----------------------------------------" << std::endl;
          PRINT_LOG(25, "Drone id", (drone+1))
          PRINT_LOG(25, "Is used ", (drone_is_used[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "v", (drone_v[drone].get(GRB_DoubleAttr_X)/SCALE))
          PRINT_LOG(25, "h_true", (drone_h[drone].get(GRB_DoubleAttr_X))/SCALE)
          PRINT_LOG(25, "fps", (sensor_fps[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "sensor_fps_true", (sensor_fps_true[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "pix", (sensor_pix[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "sensor pix x", (sensor_pix_x[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "sensor pix y", (sensor_pix_y[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "covered_area_x_t0", (covered_area_x_t0[drone].get(GRB_DoubleAttr_X)))   
          PRINT_LOG(25, "covered_area_y_t0", (covered_area_y_t0[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "covered_area_total_t0", (covered_area_total_t0[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "covered_area_total", (covered_area_total[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "covered_area_true", (covered_area_true[drone]. get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "number_of_place_covered", (number_of_place_covered[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "covered_distance", (covered_distance[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "operation_time", (operation_time[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "Actuator Power", (drone_pa_consumption[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "Sensor Power", (drone_ps_consumption[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "Power", (drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "Energy", (drone_energy_consumption[drone].get(GRB_DoubleAttr_X)*60))
          PRINT_LOG(25, "charging_cycles", (charging_cycles[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "operation_time_req", (operation_time_req[drone].get(GRB_DoubleAttr_X)))
          PRINT_LOG(25, "----------------------------------------", "")
          total_pa += drone_pa_consumption[drone].get(GRB_DoubleAttr_X);
          total_ps += drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
          total_power += drone_pa_consumption[drone].get(GRB_DoubleAttr_X) + drone_ps_consumption[drone].get(GRB_DoubleAttr_X);
          total_energy += drone_energy_consumption[drone].get(GRB_DoubleAttr_X)*60;
          total_covered_area += covered_area_true[drone].get(GRB_DoubleAttr_X);

#ifdef OPTIMIZATION_RESULT_LOG
          csv << counter << "," << drone << ","
              << drone_is_used[drone].get(GRB_DoubleAttr_X) << ","
              << drone_v[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X)/SCALE << ","
              << drone_v[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X)/SCALE << ","
              << drone_h[drone].get(GRB_DoubleAttr_X)*drone_is_used[drone].get(GRB_DoubleAttr_X)/SCALE << ","
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
#endif // OPTIMIZATION_RESULT_LOG
        }
      }
#ifdef OPTIMIZATION_RESULT_LOG
      csv.close();
#endif // OPTIMIZATION_RESULT_LOG
    
      PRINT_LOG(25, "Total Actuator Power", total_pa)
      PRINT_LOG(25, "Total Sensor Power", total_ps)
      PRINT_LOG(25, "Total Power", total_power)
      PRINT_LOG(25, "Total Energy", total_energy)
      PRINT_LOG(25, "Total operation time", (operation_time_total.get(GRB_DoubleAttr_X)))
      PRINT_LOG(25, "Total covered area", total_covered_area)
      PRINT_LOG(25, "Success counter", success)
      PRINT_LOG(25, "Model runtime", last_runtime) // model.get(GRB_DoubleAttr_Runtime)
      std::cout << "========================================" << std::endl;

#ifdef OPTIMIZATION_TIMING_LOG
      std::ofstream runtime_csv("./log/log_runtime_results.csv", std::ios::app);
      if (runtime_csv.tellp() == 0) {
        runtime_csv << "counter,last_runtime\n";
      }
      runtime_csv << counter << "," << last_runtime << "\n";
      runtime_csv.close();
#endif // OPTIMIZATION_TIMING_LOG
#endif // PRINT_OPTIMIZATION_RESULTS
    } else {
#ifdef PRINT_OPTIMIZATION_RESULTS
      PRINT_LOG(25, "Optimization status", "not successful")
      PRINT_LOG(25, "Model status", model_status)
      if (model_status == GRB_INFEASIBLE) {
      PRINT_LOG(25, "Model status", "infeasible")
      } else if (model_status == GRB_UNBOUNDED) {
      PRINT_LOG(25, "Model status", "unbounded")
      }
#endif // PRINT_OPTIMIZATION_RESULTS
    }
  } catch (GRBException &e) {
    std::cout << "Gurobi exception caught in: " << e.getMessage() << " code: " << e.getErrorCode() << std::endl;
    return;
  } catch (std::exception &e) {
    std::cout << "Standard exception caught in: " << e.what() << std::endl;
    return;
  } catch (...) {
    std::cout << "Unknown exception caught" << std::endl;
    return;
  }


  for (int i=0; i <20; i++){
    previous_observed_data[i] = observed_data[i].read();
  }

  return;
} 

//###################################################################################  helper function  ##################################################################################
  
bool Optimizer::reasoning(){
  // Stop optimization if success limit is reached
#ifdef SUCCESS_LIMIT
  if (success== SUCCESS_LIMIT){
    std::cout << "Success limit reached, stopping optimization." << std::endl;
    return false; 
  }else{
    return true; 
  }
#endif // SUCCESS_LIMIT

  // optimize at a certain interval of model parameter or weather updates 
  static int update_counter = 1;
#ifdef FIX_MODEL_PARAMETER
  static std::vector<double> last_weather_prediction(2, 0.0);
  for (int i = 0; i < 2; ++i) {
    if (weather_prediction[i].read() != last_weather_prediction[i]) {
      last_weather_prediction[i] = weather_prediction[i].read();
      update_counter++;
      break;
    }
  }
#else
  static std::vector<double> last_model_params(7, 0.0);
  for (int i = 0; i < 7; ++i) {
    if (model_parameter[i].read() != last_model_params[i]) {
      last_model_params[i] = model_parameter[i].read();
      update_counter++;
      break;
    }
  }
#endif
  return (update_counter % OPTIMIZE_INTERVAL != 0)?false:true;
}