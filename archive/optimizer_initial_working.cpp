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
  GRBEnv env = GRBEnv(true);
  env.set(GRB_IntParam_OutputFlag, 0);
  env.start();
  GRBModel model = GRBModel(env);
  model.set(GRB_IntParam_NonConvex, 2);
  GRBVar one = model.addVar(0.9, 1.0, 0.0, GRB_CONTINUOUS, "one");
  model.addQConstr(one == 1, "one_identity");

  
  //###############################################################################  drone dependent vatiable  #############################################################################
  // drone basic variables
  GRBVar drone_v = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "drone_v");
  GRBVar drone_v_2 = model.addVar(0.0, 10000, 0.0, GRB_CONTINUOUS, "drone_v_2");
  GRBVar drone_v_3 = model.addVar(0.0, 10000, 0.0, GRB_CONTINUOUS, "drone_v_3");
  GRBVar drone_v_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "v_inv");
  GRBVar drone_v_true = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "drone_v_true"); // true velocity of the drone
  GRBVar drone_v_true_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "drone_v_true_inv"); // inverse of true velocity of the drone
  GRBVar drone_h = model.addVar(0.0001, 1.0, 0.0, GRB_CONTINUOUS, "drone_h");
  GRBVar sensor_fps = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_fps");
  GRBVar sensor_fps_2 = model.addVar(0.0, 100000.0, 0.0, GRB_CONTINUOUS, "sensor_fps_2");
  GRBVar sensor_fps_true = model.addVar(15.0, 144.0, 0.0, GRB_CONTINUOUS, "sensor_fps_true"); // true fps
  GRBVar sensor_pix = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "sensor_pix");
  GRBVar sensor_pix_2 = model.addVar(0.0, 100000.0, 0.0, GRB_CONTINUOUS, "sensor_pix_2");
  GRBVar sensor_pix_x = model.addVar(PIX_X_MIN, PIX_X_MAX, 0.0, GRB_CONTINUOUS, "sensor_pix_x");
  GRBVar sensor_pix_x_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "sensor_pix_x_inv");
  GRBVar sensor_pix_y = model.addVar(PIX_Y_MIN, PIX_Y_MAX, 0.0, GRB_CONTINUOUS, "sensor_pix_y");
  GRBVar sensor_pix_y_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "sensor_pix_y_inv");
  GRBVar sensor_fps_pix = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "sensor_fps_pix");
  model.addQConstr(drone_v_2 == drone_v*drone_v, "drone_v_2_identity");
  model.addQConstr(drone_v_3 == drone_v*drone_v_2, "drone_v_3_identity");
  model.addQConstr(drone_v_inv*drone_v == 1, "drone_v_inv_identity");
  model.addQConstr(drone_v_true == drone_v * SPEED_MAX + SPEED_MIN, "drone_v_true_identity"); // true velocity of the drone is equal to the normalized velocity multiplied by the maximum speed plus the minimum speed
  model.addQConstr(drone_v_true_inv * drone_v_true == 1.0, "drone_v_true_inv_identity"); // inverse of true velocity of the drone
  model.addQConstr(sensor_fps_pix == sensor_fps * sensor_pix, "sensor_fps_pix_identity");
  model.addQConstr(sensor_fps_2 == sensor_fps*sensor_fps, "sensor_fps_2_identity");  
  model.addQConstr(sensor_fps_true == sensor_fps*FPS_MAX + FPS_MIN, "sensor_fps_true_identity"); // fps == sensor_fps
  model.addQConstr(sensor_fps_true >= drone_v*CONST_2_TAN_CAMERA_THETA_INV, "coverage_fps_lower_bound"); // fps >= v * 2 * tan(camera_theta)^-1
  model.addQConstr(sensor_pix_2 == sensor_pix*sensor_pix, "sensor_pix_2_identity");
  model.addQConstr(sensor_pix_x_inv*sensor_pix_x == 1, "sensor_pix_x_inv_identity");
  model.addQConstr(sensor_pix_y_inv*sensor_pix_y == 1, "sensor_pix_y_inv_identity");

  // pixel x and y selector
  std::vector<GRBVar> sensor_pix_selector;  // binary vars
  GRBLinExpr sensor_pix_selector_total = 0;
  GRBLinExpr sensor_pix_x_value = 0;
  GRBLinExpr sensor_pix_y_value = 0;
  GRBLinExpr drone_set_pix_value = 0;
  for (int i = 0; i < drone_set_pix.size(); ++i) { // binary variable to choose only one cfg
      sensor_pix_selector.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "sensor_pix_selector_" + std::to_string(i)));}
  for (int i = 0; i < drone_set_pix.size(); ++i) {
      sensor_pix_selector_total += sensor_pix_selector[i];}
  for (int i = 0; i < drone_set_pix_x.size(); ++i) {
      sensor_pix_x_value += drone_set_pix_x[i] * sensor_pix_selector[i]; 
      sensor_pix_y_value += drone_set_pix_y[i] * sensor_pix_selector[i];
      drone_set_pix_value += drone_set_pix_normalized[i] * sensor_pix_selector[i]; }
  model.addConstr(sensor_pix_selector_total == 1, "sensor_pix_xy_selector_total_1");
  model.addQConstr(sensor_pix_x == sensor_pix_x_value, "sensor_pix_x_value_restriction");
  model.addQConstr(sensor_pix_y == sensor_pix_y_value, "sensor_pix_y_value_restriction");
  model.addQConstr(sensor_pix == drone_set_pix_value, "drone_pix_value_restriction");

  // fps selector
  std::vector<GRBVar> drone_set_fps_selector;  // binary vars
  GRBLinExpr drone_set_fps_selector_total = 0;
  GRBLinExpr drone_set_fps_value = 0;
  for (int i = 0; i < drone_set_fps.size(); ++i) {
      drone_set_fps_selector.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "drone_set_fps_selector_" + std::to_string(i)));}
  for (int i = 0; i < drone_set_fps_selector.size(); ++i) {
      drone_set_fps_selector_total += drone_set_fps_selector[i];}
  for (int i = 0; i < drone_set_fps.size(); ++i) {
      drone_set_fps_value += drone_set_fps_normalized[i] * drone_set_fps_selector[i];}
  model.addConstr(drone_set_fps_selector_total == 1, "drone_set_fps_selector_total_1");
  model.addQConstr(sensor_fps == drone_set_fps_value, "sensor_fps_value_restriction");

  // area variables by drone
  GRBVar covered_area_x_t0 = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "covered_area_x_t0"); // area covered by the drone in x direction
  GRBVar covered_area_y_t0 = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "covered_area_y_t0"); // area covered by the drone in y direction
  GRBVar covered_area_x_t0_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "covered_area_x_t0_inv"); // inverse of area covered by the drone in x direction
  GRBVar covered_area_y_t0_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "covered_area_y_t0_inv"); // inverse of area covered by the drone in y direction
  GRBVar covered_area_total_t0 = model.addVar(0.0, 100000000.0, 0.0, GRB_CONTINUOUS, "covered_area_total_t0"); // total area covered by the drone at a time
  GRBVar covered_area_total_t0_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "covered_area_total_t0_inv");  // inverse of total area covered by the drone at a time
  GRBVar covered_area_total = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "covered_area_total"); // total area covered by a drone
  model.addQConstr(covered_area_x_t0_inv*covered_area_x_t0 == 1.0, "covered_area_x_t0_inv_identity"); // inverse of area covered by the drone in x direction
  model.addQConstr(covered_area_y_t0_inv*covered_area_y_t0 == 1.0, "covered_area_y_t0_inv_identity"); // inverse of area covered by the drone in y direction
  model.addQConstr(covered_area_total_t0_inv*covered_area_total_t0 == 1.0, "covered_area_total_t0_inv_identity"); // inverse of total area covered by the drone at a time
  model.addQConstr(covered_area_total_t0 == covered_area_x_t0 * covered_area_y_t0, "covered_area_total_t0_identity2"); // total area covered by the drone at a time is equal to the product of area covered in x and y direction
  model.addQConstr(covered_area_x_t0*covered_area_y_t0_inv == sensor_pix_x*sensor_pix_y_inv, "covered_area_x_t0_y_t0_identity"); // area covered in x direction divided by area covered in y direction is equal to pixel x resolution divided by pixel y resolution

  // operation time and distance variables
  GRBVar number_of_place_covered = model.addVar(0.0, 100000.0, 0.0, GRB_CONTINUOUS, "number_of_place_covered"); // number of places covered by the drone
  GRBVar distance_between_place = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "distance_between_place"); // distance between places covered by the drone
  GRBVar covered_area_total_inv = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "covered_area_total_inv");
  GRBVar covered_distance = model.addVar(0.0, 10000000.0, 0.0, GRB_CONTINUOUS, "covered_distance"); // distance covered by the drone
  GRBVar operation_time = model.addVar(0.0, 1000000000.0, 0.0, GRB_CONTINUOUS, "operation_time");
  model.addQConstr(number_of_place_covered == covered_area_total*covered_area_total_t0_inv, "number_of_place_covered_identity"); // number of places covered by the drone is equal to the total area covered by the drone divided by the area covered at a time
  model.addQConstr(distance_between_place == covered_area_x_t0, "distance_between_place_identity"); // distance between places covered by the drone is equal to the area covered in x direction divided by the pixel x resolution
  model.addQConstr(covered_area_total_inv * covered_area_total == 1.0, "covered_area_total_inv_identity"); // inverse of total covered area
  model.addQConstr(covered_area_y_t0*sensor_pix_y_inv <= 0.01, "covered_area_x_t0_resolution_min");
  model.addQConstr(covered_area_x_t0 == CONST_2_TAN_CAMERA_THETA * (drone_h*ALTITUDE_MAX + ALTITUDE_MIN), "covered_area_x_t0_identity"); // area covered in x direction is equal to 2 * tan(camera theta) * (drone height * maximum altitude + minimum altitude) / pixel x resolution
  model.addQConstr(covered_distance == (distance_between_place * number_of_place_covered) - distance_between_place, "covered_distance_identity"); // distance covered by the drone is equal to the distance between places covered by the drone multiplied by the number of places covered
  model.addQConstr(operation_time == covered_distance*drone_v_true_inv, "operation_time_identity"); // operation time is distance divided by velocity


  //##############################################################################  summation of expression ##############################################################################
  ///// for now, we consider only one drone **************************// sum of covered_area_total_t0 over time should equal to the total area covered by the drone
  model.addQConstr(covered_area_total == FIELD_AREA, "covered_area_total_identity"); // sum of total  area should be equal to the field area
  //// **

  // actuator power equation
  GRBQuadExpr pa_expr = pa_C_0 + 
                        pa_C_1*drone_v + 
                        pa_C_2*drone_v_2 + 
                        pa_C_3*drone_v_3 + 
                        pa_C_4*drone_h;
  model.addQConstr(pa_expr >= 0.0, "pa_expr_lower_bound");

  // sensor power equation p =  a*fps + b*pix + c*pix*fps + d*std::pow(fps,2) + e*std::pow(pix,2) + f;
  GRBQuadExpr ps_expr = ps_a*sensor_fps + 
                        ps_b*sensor_pix + 
                        ps_c*sensor_fps*sensor_pix + 
                        ps_d*sensor_fps_2 + 
                        ps_e*sensor_pix_2;
  model.addQConstr(ps_expr >= 0.0, "ps_expr_lower_bound");

  // power equation
  GRBQuadExpr p_expr = (pa_expr*POWER_ACTUATOR_MAX + POWER_ACTUATOR_MIN) + ((ps_expr + ps_f)*POWER_SENSOR_MAX + POWER_SENSOR_MIN);
  model.addQConstr(p_expr >= 0.0, "p_expr_lower_bound");

  // energy equation E = P * T
  GRBQuadExpr E_expr = 0.0; // energy expression
  //model.addQConstr(E_expr >= 0.0, "E_expr_lower_bound");
  E_expr =  
              operation_time* pa_C_0 
            + operation_time* pa_C_1* drone_v    *POWER_ACTUATOR_MAX 
            + operation_time* pa_C_2* drone_v_2  *POWER_ACTUATOR_MAX 
            + operation_time* pa_C_3* drone_v_3  *POWER_ACTUATOR_MAX 
            + operation_time* pa_C_4* drone_h    *POWER_ACTUATOR_MAX + POWER_ACTUATOR_MIN* one

            + operation_time* ps_a* sensor_fps     *POWER_SENSOR_MAX
            + operation_time* ps_b* sensor_pix     *POWER_SENSOR_MAX 
            + operation_time* ps_c* sensor_fps_pix *POWER_SENSOR_MAX 
            + operation_time* ps_d* sensor_fps_2   *POWER_SENSOR_MAX 
            + operation_time* ps_e* sensor_pix_2   *POWER_SENSOR_MAX + POWER_SENSOR_MIN* one
            + operation_time* ps_f                 *POWER_SENSOR_MAX; // total energy consumed by the drone during the operation time
            ; 

  //####################################################################################  optimization ####################################################################################
  // objective optimization
  model.setObjective(E_expr , GRB_MINIMIZE);//GRB_MAXIMIZE);//
  model.optimize();

  //handle result
  if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
    std::cout << "wind speed: " << weather_prediction[0].read() << std::endl;
    std::cout << "wind angle: " << weather_prediction[1].read() << std::endl;
    std::cout << "Optimal v: " << drone_v.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "Optimal v_true: "<< drone_v_true.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "Optimal h: " << drone_h.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "Optimal fps: " << sensor_fps.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "sensor_fps_true: " << sensor_fps_true.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "Optimal pix: " << sensor_pix.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "operation_time: " << operation_time.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "E: " << (model.get(GRB_DoubleAttr_ObjVal))<< std::endl;
    std::cout << "P: " << (p_expr.getValue()) << std::endl;
    std::cout << "Optimal sensor pix x: " << sensor_pix_x.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "Optimal sensor pix y: " << sensor_pix_y.get(GRB_DoubleAttr_X) << std::endl;

    std::cout << "covered_area_x_t0: " << covered_area_x_t0.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "covered_area_y_t0: " << covered_area_y_t0.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "covered_area_total_t0: " << covered_area_total_t0.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "covered_area_total: " << covered_area_total.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "number_of_place_covered: " << number_of_place_covered.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "distance_between_place: " << distance_between_place.get(GRB_DoubleAttr_X) << std::endl;
    std::cout << "covered_distance: " << covered_distance.get(GRB_DoubleAttr_X) << std::endl;
    
  } else {
    std::cout << "Optimization was not successful." << std::endl;
  }
  //std::cout << "pa_C_0: " << pa_C_0 << ", pa_C_1: " << pa_C_1 << ", pa_C_2: " << pa_C_2 << ", pa_C_3: " << pa_C_3 << ", pa_C_4: " << pa_C_4 << std::endl;
  //std::cout << "ps_a: " << ps_a << ", ps_b: " << ps_b << ", ps_c: " << ps_c << ", ps_d: " << ps_d << ", ps_e: " << ps_e << ", ps_f: " << ps_f << std::endl;
  return;
} 

//sc_in<double> model_parameter[10];  [eta, delta, alpha, beta][sigma, omega, epsilon]
//sc_in<double> constraints_value[10];
//sc_in<double> weather_prediction[10];
//sc_out<double> cfg[10];
//sc_out<double> power_consumption[10];
//sc_out<double> operation_time[10];
//sc_core::sc_vector<sc_in<double>> observed_data; //drone mass, payload mass, altitute, wind speed, wind angle, speed of drone,power actuator, umber of pixel, fps, power sensor