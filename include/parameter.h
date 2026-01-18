#include <iomanip>

#ifndef SIM_PARAM 
#define SIM_PARAM

////////////////////////////////////////////////////////////////////////////////// enhancement parameters /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define COEFFICIENT_SENSOR_A_INIT    0.928977
#define COEFFICIENT_SENSOR_B_INIT    0.620271
#define COEFFICIENT_SENSOR_C_INIT    0.0407869
#define COEFFICIENT_SENSOR_D_INIT    0.00561747

#define COEFFICIENT_ACTUATOR_ALPHA_INIT  0.0077451458
#define COEFFICIENT_ACTUATOR_BETA_INIT   -0.001
#define COEFFICIENT_ACTUATOR_ETA_INIT    0.7
#define COEFFICIENT_ACTUATOR_DELTA_INIT  0.0

#define EPSILON_ACTUATOR_INIT    1e-4
#define LR_ACTUATOR_INIT         1e-2
#define DECAY_ACTUATOR_INIT      1.0
#define EPSILON_SENSOR_INIT      1e-4
#define LR_SENSOR_INIT           1e-2
#define DECAY_SENSOR_INIT        1.0

///////////////////////////////////////////////////////////////////////////////////// case dependent //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// =============================================
#define SIMULATION_CASE 2
#define SIMULATION_SUB_CASE 1
#define SIMULATION_DURATION_NS      500000
#define SIMULATION_CLK_TICK_NS      10
#define AREA_DEFAULT                80*1000 
#define OPTIMIZER_GAP               0.00000
#define AREA_TEST                   100*1000   // only for case 5  
#define NUMBER_DRONE_MAX_DYNAMIC    10         // only for case 7  {3, 5, 10,15}  
#define NUMBER_DRONE_MAX_DEFAULT    8          
#define OPTIMIZE_INTERVAL           4          // Interval for optimization in number of model parameter updates
#define DRONE_SET_LENGTH            3          // only for case 6


#define CHARGING_TIME               1200
#define MAX_CHARGING_CYCLE          1000

#define SAMPEL_PIXEL_SIZE           32.0        // minimum
#define SAMPEL_SIZE_CM              20.0        //sample : (5cmx5cm)/(32x32) 
#define SHUTTER_SPEED               2000        // #define MAX_V_CAPTURING round((GSD/3)*SHUTTER_SPEED) , GSD = Ground Sampling Distance = MINIMUM_SAMPEL_SIZE_M/SAMPEL_PIXEL_SIZE
#define DRONE_IN_USE                {1,1,1,0,0} // priority of the drone in the optimization

// logging (define to enable) 
#define LEARNING_PROGRESS_LOG   
//#define LEARNING_TIMING_LOG
#define OPTIMIZATION_TIMING_LOG
#define OPTIMIZATION_RESULT_LOG 
#define PRINT_OPTIMIZATION_RESULTS
//#define WEATHER_FORECAST_LOG
//#define PRINT_GUROBI_OUTPUT_FLAG
//#define DATA_CHECK_LOG


#define ENERGY_CAPACITY_M100         99.9*60 // watt minute
#define DRONE_ENERGY_CAPACITY       {ENERGY_CAPACITY_M100, ENERGY_CAPACITY_M100, ENERGY_CAPACITY_M100, ENERGY_CAPACITY_M100, ENERGY_CAPACITY_M100}
#define DRONE_ENERGY_CAPACITY_USED  {0,0,0,0,0}



// =============================================

#define DRONE_PIX_VALUES_1      307200
#define DRONE_PIX_VALUES_2      307200, 1433600
#define DRONE_PIX_VALUES_3      307200, 1433600, 2240000
#define DRONE_PIX_X_VALUES_1    640
#define DRONE_PIX_X_VALUES_2    640, 1280
#define DRONE_PIX_X_VALUES_3    640, 1280, 1600
#define DRONE_PIX_Y_VALUES_1    480
#define DRONE_PIX_Y_VALUES_2    480, 1120
#define DRONE_PIX_Y_VALUES_3    480, 1120, 1400
#define DRONE_FPS_VALUES_1      30
#define DRONE_FPS_VALUES_2      30, 60
#define DRONE_FPS_VALUES_3      30, 60, 90
#define SELECT_PIX_VALUES(n)    DRONE_PIX_VALUES_##n
#define SELECT_PIX_X_VALUES(n)  DRONE_PIX_X_VALUES_##n
#define SELECT_PIX_Y_VALUES(n)  DRONE_PIX_Y_VALUES_##n
#define SELECT_FPS_VALUES(n)    DRONE_FPS_VALUES_##n

#define DRONE_SET_PIX_TEST      {SELECT_PIX_VALUES(DRONE_SET_LENGTH)}   // only for case 6  {307200, 1433600, 2240000}
#define DRONE_SET_PIX_X_TEST    {SELECT_PIX_X_VALUES(DRONE_SET_LENGTH)} // only for case 6  {640,1280 ,1600 }
#define DRONE_SET_PIX_Y_TEST    {SELECT_PIX_Y_VALUES(DRONE_SET_LENGTH)} // only for case 6  {480,1120 ,1400 } 
#define DRONE_SET_FPS_TEST      {SELECT_FPS_VALUES(DRONE_SET_LENGTH)}   // only for case 6  {30, 60, 90}  
#define DRONE_SET_PIX_DEFAULT   {SELECT_PIX_VALUES(3)}
#define DRONE_SET_PIX_X_DEFAULT {SELECT_PIX_X_VALUES(3)}
#define DRONE_SET_PIX_Y_DEFAULT {SELECT_PIX_Y_VALUES(3)}      
#define DRONE_SET_FPS_DEFAULT   {SELECT_FPS_VALUES(3)}   

#if SIMULATION_CASE == 0    // custom
    #define DYNAMIC_WEATHER         false
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 1    // dynamic model, static weather
    #define DYNAMIC_WEATHER         false
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 2  // static model, dynamic weather
    #define DYNAMIC_WEATHER         true
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 3  // dynamic model, dynamic weather
    #define DYNAMIC_WEATHER         true
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 4  // static model, static weather
    #define DYNAMIC_WEATHER         true
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 5  // static model, static weather , limit
    #define DYNAMIC_WEATHER         true
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_TEST
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#elif SIMULATION_CASE == 6  // static model, static weather
    #define DYNAMIC_WEATHER         true
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_TEST
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_TEST
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_TEST
    #define DRONE_SET_FPS           DRONE_SET_FPS_TEST
#elif SIMULATION_CASE == 7  // static model, static weather, diffrent set of drone range
    #define DYNAMIC_WEATHER         false
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_DEFAULT
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_DEFAULT
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_DEFAULT
    #define DRONE_SET_FPS           DRONE_SET_FPS_DEFAULT
#else
    #define DYNAMIC_WEATHER         false
    #define FMS_ON                  true
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            false
    #define FIELD_AREA              0.0
#endif

#if SIMULATION_SUB_CASE == 0
    #define NUMBER_DRONE_MAX        NUMBER_DRONE_MAX_DEFAULT
    #define DYNAMIC_DRONE           false
#elif SIMULATION_SUB_CASE == 1
    #define NUMBER_DRONE_MAX        NUMBER_DRONE_MAX_DEFAULT
    #define DYNAMIC_DRONE           true
#elif SIMULATION_SUB_CASE == 2
    #define NUMBER_DRONE_MAX        2
    #define DYNAMIC_DRONE           false
#elif SIMULATION_SUB_CASE == 3
    #define NUMBER_DRONE_MAX        1
    #define DYNAMIC_DRONE           false
#elif SIMULATION_SUB_CASE == 4
    #define NUMBER_DRONE_MAX        NUMBER_DRONE_MAX_DYNAMIC
    #define DYNAMIC_DRONE           true
#else
    #define NUMBER_DRONE_MAX        NUMBER_DRONE_MAX_DEFAULT
    #define DYNAMIC_DRONE           true
#endif        

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// simulation component
//#define OPTIMIZER_ON            true
//#define MODEL_LEARNER_ON        true
//#define FMS_ON                  true
//#define DYNAMIC_WEATHER         false

#define SUCCESS_LIMIT           50
#if !MODEL_LEARNER_ON
#define FIX_MODEL_PARAMETER
#endif

// simulation environment 
//#define NUMBER_DRONE_MAX        5              
//#define FIELD_AREA              30000.0       // Field area in m^2  https://sugarindustry.info/paper/15166/
#define GRAVITY                 9.81            // Gravity constant in m/s^2
#define H_REF                   258.0           // Reference height in meters
#define H_REF_INV               (1.0 / H_REF)


// optimization objective
#define OBJECTIVE_ENERGY_WEIGHT 1.0
#define OBJECTIVE_DRONE_WEIGHT  0.0
#define OBJECTIVE_TIME_WEIGHT   10.0
#define OBJECTIVE_CHARGING_CYCLE_WEIGHT 0.00

// optimization parameters
#define SET_GUROBI_SOLVER_PARAMS(model)                \
    model.set(GRB_DoubleParam_MIPGap, OPTIMIZER_GAP);  \
    //model.set(GRB_DoubleParam_TimeLimit, 60);          \
    /*model.set(GRB_IntParam_Threads, 8);*/            \
    /*model.set(GRB_IntParam_Presolve, 2);*/           \
    /*model.set(GRB_IntParam_Cuts, 2);*/               \
    /*model.set(GRB_IntParam_CutPasses, 20);*/         \
    /*model.set(GRB_DoubleParam_Heuristics, 1.0);*/    \
    /*model.set(GRB_IntParam_MIPFocus, 1);*/           \
    /*model.set(GRB_IntParam_RINS, 10);*/              \


// power actuator dataset
#define WIND_SPEED              std::get<2>
#define WIND_ANGLE              std::get<3>
#define BATTERY_VOLTAGE         std::get<4>
#define BATTERY_CURRENT         std::get<5>
#define POSITION_X              std::get<6>
#define POSITION_Y              std::get<7>
#define POSITION_Z              std::get<8>
#define ORIENTATION_X           std::get<9>
#define ORIENTATION_Y           std::get<10>
#define ORIENTATION_Z           std::get<11>
#define ORIENTATION_W           std::get<12>
#define VELOCITY_X              std::get<13>
#define VELOCITY_Y              std::get<14>
#define VELOCITY_Z              std::get<15>
#define ANGULAR_X               std::get<16>
#define ANGULAR_Y               std::get<17>
#define ANGULAR_Z               std::get<18>
#define LINEAR_ACCELERATION_X   std::get<19>
#define LINEAR_ACCELERATION_Y   std::get<20>
#define LINEAR_ACCELERATION_Z   std::get<21>
#define SPEED                   std::get<22>
#define PAYLOAD                 std::get<23>
#define ALTITUDE                std::get<24>   
#define FLIGHT_CAT              std::get<0>

// power actuator dataset bounds
#define BATTERY_VOLTAGE_MAX     26.0        // Maximum battery voltage
#define BATTERY_VOLTAGE_MIN     18.0        // Minimum battery voltage
#define BATTERY_CURRENT_MAX     48.0        // Maximum battery current             
#define BATTERY_CURRENT_MIN     15.0        // Minimum battery current             
#define WIND_SPEED_MAX          19.0        // Maximum wind speed
#define WIND_SPEED_MIN          0.0         // Minimum wind speed
#define WIND_ANGLE_MAX          360.0       // Maximum wind angle
#define WIND_ANGLE_MIN          0.0         // Minimum wind angle
//#define POSITION_X_MAX        -70.0       // Maximum position x
//#define POSITION_X_MIN        -80.0       // Minimum position x
//#define POSITION_Y_MAX        41.0        // Maximum position y
//#define POSITION_Y_MIN        40.0        // Minimum position y
#define POSITION_Z_MAX        120.0         // Maximum position z
#define POSITION_Z_MIN          0.0         // Minimum position z
#define ORIENTATION_X_MAX       1.0         // Maximum orientation x
#define ORIENTATION_X_MIN      -1.0         // Minimum orientation x
#define ORIENTATION_Y_MAX       1.0         // Maximum orientation y
#define ORIENTATION_Y_MIN      -1.0         // Minimum orientation y
#define ORIENTATION_Z_MAX       1.0         // Maximum orientation z
#define ORIENTATION_Z_MIN      -1.0         // Minimum orientation z
#define ORIENTATION_W_MAX       1.0         // Maximum orientation w
#define ORIENTATION_W_MIN      -1.0         // Minimum orientation w
#define VELOCITY_X_MAX          11.0        // Maximum velocity x
#define VELOCITY_X_MIN         -6.0         // Minimum velocity x
#define VELOCITY_Y_MAX          13.0        // Maximum velocity y
#define VELOCITY_Y_MIN         -5.0         // Minimum velocity y
#define VELOCITY_Z_MAX          6.0         // Maximum velocity z
#define VELOCITY_Z_MIN         -5.0         // Minimum velocity z
//#define ANGULAR_X_MAX         3.0         // Maximum angular x
//#define ANGULAR_X_MIN        -3.0         // Minimum angular x
//#define ANGULAR_Y_MAX         3.0         // Maximum angular y
//#define ANGULAR_Y_MIN        -14.0        // Minimum angular y
//#define ANGULAR_Z_MAX         3.0         // Maximum angular z
//#define ANGULAR_Z_MIN        -2.0         // Minimum angular z
//#define LINEAR_ACCELERATION_X_MAX     5.0 // Maximum linear acceleration x
//#define LINEAR_ACCELERATION_X_MIN    -6.0 // Minimum linear acceleration x
//#define LINEAR_ACCELERATION_Y_MAX     8.0 // Maximum linear acceleration y
//#define LINEAR_ACCELERATION_Y_MIN    -10.0// Minimum linear acceleration y
#define LINEAR_ACCELERATION_Z_MAX      -3.0 // Maximum linear acceleration z
#define LINEAR_ACCELERATION_Z_MIN      -23.0// Minimum linear acceleration z  
#define SPEED_MAX               12.0        // Maximum speed
#define SPEED_MIN               0.0         // Minimum speed
#define PAYLOAD_MAX             3.5//0.75        // Maximum payload
#define PAYLOAD_MIN             0.0         // Minimum payload
#define ALTITUDE_MAX            POSITION_Z_MAX       // Maximum altitude
#define ALTITUDE_MIN            POSITION_Z_MIN        // Minimum altitude
#define FLIGHT_CAT_MAX          277.0       // Maximum flight category
#define FLIGHT_CAT_MIN          1.0         // Minimum flight category

// power actuator bounds
#define POWER_ACTUATOR_MAX      1005.0      // Maximum actuator power                
#define POWER_ACTUATOR_MIN      0.0         // Minimum actuator power  

// power sensor dataset
#define FPS                 std::get<0>     // FPS
#define PIXELS              std::get<2>     // Number of pixels
#define PIX_X               std::get<3>     // Pixel x
#define PIX_Y               std::get<4>     // Pixel y
#define POWER_SENSOR        std::get<1>     // Power sensor

// power sensor dataset bounds
#define FPS_MAX             90.0           // Maximum FPS
#define FPS_MIN             30.0            // Minimum FPS
#define PIXELS_MAX          2240000         //2240000.0 // Maximum number of pixels
#define PIXELS_MIN          307200          //307200.0 // Minimum number of pixels
#define PIX_X_MAX           1600.0          // Maximum pixel x
#define PIX_X_MIN           640.0           // Minimum pixel x
#define PIX_Y_MAX           1400.0          // Maximum pixel y
#define PIX_Y_MIN           480.0           // Minimum pixel y
#define POWER_SENSOR_MAX    6.0             // Maximum power sensor
#define POWER_SENSOR_MIN    5.0             // Minimum power sensor

// folding 
#define CAMERA_THETA_DEG                          40.0                                                   // Camera theta in degrees
#define CAMERA_THETA_RAD                          (CAMERA_THETA_DEG * M_PI / 180.0)                           // Camera theta in radians
#define TAN_CAMERA_THETA                          (std::tan(CAMERA_THETA_RAD))                                // tan camera theta
#define CAMERA_OVERLAP_FACTOR                     0.03
#define OVERLAP_FACTOR                            2*TAN_CAMERA_THETA*CAMERA_OVERLAP_FACTOR

// resolution 
//#define SAMPEL_PIXEL_SIZE 32.0 // minimum
//#define SAMPEL_SIZE_CM 5.0 //sample : (5cmx5cm)/(32x32) 
#define SAMPEL_SIZE_M SAMPEL_SIZE_CM/100.0
#define RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX ceil(((SAMPEL_SIZE_M*SAMPEL_SIZE_M)/(SAMPEL_PIXEL_SIZE*SAMPEL_PIXEL_SIZE))*1000000)/1000000  
#define RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN floor(((SAMPEL_SIZE_M*SAMPEL_SIZE_M)/(SAMPEL_PIXEL_SIZE*SAMPEL_PIXEL_SIZE))*1000000)/1000000   
//#define SHUTTER_SPEED 2000
#define GSD SAMPEL_SIZE_M/SAMPEL_PIXEL_SIZE
#define MAX_V_CAPTURING floor((GSD/3)*SHUTTER_SPEED)
#define MIN_V_CAPTURING MAX_V_CAPTURING - 1

// drone constraints
#define DRONE_MASS          2.43        // Drone mass in kg
#define MASS_MAX            3.5         // Maximum mass
#define MASS_MIN            0.0         // Minimum mass
//#define DRONE_SET_PIX       {307200,1433600, 2240000}
//#define DRONE_SET_PIX_X     {640,1280 ,1600 }
//#define DRONE_SET_PIX_Y     {480,1120 ,1400 }
//#define DRONE_SET_FPS       {30, 60, 90} 
//#define DRONE_ENERGY_CAPACITY {0,0,0,0,0}

#define COVERED_AREA_X_MAX      (2*TAN_CAMERA_THETA*ALTITUDE_MAX)     // Maximum area covered in x direction in m
#define COVERED_AREA_X_MIN      0.0                                           //(CONST_2_TAN_CAMERA_THETA*ALTITUDE_MIN) // Minimum area covered in x direction in m
#define COVERED_AREA_TOTAL_MAX  (COVERED_AREA_X_MAX * COVERED_AREA_X_MAX)   // Maximum total area covered in m^2
#define COVERED_AREA_TOTAL_MIN  (COVERED_AREA_X_MIN * COVERED_AREA_X_MIN)   // Minimum total area covered in m^2

#define NUMBER_PLACE_COVERED_MAX    GRB_INFINITY
#define NUMBER_PLACE_COVERED_MIN    (FIELD_AREA / COVERED_AREA_TOTAL_MAX)
#define COVERED_DISTANCE_MAX        GRB_INFINITY
#define COVERED_DISTANCE_MIN        COVERED_AREA_X_MIN
#define OPERATION_TIME_MAX          GRB_INFINITY            //(COVERED_DISTANCE_MAX / SPEED_MIN) // Maximum operation time in seconds
#define OPERATION_TIME_MIN          0                       //Minimum operation time in seconds

// helperfunctions
#define PRINT_LOG(len, var,val) std::cout << std::left << std::setw(len) << var << ":\t" << val << std::endl; 
#define PRINT_LOG_NEW_LINE std::cout << std::endl; 

#endif // SIM_PARAM




