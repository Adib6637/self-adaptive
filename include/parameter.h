#ifndef SIM_PARAM 
#define SIM_PARAM

///////////////////////////////////////////////////////////////////////////////////// case dependent //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SIMULATION_CASE 1
#define SIMULATION_SUB_CASE 1

#define AREA_DEFAULT            80000.0 
#define LIMIT_AREA_TEST         50000                       // only for case 5  {640,1280 ,1600 }
#define DRONE_SET_PIX_TEST      {307200,1433600, 2240000}   // only for case 6  {640,1280 ,1600 }
#define DRONE_SET_PIX_X_TEST    {640,1280 ,1600 }           // only for case 6  {640,1280 ,1600 }
#define DRONE_SET_PIX_Y_TEST    {480,1120 ,1400 }           // only for case 6  {480,1120 ,1400 } 
#define DRONE_SET_FPS_TEST      {30, 60, 90}                // only for case 6  {30, 60, 90}  

#define OPTIMIZER_GAP 0.1


#define SIMULATION_DURATION_NS  2200
#define OPTIMIZER_ON            true
#define MODEL_LEARNER_ON        true
#define MANAGED_SYSTEM_ON       true
#define DYNAMIC_WEATHER         false


#if SIMULATION_CASE == 1    // dynamic model, static weather
    #define DYNAMIC_WEATHER         false
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           {307200,1433600, 2240000}
    #define DRONE_SET_PIX_X         {640,1280 ,1600 }
    #define DRONE_SET_PIX_Y         {480,1120 ,1400 }
    #define DRONE_SET_FPS           {30, 60, 90} 
#elif SIMULATION_CASE == 2  // static model, dynamic weather
    #define DYNAMIC_WEATHER         true
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           {307200,1433600, 2240000}
    #define DRONE_SET_PIX_X         {640,1280 ,1600 }
    #define DRONE_SET_PIX_Y         {480,1120 ,1400 }
    #define DRONE_SET_FPS           {30, 60, 90} 
#elif SIMULATION_CASE == 3  // dynamic model, dynamic weather
    #define DYNAMIC_WEATHER         true
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           {307200,1433600, 2240000}
    #define DRONE_SET_PIX_X         {640,1280 ,1600 }
    #define DRONE_SET_PIX_Y         {480,1120 ,1400 }
    #define DRONE_SET_FPS           {30, 60, 90} 
#elif SIMULATION_CASE == 4  // static model, static weather
    #define DYNAMIC_WEATHER         true
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        true
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           {307200,1433600, 2240000}
    #define DRONE_SET_PIX_X         {640,1280 ,1600 }
    #define DRONE_SET_PIX_Y         {480,1120 ,1400 }
    #define DRONE_SET_FPS           {30, 60, 90} 
#elif SIMULATION_CASE == 5  // static model, static weather , limit
    #define DYNAMIC_WEATHER         false
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              LIMIT_AREA_TEST
    #define DRONE_SET_PIX           {307200,1433600, 2240000}
    #define DRONE_SET_PIX_X         {640,1280 ,1600 }
    #define DRONE_SET_PIX_Y         {480,1120 ,1400 }
    #define DRONE_SET_FPS           {30, 60, 90} 
#elif SIMULATION_CASE == 6  // static model, static weather
    #define DYNAMIC_WEATHER         false
    #define MANAGED_SYSTEM_ON       true
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            true
    #define FIELD_AREA              AREA_DEFAULT
    #define DRONE_SET_PIX           DRONE_SET_PIX_TEST
    #define DRONE_SET_PIX_X         DRONE_SET_PIX_X_TEST
    #define DRONE_SET_PIX_Y         DRONE_SET_PIX_Y_TEST
    #define DRONE_SET_FPS           DRONE_SET_FPS_TEST



#else
    #define DYNAMIC_WEATHER         false
    #define MANAGED_SYSTEM_ON       false
    #define MODEL_LEARNER_ON        false
    #define OPTIMIZER_ON            false
    #define FIELD_AREA              0.0
#endif



#if SIMULATION_SUB_CASE == 1
    #define NUMBER_DRONE_MAX        5
    #define DYNAMIC_DRONE           true
#elif SIMULATION_SUB_CASE == 2
    #define NUMBER_DRONE_MAX        2
    #define DYNAMIC_DRONE           false
#elif SIMULATION_SUB_CASE == 3
    #define NUMBER_DRONE_MAX        1
    #define DYNAMIC_DRONE           false
#else
    #define NUMBER_DRONE_MAX        5
    #define DYNAMIC_DRONE           true
#endif        





#define OPTIMIZE_INTERVAL                   4      // Interval for optimization in number of model parameter updates  (FIX_MODEL_PARAMETER is not defined)
#define OPTIMIZE_WEATHER_CHANGE_INTERVAL    4      // Interval for optimization in number of weather changes (FIX_MODEL_PARAMETER is defined AND DYNAMIC_WEATHER is true)


#define LEARNING_PROGRESS_LOG   
#define LEARNING_TIMING_LOG
#define OPTIMIZATION_TIMING_LOG
#define OPTIMIZATION_RESULT_LOG 
#define PRINT_OPTIMIZATION_RESULTS
//#define PRINT_GUROBI_OUTPUT_FLAG
#define WEATHER_FORECAST_LOG


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// simulation time
#define SIMULATION_CLK_TICK_NS  10
//#define SIMULATION_DURATION_NS  10000 //2200

// simulation component
//#define OPTIMIZER_ON            true
//#define MODEL_LEARNER_ON        true
//#define MANAGED_SYSTEM_ON       true
//#define DYNAMIC_WEATHER         false
#define CONSTRAINT_TUNER_ON     false
#define SUCCESS_LIMIT           45
#if !MODEL_LEARNER_ON
#define FIX_MODEL_PARAMETER
#endif
//#define OPTIMIZE_INTERVAL                   4      // Interval for optimization in number of model parameter updates  (FIX_MODEL_PARAMETER is not defined)
//#define OPTIMIZE_WEATHER_CHANGE_INTERVAL    4      // Interval for optimization in number of weather changes (FIX_MODEL_PARAMETER is defined AND DYNAMIC_WEATHER is true)

// simulation environment 
//#define NUMBER_DRONE_MAX        5               // Number of drones
//#define FIELD_AREA              30000.0       // Field area in m^2  https://sugarindustry.info/paper/15166/
//#define FIELD_AREA_INV          (1.0 / FIELD_AREA) // Inverse of field area
#define GRAVITY                 9.81            // Gravity constant in m/s^2
#define H_REF                   100.0           // Reference height in meters

// logging (define to enable) 
//#define LEARNING_PROGRESS_LOG   
//#define LEARNING_TIMING_LOG
//#define OPTIMIZATION_TIMING_LOG
//#define OPTIMIZATION_RESULT_LOG 
//#define PRINT_OPTIMIZATION_RESULTS
//#define PRINT_GUROBI_OUTPUT_FLAG
//#define WEATHER_FORECAST_LOG



// optimization objective
#define OBJECTIVE_ENERGY_WEIGHT 1.0
#define OBJECTIVE_DRONE_WEIGHT  1000000.0
#define OBJECTIVE_TIME_WEIGHT   1000.0

// optimization parameters
#define SET_GUROBI_SOLVER_PARAMS(model)                \
    model.set(GRB_DoubleParam_MIPGap, OPTIMIZER_GAP);           \
    /*model.set(GRB_DoubleParam_TimeLimit, 60);*/      \
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
#define FLIGHT_CAT              std::get<0>(data) 

// power actuator dataset bounds
#define BATTERY_VOLTAGE_MAX     26.0        // Maximum battery voltage
#define BATTERY_VOLTAGE_MIN     19.0        // Minimum battery voltage
#define BATTERY_CURRENT_MAX     40          // Maximum battery current             
#define BATTERY_CURRENT_MIN     18          // Minimum battery current             
#define WIND_SPEED_MAX          18.0        // Maximum wind speed
#define WIND_SPEED_MIN          0.0         // Minimum wind speed
#define WIND_ANGLE_MAX          360.0       // Maximum wind angle
#define WIND_ANGLE_MIN          0.0         // Minimum wind angle
//#define POSITION_X_MAX        100.0       // Maximum position x
//#define POSITION_X_MIN       -100.0       // Minimum position x
//#define POSITION_Y_MAX        100.0       // Maximum position y
//#define POSITION_Y_MIN       -100.0       // Minimum position y
//#define POSITION_Z_MAX        100.0       // Maximum position z
//#define POSITION_Z_MIN       -100.0       // Minimum position z
#define ORIENTATION_X_MAX       1.0         // Maximum orientation x
#define ORIENTATION_X_MIN      -1.0         // Minimum orientation x
#define ORIENTATION_Y_MAX       1.0         // Maximum orientation y
#define ORIENTATION_Y_MIN      -1.0         // Minimum orientation y
#define ORIENTATION_Z_MAX       1.0         // Maximum orientation z
#define ORIENTATION_Z_MIN      -1.0         // Minimum orientation z
#define ORIENTATION_W_MAX       1.0         // Maximum orientation w
#define ORIENTATION_W_MIN      -1.0         // Minimum orientation w
//#define VELOCITY_X_MAX        100.0       // Maximum velocity x
//#define VELOCITY_X_MIN      -100.0        // Minimum velocity x
//#define VELOCITY_Y_MAX        100.0       // Maximum velocity y
//#define VELOCITY_Y_MIN       -100.0       // Minimum velocity y
//#define VELOCITY_Z_MAX        100.0       // Maximum velocity z
//#define VELOCITY_Z_MIN       -100.0       // Minimum velocity z
//#define ANGULAR_X_MAX         10.0        // Maximum angular x
//#define ANGULAR_X_MIN        -10.0        // Minimum angular x
//#define ANGULAR_Y_MAX         10.0        // Maximum angular y
//#define ANGULAR_Y_MIN        -10.0        // Minimum angular y
//#define ANGULAR_Z_MAX         10.0        // Maximum angular z
//#define ANGULAR_Z_MIN        -10.0        // Minimum angular z
//#define LINEAR_ACCELERATION_X_MAX     10.0// Maximum linear acceleration x
//#define LINEAR_ACCELERATION_X_MIN    -10.0// Minimum linear acceleration x
//#define LINEAR_ACCELERATION_Y_MAX     10.0// Maximum linear acceleration y
//#define LINEAR_ACCELERATION_Y_MIN    -10.0// Minimum linear acceleration y
#define LINEAR_ACCELERATION_Z_MAX      -9.6 // Maximum linear acceleration z
#define LINEAR_ACCELERATION_Z_MIN      -10.0// Minimum linear acceleration z
#define SPEED_MAX               12.0        // Maximum speed
#define SPEED_MIN               0.0         // Minimum speed
#define PAYLOAD_MAX             750.0       // Maximum payload
#define PAYLOAD_MIN             0.0         // Minimum payload
#define ALTITUDE_MAX            100.0       // Maximum altitude
#define ALTITUDE_MIN            0.0         // Minimum altitude
#define FLIGHT_CAT_MAX          100.0       // Maximum flight category
#define FLIGHT_CAT_MIN          0.0         // Minimum flight category

// power actuator bounds
#define POWER_ACTUATOR_MAX      800.0       // Maximum actuator power                
#define POWER_ACTUATOR_MIN      350.0       // Minimum actuator power  

// power sensor dataset
#define FPS                 std::get<0>     // FPS
#define PIXELS              std::get<2>     // Number of pixels
#define PIX_X               std::get<3>     // Pixel x
#define PIX_Y               std::get<4>     // Pixel y
#define POWER_SENSOR        std::get<1>     // Power sensor

// power sensor dataset bounds
#define FPS_MAX             144.0           // Maximum FPS
#define FPS_MIN             15.0            // Minimum FPS
#define PIXELS_MAX          3000000         //2240000.0 // Maximum number of pixels
#define PIXELS_MIN          200000          //307200.0 // Minimum number of pixels
#define PIX_X_MAX           1600.0          // Maximum pixel x
#define PIX_X_MIN           640.0           // Minimum pixel x
#define PIX_Y_MAX           1400.0          // Maximum pixel y
#define PIX_Y_MIN           480.0           // Minimum pixel y
#define PIXELS_MAX_LOG      (std::log10(PIXELS_MAX + 1e-8))             // Logarithm of maximum pixels with a small offset to avoid log(0)  
#define PIXELS_MIN_LOG      (std::log10(PIXELS_MIN + 1e-8))             // Logarithm of minimum pixels with a small offset to avoid log(0)  
#define PIXELS_NORMALIZED_DENOMINATOR (PIXELS_MAX_LOG - PIXELS_MIN_LOG) // Normalization denominator for pixels            
#define LOG_PIXEL(x)        (std::log10((x) + 1e-8)) // Logarithm of pixel value with a small offset to avoid log(0)  
#define POWER_SENSOR_MAX    6.0             // Maximum power sensor
#define POWER_SENSOR_MIN    5.0             // Minimum power sensor

// folding 
#define CAMERA_THETA_DEG    (80.0/2)                                    // Camera theta in degrees
#define CAMERA_THETA_RAD    (CAMERA_THETA_DEG * M_PI / 180.0)           // Camera theta in radians
#define TAN_CAMERA_THETA    (std::tan(CAMERA_THETA_RAD))                // tan camera theta
#define TAN_CAMERA_THETA_INV            (1.0 / TAN_CAMERA_THETA)        // Inverse of tan camera theta
#define CONST_2_TAN_CAMERA_THETA        (2.0 * TAN_CAMERA_THETA)        // 2 * tan camera theta
#define CONST_2_TAN_CAMERA_THETA_INV    (1.0 / CONST_2_TAN_CAMERA_THETA)// Inverse of 2 * tan camera theta
#define CONST_8_TAN_3_THETA_CAMERA      (CONST_2_TAN_CAMERA_THETA * CONST_2_TAN_CAMERA_THETA * CONST_2_TAN_CAMERA_THETA) // 8 * tan^3 camera theta
#define CONST_8_TAN_3_THETA_CAMERA_INV  (1.0 / CONST_8_TAN_3_THETA_CAMERA)// Inverse of 8 * tan^3 camera theta

// resolution 
#define RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX 0.000004
#define RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN 0.000002   //sample : (5cmx5cm)/(32x32)

// drone constraints
#define DRONE_MASS          3.680       // Drone mass in kg
#define MASS_MAX            10.0        // Maximum mass
#define MASS_MIN            0.0         // Minimum mass
//#define DRONE_SET_PIX       {307200,1433600, 2240000}
//#define DRONE_SET_PIX_X     {640,1280 ,1600 }
//#define DRONE_SET_PIX_Y     {480,1120 ,1400 }
//#define DRONE_SET_FPS       {30, 60, 90} 

#define COVERED_AREA_X_MAX      (CONST_2_TAN_CAMERA_THETA*ALTITUDE_MAX)     // Maximum area covered in x direction in m
#define COVERED_AREA_X_MIN      1e-8                                        //(CONST_2_TAN_CAMERA_THETA*ALTITUDE_MIN) // Minimum area covered in x direction in m
#define COVERED_AREA_TOTAL_MAX  (COVERED_AREA_X_MAX * COVERED_AREA_X_MAX)   // Maximum total area covered in m^2
#define COVERED_AREA_TOTAL_MIN  (COVERED_AREA_X_MIN * COVERED_AREA_X_MIN)   // Minimum total area covered in m^2

#define NUMBER_PLACE_COVERED_MAX    (FIELD_AREA / COVERED_AREA_TOTAL_MIN)
#define NUMBER_PLACE_COVERED_MIN    (FIELD_AREA / COVERED_AREA_TOTAL_MAX)
#define COVERED_DISTANCE_MAX        (COVERED_AREA_X_MAX*NUMBER_PLACE_COVERED_MAX - COVERED_AREA_X_MAX)
#define COVERED_DISTANCE_MIN        COVERED_AREA_X_MIN
#define OPERATION_TIME_MAX          GRB_INFINITY            //(COVERED_DISTANCE_MAX / SPEED_MIN) // Maximum operation time in seconds
#define OPERATION_TIME_MIN          0                       //Minimum operation time in seconds

#define OPERATION_MAX_PER_CHARGING      1800
#define OPERATION_MAX_PER_CHARGING_INV  (1.0 / OPERATION_MAX_PER_CHARGING) // Inverse of maximum operation time per charging in seconds
#define CHARGING_TIME                   1200
#define MAX_CHARGING_CYCLE              5

#endif // SIM_PARAM



