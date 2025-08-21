from cmath import tan
from math import ceil, floor




pa_eta =      -0.0634
pa_delta =      0.16
pa_alpha =     0.066
pa_beta =      0.0513
pa_phi =      0.5
pa_theta =     -15.7754
pa_C_0 =      195.304
pa_C_1 =      389.767
pa_C_2 =      -1.56176
pa_C_3 =      -1.04117
pa_C_4 =      -1.96654
ps_a =      -0.0362
ps_b =      -0.0387
ps_c =      -0.0036


FPS_MAX = 90.0
FPS_MIN = 30.0

CAMERA_THETA_DEG = (80.0/2)
CAMERA_THETA_RAD = (CAMERA_THETA_DEG * 3.141592653589793 / 180.0)
TAN_CAMERA_THETA = tan(CAMERA_THETA_RAD)
CONST_2_TAN_CAMERA_THETA = 2.0 * TAN_CAMERA_THETA

FIELD_AREA = 80000

H_REF = 258
H_REF_INV = 1/H_REF

t_1_60 = 1.0 / 60.0

CHARGING_TIME = 1200

SAMPEL_PIXEL_SIZE = 32.0 
SAMPEL_SIZE_CM = 20.0 
SAMPEL_SIZE_M = SAMPEL_SIZE_CM/100.0
RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX = ceil(((SAMPEL_SIZE_M*SAMPEL_SIZE_M)/(SAMPEL_PIXEL_SIZE*SAMPEL_PIXEL_SIZE))*1000000)/1000000  
RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN = floor(((SAMPEL_SIZE_M*SAMPEL_SIZE_M)/(SAMPEL_PIXEL_SIZE*SAMPEL_PIXEL_SIZE))*1000000)/1000000  

GSD = SAMPEL_SIZE_M/SAMPEL_PIXEL_SIZE
SHUTTER_SPEED = 2000
MAX_V_CAPTURING = floor((GSD/3)*SHUTTER_SPEED)



pa_exprs = 0
ps_exprs = 0
drone_used_total = 0

drone_set_pix = [307200, 1433600, 2240000]
drone_set_pix_x = [640, 1280, 1600]
drone_set_pix_y = [480, 1120, 1400]
drone_set_pix_normalized = [0,0,0] 
for i in range(len(drone_set_pix)):
    drone_set_pix_normalized[i] = (drone_set_pix[i] - drone_set_pix[0]) / (drone_set_pix[-1] - drone_set_pix[0])


def E_BAT_WH(c_mAh, v):
    return ((c_mAh*v)/1000)*6000000000000

drone_energy_capacity = E_BAT_WH(4500,22.2)
drone_energy_capacity_used = 0

drone_set_fps = [30,60,90]
drone_set_fps_normalized = [0,0,0]

for i in range(len(drone_set_fps_normalized)):
    drone_set_fps_normalized[i] = (drone_set_fps[i] - drone_set_fps[0]) / (drone_set_fps[-1] - drone_set_fps[0])








################################################



wp_v_wind = 0       
wp_theta_wind = 0
speed_factor = 1.0
drone_h = 2.0
pix_select = 1
fps_select = 1

sensor_fps = drone_set_fps[1] 
sensor_fps_2 = sensor_fps**2
sensor_fps_true = sensor_fps * ( FPS_MAX - FPS_MIN) + FPS_MIN
drone_v = MAX_V_CAPTURING*speed_factor
drone_v_2 = drone_v**2
drone_v_3 = drone_v**3
sensor_pix = drone_set_pix_normalized[pix_select]
sensor_pix_true = drone_set_pix[pix_select]
sensor_pix_2 = sensor_pix**2
sensor_pix_x = drone_set_pix_x[pix_select]
sensor_pix_y= drone_set_pix_y[pix_select]
sensor_fps_pix= sensor_fps * sensor_pix
covered_area_x_t0 = CONST_2_TAN_CAMERA_THETA * drone_h
covered_area_y_t0 = (covered_area_x_t0* sensor_pix_y)/sensor_pix_x
covered_area_total_t0 = covered_area_x_t0 * covered_area_y_t0
covered_area_total = FIELD_AREA
covered_area_true = covered_area_total
number_of_place_covered = covered_area_total/covered_area_total_t0
covered_distance = (covered_area_x_t0 * number_of_place_covered) - covered_area_x_t0
operation_time = covered_distance/drone_v
pa_exprs = 0
pa_exprs += pa_C_0
pa_exprs += pa_C_1 * drone_v
pa_exprs += pa_C_2 * drone_v_2
pa_exprs += pa_C_3 * drone_v_3
pa_exprs += pa_C_4 * drone_h*H_REF_INV
drone_pa_consumption = pa_exprs
ps_exprs = 0
ps_exprs += ps_a*sensor_fps
ps_exprs += ps_b*sensor_pix
ps_exprs += ps_c
drone_ps_consumption = ps_exprs
drone_energy_consumption = (operation_time*t_1_60) * (drone_pa_consumption + drone_ps_consumption)
charging_cycles = (drone_energy_consumption + drone_energy_capacity_used)/drone_energy_capacity
operation_time_req = operation_time + CHARGING_TIME*(charging_cycles-1)   
OBJECTIVE_ENERGY_WEIGHT = 1
OBJECTIVE_DRONE_WEIGHT = 1
OBJECTIVE_TIME_WEIGHT = 1
OBJECTIVE_CHARGING_CYCLE_WEIGHT = 1
total_energy_consumed = drone_energy_consumption
operation_time_total = operation_time_req
charging_cycle_total = charging_cycles
objective_expr = OBJECTIVE_ENERGY_WEIGHT*total_energy_consumed + OBJECTIVE_DRONE_WEIGHT*drone_used_total + OBJECTIVE_TIME_WEIGHT*operation_time_total + OBJECTIVE_CHARGING_CYCLE_WEIGHT*charging_cycle_total


print("charging cycle: " + str(charging_cycles))
print("drone_energy_consumption: " + str(drone_energy_consumption))
print("operation_time_req: " + str(operation_time_req))
print("drone_v: " + str(drone_v))

print("drone_pa_consumption: " + str(drone_pa_consumption))
print("drone_ps_consumption: " + str(drone_ps_consumption))

print("covered_area_x_t0: " + str(covered_area_x_t0))
print("covered_area_total_t0: " + str(covered_area_total_t0))
print("number_of_place_covered: " + str(number_of_place_covered))
print("covered_distance: " + str(covered_distance))