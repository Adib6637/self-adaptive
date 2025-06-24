import numpy as np
import math

# Example coefficients (replace with your set)

data = [0.31961217051327650562342341800104, 0.025757735694765533651651168156604, 0.097316016522270870625632710471109, 0.0020472216049019411471254770162886, 1.6310688411653309071169815069879, 31.594803680796367473249119939283, -6.1650594808696199677910954051185]
pa_eta = data[0]
pa_delta = data[1]
pa_alpha = data[2]
pa_beta = data[3]
ps_a = data[4]
ps_b = data[5]
ps_c = data[6]



# Constants (replace with your values)
FIELD_AREA = 1000000000.0
GRAVITY = 9.81
DRONE_MASS = 3.6
MASS_MAX = 10
LINEAR_ACCELERATION_Z_MAX = -9.6
LINEAR_ACCELERATION_Z_MIN = -10.0
PIXELS_MIN = 200000
PIXELS_MAX = 3000000
FPS_MIN = 15.0
FPS_MAX = 144.0
SPEED_MIN = 4.0
SPEED_MAX = 12.0
ALTITUDE_MIN = 0.0
ALTITUDE_MAX = 100.0
POWER_ACTUATOR_MAX = 800.0
POWER_ACTUATOR_MIN = 350.0
POWER_SENSOR_MAX = 6.0
POWER_SENSOR_MIN = 5.0

PIXELS_x_MIN = 640
PIXELS_x_MAX = 1600
PIXELS_y_MIN = 4880
PIXELS_y_MAX = 1400

COVERED_AREA_X_MIN = 1e-8
COVERED_AREA_TOTAL_MIN = COVERED_AREA_X_MIN * COVERED_AREA_X_MIN
NUMBER_PLACE_COVERED_MAX = FIELD_AREA / COVERED_AREA_TOTAL_MIN
CONST_2_TAN_CAMERA_THETA = 2.0 * np.tan(np.deg2rad(80.0 / 2))  # 2 * tan camera theta
COVERED_AREA_X_MAX = CONST_2_TAN_CAMERA_THETA*ALTITUDE_MAX
COVERED_DISTANCE_MAX = COVERED_AREA_X_MAX*NUMBER_PLACE_COVERED_MAX - COVERED_AREA_X_MAX


OPERATION_TIME_MIN = 0.0
OPERATION_TIME_MAX = COVERED_DISTANCE_MAX / SPEED_MIN
CHARGING_TIME = 1200

def actuator_power(v, h, phi=1.0):
    pa_theta = 1/(pa_eta + 1e-8)
    pa_C_0 = pa_theta * (((GRAVITY-(LINEAR_ACCELERATION_Z_MAX*(-1)))/((LINEAR_ACCELERATION_Z_MIN*(-1))-(LINEAR_ACCELERATION_Z_MAX*(-1)))) * ((DRONE_MASS)/MASS_MAX)) * phi + pa_theta*pa_alpha*phi**3 + pa_delta
    pa_C_1 = pa_theta * (((GRAVITY-(LINEAR_ACCELERATION_Z_MAX*(-1)))/((LINEAR_ACCELERATION_Z_MIN*(-1))-(LINEAR_ACCELERATION_Z_MAX*(-1)))) * ((DRONE_MASS)/MASS_MAX)) + 3*pa_theta*pa_alpha*phi**2
    pa_C_2 = 3 * pa_theta * pa_alpha * phi
    pa_C_3 = pa_theta * pa_alpha
    pa_C_4 = pa_theta * pa_beta * ((DRONE_MASS)/MASS_MAX)
    return pa_C_0 + pa_C_1*v + pa_C_2*v**2 + pa_C_3*v**3 + pa_C_4*h

def sensor_power(fps, pix):
    return ps_a*fps + ps_b*pix + ps_c

def energy_consumption(op_time, pa, ps):
    return op_time * (pa+ps)

def main():
    print('Enter values for speed, height, fps, and pixel:')
    try:
        v = 12.0 #float(input(f'Speed [{SPEED_MIN}-{SPEED_MAX}]: '))
        h = 100.0 #float(input(f'Height [{ALTITUDE_MIN}-{ALTITUDE_MAX}]: '))
        f = 90.0 #float(input(f'FPS [{FPS_MIN}-{FPS_MAX}]: '))
        p_x = 1600 #float(input(f'Pixel [{PIXELS_x_MIN}-{PIXELS_x_MAX}]: '))
        p_y = 1400 #float(input(f'Pixel [{PIXELS_y_MIN}-{PIXELS_y_MAX}]: '))
    except Exception as e:
        print('Invalid input:', e)
        return
    
    p = ((p_x*p_y) - math.log(200000))/(math.log(3000000000) - math.log(200000))
    v_norm = (v - SPEED_MIN) / (SPEED_MAX - SPEED_MIN)
    h_norm = (h - ALTITUDE_MIN) / (ALTITUDE_MAX - ALTITUDE_MIN)
    f_norm = (f - FPS_MIN) / (FPS_MAX - FPS_MIN)
    p_norm = (p - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN)

    pa = actuator_power(v_norm, h_norm)* (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN) + POWER_ACTUATOR_MIN
    ps = sensor_power(f_norm, p_norm)* (POWER_SENSOR_MAX - POWER_SENSOR_MIN) + POWER_SENSOR_MIN


    ratio_p_x_y = p_x/p_y


    covered_area_x_t0 = CONST_2_TAN_CAMERA_THETA * (h * ( ALTITUDE_MAX - ALTITUDE_MIN ) + ALTITUDE_MIN)
    covered_area_total = FIELD_AREA
    covered_area_y_t0 = 1/ratio_p_x_y * covered_area_x_t0
    covered_area_total_t0 = covered_area_x_t0 * covered_area_y_t0
    number_of_place_covered = covered_area_total * (1/covered_area_total_t0)
    covered_distance = (covered_area_x_t0 * number_of_place_covered) - covered_area_x_t0
    op_time = covered_distance/v
    energy = energy_consumption(op_time, pa, ps)
    feasible = (pa >= 0) and (ps >= 0) and (energy > 0)
    
    # Normalize variables as in optimizer.cpp
    v_norm = (v - SPEED_MIN) / (SPEED_MAX - SPEED_MIN)
    h_norm = (h - ALTITUDE_MIN) / (ALTITUDE_MAX - ALTITUDE_MIN)
    f_norm = (f - FPS_MIN) / (FPS_MAX - FPS_MIN)
    p_norm = (p - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN)

    # Constraints mimicking optimizer.cpp
    constraints = []
    if not (0.0 <= v_norm <= 1.0):
        constraints.append('Speed normalization out of bounds')
    if not (0.0 <= h_norm <= 1.0):
        constraints.append('Height normalization out of bounds')
    if not (0.0 <= f_norm <= 1.0):
        constraints.append('FPS normalization out of bounds')
    if not (0.0 <= p_norm <= 1.0):
        constraints.append('Pixel normalization out of bounds')
    if not (POWER_ACTUATOR_MIN <= pa <= POWER_ACTUATOR_MAX):
        constraints.append('Actuator power out of bounds')
    if not (POWER_SENSOR_MIN <= ps <= POWER_SENSOR_MAX):
        constraints.append('Sensor power out of bounds')
    if not (energy > 0):
        constraints.append('Energy consumption not positive')
    # Add more constraints as needed to match optimizer.cpp

    if constraints:
        print('--- Constraint Violations ---')
        for c in constraints:
            print(c)
    else:
        print('All constraints satisfied.')
    
    print(f'--- Results ---')
    print(f'Speed: {v:.2f} | Height: {h:.2f} | FPS: {f:.2f} | Pixel: {p:.2f}')
    print(f'covered_area_x_t0: {covered_area_x_t0:.2f} m')
    print(f'covered_area_y_t0: {covered_area_y_t0:.2f} m')
    print(f'covered_area_total_t0: {covered_area_total_t0:.2f} m^2')
    print(f'Number of places covered: {number_of_place_covered:.2f}')
    print(f'Covered Distance: {covered_distance:.2f} m')
    print(f'çovered_area_total: {covered_area_total:.2f} m^2')
    print(f'Actuator Power: {pa:.2f} W')
    print(f'Sensor Power: {ps:.2f} W')
    print(f'Operation Time: {op_time:.2f} min')
    print(f'Energy Consumption: {energy:.2f} Wh')
    print(f'Feasible: {"YES" if feasible else "NO"}')

if __name__ == '__main__':
    main()
