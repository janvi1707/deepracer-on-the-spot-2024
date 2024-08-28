import math
import numpy as np
class PARAMS:
    prev_speed = None
    prev_steering_angle = None 
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None

def smooth_central_line(center_line, max_offset, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
    if max_offset < 0.0001:
        return center_line
    if skip_step < 1:
        skip_step = 1
    smoothed_line = center_line
    for i in range(0, iterations):
        smoothed_line = smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
    return smoothed_line
def circle_radius(x1, y1, x2, y2, x3, y3):
    a = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2
    b = (x1**2+y1**2)*(y3-y2) + (x2**2+y2**2)*(y1-y3) + (x3**2+y3**2)*(y2-y1)
    c = (x1**2+y1**2)*(x2-x3) + (x2**2+y2**2)*(x3-x1) + (x3**2+y3**2)*(x1-x2)
    d = (x1**2+y1**2)*(x3*y2-x2*y3) + (x2**2+y2**2) * \
        (x1*y3-x3*y1) + (x3**2+y3**2)*(x2*y1-x1*y2)

    # In case a is zero (so radius is infinity)
    try:
        r = abs((b**2+c**2-4*a*d) / abs(4*a**2)) ** 0.5
    except:
        r = 999

    return r
def generate_optimal_speed_array(track_points):
    list =[]
    length= len(track_points)
    for i in range(0,length):
        next_point_1 = track_points[i]
        next_point_2 = track_points[(i+1)%length]
        prev_point_1 = track_points[(i-1+length)%length]
        list.append(min(1.69*math.sqrt(circle_radius(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],prev_point_1[0],prev_point_1[1])),4))
    return np.array(list)
def smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step):
    length = len(center_line)
    new_line = [[0.0 for _ in range(2)] for _ in range(length)]
    for i in range(0, length):
        wpp = smoothed_line[(i - 2 * skip_step + length) % length]
        wp = smoothed_line[(i - skip_step + length) % length]
        wc = smoothed_line[i]
        wn = smoothed_line[(i + skip_step) % length]
        wnn = smoothed_line[(i + 2 * skip_step) % length]
        new_line[i][0] = pp * wpp[0] + p * wp[0] + c * wc[0] + n * wn[0] + nn * wnn[0]
        new_line[i][1] = pp * wpp[1] + p * wp[1] + c * wc[1] + n * wn[1] + nn * wnn[1]
        while calc_distance(new_line[i], center_line[i]) >= max_offset:
            new_line[i][0] = (0.98 * new_line[i][0]) + (0.02 * center_line[i][0])
            new_line[i][1] = (0.98 * new_line[i][1]) + (0.02 * center_line[i][1])
    return new_line
def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.hypot(delta_x, delta_y)
def eucledian_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
def angle(x1,y1,x2,y2):
    return math.degrees(math.atan2(y2-y1,x2-x1))
def closest_route_point(x,y,optimal_waypoints):
    opt_len = len(optimal_waypoints)
    list = []
    for i in range(0,opt_len):
        list.append((eucledian_distance(x,y,optimal_waypoints[i][0],optimal_waypoints[i][1]),optimal_waypoints[i][0],optimal_waypoints[i][1],i))
    dist = sorted(list, key = lambda x: x[0])
    print("Two closest points:",dist[0],dist[1])
    if dist[0][3] < dist[1][3] or dist[1][3]==0:
        print("Closest optimal point :", dist[0])
        return dist[0][3],dist[1][3]
    else:
        print("Closest optimal point :",dist[1])
        return dist[1][3],dist[0][3]
def normalized_distance(prev_point,next_point,vehicle_x,vehicle_y):
    x1=prev_point[0]
    y1=prev_point[1]
    x2=next_point[0]
    y2=next_point[1]
    x0=vehicle_x
    y0=vehicle_y
    return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/ np.sqrt(np.square(x2-x1) + np.square(y2-y1))
def reward_function(params):
    curve_points=[17,18,19,20,21,22,23,24,25,26,27,28,29,30,54,55,56,57,58,59,60,61,62,97,98,99,100,101,102,103,104,105,106,107,108,109,110,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,153,154,155,156,157,158,159,160,183,184,185,186,187,188,189,190,191,192,193,194,195,196]
    straight_points=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 64, 65, 66, 67, 68, 69, 70, 71, 72, 81, 82, 83, 84, 85,86,87,88,89,90, 91, 92, 93, 94, 118, 119, 120, 127, 128, 129, 130, 131, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213]
    # Read input parameters
    heading = params['heading']
    IC=0
    distance_from_center = params['distance_from_center']
    steps = params['steps']
    steering_angle = params['steering_angle']
    speed = params['speed']
    track_width =params['track_width']
    waypoints = params['waypoints']
    progress = params['progress']
    RACING_LINE_VS_CENTRAL_LINE = 0.60
    max_offset = track_width * RACING_LINE_VS_CENTRAL_LINE * 0.5
    optimal_waypoints = smooth_central_line(waypoints, max_offset)
    waypoints_length= len(optimal_waypoints)
    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None
        PARAMS.prev_direction_diff = None
        PARAMS.prev_normalized_distance_from_route = None
    optimal_speed_array = generate_optimal_speed_array(optimal_waypoints)
###########################################################
    #Heading Reward
###########################################################
    heading = params['heading']
    vehicle_x = params['x']
    vehicle_y = params['y']
    prev_closest_point_index,next_closest_point_index = closest_route_point(vehicle_x,vehicle_y,optimal_waypoints)
    next_route_point_x = optimal_waypoints[(next_closest_point_index+4)%waypoints_length][0]
    next_route_point_y = optimal_waypoints[(next_closest_point_index+4)%waypoints_length][1]
    print("Next closest point index:",next_closest_point_index)
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians between target and current vehicle position
    route_direction = math.atan2(next_route_point_y - vehicle_y, next_route_point_x - vehicle_x) 
    # Convert to degree
    route_direction = math.degrees(route_direction)
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = route_direction - heading
    #Check that the direction_diff is in valid range
    #Then compute the heading reward
    heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 10
    if abs(direction_diff) <= 20:
        heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 4
    print("Route Direction:",route_direction)
    print("Direction Diff:",direction_diff)
    print("Heading Reward:",heading_reward)

###########################################################    
    #Distance Reward
###########################################################
    #distance reward is value of the standard normal scaled back to 1. #Hence the 1/2*pi*sigma term is cancelled out
    distance_reward = 0
    normalized_car_distance_from_route = (normalized_distance(optimal_waypoints[prev_closest_point_index],optimal_waypoints[next_closest_point_index],vehicle_x,vehicle_y))/track_width
    print("Normalized distance from track:",normalized_car_distance_from_route)
    if normalized_car_distance_from_route == 0: #i.e. on the route line
        distance_from_route = 0
        distance_reward = 1
    elif normalized_car_distance_from_route > 0: #i.e. on right side of the route line
        normalized_route_distance_from_inner_border = params['distance_from_center']
        if params['is_left_of_center']:
            normalized_route_distance_from_inner_border = 0.5*params['track_width'] - normalized_route_distance_from_inner_border
        else:
            normalized_route_distance_from_inner_border+= 0.5*params['track_width']
        normalized_route_distance_from_inner_border/=track_width
        print("Normalized distance from inner border:",normalized_route_distance_from_inner_border)
        sigma=abs(normalized_route_distance_from_inner_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
    elif normalized_car_distance_from_route < 0: #i.e. on left side of the route line
        normalized_route_distance_from_outer_border = params['distance_from_center']
        if not params['is_left_of_center']:
            normalized_route_distance_from_outer_border = 0.5*params['track_width'] - normalized_route_distance_from_outer_border
        else:
            normalized_route_distance_from_outer_border+= 0.5*params['track_width']
        normalized_route_distance_from_outer_border/=track_width
        print("Normalized distance from outer border:",normalized_route_distance_from_outer_border)
        sigma=abs(normalized_route_distance_from_outer_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
    print("Distance Reward:",distance_reward)
###########################################################
    #Speed Reward
###########################################################
    MIN_SPEED = 1.4
    MAX_SPEED = 4.0
    optimal_speed = 0
    #chosen such that 6 standard deviations covers the entire range
    sigma_speed = abs(MAX_SPEED - MIN_SPEED)/6.0
    i = next_closest_point_index
    nsteps = 5
    if i+nsteps < waypoints_length:
        optimal_speed = min( optimal_speed_array[i:(i+nsteps)%waypoints_length] )
    else:
        optimal_speed = min( min(optimal_speed_array[i:]), min(optimal_speed_array[:(i+nsteps)%waypoints_length+1]) )
        
    #PARAMS.optimal speed is unbounded and hence must be bounded from above to reflect action space realities
    optimal_speed = min(MAX_SPEED,optimal_speed)
    speed_reward = math.exp(-0.5*abs(speed-optimal_speed)**2 / sigma_speed**2)
    print("Optimal Speed:",optimal_speed)
    print("Speed Reward:",speed_reward)

# Reinitialize previous parameters if it is a new episode
    #Check if the speed has dropped
    has_speed_dropped = False
    if PARAMS.prev_speed is not None:
        if PARAMS.prev_speed > speed:
            has_speed_dropped = True
    speed_maintain_bonus= 1
    #Penalize slowing down without good reason on straight portions
    if has_speed_dropped and not params['closest_waypoints'][1] in curve_points: 
        speed_maintain_bonus = min( speed / PARAMS.prev_speed, 1 )
    #Penalize making the heading direction worse
    heading_decrease_bonus = 0
    is_heading_in_right_direction = True if abs(direction_diff) <=20 else False
    if PARAMS.prev_direction_diff is not None:
        if is_heading_in_right_direction:
            if abs( PARAMS.prev_direction_diff / direction_diff ) > 1:
                heading_decrease_bonus = min(10, abs( PARAMS.prev_direction_diff / direction_diff ))
    #has the steering angle changed
    has_steering_angle_changed = False
    if PARAMS.prev_steering_angle is not None:
        if not(math.isclose(PARAMS.prev_steering_angle,steering_angle)):
            has_steering_angle_changed = True
    steering_angle_maintain_bonus = 1 
    #Not changing the steering angle is a good thing if heading in the right direction
    if is_heading_in_right_direction and not has_steering_angle_changed:
        if abs(direction_diff) < 10:
            steering_angle_maintain_bonus *= 2
        if abs(direction_diff) < 5:
            steering_angle_maintain_bonus *= 2
        if PARAMS.prev_direction_diff is not None and abs(PARAMS.prev_direction_diff) > abs(direction_diff):
            steering_angle_maintain_bonus *= 2
    #Reward reducing distance to the race line
    distance_reduction_bonus = 1
    if PARAMS.prev_normalized_distance_from_route is not None and PARAMS.prev_normalized_distance_from_route > normalized_car_distance_from_route:
        if abs(normalized_car_distance_from_route) > 0:
            distance_reduction_bonus = min( abs( PARAMS.prev_normalized_distance_from_route / normalized_car_distance_from_route ), 2)
    # Before returning reward, update the variables
    PARAMS.prev_speed = speed
    PARAMS.prev_steering_angle = steering_angle
    PARAMS.prev_direction_diff = direction_diff
    PARAMS.prev_steps = steps
    PARAMS.prev_normalized_distance_from_route = normalized_car_distance_from_route
    #heading component of reward
    HC = ( 10 * heading_reward * steering_angle_maintain_bonus )
    #distance component of reward
    DC = ( 10 * distance_reward * distance_reduction_bonus )
    #speed component of reward
    SC = ( 5 * speed_reward * speed_maintain_bonus )
    #Immediate component of reward
    IC = ( HC + DC + SC ) ** 2 + ( HC * DC * SC )

    print("Speed Maintain Bonus :",speed_maintain_bonus)
    print("Distance Reduction Bonus :",distance_reduction_bonus)
    print("Steering angle Maintain Bonus :",steering_angle_maintain_bonus) 
#If an unpardonable action is taken, then the immediate reward is 0
#########################################################
# Unpardonable Actions
#########################################################
    if params['is_offtrack'] or params['is_crashed']:
        IC = 0
    if direction_diff > 30:
        IC = 0
    if normalized_car_distance_from_route * params['steering_angle'] < 0 :
        IC = 0
    if params['speed'] > optimal_speed+1.3 and params['closest_waypoints'][1] in curve_points:
        IC = 0
    if params['speed'] + 1.5 < optimal_speed and params['closest_waypoints'][1] in straight_points:
        IC = 0
    print("IC:",IC)
    # Reward for making steady progress
    progress_reward = 10 * progress / steps
    if steps <= 5:
        progress_reward = 1 #ignore progress in the first 5 steps
    # Bonus that the agent gets for completing every 10 percent of track
    # Is exponential in the progress / steps. 
    # exponent increases with an increase in fraction of lap completed
    intermediate_progress_bonus = 0
    pi = int(progress//10)
    if pi != 0:
        if pi==10: # 100% track completion
            intermediate_progress_bonus = progress_reward ** 14
    else:
        intermediate_progress_bonus = progress_reward ** (5+0.75*pi)
    print("Progress Reward :",progress_reward)
    print("Intermediate Progres Bonus:",intermediate_progress_bonus)
    #Long term component of reward
    #TO-DO
    curve_bonus=0
    straight_section_bonus=0
    LC = ( curve_bonus + intermediate_progress_bonus + straight_section_bonus )
    return max(IC + LC,1e-3)
    