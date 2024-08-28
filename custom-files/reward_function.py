import math
import numpy as np
class PARAMS:
    prev_speed = None
    prev_steering_angle = None 
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None

def angle_between_lines(x1, y1, x2, y2, x3, y3, x4, y4):
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3
    angle = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
    deg= math.degrees(angle)
    if deg>180:
        deg=deg-360
    if deg <-180:
        deg= deg+360
    return deg

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
def reward_function(params):
    curve_points=[17,18,19,20,21,22,23,24,25,26,27,28,29,30,54,55,56,57,58,59,60,61,62,97,98,99,100,101,102,103,104,105,106,107,108,109,110,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,153,154,155,156,157,158,159,160,183,184,185,186,187,188,189,190,191,192,193,194,195,196]
    straight_points=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 64, 65, 66, 67, 68, 69, 70, 71, 72, 81, 82, 83, 84, 85,86,87,88,89,90, 91, 92, 93, 94, 118, 119, 120, 127, 128, 129, 130, 131, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213]
    
    left_turn=[18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,55,56,57,58,59,60,61,62,63,64,77,78,79,80,81,82,
               99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,
               186,187,188,189,190,191,192,193,194,195,196]
    not_very_left=[16,17,33,53,54,76,83,98,125,126,185,197,198]
    left_mid=[13,14,15,34,50,51,52,65,75,84,97,127,128,182,183,184,199,200,201,202]
    basic_left=[6,7,8,9,10,11,12,35,36,47,48,49,66,74,85,86,96,129,130,177,178,179,180,181,203,204,205,206,207]
    straight_waypoints=[0,1,2,3,4,5,37,46,67,73,87,95,131,172,173,174,175,176,208,209,210,211,212,213]
    basic_right=[38,39,44,45,68,72,88,89,90,94,132,168,169,170,171]
    right_mid_path=[40,41,42,43,69,70,71,91,92,93,133,165,166,167]
    not_very_right_waypoints=[134,142,143,144,163,164]
    right_turn=[135,136,137,138,139,140,141,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162]
    
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
##################
#steering
#############
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_point_1 = waypoints[next]
    next_point_2 = waypoints[(next+1)%waypoints_length]
    next_point_3 = waypoints[(next+2)%waypoints_length]
    next_point_4 = waypoints[(next+3)%waypoints_length]
    next_point_5 = waypoints[(next+4)%waypoints_length]
    next_point_6 = waypoints[(next+5)%waypoints_length]
    prev_point = waypoints[prev]
    prev_point_2 = waypoints[(prev-1+waypoints_length)%waypoints_length]
    prev_point_3 = waypoints[(prev-2+waypoints_length)%waypoints_length]
    prev_point_4 = waypoints[(prev-3+waypoints_length)%waypoints_length]
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    reward = 1e-9
    total_angle = angle_f
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if total_angle >30:
        total_angle=30
    elif total_angle <-30:
        total_angle=-30
    if abs(total_angle)<=7:
        total_angle=0
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    steering_reward = 100/(1+abs(params['steering_angle']-total_angle))
    if abs(total_angle) >=30 and abs(params['steering_angle'])>25 and total_angle*params['steering_angle']>=0:
        steering_reward=100

    if abs(params['steering_angle']-total_angle) >=10:
        steering_reward*=0.25
    if abs(params['steering_angle'])<10 and abs(total_angle)>20:
        return 1e-3

    if abs(params['steering_angle'])>=25 and abs(total_angle)>=25 and total_angle*params['steering_angle']>=0:
        steering_reward=100
    if abs(params['steering_angle'])>7 and abs(total_angle)<9 and total_angle*params['steering_angle']>=0:
        return 1e-3

# ###########################################################
#     #Optimal path
# ###########################################################
    vehicle_x = params['x']
    vehicle_y = params['y']
    opt_reward=0
    prev_closest_point_index,next_closest_point_index = closest_route_point(vehicle_x,vehicle_y,optimal_waypoints)
    if next in left_turn and params['is_left_of_center']:
        opt_reward+=30
        if params['distance_from_center']>=0.25*params['track_width']:
            opt_reward+=70
        elif params['distance_from_center']>=0.18*params['track_width']:
            opt_reward+=40
        elif  params['distance_from_center']>=0.1*params['track_width']:
            opt_reward+=20
    if next in left_turn and not params['is_left_of_center']:
        opt_reward-=30
    if next in right_turn and not params['is_left_of_center']:
        opt_reward+=30
        if params['distance_from_center']>=0.25*params['track_width']:
            opt_reward+=70
        elif params['distance_from_center']>=0.18*params['track_width']:
            opt_reward+=40
        elif  params['distance_from_center']>=0.1*params['track_width']:
            opt_reward+=20
    if next in right_turn and params['is_left_of_center']:
        opt_reward-=30
    if next in not_very_right_waypoints and not params['is_left_of_center']:
        opt_reward+=30
        if params['distance_from_center']>=0.2*params['track_width']:
            opt_reward+=70
    if next in not_very_left and params['is_left_of_center']:
        opt_reward+=30
        if  params['distance_from_center']>=0.2*params['track_width']:
            opt_reward+=70
    if next in basic_left:
        if params['is_left_of_center'] or params['distance_from_center']==0:
            opt_reward+=30
            if params['distance_from_center']<=0.2*params['track_width']:
                opt_reward+=70
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            opt_reward+=30
            if params['distance_from_center']<=0.2*params['track_width']:
                opt_reward+=70
    if next in right_mid_path and not params['is_left_of_center']:
        opt_reward+=30
        if params['distance_from_center']>=0.18*params['track_width'] and params['distance_from_center']<0.25*params['track_width']:
            opt_reward+=70

    if next in left_mid and params['is_left_of_center']:
        opt_reward+=30
        if params['distance_from_center']>=0.18*params['track_width'] and params['distance_from_center']<0.25*params['track_width']:
            opt_reward+=70
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
    speed_reward*=100
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
    ##########
    ##progress#####
    ######
    step_reward=0
    if steps<5:
        step_reward=1

    if steps>5 and steps%30==0:
        step_reward+= ((progress*31)/steps)**4

   # Before returning reward, update the variables
    PARAMS.prev_speed = speed
    PARAMS.prev_steering_angle = steering_angle
    PARAMS.prev_direction_diff = direction_diff
    PARAMS.prev_steps = steps
    #heading component of reward
    # HC = ( 10 * heading_reward * steering_angle_maintain_bonus )
    HC = ( 10 * steering_reward )
    #speed component of reward
    SC = ( 5 * speed_reward * speed_maintain_bonus )

    OC = 5 * opt_reward
    #Immediate component of reward
    IC = ( HC + SC + OC + step_reward )

    print("Speed Maintain Bonus :",speed_maintain_bonus)
#If an unpardonable action is taken, then the immediate reward is 0
#########################################################
# Unpardonable Actions
#########################################################
    if params['is_offtrack'] or params['is_crashed']:
        IC = 0
    if direction_diff > 30:
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
    #Long term component of reward
    #TO-DO
    curve_bonus=0
    straight_section_bonus=0
    # LC = ( curve_bonus + intermediate_progress_bonus + straight_section_bonus )
    return max(IC ,1e-3)
    