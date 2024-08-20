import math

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
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    steps=params['steps']
    progress = params['progress']
    closest_waypoints = params['closest_waypoints']    
    left_turn=[14,15,16,17,18,19,20,21,22,23,24,25,26,27,53,54,55,56,57,58,59,60,61,77,78,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,187,188,189,190,191,192,193,194]
    right_turn=[135,136,137,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159]
    straight_waypoints=[33,44,66,72,85,94,130,171]
    not_very_right_waypoints=[89,90,91,132,133,134,138,139,140,141,142,143,144,160,161,162,163]
    not_very_left=[9,10,11,12,13,28,29,30,52,62,74,75,76,79,80,81,82,97,98,99,123,124,125,183,184,185,186,195,196,197,198]
    basic_left=[1,2,3,4,5,6,7,8,31,32,45,46,47,48,49,50,51,63,64,65,73,83,84,95,96,126,127,128,129,172,173,174,175,176,177,178,179,180,181,182,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213]
    basic_right=[34,35,36,37,38,39,40,41,42,43,67,68,69,70,71,86,87,88,92,93,131,164,165,166,167,168,169,170]
    straight_lines=[30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,200,201,202,203,204,205,206,207,208,209,210,211,212,213,1,2,3,4,5]
    curve_points= left_turn + right_turn
    almost_straight= basic_left + basic_right
    # Calculate the direction of the center line based on the closest waypoints
    track_width =params['track_width']
    RACING_LINE_VS_CENTRAL_LINE = 0.90
    max_offset = track_width * RACING_LINE_VS_CENTRAL_LINE * 0.5
    optimal_waypoints = smooth_central_line(waypoints, max_offset)
    waypoints_length= len(optimal_waypoints)

    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_point_1 = optimal_waypoints[next]
    next_point_2 = optimal_waypoints[(next+1)%waypoints_length]
    next_point_3 = optimal_waypoints[(next+2)%waypoints_length]
    next_point_4 = optimal_waypoints[(next+3)%waypoints_length]
    prev_point =   optimal_waypoints[prev]
    prev_point_2 = optimal_waypoints[(prev-1+waypoints_length)%waypoints_length]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    straight_direction_diff = abs(track_direction - params['heading']-params['steering_angle'])
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if straight_direction_diff>180:
        straight_direction_diff= 360-straight_direction_diff

    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    reward = 1e-9
    total_angle = (angle_f+angle_b)/2
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if total_angle >30:
        total_angle=30
    elif total_angle <-30:
        total_angle=-30
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle = 0
    steering_reward=1e-4
    if next not in curve_points:
        steering_reward = 160*math.tanh(10/(1+abs(straight_direction_diff - total_angle)))
    else:
        steering_reward = 160*math.tanh(10/(1+abs(params['steering_angle']-total_angle)))

    reward=reward+ steering_reward
    if next in straight_waypoints or next in almost_straight:
        if params['speed'] >=2.8:
            reward+=10
        if params['speed'] >=3:
            reward+=10
        if params['speed']>=3.2:
            reward+=10
        if params['speed'] >=3.4:
            reward+=20
        if params['speed'] >=3.7:
            reward+=30
        if params['speed'] >=4:
            reward+=40
        if params['speed'] >=4.2:
            reward+=15
        if params['speed'] >=4.4:
            reward+=15
    elif next not in curve_points:
        if params['speed'] >=2.0:
            reward+=10
        if params['speed'] >=2.2:
            reward+=20
        if params['speed'] >=2.4:
            reward+=30
        if params['speed'] >=2.6:
            reward+=30
        if params['speed'] >=2.8:
            reward+=20
        if params['speed'] >=3.0:
            reward+=20
        if params['speed'] >=3.2:
            reward+=5
    elif abs(total_angle) <=20:
        opt_speed= 5*math.tanh(10/(1+abs(total_angle)))
        opt_speed=max(1.9,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3
    else:
        opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
        opt_speed=max(1.4,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3


    if next in straight_waypoints:

        if params['distance_from_center']==0:
            reward=reward+30+params['speed']**3
        elif params['distance_from_center']<=0.1*params['track_width']:
            reward+=10+ params['speed']**3

    if next in left_turn and params['is_left_of_center']:
        reward+=50.0
        if params['distance_from_center']>=0.3*params['track_width']:
           reward+=100
        elif params['distance_from_center']>=0.2*params['track_width']:
           reward+=40
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10
    if next in right_turn and not params['is_left_of_center']:
        reward+=50.0
        if params['distance_from_center']>=0.3*params['track_width']:
           reward+=100
        elif params['distance_from_center']>=0.2*params['track_width']:
           reward+=40
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10
    if next in not_very_right_waypoints and not params['is_left_of_center']:
        reward+=50.0
        if params['distance_from_center']>=0.2*params['track_width']:
           reward+=100
    if next in not_very_left and params['is_left_of_center']:
        reward+=50.0
        if  params['distance_from_center']>=0.2*params['track_width']:
            reward+=100
    if next in basic_left:
        if params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=50
            if params['distance_from_center']<=0.3*params['track_width']:
                reward+=50+params['speed']**3
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=50
            if params['distance_from_center']<=0.3*params['track_width']:
                reward+=50+params['speed']**3
    if progress ==100:
        if steps <=315:
            reward+=2000
        if steps <=305:
            reward+=2000
        if steps <=295:
            reward+=1000
        if steps <=285:
            reward+=1000
        if steps <=275:
            reward+=1000
        if steps <=265:
            reward+=500
    threshold_1=295
    threshold_2=305
    threshold_3=314
    steps_t1= (threshold_1*progress)/100
    steps_t2= (threshold_2*progress)/100
    steps_t3= (threshold_3*progress)/100
    if steps>=5 and steps%30==0:
        if steps<= steps_t3:
            reward+=900
        if steps<= steps_t2:
            reward+=300
        if steps<= steps_t1:
            reward+=300
    steps_weight =1
    if steps>5:
        expected_steps = progress*3.40
        if steps > expected_steps:
            steps_weight = 1 - ((4*(steps-expected_steps))/steps)
        if steps_weight <= 0.1:
            steps_weight = 0.1
    return float(reward)