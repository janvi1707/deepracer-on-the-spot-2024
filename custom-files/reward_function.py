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
    closest_waypoints = params['closest_waypoints']
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
    prev_point_3 = optimal_waypoints[(prev-2+waypoints_length)%waypoints_length]
    prev_point_4 = optimal_waypoints[(prev-3+waypoints_length)%waypoints_length]
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
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])

    reward = 1e-9
    total_angle = (angle_f + angle_b)/2
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if total_angle >30:
        total_angle=30
    elif total_angle <-30:
        total_angle=-30
    if abs(total_angle)<7:
        total_angle=0
    print("waypoint_id : {},Total angle : {},steering_angle : {}".format(next,total_angle,params['steering_angle']))
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    steering_reward = 200/(1+abs(params['steering_angle']-total_angle))
    if abs(total_angle) >30 and abs(params['steering_angle'])>25 and total_angle*params['steering_angle']>=0:
        steering_reward=200
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])+ params['progress']//2
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    if direction_diff <=10.0:
        reward+=10.0
    if abs(total_angle)<=5:
        if params['speed'] >=3:
            reward+=30
        if params['speed'] >=3.4:
            reward+=30
        if params['speed'] >=3.8:
            reward+=30
        if params['speed'] >=4:
            reward+=30
        if params['speed'] >=4.2:
            reward+=30
        if params['speed'] >=4.4:
            reward+=50
    else:
        opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
        opt_speed=max(1.2,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3
        
    if abs(params['steering_angle']-total_angle) >=10:
        reward*=0.25
    if abs(params['steering_angle'])<10 and abs(total_angle)>20:
        return 1e-3
    if total_angle >10 and params['is_left_of_center']:
        reward+=100.0
    if total_angle < -10 and not params['is_left_of_center']:
        reward+=100.0
    if abs(params['steering_angle'])>=25 and abs(total_angle)>=25 and total_angle*params['steering_angle']>=0:
        reward+=100.0
    if total_angle>26 and params['is_left_of_center']:
        reward+=30.0
    if total_angle<-26 and not params['is_left_of_center']:
        reward+=30.0
    return float(reward)