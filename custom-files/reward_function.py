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
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    steps=params['steps']
    progress = params['progress']
    closest_waypoints = params['closest_waypoints']
    straight_waypoints = [149,150,151,152,153,154]
    left_waypoints=[9,10,11,12,13,14,15,16,17,18,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,94,95,96,97,98,99,100,101,102,103,104,105,106,130,131,132,133,134,135,136,137]
    right_waypoints=[65,66,67,68,69,70,71,72,73,74,75,76,77,78,79]
    not_very_right_waypoints=[63,64,80,81,82]
    not_very_left=[5,6,7,8,19,20,21,22,31,32,33,34,54,55,91,92,93,107,108,109,110,127,128,129,138,139,140]
    basic_left=[1,2,3,4,23,24,25,26,27,28,29,30,56,57,58,59,87,88,89,90,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,141,142,143,144,145,146,147,148,149,150,151,152,153,154]
    basic_right=[60,61,62,83,84,85,86]
    curve_points= left_waypoints + right_waypoints
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_point_1 = waypoints[next]
    next_point_2 = waypoints[(next+1)%waypoints_length]
    next_point_3 = waypoints[(next+2)%waypoints_length]
    next_point_4 = waypoints[(next+3)%waypoints_length]
    prev_point = waypoints[prev]
    prev_point_2 = waypoints[(prev-1+waypoints_length)%waypoints_length]

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
        steering_reward = 100/(1+abs(straight_direction_diff - total_angle))
    else:
        steering_reward = 100/(1+abs(params['steering_angle']-total_angle))
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])+ params['progress']//2
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    if direction_diff <=10.0:
        reward+=10.0
    if next in straight_waypoints:
        if params['speed'] >=2.8:
            reward+=5
        if params['speed'] >=3:
            reward+=25
        if params['speed']>=3.2:
            reward+=20
        if params['speed'] >=3.4:
            reward+=20
        if params['speed'] >=3.7:
            reward+=50
        if params['speed'] >=4:
            reward+=80
        if params['speed'] >=4.2:
            reward+=80
        if params['speed'] >=4.4:
            reward+=80
    elif next not in curve_points:
        if params['speed'] >=2.0:
            reward+=10
        if params['speed'] >=2.2:
            reward+=20
        if params['speed'] >=2.4:
            reward+=20
        if params['speed'] >=2.6:
            reward+=20
        if params['speed'] >=2.8:
            reward+=20
        if params['speed'] >=3.0:
            reward+=30
        if params['speed'] >=3.2:
            reward+=30
    else:
        opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
        opt_speed=max(1.4,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**2


    if next in straight_waypoints:

        if params['distance_from_center']==0:
            reward=reward+5*(params['speed']**2)
        elif params['distance_from_center']<=0.1*params['track_width']:
            reward+=3*(params['speed']**2)

    if next in left_waypoints and params['is_left_of_center']:
        reward+=60.0
        if params['distance_from_center']>=0.3*params['track_width']:
           reward+=50
        elif params['distance_from_center']>=0.2*params['track_width']:
           reward+=30
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10
    if next in right_waypoints and not params['is_left_of_center']:
        reward+=60.0
        if params['distance_from_center']>=0.3*params['track_width']:
           reward+=50
        elif params['distance_from_center']>=0.2*params['track_width']:
           reward+=30
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10 
    if next in not_very_right_waypoints and not params['is_left_of_center']:
        reward+=60.0
        if params['distance_from_center']>=0.2*params['track_width']:
           reward+=50
    if next in not_very_left and params['is_left_of_center']:
        reward+=60.0
        if  params['distance_from_center']>=0.2*params['track_width']:
            reward+=50
    if next in basic_left:
        if params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=100
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=100
    if progress ==100:
        if steps <=270:
            reward+=2000
        if steps <=250:
            reward+=3500
        if steps <=230:
            reward+=1500
        if steps <=210:
            reward+=1500
        if steps <=190:
            reward+=1000
        if steps <=170:
            reward+=500
    threshold_1=210
    threshold_2=240
    threshold_3=270
    steps_t1= (threshold_1*progress)/100
    steps_t2= (threshold_2*progress)/100
    steps_t3= (threshold_3*progress)/100
    if steps>=5 and steps%30==0:
        if steps<= steps_t3:
            reward+=500
        if steps<= steps_t2:
            reward+=400
        if steps<= steps_t1:
            reward+=600
    return float(reward)