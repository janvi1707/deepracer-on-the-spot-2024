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
    left_turn=[14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,53,54,55,56,57,58,59,60,61,62,76,77,78,79,80,99,100,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,186,187,188,189,190,191,192,193,194,195]
    right_turn=[134,135,136,137,138,139,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161]
    straight_waypoints=[1,2,3,65,66,85,94,129,130]
    not_very_right_waypoints=[88,89,90,91,92,132,133,140,141,142,143,144,162,163,164,165,166]
    not_very_left=[8,9,10,11,12,13,30,49,50,51,52,63,73,74,75,81,82,83,96,97,98,101,102,103,125,126,127,176,177,178,179,180,181,182,183,184,185,196,197,198,199,200,201]
    basic_left=[4,5,6,7,31,32,45,46,47,48,64,72,84,95,128,172,173,174,175,202,203,204,205,206,207,208,209,210,211,212,213]
    basic_right=[33,34,35,36,37,38,39,40,41,42,43,44,67,68,69,70,71,86,87,93,131,167,168,169,170,171]
    curve_points= left_turn + right_turn
    almost_straight= basic_left + basic_right
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
    total_angle = angle_f
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
        if params['speed'] >=2.4:
            reward+=10
        if params['speed'] >=2.6:
            reward+=20
        if params['speed'] >=2.8:
            reward+=30
        if params['speed'] >=3.0:
            reward+=30
        if params['speed'] >=3.2:
            reward+=20
        if params['speed'] >=3.4:
            reward+=20
        if params['speed'] >=3.5:
            reward+=5
    elif abs(total_angle) <=20:
        opt_speed= 5*math.tanh(10/(1+abs(total_angle)))
        opt_speed=max(2.2,opt_speed)
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
            if params['distance_from_center']<=0.2*params['track_width']:
                reward+=50+params['speed']**3
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=50
            if params['distance_from_center']<=0.2*params['track_width']:
                reward+=50+params['speed']**3

    # if steps>0:
    #     reward+= ((progress*70)/steps)**2


    expected_steps= progress*3.00
    if steps>0:
        if steps<=expected_steps:
            reward+=500
        else:
            reward+= 160/(1+abs((expected_steps-steps)**2))

    return float(reward)