import math

class PARAMS:
    prev_speed= None
    prev_steering_angle= None
    prev_steps = None


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

    steps = params['steps']
    steering_angle = params['steering_angle']
    speed = params['speed'] 
    waypoints = params['waypoints']
    progress = params['progress']
    closest_waypoints = params['closest_waypoints']

    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None

    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    
    has_speed_dropped = False
    if PARAMS.prev_speed is not None:
        if PARAMS.prev_speed > speed:
            has_speed_dropped = True

    
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
    straight_lines=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,
                    62,63,64,65,66,67,68,69,70,71,72,73,74,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,
                    113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,149,150,151,
                    164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,
                    197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213]
    curve_points= left_turn + right_turn
    almost_straight= basic_left + basic_right
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_2=next+1
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
    straight_direction_diff = abs(track_direction - params['heading']-steering_angle)
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if straight_direction_diff>180:
        straight_direction_diff= 360-straight_direction_diff

    is_turn_upcoming= True
    if next_2 in straight_lines:
        is_turn_upcoming=False

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
    speed_reward=1e-4
    opt_reward=1e-4

    steering_reward = 13*math.tanh(10/(1+abs(params['steering_angle']-total_angle)))

    if next in straight_lines:
        if speed >=2.8:
            speed_reward+=3
        if speed >=3:
            speed_reward+=3
        if speed>=3.2:
            speed_reward+=3
        if speed >=3.4:
            speed_reward+=2
        if speed >=3.7:
            speed_reward+=2
        if speed >=4:
            speed_reward+=2
        if speed >=4.2:
            speed_reward+=1
        if speed >=4.4:
            speed_reward+=1

    if has_speed_dropped and not is_turn_upcoming: 
        speed_reward*=0.2
    

    if next in straight_waypoints:

        if params['distance_from_center']==0:
            opt_reward=reward+10
        elif params['distance_from_center']<=0.1*params['track_width']:
            opt_reward+=5

    if next in left_turn and params['is_left_of_center']:
        opt_reward+=9
        if params['distance_from_center']>=0.25*params['track_width']:
           opt_reward+=6
        elif params['distance_from_center']>=0.18*params['track_width']:
           opt_reward+=4
        elif  params['distance_from_center']>=0.1*params['track_width']:
            opt_reward+=1
    if next in left_turn and not params['is_left_of_center']:
        opt_reward-=5
    if next in right_turn and not params['is_left_of_center']:
        opt_reward+=9
        if params['distance_from_center']>=0.25*params['track_width']:
           opt_reward+=6
        elif params['distance_from_center']>=0.18*params['track_width']:
           opt_reward+=4
        elif  params['distance_from_center']>=0.1*params['track_width']:
            opt_reward+=1
    if next in right_turn and params['is_left_of_center']:
        opt_reward-=5
    if next in not_very_right_waypoints and not params['is_left_of_center']:
        opt_reward+=9
        if params['distance_from_center']>=0.2*params['track_width']:
           opt_reward+=6
    if next in not_very_left and params['is_left_of_center']:
        opt_reward+=9
        if  params['distance_from_center']>=0.2*params['track_width']:
            opt_reward+=6
    if next in basic_left:
        if params['is_left_of_center'] or params['distance_from_center']==0:
            opt_reward+=9
            if params['distance_from_center']<=0.2*params['track_width']:
                opt_reward+=6
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            opt_reward+=9
            if params['distance_from_center']<=0.2*params['track_width']:
                opt_reward+=6
    if next in right_mid_path and not params['is_left_of_center']:
        opt_reward+=9
        if params['distance_from_center']>=0.18*params['track_width'] and params['distance_from_center']<0.25*params['track_width']:
            opt_reward+=6

    if next in left_mid and params['is_left_of_center']:
        opt_reward+=9
        if params['distance_from_center']>=0.18*params['track_width'] and params['distance_from_center']<0.25*params['track_width']:
            opt_reward+=6
    
    step_reward=1e-2
    step_reward_long=1e-2
    if steps>0 and steps%45==0:
        step_reward_long+= ((progress*8)/steps)**4

    if steps>0:
        step_reward+= ((progress*15)/steps)**2


    reward = ( steering_reward + speed_reward + opt_reward + step_reward) ** 2 + ( steering_reward*speed_reward*opt_reward*step_reward )/225
    reward+=step_reward_long
    PARAMS.prev_speed = speed
    PARAMS.prev_steering_angle = steering_angle
    PARAMS.prev_steps = steps
    return float(reward)