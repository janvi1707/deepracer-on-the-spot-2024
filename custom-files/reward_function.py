import math

prev_steer = None

def reward_function(params):
    global prev_steer
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    
    closest_waypoints = params['closest_waypoints']    
    left_turn=[14,15,16,17,18,19,20,21,22,23,24,25,26,27,53,54,55,56,57,58,59,60,61,77,78,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,187,188,189,190,191,192,193,194]
    right_turn=[135,136,137,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159]
    straight_waypoints=[33,44,66,72,85,94,130,171]
    not_very_right_waypoints=[89,90,91,132,133,134,138,139,140,141,142,143,144,160,161,162,163]
    not_very_left=[9,10,11,12,13,28,29,30,52,62,74,75,76,79,80,81,82,97,98,99,123,124,125,183,184,185,186,195,196,197,198]
    basic_left=[1,2,3,4,5,6,7,8,31,32,45,46,47,48,49,50,51,63,64,65,73,83,84,95,96,126,127,128,129,172,173,174,175,176,177,178,179,180,181,182,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213]
    basic_right=[34,35,36,37,38,39,40,41,42,43,67,68,69,70,71,86,87,88,92,93,131,164,165,166,167,168,169,170]
    straight_lines=[30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,200,201,202,203,204,205,206,207,208,209,210,211,212,213,1,2,3,4,5]
    waypoints = params['waypoints']
    waypoints_length= len(waypoints)
    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_point_1 = waypoints[next]
    next_point_2 = waypoints[(next+1)%waypoints_length]
    next_point_3 = waypoints[(next+2)%waypoints_length]
    next_point_4 = waypoints[(next+3)%waypoints_length]
    prev_point = waypoints[prev]
    prev_point_2 = waypoints[(prev-1+waypoints_length)%waypoints_length]
    if prev_steer is not None:
        if next_point_2 in straight_lines and prev_point in straight_lines:
            if abs(prev_steer-params['steering_angle'])<3:
                reward+=160

    reward = 1e-9
    if next in straight_lines:
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
    
    prev_steer=params['steering_angle']

    if prev_steer is not None:
        print("janvi : steeer angle",prev_steer)
    else:
        print("prev_steering null")

    return float(reward)
