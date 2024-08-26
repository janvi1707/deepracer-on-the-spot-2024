import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9

    steps=params['steps']
    progress = params['progress']
    closest_waypoints = params['closest_waypoints']    
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
    next = int(closest_waypoints[1])
    reward = 1e-9
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
           reward+=50
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10
    if next in left_turn and not params['is_left_of_center']:
        reward-=50 
    if next in right_turn and not params['is_left_of_center']:
        reward+=50.0
        if params['distance_from_center']>=0.3*params['track_width']:
           reward+=100
        elif params['distance_from_center']>=0.2*params['track_width']:
           reward+=50
        elif  params['distance_from_center']>=0.1*params['track_width']:
            reward+=10
    if next in right_turn and params['is_left_of_center']:
        reward-=50 
    if next in not_very_right_waypoints and not params['is_left_of_center']:
        reward+=50.0
        if params['distance_from_center']>=0.25*params['track_width']:
           reward+=100
    if next in not_very_left and params['is_left_of_center']:
        reward+=50.0
        if  params['distance_from_center']>=0.25*params['track_width']:
            reward+=100
    if next in basic_left:
        if params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=50
            if params['distance_from_center']<=0.25*params['track_width']:
                reward+=50+params['speed']**3
    if next in basic_right:
        if not params['is_left_of_center'] or params['distance_from_center']==0:
            reward+=50
            if params['distance_from_center']<=0.25*params['track_width']:
                reward+=50+params['speed']**3
    if next in right_mid_path and not params['is_left_of_center']:
        reward+=50
        if params['distance_from_center']>=0.2*params['track_width'] and params['distance_from_center']<0.3*params['track_width']:
            reward+=50+params['speed']**3

    if next in left_mid and params['is_left_of_center']:
        reward+=50
        if params['distance_from_center']>=0.2*params['track_width'] and params['distance_from_center']<0.3*params['track_width']:
            reward+=50+params['speed']**3
    
    if next in straight_lines:
        if params['speed'] >=2.8:
            reward+=30
        if params['speed'] >=3:
            reward+=30
        if params['speed']>=3.2:
            reward+=30
        if params['speed'] >=3.4:
            reward+=35
        if params['speed'] >=3.7:
            reward+=30
        if params['speed'] >=4:
            reward+=20
        if params['speed'] >=4.2:
            reward+=15
        if params['speed'] >=4.4:
            reward+=15
        if params['speed']<2.8:
            reward-=50
    else:
        opt_speed= 5*math.tanh(10/(1+abs(params['steering_angle'])))
        opt_speed=max(1.4,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3

    step_reward=1

    if steps>0 and steps%45==0:
        step_reward+= ((progress*36)/steps)**4

    if steps>0:
        step_reward+= ((progress*45)/steps)**2

    reward+=step_reward
    
    return float(reward)

