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
    closest_waypoints = params['closest_waypoints']
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    reward = 0
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

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    angle_f2= angle_between_lines(prev_point_4[0],prev_point_4[1],prev_point_3[0],prev_point_3[1],prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1])

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
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])+ params['progress']//2
        reward += progress_reward
    else:
        return 1e-9
    if abs(total_angle) <=22:
        opt_speed= 5*math.tanh(10/(1+abs(total_angle)))
        opt_speed=max(2.2,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3
    else:
        opt_speed= 5*math.tanh(10/(1+abs(total_angle)))
        opt_speed=max(1.6,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**3
    if total_angle >10 and params['is_left_of_center']:
        reward+=20.0
    if total_angle < -10 and not params['is_left_of_center']:
        reward+=20.0
    if total_angle >15 and params['distance_from_center']>=0.2*params['track_width']  and params['is_left_of_center']:
        reward+=20
    if total_angle <-22 and params['distance_from_center']>=0.2*params['track_width'] and not params['is_left_of_center']:
        reward+=20
    # if total_angle>26 and params['is_left_of_center']:
    #     reward+=30.0
    # if total_angle<-26 and not params['is_left_of_center']:
    #     reward+=30.0

    steps=params['steps']
    progress= params['progress']
    step_reward = 0
    if steps>0:
        step_reward= ((progress*20)/steps)**3

    reward+=step_reward
    return float(reward)