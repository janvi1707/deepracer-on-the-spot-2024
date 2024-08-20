import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])


    best_angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0, 14.0, 17.0, 5.0, -1.0, 0.0, 0.0, 11.0, 25.0, 30.0, 30.0, 20.0, 0.0, -1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 4.0, -6.0, 19.0, 11.0, -3.0, 30.0, 22.0, 21.0, 18.0, 14.0, 13.0, 10.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 12.0, 23.0, 14.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, -0.0, 14.0, 30.0, 18.0, 6.0, 10.0, 14.0, 23.0, 30.0, 30.0, 30.0, 30.0, 28.0, 12.0, 8.0, 7.0, 4.0, 4.0, 5.0, 0.0, 4.0, 7.0, 0.0, 0.0, 8.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -25.0, -30.0, -30.0, -30.0, -20.0, -1.0, -0.0, -0.0, -2.0, 5.0, -10.0, -30.0, -30.0, -16.0, -2.0, 0.0, -1.0, 0.0, -5.0, -18.0, -21.0, -25.0, -29.0, -30.0, -25.0, -4.0, 0.0, -2.0, -1.0, -2.0, -1.0, -2.0, -1.0, -1.0, -1.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 15.0, 23.0, 25.0, 25.0, 30.0, 29.0, 18.0, 22.0, 30.0, 30.0, 30.0, 11.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    best_angles_len = len(best_angles)
    waypoints_length= len(waypoints)

    reward = 1e-9


    total_angle = best_angles[next%best_angles_len]
    
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0

    print("waypoint_id : {},Total angle : {},steering_angle : {}".format(next,total_angle,params['steering_angle']))
    
    steering_reward = 200*math.tanh(10/(1+abs(params['steering_angle'] - total_angle)))


    reward=reward+ steering_reward

    if abs(total_angle)<5:
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
        

    return float(reward)