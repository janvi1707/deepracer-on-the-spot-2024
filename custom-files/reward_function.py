import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    closest_waypoints = params['closest_waypoints']
    steps =params['steps']
    progress = params['progress']  
    best_angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0, 14.0, 17.0, 5.0, -1.0, 0.0, 0.0, 11.0, 25.0, 30.0, 30.0, 20.0, 0.0, -1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 4.0, -6.0, 19.0, 11.0, -3.0, 30.0, 22.0, 21.0, 18.0, 14.0, 13.0, 10.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 12.0, 23.0, 14.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, -0.0, 14.0, 30.0, 18.0, 6.0, 10.0, 14.0, 23.0, 30.0, 30.0, 30.0, 30.0, 28.0, 12.0, 8.0, 7.0, 4.0, 4.0, 5.0, 0.0, 4.0, 7.0, 0.0, 0.0, 8.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -25.0, -30.0, -30.0, -30.0, -20.0, -1.0, -0.0, -0.0, -2.0, 5.0, -10.0, -30.0, -30.0, -16.0, -2.0, 0.0, -1.0, 0.0, -5.0, -18.0, -21.0, -25.0, -29.0, -30.0, -25.0, -4.0, 0.0, -2.0, -1.0, -2.0, -1.0, -2.0, -1.0, -1.0, -1.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 15.0, 23.0, 25.0, 25.0, 30.0, 29.0, 18.0, 22.0, 30.0, 30.0, 30.0, 11.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Calculate the direction of the center line based on the closest waypoints
    best_angles_len= len(best_angles)
    reward = 100/(1+abs(params['steering_angle']-best_angles[(closest_waypoints[1])%best_angles_len]))
    expected_steps1= progress*3.00
    expected_steps2= progress*3.30
    expected_steps3= progress*3.40

    if steps>0 and steps%10==0:
        if steps<=expected_steps1:
            reward+=1000
        elif steps<=expected_steps2:
            reward+=600
        elif steps<=expected_steps3:
            reward+=300
        else:
            reward= 2*reward/(1+abs((expected_steps3-steps)**2))
    return reward
