import math


def progress_reward(params):
    progress = params['progress']
    reward = (progress)/100
    return 200*(1+reward);

def get_abs_speed(diff):
    return min(4,91.63/(22.9+diff))

def fetch_required_steering_angle(waypoints,closest_waypoints,x,y,track_width,heading):
    w_len = len(waypoints);
    start = int(closest_waypoints[0]);
    end = int(closest_waypoints[0]+20);
    resp = int(closest_waypoints[1])
    while(start<=end):
        mid = (start+end)//2
        mid_wp = waypoints[mid%w_len];

        mid_x,mid_y = (x+waypoints[(end+w_len)%w_len][0])/2,(y+waypoints[(w_len+ end)%w_len][1])/2;
        dist = math.sqrt((mid_x-mid_wp[0])**2 + (mid_y-mid_wp[1])**2);

        if(dist<track_width):
            resp = mid;
            start = mid+1;
        else:
            end = mid - 1;
    end = waypoints[(w_len+resp)%w_len];
    start = waypoints[int(closest_waypoints[0])];
    
    track_direction = math.atan2(end[1]-y,end[0]-x)
    track_direction = math.degrees(track_direction)

    direction_diff = track_direction-heading

    if(direction_diff>180):
        direction_diff = direction_diff-360;
    elif(direction_diff<-180):
        direction_diff = direction_diff+360;
    req_steer = direction_diff;

    track_direction = math.atan2(end[1]-start[1],end[0]-start[0])
    track_direction = math.degrees(track_direction)

    direction_diff = track_direction-heading

    if(direction_diff>180):
        direction_diff = direction_diff-360;
    elif(direction_diff<-180):
        direction_diff = direction_diff+360;
    track_direction = direction_diff;

    return [req_steer, track_direction]

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    track_width = int(params['track_width'])/2
    heading = params['heading']
    speed = params['speed']
    steering_angle = params['steering_angle']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    x = params['x']
    y = params['y']
    resp = fetch_required_steering_angle(waypoints,closest_waypoints,x,y,track_width,heading);
    
    direction_diff = abs(round(resp[0])-steering_angle)
    if direction_diff>180:
        direction_diff = 360-direction_diff;

    dfc_reward = 100;
    
    if(resp[1]<-5 and not is_left_of_center):
        dfc_reward=100;
        dfc = track_width * (abs(resp[1])/60)
        if(resp[1]*steering_angle < 0):
            return 1e-9
        dfc = 500/(1 + 100*abs(distance_from_center - dfc))
        dfc_reward = dfc_reward + dfc
    elif(resp[1]>5 and is_left_of_center):
        dfc_reward=100;
        dfc = track_width * (abs(resp[1])/50)
        if(resp[1]*steering_angle < 0):
            return 1e-9
        dfc = 500/(1 + 100*abs(distance_from_center - dfc))
        dfc_reward = dfc_reward + dfc
    elif(abs(resp[1])>5):
        dfc_reward=1e-9;
    else:
        dfc = 500/(1 + 100*(distance_from_center - track_width));
        dfc_reward = dfc_reward + dfc;

    
    steering_and_distance_from_center_reward = (dfc_reward + (500/(1+100*abs(resp[0]-steering_angle))))
    speed_reward = 0;
    if(abs(resp[1])<=10):
        max_speed = 0;        
        if(speed>2):
            max_speed = 2;
            speed_reward=speed_reward+100;
        if(speed>2.5):
            max_speed = 2.5;
            speed_reward=speed_reward+100;
        if(speed>3):
            max_speed = 3;
            speed_reward=speed_reward+100;
        if(speed>3.5):
            max_speed = 3.5
            speed_reward=speed_reward+100;
        if(speed>4):
            max_speed=4
            speed_reward = speed_reward+100;
            
        speed_reward = speed_reward + 100*((speed-max_speed)/0.5)
    else:
        req_speed = get_abs_speed(abs(resp[0]))
        speed_reward= 500/(1+abs(req_speed-speed))
    
    print('Required Steering: {}, Track Direction : "{}!"'.format(resp[0], resp[1]))
    return float(speed_reward) * float(steering_and_distance_from_center_reward) * float(progress_reward(params)) * 0.00001;
