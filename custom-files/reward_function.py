import math


def progress_reward(params):
    progress = params['progress']
    reward = (progress)/100
    return 100;
    
def direction_reward(direction_diff):
    direction_reward = 0;
    max_angle = 30;
    if(direction_diff<=30):
        direction_reward += 100;
        max_angle = 25;
    if(direction_diff<=25):
        direction_reward += 100;
        max_angle = 20;
    if(direction_diff<=20):
        direction_reward += 100;
        max_angle = 15;
    if(direction_diff<=15):
        direction_reward += 100;
        max_angle = 10;
    if(direction_diff<=10):
        direction_reward += 100;
        max_angle = 5;
    if(direction_diff<=5):
        direction_reward += 100;
        max_angle = 0;
    
        
    direction_reward = direction_reward + (100/(1+(direction_diff-max_angle)));

    return direction_reward;
    
def get_abs_speed(diff):
    steer = abs(diff)
    if(steer>=20):
        return max(1.4,55/(5+steer));
    return max(1.4,min(4,63.6/(8.9+steer)))

def fetch_required_steering_angle(waypoints,closest_waypoints,x,y,track_width,heading):
    w_len = len(waypoints);
    start = int(closest_waypoints[0]);
    end = int(closest_waypoints[1]+20);
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
    
    next_start = waypoints[int(closest_waypoints[1])]
    
    ideal_heading = math.atan2(next_start[1]-start[1],next_start[0]-start[0])
    ideal_heading = math.degrees(ideal_heading)
    direction_diff = track_direction-ideal_heading

    if(direction_diff>180):
        direction_diff = direction_diff-360;
    elif(direction_diff<-180):
        direction_diff = direction_diff+360;
    track_direction = direction_diff;


    
    return [round(req_steer), round(track_direction)]
    

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
    
    req_steering_angle = resp[0];
    track_direction = resp[1];
    
    direction_diff = abs(round(resp[0])-steering_angle)
    if direction_diff>180:
        direction_diff = 360-direction_diff;

    direction_reward = direction_reward(direction_diff);
    distance_from_center_reward = 0;
    
    if(resp[1]<-7 and not is_left_of_center):
        distance_from_center_reward = 100;
        dfc = track_width * (abs(resp[1])/60)
        dfc = 500/(1 + 10*abs(distance_from_center - dfc))
        distance_from_center_reward = distance_from_center_reward + dfc
    elif(resp[1]>7 and is_left_of_center):
        distance_from_center_reward = 100;
        dfc = track_width * (abs(resp[1])/60)
        dfc = 500/(1 + 10*abs(distance_from_center - dfc))
        distance_from_center_reward = distance_from_center_reward + dfc
    elif(abs(resp[1])>7):
        distance_from_center_reward = 100/(1 + 100*(distance_from_center/track_width));
    else:
        distance_from_center_reward = 600/(1 + 100*(distance_from_center/track_width));
    
    speed_reward = 0;
    max_speed = 0;
    if(abs(resp[1])<=7):
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
        speed_reward = 500/(1+10*abs(req_speed-speed))
    
    start = int(closest_waypoints[0]);
    end = int(closest_waypoints[1]);
    if(waypoints[start][0]==waypoints[end][0] and waypoints[start][1]==waypoints[end][1]):
        start = (len(waypoints)+start-1)%len(waypoints);
    track_direction = math.atan2(waypoints[end][1]-waypoints[start][1],waypoints[end][0]-waypoints[start][0])
    track_direction = math.degrees(track_direction)

    heading_diff = track_direction-heading


    print("Calculated Steering {}, Track direction : {} , Heading diff : {}".format(resp[0],resp[1],heading_diff))
    if(heading_diff>180):
        heading_diff = heading_diff-360;
    elif(heading_diff<-180):
        heading_diff = heading_diff+360;

    heading_reward = direction_reward(abs(heading_diff));
    
    if(abs(resp[1])<=7):
        return (((5*float(speed_reward)+3*float(distance_from_center_reward)+3*float(heading_reward))/11)**2 + float(direction_reward)) * 0.0001;
    else:
        return ((float(direction_reward)+float(distance_from_center_reward))**2 + float(speed_reward)) * 0.0001;
