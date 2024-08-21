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
def smooth_central_line(center_line, max_offset, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
    if max_offset < 0.0001:
        return center_line
    if skip_step < 1:
        skip_step = 1
    smoothed_line = center_line
    for i in range(0, iterations):
        smoothed_line = smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
    return smoothed_line

def smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step):
    length = len(center_line)
    new_line = [[0.0 for _ in range(2)] for _ in range(length)]
    for i in range(0, length):
        wpp = smoothed_line[(i - 2 * skip_step + length) % length]
        wp = smoothed_line[(i - skip_step + length) % length]
        wc = smoothed_line[i]
        wn = smoothed_line[(i + skip_step) % length]
        wnn = smoothed_line[(i + 2 * skip_step) % length]
        new_line[i][0] = pp * wpp[0] + p * wp[0] + c * wc[0] + n * wn[0] + nn * wnn[0]
        new_line[i][1] = pp * wpp[1] + p * wp[1] + c * wc[1] + n * wn[1] + nn * wnn[1]
        while calc_distance(new_line[i], center_line[i]) >= max_offset:
            new_line[i][0] = (0.98 * new_line[i][0]) + (0.02 * center_line[i][0])
            new_line[i][1] = (0.98 * new_line[i][1]) + (0.02 * center_line[i][1])
    return new_line
def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.hypot(delta_x, delta_y)
def orientation(x1,y1,x2,y2, x3, y3):
     
    # to find the orientation of 
    # an ordered triplet (p1,p2,p3)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
    val = (float(y2 - y1) * (x3 - x2)) - \
           (float(x2 - x1) * (y3 - y2))
    if (val > 0):
         
        # Clockwise orientation
        return False
    elif (val < 0):
         
        # Counterclockwise orientation
        return True
    else:
         
        # Collinear orientation
        return True if x2 == (x1+x3)/2 and y2 ==(y1+y3)/2 else False
def eucledian_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
def closest_distance(x,y,optimal_waypoints):
    opt_len = len(optimal_waypoints)
    dist = 10000
    for i in range(0,opt_len):
        if orientation(x,y,optimal_waypoints[i][0],optimal_waypoints[i][1],optimal_waypoints[(i+1)%opt_len][0],optimal_waypoints[(i+1)%opt_len][1]):
            dist = min(dist,eucledian_distance(x,y,optimal_waypoints[i][0],optimal_waypoints[i][1]))
    return dist if dist!=10000 else -1
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    reward = 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    # Calculate the direction of the center line based on the closest waypoints
    track_width =params['track_width']
    RACING_LINE_VS_CENTRAL_LINE = 0.90
    max_offset = track_width * RACING_LINE_VS_CENTRAL_LINE * 0.5
    optimal_waypoints = smooth_central_line(waypoints, max_offset)
    dist = closest_distance(params['x'],params['y'],optimal_waypoints)
    reward =1e-9
    print(dist)
    if dist!=-1:
        reward = 100/(1+ 5*closest_distance(params['x'],params['y'],optimal_waypoints))
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])
        reward += progress_reward
    else:
        return 1e-3
    reward=reward+ params['speed']**2
    return float(reward)