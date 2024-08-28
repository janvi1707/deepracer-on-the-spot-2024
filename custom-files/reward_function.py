import math
import numpy as np
class PARAMS:
    prev_speed = None
    prev_steering_angle = None 
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None

def smooth_central_line(center_line, max_offset, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
    if max_offset < 0.0001:
        return center_line
    if skip_step < 1:
        skip_step = 1
    smoothed_line = center_line
    for i in range(0, iterations):
        smoothed_line = smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
    return smoothed_line
def circle_radius(x1, y1, x2, y2, x3, y3):
    a = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2
    b = (x1**2+y1**2)*(y3-y2) + (x2**2+y2**2)*(y1-y3) + (x3**2+y3**2)*(y2-y1)
    c = (x1**2+y1**2)*(x2-x3) + (x2**2+y2**2)*(x3-x1) + (x3**2+y3**2)*(x1-x2)
    d = (x1**2+y1**2)*(x3*y2-x2*y3) + (x2**2+y2**2) * \
        (x1*y3-x3*y1) + (x3**2+y3**2)*(x2*y1-x1*y2)

    # In case a is zero (so radius is infinity)
    try:
        r = abs((b**2+c**2-4*a*d) / abs(4*a**2)) ** 0.5
    except:
        r = 999

    return r
def generate_optimal_speed_array(track_points):
    list =[]
    length= len(track_points)
    for i in range(0,length):
        next_point_1 = track_points[i]
        next_point_2 = track_points[(i+1)%length]
        prev_point_1 = track_points[(i-1+length)%length]
        list.append(min(1.69*math.sqrt(circle_radius(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],prev_point_1[0],prev_point_1[1])),4))
    return np.array(list)
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
def eucledian_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
def angle(x1,y1,x2,y2):
    return math.degrees(math.atan2(y2-y1,x2-x1))
def closest_route_point(x,y,optimal_waypoints):
    opt_len = len(optimal_waypoints)
    list = []
    for i in range(0,opt_len):
        list.append((eucledian_distance(x,y,optimal_waypoints[i][0],optimal_waypoints[i][1]),optimal_waypoints[i][0],optimal_waypoints[i][1],i))
    dist = sorted(list, key = lambda x: x[0])
    print("Two closest points:",dist[0],dist[1])
    if dist[0][3] < dist[1][3] or dist[1][3]==0:
        print("Closest optimal point :", dist[0])
        return dist[0][3],dist[1][3]
    else:
        print("Closest optimal point :",dist[1])
        return dist[1][3],dist[0][3]
def normalized_distance(prev_point,next_point,vehicle_x,vehicle_y):
    x1=prev_point[0]
    y1=prev_point[1]
    x2=next_point[0]
    y2=next_point[1]
    x0=vehicle_x
    y0=vehicle_y
    return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/ np.sqrt(np.square(x2-x1) + np.square(y2-y1))
def reward_function(params):
    curve_points=[17,18,19,20,21,22,23,24,25,26,27,28,29,30,54,55,56,57,58,59,60,61,62,97,98,99,100,101,102,103,104,105,106,107,108,109,110,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,153,154,155,156,157,158,159,160,183,184,185,186,187,188,189,190,191,192,193,194,195,196]
    straight_points=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 64, 65, 66, 67, 68, 69, 70, 71, 72, 81, 82, 83, 84, 85,86,87,88,89,90, 91, 92, 93, 94, 118, 119, 120, 127, 128, 129, 130, 131, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213]
    # Read input parameters
    heading = params['heading']
    IC=0
    distance_from_center = params['distance_from_center']
    steps = params['steps']
    steering_angle = params['steering_angle']
    speed = params['speed']
    track_width =params['track_width']
    waypoints = params['waypoints']
    progress = params['progress']
    RACING_LINE_VS_CENTRAL_LINE = 0.60
    max_offset = track_width * RACING_LINE_VS_CENTRAL_LINE * 0.5
    optimal_waypoints = [[-0.63851828 ,-5.40203879],
                        [-0.50851834 ,-5.40191698],
                        [-0.3785184  ,-5.40179178],
                        [-0.2069995  ,-5.40162659],
                        [ 0.09451947 ,-5.40133858],
                        [ 0.3960389  ,-5.40108657],
                        [ 0.69747887 ,-5.39988982],
                        [ 0.99869665 ,-5.39663928],
                        [ 1.29950383 ,-5.39035426],
                        [ 1.59964687 ,-5.37999128],
                        [ 1.89881002 ,-5.36451067],
                        [ 2.19665229 ,-5.34304757],
                        [ 2.49273284 ,-5.31458538],
                        [ 2.78651842 ,-5.27809399],
                        [ 3.0773626  ,-5.23253328],
                        [ 3.36448276 ,-5.17686648],
                        [ 3.64694145 ,-5.11009174],
                        [ 3.92364131 ,-5.0312971 ],
                        [ 4.19333449 ,-4.93971925],
                        [ 4.45465082 ,-4.83478842],
                        [ 4.7061744  ,-4.7161881 ],
                        [ 4.94659414 ,-4.5839437 ],
                        [ 5.17493849 ,-4.43854146],
                        [ 5.39077431 ,-4.28097505],
                        [ 5.59348243 ,-4.11194776],
                        [ 5.78131375 ,-3.93127588],
                        [ 5.95185821 ,-3.7385911 ],
                        [ 6.10141609 ,-3.53316711],
                        [ 6.22541583 ,-3.31451204],
                        [ 6.31598113 ,-3.08173776],
                        [ 6.36351641 ,-2.83533361],
                        [ 6.38205328 ,-2.58189332],
                        [ 6.37720704 ,-2.32356466],
                        [ 6.35305789 ,-2.0616314 ],
                        [ 6.31278016 ,-1.79691244],
                        [ 6.26071735 ,-1.53008   ],
                        [ 6.20075856 ,-1.26151856],
                        [ 6.13148151 ,-0.97499586],
                        [ 6.06725969 ,-0.68699413],
                        [ 6.00823021 ,-0.39763843],
                        [ 5.95430626 ,-0.10706366],
                        [ 5.90555805 , 0.18467244],
                        [ 5.86167859 , 0.47745648],
                        [ 5.82214921 , 0.77115282],
                        [ 5.78658551 , 1.06566694],
                        [ 5.75499429 , 1.36099   ],
                        [ 5.7273929  , 1.65711642],
                        [ 5.70403118 , 1.95408586],
                        [ 5.67622257 , 2.24136477],
                        [ 5.64203015 , 2.5248385 ],
                        [ 5.59952207 , 2.80318752],
                        [ 5.5476693  , 3.07557648],
                        [ 5.48424899 , 3.34029551],
                        [ 5.40673494 , 3.5951855 ],
                        [ 5.31269497 , 3.83778359],
                        [ 5.19945093 , 4.06489432],
                        [ 5.06405847 , 4.27221525],
                        [ 4.90357323 , 4.45391204],
                        [ 4.71624975 , 4.60305295],
                        [ 4.50929325 , 4.72427619],
                        [ 4.28749415 , 4.82192623],
                        [ 4.05341077 , 4.89827963],
                        [ 3.80913554 , 4.95560131],
                        [ 3.55636319 , 4.99618293],
                        [ 3.29647986 , 5.0226788 ],
                        [ 3.03050225 , 5.03816303],
                        [ 2.75933219 , 5.04598488],
                        [ 2.4847027  , 5.04973247],
                        [ 2.20105649 , 5.05286287],
                        [ 1.91741332 , 5.0562718 ],
                        [ 1.63377218 , 5.05986783],
                        [ 1.35013407 , 5.06374228],
                        [ 1.06913544 , 5.0656566 ],
                        [ 0.79099924 , 5.06326493],
                        [ 0.51523701 , 5.05453417],
                        [ 0.24184319 , 5.03734901],
                        [-0.02891155 , 5.00954634],
                        [-0.2966027  , 4.96896438],
                        [-0.56076833 , 4.91369043],
                        [-0.82104174 , 4.84251073],
                        [-1.07719845 , 4.75500635],
                        [-1.32918907 , 4.65155482],
                        [-1.57717039 , 4.53330455],
                        [-1.82152917 , 4.40213017],
                        [-2.0628913  , 4.26057004],
                        [-2.30209933 , 4.11170762],
                        [-2.54013819 , 3.9589377 ],
                        [-2.78502742 , 3.80260483],
                        [-3.03076344 , 3.64774898],
                        [-3.27767744 , 3.49494852],
                        [-3.52591953 , 3.34446632],
                        [-3.77544302 , 3.19622181],
                        [-4.02622004 , 3.05016727],
                        [-4.27822235 , 2.90625443],
                        [-4.53158624 , 2.76472314],
                        [-4.76856706 , 2.62891224],
                        [-5.00013379 , 2.48946189],
                        [-5.22346031 , 2.34462087],
                        [-5.43561287 , 2.19275164],
                        [-5.63351654 , 2.03240911],
                        [-5.81420206 , 1.86257991],
                        [-5.97209628 , 1.68158528],
                        [-6.10177822 , 1.48932432],
                        [-6.20881029 , 1.29001415],
                        [-6.29165001 , 1.08483022],
                        [-6.34568432 , 0.87486589],
                        [-6.36563899 , 0.66238044],
                        [-6.34463336 , 0.45145615],
                        [-6.27273298 , 0.2501886 ],
                        [-6.14251428 , 0.07320818],
                        [-5.97402651 ,-0.07844622],
                        [-5.7749891  ,-0.20366538],
                        [-5.55225412 ,-0.30312909],
                        [-5.31066221 ,-0.37696577],
                        [-5.0547667  ,-0.42588075],
                        [-4.78875118 ,-0.45104676],
                        [-4.51626663 ,-0.45390519],
                        [-4.24034256 ,-0.43611482],
                        [-3.96335038 ,-0.39951836],
                        [-3.68705067 ,-0.34569911],
                        [-3.4127742  ,-0.27583829],
                        [-3.14154961 ,-0.19091664],
                        [-2.87415908 ,-0.09184913],
                        [-2.61117104 , 0.02045709],
                        [-2.35315422 , 0.1454891 ],
                        [-2.10014508 , 0.28175347],
                        [-1.85244772 , 0.428568  ],
                        [-1.61006185 , 0.58462117],
                        [-1.37281869 , 0.74838715],
                        [-1.14030905 , 0.91819708],
                        [-0.91170669 , 1.09227208],
                        [-0.68551358 , 1.2686764 ],
                        [-0.47579284 , 1.43097103],
                        [-0.26488665 , 1.58359584],
                        [-0.05155004 , 1.71849625],
                        [ 0.16557884 , 1.82851951],
                        [ 0.38744094 , 1.9072072 ],
                        [ 0.61392329 , 1.94740563],
                        [ 0.84285481 , 1.93679984],
                        [ 1.06952177 , 1.88404572],
                        [ 1.2901971  , 1.79516546],
                        [ 1.50185283 , 1.67565272],
                        [ 1.70166851 , 1.52937005],
                        [ 1.88739242 , 1.36022386],
                        [ 2.05649161 , 1.1710819 ],
                        [ 2.2077775  , 0.96577562],
                        [ 2.3398285  , 0.74719801],
                        [ 2.45155114 , 0.5181329 ],
                        [ 2.54266106 , 0.28136119],
                        [ 2.61242796 , 0.03919568],
                        [ 2.660511   ,-0.20611522],
                        [ 2.68704272 ,-0.45248701],
                        [ 2.69254879 ,-0.69807102],
                        [ 2.67744704 ,-0.94122461],
                        [ 2.64079746 ,-1.18011295],
                        [ 2.5818738  ,-1.41276923],
                        [ 2.49749273 ,-1.63601249],
                        [ 2.38432597 ,-1.84555704],
                        [ 2.23641854 ,-2.03389618],
                        [ 2.04725217 ,-2.18885775],
                        [ 1.83543469 ,-2.32122959],
                        [ 1.60482601 ,-2.43276452],
                        [ 1.35816903 ,-2.52477566],
                        [ 1.09808521 ,-2.59897351],
                        [ 0.82684064 ,-2.65706579],
                        [ 0.54681886 ,-2.70140387],
                        [ 0.26007625 ,-2.73434183],
                        [-0.03170644 ,-2.7580615 ],
                        [-0.32717589 ,-2.77453832],
                        [-0.62524903 ,-2.78554114],
                        [-0.92506184 ,-2.79264569],
                        [-1.22589271 ,-2.79733703],
                        [-1.52727103 ,-2.80067146],
                        [-1.82877499 ,-2.80369198],
                        [-2.13027906 ,-2.80671251],
                        [-2.43172492 ,-2.80984863],
                        [-2.73229746 ,-2.81476426],
                        [-3.0310403  ,-2.82324026],
                        [-3.32690058 ,-2.83698214],
                        [-3.61870945 ,-2.85764578],
                        [-3.90518212 ,-2.88681246],
                        [-4.18492487 ,-2.92594847],
                        [-4.45626615 ,-2.97659488],
                        [-4.71724214 ,-3.04027957],
                        [-4.96567569 ,-3.11832302],
                        [-5.19904403 ,-3.21190689],
                        [-5.41446457 ,-3.32196302],
                        [-5.60780474 ,-3.44964291],
                        [-5.77455439 ,-3.59517728],
                        [-5.90899949 ,-3.75810113],
                        [-6.00396797 ,-3.93655454],
                        [-6.05068117 ,-4.12573167],
                        [-6.06155256 ,-4.31760491],
                        [-6.03306928 ,-4.5082205 ],
                        [-5.95720439 ,-4.69132616],
                        [-5.81771953 ,-4.85102768],
                        [-5.64229614 ,-4.98955805],
                        [-5.43618141 ,-5.10552841],
                        [-5.20466676 ,-5.19901526],
                        [-4.95270824 ,-5.27123116],
                        [-4.68477105 ,-5.32426217],
                        [-4.40483081 ,-5.36091715],
                        [-4.11616987 ,-5.3842177 ],
                        [-3.82150848 ,-5.39731472],
                        [-3.52292416 ,-5.40306987],
                        [-3.22218907 ,-5.40444136],
                        [-2.92066991 ,-5.40417147],
                        [-2.61915147 ,-5.40389252],
                        [-2.31763208 ,-5.40360737],
                        [-2.01611352 ,-5.40332508],
                        [-1.71459448 ,-5.40304351],
                        [-1.41307551 ,-5.40276241],
                        [-1.11155647 ,-5.4024806 ],
                        [-0.81003734 ,-5.40219951]]
    waypoints_length= len(optimal_waypoints)
    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None
        PARAMS.prev_direction_diff = None
        PARAMS.prev_normalized_distance_from_route = None
    optimal_speed_array = generate_optimal_speed_array(optimal_waypoints)
###########################################################
    #Heading Reward
###########################################################
    heading = params['heading']
    vehicle_x = params['x']
    vehicle_y = params['y']
    prev_closest_point_index,next_closest_point_index = closest_route_point(vehicle_x,vehicle_y,optimal_waypoints)
    next_route_point_x = optimal_waypoints[(next_closest_point_index+4)%waypoints_length][0]
    next_route_point_y = optimal_waypoints[(next_closest_point_index+4)%waypoints_length][1]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians between target and current vehicle position
    route_direction = math.atan2(next_route_point_y - vehicle_y, next_route_point_x - vehicle_x) 
    # Convert to degree
    route_direction = math.degrees(route_direction)
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = route_direction - heading
    #Check that the direction_diff is in valid range
    #Then compute the heading reward
    heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 10
    if abs(direction_diff) <= 20:
        heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 4
###########################################################    
    #Distance Reward
###########################################################
    #distance reward is value of the standard normal scaled back to 1. #Hence the 1/2*pi*sigma term is cancelled out
    distance_reward = 0
    normalized_car_distance_from_route = normalized_distance(optimal_waypoints[prev_closest_point_index],optimal_waypoints[next_closest_point_index],vehicle_x,vehicle_y)/track_width
    if normalized_car_distance_from_route == 0: #i.e. on the route line
        distance_from_route = 0
        distance_reward = 1
    elif normalized_car_distance_from_route > 0: #i.e. on right side of the route line
        normalized_route_distance_from_inner_border = params['distance_from_center']
        if params['is_left_of_center']:
            normalized_route_distance_from_inner_border = 0.5*params['track_width'] - normalized_route_distance_from_inner_border
        else:
            normalized_route_distance_from_inner_border+= 0.5*params['track_width']
        normalized_route_distance_from_inner_border/=track_width
        sigma=abs(normalized_route_distance_from_inner_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
    elif normalized_car_distance_from_route < 0: #i.e. on left side of the route line
        normalized_route_distance_from_outer_border = params['distance_from_center']
        if not params['is_left_of_center']:
            normalized_route_distance_from_outer_border = 0.5*params['track_width'] - normalized_route_distance_from_outer_border
        else:
            normalized_route_distance_from_outer_border+= 0.5*params['track_width']
        normalized_route_distance_from_outer_border/=track_width
        sigma=abs(normalized_route_distance_from_outer_border / 4) 
        distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
###########################################################
    #Speed Reward
###########################################################
    MIN_SPEED = 1.4
    MAX_SPEED = 4.0
    optimal_speed = 0
    #chosen such that 6 standard deviations covers the entire range
    sigma_speed = abs(MAX_SPEED - MIN_SPEED)/6.0
    i = next_closest_point_index
    nsteps = 5
    if i+nsteps < waypoints_length:
        optimal_speed = min( optimal_speed_array[i:(i+nsteps)%waypoints_length] )
    else:
        optimal_speed = min( min(optimal_speed_array[i:]), min(optimal_speed_array[:(i+nsteps)%waypoints_length+1]) )
        
    #PARAMS.optimal speed is unbounded and hence must be bounded from above to reflect action space realities
    optimal_speed = min(MAX_SPEED,optimal_speed)
    speed_reward = math.exp(-0.5*abs(speed-optimal_speed)**2 / sigma_speed**2)

# Reinitialize previous parameters if it is a new episode
    #Check if the speed has dropped
    has_speed_dropped = False
    if PARAMS.prev_speed is not None:
        if PARAMS.prev_speed > speed:
            has_speed_dropped = True
    speed_maintain_bonus= 1
    #Penalize slowing down without good reason on straight portions
    if has_speed_dropped and not params['closest_waypoints'][1] in curve_points: 
        speed_maintain_bonus = min( speed / PARAMS.prev_speed, 1 )
    #Penalize making the heading direction worse
    heading_decrease_bonus = 0
    is_heading_in_right_direction = True if abs(direction_diff) <=30 else False
    if PARAMS.prev_direction_diff is not None:
        if is_heading_in_right_direction:
            if abs( PARAMS.prev_direction_diff / direction_diff ) > 1:
                heading_decrease_bonus = min(10, abs( PARAMS.prev_direction_diff / direction_diff ))
    #has the steering angle changed
    has_steering_angle_changed = False
    if PARAMS.prev_steering_angle is not None:
        if not(math.isclose(PARAMS.prev_steering_angle,steering_angle)):
            has_steering_angle_changed = True
    steering_angle_maintain_bonus = 1 
    #Not changing the steering angle is a good thing if heading in the right direction
    if is_heading_in_right_direction and not has_steering_angle_changed:
        if abs(direction_diff) < 10:
            steering_angle_maintain_bonus *= 2
        if abs(direction_diff) < 5:
            steering_angle_maintain_bonus *= 2
        if PARAMS.prev_direction_diff is not None and abs(PARAMS.prev_direction_diff) > abs(direction_diff):
            steering_angle_maintain_bonus *= 2
    #Reward reducing distance to the race line
    distance_reduction_bonus = 1
    if PARAMS.prev_normalized_distance_from_route is not None and PARAMS.prev_normalized_distance_from_route > normalized_car_distance_from_route:
        if abs(normalized_car_distance_from_route) > 0:
            distance_reduction_bonus = min( abs( PARAMS.prev_normalized_distance_from_route / normalized_car_distance_from_route ), 2)
    # Before returning reward, update the variables
    PARAMS.prev_speed = speed
    PARAMS.prev_steering_angle = steering_angle
    PARAMS.prev_direction_diff = direction_diff
    PARAMS.prev_steps = steps
    PARAMS.prev_normalized_distance_from_route = normalized_car_distance_from_route
    #heading component of reward
    HC = ( 10 * heading_reward * steering_angle_maintain_bonus )
    #distance component of reward
    DC = ( 10 * distance_reward * distance_reduction_bonus )
    #speed component of reward
    SC = ( 5 * speed_reward * speed_maintain_bonus )
    #Immediate component of reward
    IC = ( HC + DC + SC ) ** 2 + ( HC * DC * SC ) 
#If an unpardonable action is taken, then the immediate reward is 0
#########################################################
# Unpardonable Actions
#########################################################
    if params['is_offtrack'] or params['is_crashed']:
        IC = 0
    if abs(direction_diff) > 30:
        IC = 0
    if normalized_car_distance_from_route * params['steering_angle'] < 0 :
        IC = 0
    if params['speed'] > optimal_speed+1.3 and params['closest_waypoints'][1] in curve_points:
        IC = 0
    if params['speed'] + 1.5 < optimal_speed and params['closest_waypoints'][1] in straight_points:
        IC = 0
    # Reward for making steady progress
    progress_reward = 10 * progress / steps
    if steps <= 5:
        progress_reward = 1 #ignore progress in the first 5 steps
    # Bonus that the agent gets for completing every 10 percent of track
    # Is exponential in the progress / steps. 
    # exponent increases with an increase in fraction of lap completed
    intermediate_progress_bonus = 0
    pi = int(progress//10)
    if pi != 0:
        if pi==10: # 100% track completion
            intermediate_progress_bonus = progress_reward ** 14
    else:
        intermediate_progress_bonus = progress_reward ** (5+0.75*pi)
    #Long term component of reward
    #TO-DO
    curve_bonus=0
    straight_section_bonus=0
    LC = ( curve_bonus + intermediate_progress_bonus + straight_section_bonus )
    return max(IC + LC,1e-3)
    