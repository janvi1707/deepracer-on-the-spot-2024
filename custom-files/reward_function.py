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
def circle_indexes(mylist, index_car, add_index_1=0, add_index_2=0):

    list_len = len(mylist)

    # if index >= list_len:
    #     raise ValueError("Index out of range in circle_indexes()")

    # Use modulo to consider that track is cyclical
    index_1 = (index_car + add_index_1) % list_len
    index_2 = (index_car + add_index_2) % list_len

    return [index_car, index_1, index_2]
def circle_radius(coords):

    # Flatten the list and assign to variables (makes code easier to read later)
    x1, y1, x2, y2, x3, y3 = [i for sub in coords for i in sub]

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
def optimal_velocity(track, min_speed, max_speed, look_ahead_points):

    # Calculate the radius for every point of the track
    radius = []
    for i in range(len(track)):
        indexes = circle_indexes(track, i, add_index_1=-1, add_index_2=1)
        coords = [track[indexes[0]],
                  track[indexes[1]], track[indexes[2]]]
        radius.append(circle_radius(coords))

    # Get the max_velocity for the smallest radius
    # That value should multiplied by a constant multiple
    v_min_r = min(radius)**0.5
    constant_multiple = min_speed / v_min_r
    print(f"Constant multiple for optimal speed: {constant_multiple}")

    if look_ahead_points == 0:
        # Get the maximal velocity from radius
        max_velocity = [(constant_multiple * i**0.5) for i in radius]
        # Get velocity from max_velocity (cap at MAX_SPEED)
        velocity = [min(v, max_speed) for v in max_velocity]
        return velocity

    else:
        # Looks at the next n radii of points and takes the minimum
        # goal: reduce lookahead until car crashes bc no time to break
        LOOK_AHEAD_POINTS = look_ahead_points
        radius_lookahead = []
        for i in range(len(radius)):
            next_n_radius = []
            for j in range(LOOK_AHEAD_POINTS+1):
                index = circle_indexes(
                    mylist=radius, index_car=i, add_index_1=j)[1]
                next_n_radius.append(radius[index])
            radius_lookahead.append(min(next_n_radius))
        max_velocity_lookahead = [(constant_multiple * i**0.5)
                                  for i in radius_lookahead]
        velocity_lookahead = [min(round(v,1), round(max_speed,1))
                              for v in max_velocity_lookahead]
        return velocity_lookahead
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    # Calculate the direction of the center line based on the closest waypoints
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
    prev = int(closest_waypoints[0])
    next_index = int(closest_waypoints[1])
    next_point_1 = optimal_waypoints[next_index%waypoints_length]
    next_point_2 = optimal_waypoints[(next_index+1)%waypoints_length]
    next_point_3 = optimal_waypoints[(next_index+2)%waypoints_length]
    next_point_4 = optimal_waypoints[(next_index+3)%waypoints_length]
    next_point_5 = optimal_waypoints[(next_index+4)%waypoints_length]
    next_point_6 = optimal_waypoints[(next_index+5)%waypoints_length]
    prev_point = optimal_waypoints[prev%waypoints_length]
    prev_point_2 = optimal_waypoints[(prev-1+waypoints_length)%waypoints_length]
    prev_point_3 = optimal_waypoints[(prev-2+waypoints_length)%waypoints_length]
    prev_point_4 = optimal_waypoints[(prev-3+waypoints_length)%waypoints_length]
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    angle_f2= angle_between_lines(prev_point_4[0],prev_point_4[1],prev_point_3[0],prev_point_3[1],prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1])

    reward = 1e-9
    total_angle = angle_f
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if abs(total_angle)<5:
        total_angle=0
    if next_index ==1 or prev==1 or (next_index+1)%waypoints_length ==1 or (next_index+2)%waypoints_length ==1 or (next_index+3)%waypoints_length ==1 or (next_index+4)%waypoints_length ==1 or (next_index+5)%waypoints_length ==1 or (next_index+6)%waypoints_length ==1 or (next_index+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    steering_reward = 100/(1+abs(params['steering_angle']-total_angle))
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])+ params['progress']//2
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    if direction_diff <=10.0:
        reward+=10.0
    LOOK_AHEAD_POINTS = 4
    MIN_SPEED = 1.3
    MAX_SPEED = 4

    # Calculate optimal speed
    optimal_speed_waypoints = optimal_velocity(track=optimal_waypoints, 
        min_speed=MIN_SPEED, max_speed=MAX_SPEED, look_ahead_points=LOOK_AHEAD_POINTS)
    reward = reward + 200/(1+abs(optimal_speed_waypoints[next_index%waypoints_length]-params['speed']))
        
    if abs(params['steering_angle'])<10 and abs(total_angle)>20:
        return 1e-3
    if total_angle >20 and params['is_left_of_center']:
        reward+=100.0
    if total_angle < -20 and not params['is_left_of_center']:
        reward+=100.0


    # if total_angle>26 and params['is_left_of_center']:
    #     reward+=30.0
    # if total_angle<-26 and not params['is_left_of_center']:
    #     reward+=30.0

    steps=params['steps']
    progress= params['progress']
    if steps>0:
        step_reward= ((progress*25)/steps)**3

    reward+=step_reward

    if abs(params['steering_angle']-total_angle) >=20:
        reward*=0.25
    return float(reward)