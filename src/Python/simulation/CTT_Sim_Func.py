from array import array
import numpy as np

# function to calculate point stabilization output (speed and yaw rate)
def calculate_point_stabilization(xnow, ynow, yaw_now, x_target, y_target, yaw_target, k_rho, k_alpha, l_car, kv_alpha):
  # error calc
  # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
  # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
  # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
  # The orientation value is substracted by pi, to change the pi/2 to -pi/2
  rho_ = np.sqrt((xnow-x_target)**2+(ynow-y_target)**2)
  errx_ = np.sqrt((xnow-x_target)**2)
  vi_ = np.arctan2(-(ynow-y_target),-(xnow-x_target))-(yaw_target-np.pi)+10**(-32)
  alpha_ = np.arctan2(-(ynow-y_target),-(xnow-x_target))-(yaw_now-np.pi)+10**(-32)
  # linear v
  v = (k_rho * rho_ * np.cos(alpha_))
  omega = (k_alpha * alpha_ + kv_alpha * k_rho * (vi_ + alpha_) * np.cos(alpha_) * (np.sin(alpha_)/alpha_))
  cs_steer = -np.arctan((l_car/v)*(omega))
  cs_steer = min(max(cs_steer, -0.785),0.785)

  return -v, cs_steer, rho_, alpha_, errx_

# update position, point stabilization special (only if using omega input)
def update_position_point_stab(X, Y, V, yaw, cs_steer, dt, l_car):
    omega = ((V/(l_car))*np.tan(cs_steer))
    yaw = yaw + omega*dt
    X_AV = X + V * np.cos(yaw) * dt
    Y_AV = Y + V * np.sin(yaw) * dt
    yaw_pub = yaw
    return X_AV, Y_AV, yaw_pub

# to calculate steer control signal using Stanley Controller (Trajectory Tracking)
def calculate_stanley_control_traj(X, Y, yaw, t_now, X_wp, Y_wp, dt_wp, v_wp, yaw_wp):
    # check waypoint to follow based on time trajectory
    T_tr_idx = (np.argmin(np.square(np.array(dt_wp)[0:]-t_now)))
    if t_now > dt_wp[T_tr_idx]: 
        T_tr_idx += 1
    
    # in case, it going to maintain (actually its prepared on the other function)
    if len(X_wp) == 1: 
        T_tr_idx = 0

    dt = dt_wp[T_tr_idx] - t_now

    V_AV = v_wp[T_tr_idx]

    # delta (Use Stanley!)
    if dt == 0 : 
      cs_steer = 0
    else: 
      _e_yaw = yaw_wp[T_tr_idx] - yaw
      _e_yaw = (_e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)
      _dtrx = X_wp[T_tr_idx] - X_wp[T_tr_idx-1]
      _dtry = Y_wp[T_tr_idx] - Y_wp[T_tr_idx-1]
      c = _dtrx * Y_wp[T_tr_idx-1] -  _dtry * X_wp[T_tr_idx-1]
      _e_lat = ((X*_dtry)+ c - (Y*_dtrx))/np.sqrt(_dtrx**2+_dtry**2) + 10**(-32)
      cs_steer = _e_yaw + np.arctan(50*_e_lat / (5 + V_AV))
      cs_steer = min(max(cs_steer, -0.785),0.785) #maximum 45 degree

    if np.isnan(cs_steer): 
        cs_steer = 0
        print("nan!")
    
    return V_AV, cs_steer

# to calculate steer control signal using Stanley Controller (Path Tracking)
def calculate_stanley_control_path(X, Y, yaw, X_wp, Y_wp, v_wp, yaw_wp):
    # check waypoint to follow based on nearest path
    # T_tr_idx = np.argmin(np.sum(np.square(np.array([X_wp, Y_wp]) - np.array([X, Y])), axis=-1))
    T_tr_idx = np.argmin((np.array(X_wp)[0:]-np.array(X))**2 + (np.array(Y_wp)[0:]-np.array(Y))**2)
    # T_tr_idx = 0
    # always track the first waypoints
    if T_tr_idx == 0: 
      T_tr_idx = 1
    print(T_tr_idx)
    V_AV = v_wp[T_tr_idx]

    # delta (Use Stanley!)
    _e_yaw = yaw_wp[T_tr_idx] - yaw
    _e_yaw = (_e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)
    _dtrx = X_wp[T_tr_idx] - X_wp[T_tr_idx-1]
    _dtry = Y_wp[T_tr_idx] - Y_wp[T_tr_idx-1]
    c = _dtrx * Y_wp[T_tr_idx-1] -  _dtry * X_wp[T_tr_idx-1]
    _e_lat = ((X*_dtry)+ c - (Y*_dtrx))/np.sqrt(_dtrx**2+_dtry**2) + 10**(-32)
    cs_steer = _e_yaw + np.arctan(20*_e_lat / (5 + V_AV))
    cs_steer = min(max(cs_steer, -0.785),0.785) #maximum 45 degree

    if np.isnan(cs_steer): 
        cs_steer = 0
        print("nan!")
    
    return V_AV, cs_steer

# to update the AV position
def update_AV_position(X, Y, V, yaw, cs_steer, dt, l_car):
    omega = ((V/(l_car))*np.tan(cs_steer))
    yaw = yaw + omega*dt
    X_AV = X + V * np.cos(yaw) * dt
    Y_AV = Y + V * np.sin(yaw) * dt
    yaw_pub = yaw
    return X_AV, Y_AV, yaw_pub

def Trailer_(X_pos, Y_pos, yaw_,l_, w_): #(x position, y position, yaw angle, length, width)
  # Create polygon coordinates
  transform_coor = [[X_pos-l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)],
          [X_pos-l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
          [X_pos+l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
          [X_pos+l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)]]
  return transform_coor  

def head_(X_pos_t, Y_pos_t, yaw_t, l_t, w_t, l_h, w_h, yaw_h): 
  # create head center point
  h_center = [X_pos_t + (l_t/2-0.755)*np.cos(yaw_t), Y_pos_t + (l_t/2-0.755)*np.sin(yaw_t)]
  h_transform = [[h_center[0]-l_h/2*np.cos(yaw_h)+w_h/2*np.sin(yaw_h),h_center[1]-l_h/2*np.sin(yaw_h)-w_h/2*np.cos(yaw_h)],
          [h_center[0]-l_h/2*np.cos(yaw_h)-w_h/2*np.sin(yaw_h),h_center[1]-l_h/2*np.sin(yaw_h)+w_h/2*np.cos(yaw_h)],
          [h_center[0]+l_h/2*np.cos(yaw_h)-w_h/2*np.sin(yaw_h),h_center[1]+l_h/2*np.sin(yaw_h)+w_h/2*np.cos(yaw_h)],
          [h_center[0]+l_h/2*np.cos(yaw_h)+w_h/2*np.sin(yaw_h),h_center[1]+l_h/2*np.sin(yaw_h)-w_h/2*np.cos(yaw_h)]]
  return h_transform

def create_sigmoid_wp(X,Y,inc,Dx,Dy,k1,k2):
    rot_x = []
    rot_y = []
    # creating sigmoid waypoints
    for i in range(inc):
        X.append(X[-1]+Dx/inc)
        Y.append(Dy/(1+np.e**(-k1*(X[-1]-X[0]-k2)))+Y[0])

    # creating rotation matrices
    rot_angle = 0.5 * 3.14 #in rad
    rot_ = [[np.cos(rot_angle), -np.sin(rot_angle)],[np.sin(rot_angle),np.cos(rot_angle)]]
    for i in range(len(X)):
        rot_pos = np.dot([np.array(X)[i]-X[0],np.array(Y)[i]-Y[0]],np.transpose(rot_))
        rot_x.append(rot_pos[0]+X[0])
        rot_y.append(rot_pos[1]+Y[0])
    return rot_x, rot_y

def calc_time_trajectory(xcoor, ycoor, vmax):
  # Calculating trajectory time based on max speed and curvature
  x = np.asarray(xcoor)
  y = np.asarray(ycoor)
  yaw = np.zeros(len(xcoor))
  arc_length = np.zeros(len(xcoor))
  V_curv = []
  dt = np.zeros(len(xcoor)) 
  delta = np.zeros(len(xcoor)) 
  yaw_rate = np.zeros(len(xcoor)) 
  Kv = 100

  # print("coor_shape: "+str(x.shape))

  # calculate yaw
  diffx = x[2:] - x[:-2]
  diffy = y[2:] - y[:-2]
  yaw[1:-1] = np.arctan2(diffy, diffx)
  # print("this shape: "+str(np.arctan2(diffy, diffx).shape))
  yaw[0] = yaw[1]
  yaw[-1] = yaw[-2]

  # calculate yaw and arc length
  for i in range(len(xcoor)):
    dy = (y[i]-y[i-1])
    dx = (x[i]-x[i-1])
    e_dist = np.sqrt((dx**2+dy**2))
    arc_length[i] = (arc_length[i-1] + e_dist)
  curvature = np.gradient(yaw, arc_length)

  # calculate curvature speed
  V_curv = vmax/(1+Kv*(curvature)**2)

  # calculate dt at each waypoints
  for i in range(len(xcoor)):
    dy = (y[i]-y[i-1])
    dx = (x[i]-x[i-1])
    e_dist = np.sqrt((dx**2+dy**2))
    dt[i] = abs((e_dist/V_curv[i]))
  
  dt[0] = dt[1]

  return curvature, yaw, arc_length, V_curv, dt