import matplotlib.pyplot as plt

# X_start,            Y_Start,     X_end_pointstab,   Y_end_pointstab,   X_end_dock,       Y_end_dock
# 558098.5399999999   9158001.32   558091.9866405     9157952.32         558091.9866405    9157942.32

# Yaw Pos lane 2-7 (dihitung dari paling timur) 
#      Lane 2            Lane 3             Lane 4            Lane 5             Lane 6              Lane 7
# Deg: 90.24888516460581 90.24659049383784  90.3908420592535  90.3808135597888   90.35716044482973   90.28002268266489
# rad: 1.575140192376636 1.5751001428164857 1.577617807584145 1.5774427772494515 1.5770299525150628  1.5756836445876663

# Pos 1 to pos 16 (garis tambangan, start dari weld 3 utara ke ujung selatan tambangan)
X_bridge = [558106.622215, 558103.485361, 558100.124337, 558096.585934, 558093.201858, 558089.771423, \
         558086.438205, 558083.146216, 558083.805074, 558086.989303, 558090.473591, 558093.950218, \
         558097.354744, 558100.609169, 558103.974908, 558107.324993, 558106.622215]
Y_bridge = [9158033.23834, 9158033.10195, 9158033.06035, 9158033.04034, 9158032.9398,  9158032.96462, \
         9158032.96076, 9158032.99646, 9157920.25556, 9157920.20083, 9157920.32409, 9157920.34602, \
         9157920.33782, 9157920.40939, 9157920.40419, 9157920.37652, 9158033.23834]

plt.figure(figsize=[10,10])
# new pos
plt.plot(X_utm, Y_utm)
plt.plot([X_utm[1],X_utm[14]],[Y_utm[1], Y_utm[14]], '--', c='b')
plt.plot([X_utm[2],X_utm[13]],[Y_utm[2], Y_utm[13]], '--', c='b')
plt.plot([X_utm[3],X_utm[12]],[Y_utm[3], Y_utm[12]], '--', c='b')
plt.plot([X_utm[4],X_utm[11]],[Y_utm[4], Y_utm[11]], '--', c='b')
plt.plot([X_utm[5],X_utm[10]],[Y_utm[5], Y_utm[10]], '--', c='b')
plt.plot([X_utm[6],X_utm[9]],[Y_utm[6], Y_utm[9]], '--', c='b')

# old pos
X_start_pointstab = 558096.84 + 1.7
Y_start_pointstab = 9158001.32
# X_end_pointstab = 558092.34
# Y_end_pointstab = 9157952.32
plt.scatter(X_start_pointstab, Y_start_pointstab)
# plt.scatter(X_end_pointstab, Y_end_pointstab)

# check truck old
# init pos
X_pos_t = X_start_pointstab 
Y_pos_t = Y_start_pointstab
yaw_t = 0.5*3.14
yaw_h = 0.5*3.14
l_t = 13.75
w_t = 2.682
l_h = 3.533
w_h = 2.682

trailer = Trailer_(X_pos_t, Y_pos_t, yaw_t, l_t, w_t)
head_center, head_transform = head_(X_pos_t, Y_pos_t, yaw_t, l_t, w_t, l_h, w_h, yaw_h)
x_t,y_t = Polygon(trailer).exterior.xy 
x_h,y_h = Polygon(head_transform).exterior.xy

# end pos
# X_end_t = 558092.34
# Y_end_t = 9157952.32
X_end_t = X_utm[5] + (abs(X_utm[5] - X_utm[4])/2) + 0.5
Y_end_t = 9157952.32
yaw_end_t = 1.5774427772494515
yaw_end_h = 1.5774427772494515
plt.scatter(X_end_t, Y_end_t)

trailer = Trailer_(X_end_t, Y_end_t, yaw_end_t, l_t, w_t)
head_center, head_transform = head_(X_end_t, Y_end_t, yaw_end_t, l_t, w_t, l_h, w_h, yaw_end_h)
x_endt,y_endt = Polygon(trailer).exterior.xy 
x_endh,y_endh = Polygon(head_transform).exterior.xy

plt.plot(x_t,y_t, 'g')
plt.plot(x_h,y_h, 'g')

# plt.plot(x_endt,y_endt, 'g')
# plt.plot(x_endh, y_endh, 'g')

# dock pos
# X_dock_t = 558092.34
Y_dock_t = 9157942.32
X_dock_t = X_utm[5] + (abs(X_utm[5] - X_utm[4])/2) + 0.5
# Y_dock_t = 9157942.32
yaw_dock_t = 1.5774427772494515
yaw_dock_h = 1.5774427772494515
plt.scatter(X_dock_t, Y_dock_t)

trailer = Trailer_(X_dock_t, Y_dock_t, yaw_dock_t, l_t, w_t)
head_center, head_transform = head_(X_dock_t, Y_dock_t, yaw_dock_t, l_t, w_t, l_h, w_h, yaw_dock_h)
x_dockt,y_dockt = Polygon(trailer).exterior.xy 
x_dockh,y_dockh = Polygon(head_transform).exterior.xy

plt.plot(x_dockt,y_dockt, 'g')
plt.plot(x_dockh, y_dockh, 'g')

plt.axis("equal")

print("X_start, Y_Start, X_end_pointstab, Y_end_pointstab, X_end_dock, Y_end_dock")
print(X_start_pointstab, Y_start_pointstab, X_end_t, Y_end_t, X_dock_t, Y_dock_t)