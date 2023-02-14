




 # Initilize for camera position tracker
self.total_dis = 0
self.goal = [0, 0]




# Get the pose of the left eye of the camera with reference to the world frame
self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
self.zed.get_sensors_data(self.zed_sensors, sl.TIME_REFERENCE.IMAGE)
zed_imu = self.zed_sensors.get_imu_data()

# Display the translation and timestamp
pos_translation = sl.Translation()
tx = round(self.zed_pose.get_translation(pos_translation).get()[0], 3)
ty = round(self.zed_pose.get_translation(pos_translation).get()[1], 3)
#tz = round(self.zed_pose.get_translation(self.pos_translation).get()[2], 3)
print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, self.zed_pose.timestamp.get_milliseconds()))

norm2 = np.linalg.norm([tx, ty])
self.total_dis = self.total_dis + norm2

rot_y = round(self.zed_pose.get_euler_angles(radian = False)[1], 3)

direction_vector = []
direction_vector[0] = math.cos(rot_y - (math.pi/2)) # x value
direction_vector[1] = math.sin(rot_y - (math.pi/2)) # y value

heading_diff_ang = math.atan2(self.goal[0] * direction_vector[1] - self.goal[1] * direction_vector[0], self.goal[0] * direction_vector[0] + self.goal[1] * direction_vector[1])

self.total_dis = self.total_dis + norm2

return sampled_heightData2, 