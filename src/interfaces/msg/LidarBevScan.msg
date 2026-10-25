# One VLP16 revolution rasterised by morai_vla.lidar_bev.scan_bev_stream (vlp16_bev node), the per-scan unit the
# training data stores (step 0 lidar.bin): every packet motion-compensated into the ego frame at the scan's last packet.
# Two of these, the newer one's frame, make one model BEV (lidar_bev.pack_bev, done by the model server).
std_msgs/Header header    # stamp: receipt time of the scan's last packet (velodyne_driver convention); frame_id: base_link
uint16 rows               # 320: x 60 .. -20 m forward, 0.25 m
uint16 cols               # 160: y +20 .. -20 m left, 0.25 m
uint8[] height            # rows*cols, 0 = empty, else (h + 1 m) / 0.05 m above base_link ground (max per cell)
uint8[] density           # rows*cols, point count capped at 255
float64 pose_x            # lidar_bev.Odometry pose at the last packet, in the frame of the node's first sample
float64 pose_y
float64 pose_yaw
uint32 npts               # points kept in the BEV window
uint32 npackets           # packets in the scan (76)
uint32 odom_epoch         # +1 whenever the node restarts its Odometry; poses of different epochs are not comparable
