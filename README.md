# Lidar_Quadtree_Segmentation
1. get LiDAR data (Ouster OS1 64 or Velodyne VLP 16, or any 3d 360 degree LiDAR)
2. subscribe '/points_raw' topic
3. saving Z axis information
4. 3d Data -> 2d Data ( project onto Road Plane that was estimated by RANSAC )
5. point cloud data segmentation, Using quadtree
6. clustering
7. 2d -> 3d as bounding boxes, Using Z axis information

# Hardware
1. HYUNDAI i30
2. Ouster OS1 64 channel LiDAR

# Input topic
'/points_raw'

# Output topic
'/detected_boxes'
'/projected_cloud'
'/quad_cluster'

# Run 
roslaunch lidar_detect quadtree_seg_launch.launch --screen 


# result
1. segmentation
![Screenshot from 2020-01-15 20-22-09](https://user-images.githubusercontent.com/46434674/72430251-34b41400-37d5-11ea-83a4-d790f9ea17ff.png)

2. 3D object detection
![Screenshot from 2020-01-15 20-31-24](https://user-images.githubusercontent.com/46434674/72430573-14d12000-37d6-11ea-83db-b62f73f6f495.png)

3. 3D view with PointCloud2(3d pointcloud data)
![Screenshot from 2020-01-15 20-34-34](https://user-images.githubusercontent.com/46434674/72430763-85783c80-37d6-11ea-8f5a-fbdc71599d87.png)
