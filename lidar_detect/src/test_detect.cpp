#include<ros/ros.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<sensor_msgs/PointCloud2.h>
#include<vector>
#include<string>
#include<deque>
#include<cmath>
#include<Eigen/Dense>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>

    std::vector<jsk_recognition_msgs::BoundingBoxArray>
    init_box(const int x, const int y, const double size){
        if(x * y <= 0) { 
            ROS_ERROR("Bounding box Array size is minus!"); exit(1);
        }

        jsk_recognition_msgs::BoundingBoxArray boxArr;
        jsk_recognition_msgs::BoundingBox box;
        std::vector<jsk_recognition_msgs::BoundingBoxArray> bounding_box_vec;
        boxArr.header.seq = 0;
        boxArr.header.frame_id = "bound";
        boxArr.header.stamp = ros::Time();

        bounding_box_vec.resize(0);
        boxArr.boxes.resize(0);
        double next_box_x_pose = 0.25;
        double next_box_y_pose = 0.25; 
        for(unsigned int i = 0; i < y; ++i){
            next_box_x_pose = 0.25;
            for(unsigned int j = 0; j < x; ++j){
                box.header.frame_id = "bound";
                box.header.seq = 0;
                box.header.stamp = ros::Time();
                box.dimensions.x = 0.5;
                box.dimensions.y = 0.5;
                box.dimensions.z = 0.001;
                box.pose.position.x = next_box_x_pose;
                box.pose.position.y = next_box_y_pose;
                box.pose.position.z = 0.0;
                box.pose.orientation.x = 0.0;
                box.pose.orientation.y = 0.0;
                box.pose.orientation.z = 0.0;
                box.pose.orientation.w = 0.0;
                box.value = 1;
                box.label = 1;
                next_box_x_pose += size;

                boxArr.boxes.push_back(box);
            }
            bounding_box_vec.push_back(boxArr);
            boxArr.boxes.resize(0);
            next_box_y_pose += size;
        }
        return bounding_box_vec;
    }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_detect");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/test_boxes", 1000);
    jsk_recognition_msgs::BoundingBoxArray out_boxArr;
    std::vector<jsk_recognition_msgs::BoundingBoxArray> vec;
    vec = init_box(100, 100, 0.5);
    for(int i = 1; i < vec.size(); ++i){
        vec[0].boxes.insert(vec[0].boxes.end(), vec[i].boxes.begin(), vec[i].boxes.end());
    }
    int a = 0;
    ros::Rate loop_rate(100);
    out_boxArr.header.frame_id = "bound";
    out_boxArr.boxes.insert(out_boxArr.boxes.end(), vec[0].boxes.begin(), vec[0].boxes.end());
    while(ros::ok()){
        a++;
        out_boxArr.header.seq = a;
        out_boxArr.header.stamp = ros::Time();
        pub.publish(out_boxArr);
        loop_rate.sleep();
    }
    return 0;
}