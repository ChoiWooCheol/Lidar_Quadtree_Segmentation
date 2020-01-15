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

class GTbox{
public:
    GTbox(){
        pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/groundtruth_boxes", 1000);
        sub = nh.subscribe("/points_raw", 10000, &GTbox::callback, this);
        init_box();
    }

    void init_box(){ 
        jsk_recognition_msgs::BoundingBox box;
        boxArr.header.seq = 0;
        boxArr.header.frame_id = "velodyne";
        boxArr.header.stamp = ros::Time();
        boxArr.boxes.resize(0);
        /* ============== box 1 ============== */
        box.header.frame_id = "velodyne";
        box.header.seq = 0;
        box.header.stamp = ros::Time();
        box.dimensions.x = 5.48743;
        box.dimensions.y = 4.13493;
        box.dimensions.z = 3.22674;
        box.pose.position.x = 7.55781;
        box.pose.position.y = 11.9244;
        box.pose.position.z = -0.664914;
        box.pose.orientation.x = 0.0;
        box.pose.orientation.y = 0.0;
        box.pose.orientation.z = 0.0;
        box.pose.orientation.w = 0.0;
        box.value = 1;
        box.label = 1;
        boxArr.boxes.push_back(box);

        /* ============== box 2 ============== */
        box.dimensions.x = 12.2985;
        box.dimensions.y = 4.87904;
        box.dimensions.z = 6.2702;
        box.pose.position.x = -5.84367;
        box.pose.position.y = 8.24745;
        box.pose.position.z = 0.923352;
        boxArr.boxes.push_back(box);

        /* ============== box 3 ============== */
        box.dimensions.x = 6.59916;
        box.dimensions.y = 3.44185;
        box.dimensions.z = 5.95687;
        box.pose.position.x = -10.1752;
        box.pose.position.y = -3.51536;
        box.pose.position.z = 0.98071;
        boxArr.boxes.push_back(box);

        /* ============== box 4 ============== */
        box.dimensions.x = 8.45178;
        box.dimensions.y = 4.95797;
        box.dimensions.z = 6.78154;
        box.pose.position.x = 10.12;
        box.pose.position.y = 7.88263;
        box.pose.position.z = 1.30599;
        boxArr.boxes.push_back(box);
        
        /* ============== box 5 ============== */
        box.dimensions.x = 8.89158;
        box.dimensions.y = 4.20399;
        box.dimensions.z = 3.15669;
        box.pose.position.x = -0.0639453;
        box.pose.position.y = -4.07118;
        box.pose.position.z = -0.00998831;
        boxArr.boxes.push_back(box);
        
        /* ============== box 6 ============== */
        box.dimensions.x = 0.631154;
        box.dimensions.y = 0.552995;
        box.dimensions.z = 1.37066;
        box.pose.position.x = 4.87303;
        box.pose.position.y = 3.42407;
        box.pose.position.z = -1.15397;
        boxArr.boxes.push_back(box);
        
        /* ============== box 7 ============== */
        box.dimensions.x = 4.16238;
        box.dimensions.y = 5.55131;
        box.dimensions.z = 3.66309;
        box.pose.position.x = 6.30385;
        box.pose.position.y = -4.89002;
        box.pose.position.z = 0.0277261;
        boxArr.boxes.push_back(box);
        
        /* ============== box 8 ============== */
        box.dimensions.x = 0.678302;
        box.dimensions.y = 0.704119;
        box.dimensions.z = 1.61558;
        box.pose.position.x = 7.01108;
        box.pose.position.y = 4.30576;
        box.pose.position.z = -1.30295;
        boxArr.boxes.push_back(box);
        
        /* ============== box 9 ============== */
        box.dimensions.x = 0.876642;
        box.dimensions.y = 0.590178;
        box.dimensions.z = 1.51841;
        box.pose.position.x = 6.57557;
        box.pose.position.y = 2.31763;
        box.pose.position.z = -1.25215;
        boxArr.boxes.push_back(box);
        
        /* ============== box 10 ============== */
        box.dimensions.x = 0.49084;
        box.dimensions.y = 0.60796;
        box.dimensions.z = 1.00983;
        box.pose.position.x = -1.15216;
        box.pose.position.y = 3.89934;
        box.pose.position.z = -0.775221;
        boxArr.boxes.push_back(box);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& ptr){
        pub.publish(boxArr);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    jsk_recognition_msgs::BoundingBoxArray boxArr;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_detect");
    GTbox GT;
    ros::spin();

    return 0;
}