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

static constexpr double PI = 3.1415926; 
class DetectObj{
public:
    struct XY{ int x, y; };
    struct DXY { double x, y; };
    DetectObj() : box_size(0.5), boxX(104), boxY(104), seq(0){
        filterd_points_sub = nh.subscribe("/filtered_points", 10000, &DetectObj::pointCallBack, this);
        detect_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_boxs", 1000);
        boxes_vec.resize(0);
        boxes_vec = init_box(boxX, boxY, box_size);
        init_checkArr();
        ROS_INFO("boxX size : %ld", boxes_vec[0].boxes.size());
        ROS_INFO("boxY size : %ld", boxes_vec.size());
    }

    void pointCallBack(const sensor_msgs::PointCloud2::ConstPtr& in_points){
        uint count = 0;
        XY indexes;
        DXY box_pose;
        seq++;
        pcl::PointCloud<pcl::PointXYZI> scan;
        jsk_recognition_msgs::BoundingBoxArray out_boxArr;
        pcl::fromROSMsg(*in_points, scan);

        out_boxArr.boxes.resize(0);
        out_boxArr.header.seq = seq;
        out_boxArr.header.stamp = ros::Time();
        out_boxArr.header.frame_id = "velodyne";
        int out_boxX, out_boxY;
        init_checkArr();
        for(auto const point : scan.points){
            indexes = indexed_searching(point.x, point.y, box_size);
            if(checkArr[indexes.x][indexes.y] == false){
                out_boxArr.boxes.push_back(boxes_vec[indexes.x].boxes[indexes.y]);
                out_boxArr.boxes[count].header.frame_id = "velodyne";
                checkArr[indexes.x][indexes.y] = true;
                count++;
            }
        }
        detect_pub.publish(out_boxArr);
    }
    

/*
    void Quadsearching(int startx, int endx,
                        int starty, int endy,
                        const double pointx, const double pointy)
    {
        if()
    }
*/

     XY indexed_searching(const double pointx, const double pointy, const double in_size){
        XY xy;
        int bound_box_x, bound_box_y;
        if(pointx / in_size <= 0) { bound_box_x = ceil(pointx / in_size); }
        else { bound_box_x = ceil(pointx / in_size); }
        if(pointy / in_size <= 0) { bound_box_y = ceil(pointy / in_size) - 2; }
        else { bound_box_y = ceil(pointy / in_size); }
        xy.x = bound_box_x;
        xy.y = bound_box_y;
        xy = xyz_to_box(PI, xy);
        return xy;
    }

    inline void init_checkArr(){
        checkArr.assign(boxY, std::vector<bool>(boxX, false));
    }

    /*
        get_transform_matrix() function
        using Eigen
        bounding_box_pose * (TFmatrix) = xy_pose
        TFmatrix = (bounding_box_pose)^-1 * xy_pose
    */
    Eigen::Matrix4d get_transform_matrix(double theta, int x, int y){ // x : box_vec's xsize, y : box_vec's ysize
        Eigen::Matrix4d vecMat;
        vecMat << 1, 0, 0, -x,
                  0, cos(theta), -sin(theta), y,
                  0, sin(theta), cos(theta), 0,
                  0, 0, 0, 1;
        return vecMat;
    }

    Eigen::Matrix4d rotation_z(double theta){
        Eigen::Matrix4d vecMat;
        vecMat << cos(theta), -sin(theta), 0, 0,
                  sin(theta), cos(theta), 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
        return vecMat;
    }

    Eigen::Matrix4d rotation_y(double theta){
        Eigen::Matrix4d vecMat;
        vecMat << cos(theta), 0, sin(theta), 0,
                  0, 1, 0, 0,
                  -sin(theta), 0, cos(theta), 0,
                  0, 0, 0, 1;
        return vecMat;
    }

    DXY box_to_xyz(double theta, DXY xy){
        Eigen::Matrix4d tfMat;
        Eigen::Vector4d boxMat;
        DXY out_xy;
        boxMat(0) = xy.x;
        boxMat(1) = xy.y;
        boxMat(2) = 0;
        boxMat(3) = 1;

        double ix, iy; // matrix sum number
        ix = 0;
        iy = 0;

        tfMat = rotation_y(PI) * rotation_z(-PI / 2) * get_transform_matrix(theta, boxX / 4, boxY / 4) ;
        //tfMat = tfMat * rotation_z(-PI / 2);
        for(int i = 0; i < 4; ++i){
            ix += tfMat(0, i) * boxMat(i);
        }
        for(int i = 0; i < 4; ++i){
            iy += tfMat(1, i) * boxMat(i);
        }

        out_xy.x = ix;
        out_xy.y = iy;

        return out_xy;
    }

    XY xyz_to_box(double theta, XY xy){
        Eigen::Matrix4d tfMat, inverse_mat;
        Eigen::Vector4d xyzMat;
        XY out_xy;
        xyzMat(0) = xy.x;
        xyzMat(1) = xy.y;
        xyzMat(2) = 0;
        xyzMat(3) = 1;

        int ix, iy; // matrix sum number
        ix = 0;
        iy = 0;

        inverse_mat = get_transform_matrix(theta, boxX / 2, boxY / 2);
        tfMat = inverse_mat.inverse();
        for(int i = 0; i < 4; ++i){
            ix += tfMat(0, i) * xyzMat(i);
        }
        for(int i = 0; i < 4; ++i){
            iy += tfMat(1, i) * xyzMat(i);
        }

        out_xy.x = ix;
        out_xy.y = iy;
        
        return out_xy;
    }

    std::vector<jsk_recognition_msgs::BoundingBoxArray>
    init_box(const int x, const int y, const double size){
        if(x * y <= 0) { 
            ROS_ERROR("Bounding box Array size is minus!"); exit(1);
        }

        jsk_recognition_msgs::BoundingBoxArray boxArr;
        jsk_recognition_msgs::BoundingBox box;
        std::vector<jsk_recognition_msgs::BoundingBoxArray> bounding_box_vec;
        DXY trans_pose;
        boxArr.header.seq = 0;
        boxArr.header.frame_id = box_frame_id;
        boxArr.header.stamp = ros::Time();
        
        bounding_box_vec.resize(0);
        boxArr.boxes.resize(0);
        double next_box_x_pose;
        double next_box_y_pose = size / 2; 
        for(unsigned int i = 0; i < y; ++i){
            next_box_x_pose = size / 2;
            for(unsigned int j = 0; j < x; ++j){
                box.header.frame_id = box_frame_id;
                box.header.seq = 0;
                box.header.stamp = ros::Time();
                box.dimensions.x = size;
                box.dimensions.y = size;
                box.dimensions.z = 0.0001;
                trans_pose.x = next_box_x_pose;
                trans_pose.y = next_box_y_pose;
                trans_pose = box_to_xyz(PI, trans_pose);
                box.pose.position.x = trans_pose.x;
                box.pose.position.y = trans_pose.y;
                box.pose.position.z = -1.785;
                box.pose.orientation.x = 0.0;
                box.pose.orientation.y = 0.0;
                box.pose.orientation.z = 0.0;
                box.pose.orientation.w = 0.0;
                box.value = boxValue;
                box.label = boxLable;
                next_box_x_pose += size;

                boxArr.boxes.push_back(box);
            }
            bounding_box_vec.push_back(boxArr);
            boxArr.boxes.resize(0);
            next_box_y_pose += size;
        }
        return bounding_box_vec;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber filterd_points_sub;
    ros::Publisher detect_pub;
    std::vector<jsk_recognition_msgs::BoundingBoxArray> boxes_vec;
    std::vector< std::vector<bool> > checkArr;
    double boxValue;
    unsigned int boxLable, boxX, boxY, seq;
    double box_size;
    std::string box_frame_id;

    
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obj_detect");
    DetectObj Dobj;
    ros::spin();    
    return 0;
}