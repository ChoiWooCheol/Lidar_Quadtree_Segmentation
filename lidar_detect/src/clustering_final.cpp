#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>


/* Quadtree header */
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>
#include<string>
#include<cmath>
#include<Eigen/Dense>
#include<pcl_conversions/pcl_conversions.h>
#include<ctime>
#include<cstdlib>

/* user header */
#include "clustering/find_road_points.h"

using namespace pcl;
using namespace std;

/* Quadtree macro variable */
#define LEVEL0_BOX_PIXEL 128
#define LEVEL1_BOX_PIXEL 64
#define LEVEL2_BOX_PIXEL 32
#define LEVEL3_BOX_PIXEL 16
#define LEVEL4_BOX_PIXEL 8
#define LEVEL5_BOX_PIXEL 4
#define LEVEL6_BOX_PIXEL 2
#define LEVEL7_BOX_PIXEL 1

#define LEVEL0_THRESHOLD 16384      // 16384 
#define LEVEL1_THRESHOLD 4096       // 4096
#define LEVEL2_THRESHOLD 1024       // 1024 or 700
#define LEVEL3_THRESHOLD 256       // 256 or 180
#define LEVEL4_THRESHOLD 64        // 64 or 40
#define LEVEL5_THRESHOLD 16         // 16 or 13
#define LEVEL6_THRESHOLD 4          // 4 or 2
#define LEVEL7_THRESHOLD 1          // 1

/* ===================================== Quadtree Code ===================================== */

/* MAIN CLASS */
struct XYI{
    int count;
    double x, y, z;
    double maxZ;
};

static jsk_recognition_msgs::BoundingBoxArray totalBox;
static jsk_recognition_msgs::BoundingBoxArray clusteredBox;
static std::vector< std::vector<XYI> > total_pixel;
static double boxSize;
static double default_x;
static double default_y;
static double default_size;
static int start_Xindex;
static int start_Yindex;
static int default_pixel;
static double box_height;
static double box_z;
static double correction_dist;

class BOX{
public:
    BOX(double xpose, double ypose, double bsize, int pixel) : Xpose(xpose), Ypose(ypose), size(bsize), pixelSize(pixel) {

    } // constant -> parameter
    
    BOX() : Xpose(default_x), Ypose(default_y), size(default_size), pixelSize(default_pixel) {

    } // constant -> parameter

    ~BOX() {

    }

    BOX getUR()
    {
        double in_x = this->Xpose + this->size / 4.0;
        double in_y = this->Ypose + this->size / 4.0;
        double in_size = this->size / 2.0;
        BOX box(in_x, in_y, in_size, pixelSize / 2);
        return box;
    }

    BOX getUL()
    {
        double in_x = this->Xpose - this->size / 4.0;
        double in_y = this->Ypose + this->size / 4.0;
        double in_size = this->size / 2.0;
        BOX box(in_x, in_y, in_size, pixelSize / 2);
        return box;
    }

    BOX getDR()
    {
        double in_x = this->Xpose + this->size / 4.0;
        double in_y = this->Ypose - this->size / 4.0;
        double in_size = this->size / 2.0;
        BOX box(in_x, in_y, in_size, pixelSize / 2);
        return box;
    }

    BOX getDL()
    {
        double in_x = this->Xpose - this->size / 4.0;
        double in_y = this->Ypose - this->size / 4.0;
        double in_size = this->size / 2.0;
        BOX box(in_x, in_y, in_size, pixelSize / 2);
        return box;
    }

public:
    ros::NodeHandle nh;
    double size;
    double Xpose, Ypose;
    int pixelSize;
};

class QuadNode{
public:
    QuadNode()
    {
        this->Children[0] = NULL;
        this->Children[1] = NULL;
        this->Children[2] = NULL;
        this->Children[3] = NULL;

        rect = BOX();
        HasChildren = false;
        HasPoints = true;
    }

    QuadNode(BOX& re)
    {
        this->Children[0] = NULL;
        this->Children[1] = NULL;
        this->Children[2] = NULL;
        this->Children[3] = NULL;

        rect = re;
        HasChildren = false;
        HasPoints = true;
    }
    
    ~QuadNode(){

    }

    int divideQuadTree()
    {
        BOX box = rect;
        BOX URbox = box.getUR();
        BOX ULbox = box.getUL();
        BOX DRbox = box.getDR();
        BOX DLbox = box.getDL();

        if(URbox.pixelSize < 1) return 0;

        HasChildren = true;
        HasPoints = false;

        Children[0] = new QuadNode(URbox);
        Children[1] = new QuadNode(ULbox);
        Children[2] = new QuadNode(DRbox);
        Children[3] = new QuadNode(DLbox);

        if(check_child(total_pixel, URbox)) Children[0]->divideQuadTree(); 
        delete Children[0];

        if(check_child(total_pixel, ULbox)) Children[1]->divideQuadTree(); 
        delete Children[1];

        if(check_child(total_pixel, DRbox)) Children[2]->divideQuadTree(); 
        delete Children[2];

        if(check_child(total_pixel, DLbox)) Children[3]->divideQuadTree(); 
        delete Children[3];

        return 1;
    }

    bool check_child(std::vector< std::vector<XYI> >& in_pixel, BOX& in_box){
        int pixelCnt = 0;
        int start_X, start_Y;
        double longest_z = -1.785;
        double result_Z = 0, total_Z =0;
        start_Y = start_Yindex - (in_box.Ypose / boxSize);
        start_X = start_Xindex + (in_box.Xpose / boxSize);
        start_Y = start_Y - (in_box.pixelSize / 2) + 1;
        start_X = start_X - (in_box.pixelSize / 2);


        for(uint i = start_Y; i < start_Y + in_box.pixelSize; ++i){
            for(uint j = start_X; j < start_X + in_box.pixelSize; ++j){
                if(in_pixel[i][j].maxZ > longest_z) longest_z = in_pixel[i][j].maxZ;
                pixelCnt += in_pixel[i][j].count;
                total_Z += in_pixel[i][j].z;
            }
        }

        result_Z = total_Z / pixelCnt + (longest_z + 1.785) / 2;// + 1.785;// + box_height;
        //result_Z = in_pixel[start_Y][start_X].z + longest_z;
        //if(result_Z > longest_z) return false;//longest_z = result_Z+0.1;
        /* if point exists in box which is minimum size box, bounding box is maked */
        if (pixelCnt == 0) return false;
        else if(check_threshold(in_box, pixelCnt)) return true;
        else if(!check_threshold(in_box, pixelCnt)){
            jsk_recognition_msgs::BoundingBox box_s;
            box_s.header.frame_id = "velodyne";
            box_s.header.seq = 0;
            box_s.header.stamp = ros::Time();
            box_s.dimensions.x = in_box.size;
            box_s.dimensions.y = in_box.size;

            // box_s.dimensions.z = longest_z + 1.785;//Try
            box_s.dimensions.z = 0.0001;

            box_s.pose.position.x = in_box.Xpose;
            box_s.pose.position.y = in_box.Ypose;
            // box_s.pose.position.z = result_Z;// + (longest_z + 1.785) / 2;//Try
            box_s.pose.position.z = -1.785;// + (longest_z + 1.785) / 2;//Try

            box_s.pose.orientation.x = 0.0;
            box_s.pose.orientation.y = 0.0;
            box_s.pose.orientation.z = 0.0;
            box_s.pose.orientation.w = 0.0;
            box_s.value = 1;
            box_s.label = 1;
            totalBox.boxes.emplace_back(box_s);
            return false;
        }
        else{
            ROS_ERROR("check children error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
            exit(1);
        }
    }

    inline bool check_threshold(BOX& in_box, int in_Cnt){
        if(LEVEL0_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL0_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL1_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL1_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL2_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL2_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL3_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL3_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL4_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL4_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL5_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL5_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL6_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL6_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else if(LEVEL7_BOX_PIXEL == in_box.pixelSize){
            if(LEVEL7_THRESHOLD > in_Cnt) return true;
            else return false;
        }
        else{
            ROS_ERROR("check_threshold error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
            exit(1);
        }
    }



public:
    ros::NodeHandle nh;
    double boxPose_x, boxPose_y;
    bool HasChildren, HasPoints;
    BOX rect;
    QuadNode* Children[4];
    
};

class BoxCluster{
public:
    BoxCluster(){        
        clusteredBox.boxes.resize(0);
        clusteredBox.header.seq = totalBox.header.seq;
        clusteredBox.header.stamp = ros::Time();
        clusteredBox.header.frame_id = "velodyne";
        // grouping_count = 0;
    }

    ~BoxCluster(){

    }

    void grouping(int index, int box_label){
        // ++grouping_count;
        jsk_recognition_msgs::BoundingBox tmp;
        tmp = totalBox.boxes[index];
        totalBox.boxes.erase(totalBox.boxes.begin() + index);
        tmp.label = box_label;
        clusteredBox.boxes.emplace_back(tmp);

        for(int i = 0; i < totalBox.boxes.size(); ++i){
            if(check_dist(tmp, totalBox.boxes[i])){
                grouping(i, box_label);
                --i;
            }
        }
    }

    void makeGroup(){
        int _label_;
        srand((unsigned)time(NULL));
        while(totalBox.boxes.size() != 0){
            _label_ = rand() % 255 + 1;
            grouping(0, _label_);
        }
    }

    bool check_dist(jsk_recognition_msgs::BoundingBox& in_box1, jsk_recognition_msgs::BoundingBox& in_box2){
        double box1_size = in_box1.dimensions.x;
        double box2_size = in_box2.dimensions.x;
        double box1_dist, box2_dist, box1_to_box2;
        double box1_Xpose, box1_Ypose, box2_Xpose, box2_Ypose;
        box1_Xpose = in_box1.pose.position.x;
        box1_Ypose = in_box1.pose.position.y;
        box2_Xpose = in_box2.pose.position.x;
        box2_Ypose = in_box2.pose.position.y;

        box1_dist = sqrt(pow(box1_size / 2, 2) + pow(box1_size / 2, 2));
        box2_dist = sqrt(pow(box2_size / 2, 2) + pow(box2_size / 2, 2));

        box1_to_box2 = sqrt(pow(box1_Xpose - box2_Xpose, 2) + pow(box1_Ypose - box2_Ypose, 2));

        if((box1_dist + box2_dist) + correction_dist >= box1_to_box2) return true; // correction_dist : error correction
        else return false;
    }

private:
    // int grouping_count;
};

class QuadTree{
public:
    QuadTree() : seq(0){
        ros::NodeHandle private_nh("~");
        
        if(!private_nh.getParam("/quadtree_params/box_z",               box_z))             throw std::runtime_error("set box_z");
        if(!private_nh.getParam("/quadtree_params/box_height",          box_height))        throw std::runtime_error("set box_height");
        if(!private_nh.getParam("/quadtree_params/minimum_pixel",       boxSize))           throw std::runtime_error("set boxSize");
        if(!private_nh.getParam("/quadtree_params/pixel_x",             point_pixel_x))     throw std::runtime_error("set point_pixel_x");
        if(!private_nh.getParam("/quadtree_params/pixel_y",             point_pixel_y))     throw std::runtime_error("set point_pixel_y");
        if(!private_nh.getParam("/quadtree_params/QTsub_topic",         subtopic))          throw std::runtime_error("set subtopic");
        if(!private_nh.getParam("/quadtree_params/QTpub_topic",         pubtopic))          throw std::runtime_error("set pubtopic");
        if(!private_nh.getParam("/quadtree_params/pixel_Xmax",          pixel_Xmax))        throw std::runtime_error("set pixel_Xmax");
        if(!private_nh.getParam("/quadtree_params/pixel_Ymax",          pixel_Ymax))        throw std::runtime_error("set pixel_Ymax");
        if(!private_nh.getParam("/quadnode_params/start_Xindex",        start_Xindex))      throw std::runtime_error("set start_Xindex");
        if(!private_nh.getParam("/quadnode_params/start_Yindex",        start_Yindex))      throw std::runtime_error("set start_Yindex");
        if(!private_nh.getParam("/box_params/default_x",                default_x))         throw std::runtime_error("set default_x");
        if(!private_nh.getParam("/box_params/default_y",                default_y))         throw std::runtime_error("set default_y");
        if(!private_nh.getParam("/box_params/default_size",             default_size))      throw std::runtime_error("set default_size");
        if(!private_nh.getParam("/box_params/default_pixel",            default_pixel))     throw std::runtime_error("set default_pixel");
        if(!private_nh.getParam("/boxcluster_params/correction_dist",   correction_dist))   throw std::runtime_error("set correction_dist");

        ROS_INFO("==================PARAMS==================");
        ROS_INFO("box params:");
        ROS_INFO("  default_x = %f", default_x);
        ROS_INFO("  default_y = %f", default_y);
        ROS_INFO("  default_size = %f", default_size);
        ROS_INFO("  default_pixel = %d", default_pixel);
        ROS_INFO("quadnode params:");
        ROS_INFO("  start_Xindex = %d", start_Xindex);
        ROS_INFO("  start_Yindex = %d", start_Yindex);
        ROS_INFO("quadtree params:");
        ROS_INFO("  box_z = %f", box_z);
        ROS_INFO("  box_height = %f", box_height);
        ROS_INFO("  minimum_pixel = %f", boxSize);
        ROS_INFO("  pixel_x = %d", point_pixel_x);
        ROS_INFO("  pixel_y = %d", point_pixel_y);
        ROS_INFO("  QTsub_topic = %s", subtopic.c_str());
        ROS_INFO("  QTpub_topic = %s", pubtopic.c_str());
        ROS_INFO("  pixel_Xmax = %d", pixel_Xmax);
        ROS_INFO("  pixel_Ymax = %d", pixel_Ymax);
        ROS_INFO("boxcluster_params:");
        ROS_INFO("  correction_dist = %f", correction_dist);
        ROS_INFO("==========================================");

        default_x       = const_cast<const double&>(default_x);
        default_y       = const_cast<const double&>(default_y);
        default_size    = const_cast<const double&>(default_size);
        box_z           = const_cast<const double&>(box_z);
        box_height      = const_cast<const double&>(box_height);
        boxSize         = const_cast<const double&>(boxSize);
        correction_dist = const_cast<const double&>(correction_dist);
        
        start_Xindex    = const_cast<const int&>(start_Xindex);
        start_Yindex    = const_cast<const int&>(start_Yindex);
        point_pixel_x   = const_cast<const int&>(point_pixel_x);
        point_pixel_y   = const_cast<const int&>(point_pixel_y);
        pixel_Xmax      = const_cast<const int&>(pixel_Xmax);
        pixel_Ymax      = const_cast<const int&>(pixel_Ymax);
        default_pixel   = const_cast<const int&>(default_pixel);
        
        // boxSize = 0.390625; // 50 x 50 image, fake pixel size 128 x 128. // constant -> parameter
        // point_pixel_x = 128; // constant -> parameter
        // point_pixel_y = 128; // constant -> parameter
        // filtered_sub = nh.subscribe(subtopic.c_str(), 10000, &QuadTree::pointCallBack, this);
        detect_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(pubtopic.c_str(), 1000); // string -> parameter
        init_pixel();
    }

    void pointCallBack(sensor_msgs::PointCloud2* in_points, sensor_msgs::PointCloud2* pcl_points){
        QuadNode*   QN = new QuadNode();
        BoxCluster* BC = new BoxCluster();
        ++seq;
        pcl::PointCloud<pcl::PointXYZI> scan;
        pcl::PointCloud<pcl::PointXYZI> pcl;
        pcl::fromROSMsg(*in_points, scan);
        pcl::fromROSMsg(*pcl_points,pcl);


        double poseX = point_pixel_x / 2;
        double poseY = (point_pixel_y / 2) - 1;
        int indexX, indexY;
        for(auto point : scan.points){
            if(fabs(point.x) < 0.1 && fabs(point.y) < 0.1) continue;
            indexX = poseX + (ceil(point.x / boxSize) - 1);
            indexY = poseY - (ceil(point.y / boxSize) - 1);
            if(total_pixel[indexY][indexX].maxZ < point.z)
                total_pixel[indexY][indexX].maxZ = point.z;
            if(total_pixel[indexY][indexX].count != 0) continue;
            total_pixel[indexY][indexX].count++;
        }
        
        for(auto point : pcl.points){
            indexX = poseX + (ceil(point.x / boxSize) - 1);
            indexY = poseY - (ceil(point.y / boxSize) - 1);
            total_pixel[indexY][indexX].z = point.z;
        }

        double maxX = pixel_Xmax; // constant -> parameter
        double maxY = pixel_Ymax; // constant -> parameter
        for(uint i = 0; i < point_pixel_y; ++i){
            maxY = maxY - boxSize / 2;
            maxX = pixel_Xmax;
            for(uint j = 0; j < point_pixel_x; ++j){
                maxX = maxX + boxSize / 2;
                if(total_pixel[i][j].count != 0){
                    total_pixel[i][j].x = maxX;
                    total_pixel[i][j].y = maxY;
                }
                maxX = maxX + boxSize / 2;
            }
            maxY = maxY - boxSize / 2;
        }

        QN->divideQuadTree();
        BC->makeGroup();

        detect_pub.publish(clusteredBox);

        for(uint i = 0; i < point_pixel_y; ++i)
            for(uint j = 0; j < point_pixel_x; ++j)
                if(total_pixel[i][j].count != 0){
                    total_pixel[i][j].count = 0;
                    total_pixel[i][j].maxZ = -1.785;
                }
        

        clusteredBox.boxes.resize(0);
        totalBox.boxes.resize(0);
        totalBox.header.seq = seq++;
        totalBox.header.stamp = ros::Time();
        totalBox.header.frame_id = "velodyne";

        delete QN;
    }

    inline void init_pixel(){
        XYI xyi;
        std::vector<XYI> xyi_vec;
        double poseX = static_cast<double>(point_pixel_x);
        double poseY = static_cast<double>(point_pixel_y);
        double next_x, next_y;
        next_y = boxSize;
        for(uint i = 0; i < point_pixel_y; ++i){
            xyi_vec.resize(0);
            next_x = boxSize;
            for(uint j = 0; j < point_pixel_x; ++j){
                xyi.count = 0;
                xyi.x = -poseX + next_x;
                xyi.y = poseY - next_y;
                xyi.maxZ = -1.785;
                next_x = next_x + 2*boxSize;
                xyi_vec.emplace_back(xyi);
            }
            total_pixel.emplace_back(xyi_vec);
            next_y = next_y + 2*boxSize;
        }
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber filtered_sub;
    ros::Publisher detect_pub;


    std::string pubtopic, subtopic;
    int point_pixel_x, point_pixel_y;
    int pixel_Xmax;
    int pixel_Ymax;
    uint seq;
};

/* ===================================== Quadtree Code END ===================================== */


/* RANSACK code */
class Plane
{
    public:
        Plane() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
        {   
            sub = nh.subscribe("/points_raw", 10000, &Plane::callback, this);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10000);
            QT = new QuadTree();
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr &ptr)
        {
            sensor_msgs::PointCloud2 point_msg, filtered_msg, pcl_msg;
        
            pcl::fromROSMsg(*ptr, scan);
            pcl::fromROSMsg(*ptr, filterd_scan);
            pcl::fromROSMsg(*ptr, pcl_scan);
            filterd_scan.clear();
            pcl_scan.clear();
            // ROS_INFO("points raw : %ld", scan.points.size());
            pcl::VoxelGrid<pcl::PointXYZ> vg;

            vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
            vg.setLeafSize(0.10f,0.10f,0.10f);//set the voxel grid size //10cm
            vg.filter(*cloud_filtered);//create the filtering object
            // ROS_INFO("voxel points : %ld", cloud_filtered->points.size());
            make_plane_RANSAC();
            projection_onto_plane();
            pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filterd_scan));
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
            pcl::toROSMsg(*scan_ptr, filtered_msg);
            pcl::toROSMsg(*pcl_ptr, pcl_msg);

            QT->pointCallBack(&filtered_msg, &pcl_msg);
        }

        void make_plane_RANSAC()
        {
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;
            
            // z 값이 filtering 된 point들을 가지고 pointcloud 만드는 작업. RANSAC 알고리즘에 넣어주기 위해
            for(int k = 0; k < cloud_filtered->points.size(); ++k)
            {
                if(fabs(cloud_filtered->points[k].x) < 10 && fabs(cloud_filtered->points[k].y) < 10 && cloud_filtered->points[k].z < -1.5)
                {
                    pcl::PointXYZ z_filtered_point;
                    z_filtered_point.x = cloud_filtered->points[k].x;
                    z_filtered_point.y = cloud_filtered->points[k].y;
                    z_filtered_point.z = cloud_filtered->points[k].z;
                    filtered_points_cloud_z.push_back(z_filtered_point);
                }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
            seg.setInputCloud (point_ptr);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            }
            normal_vector.x = coefficients->values[0];
            normal_vector.y = coefficients->values[1];
            normal_vector.z = coefficients->values[2];

            D = (-1)*coefficients->values[3];
            normal_vector_queue.push(normal_vector);
            D_queue.push(D);
            extract_normal_vector(); // normal_vector의 n개의 평균을 구한다.
        }

        void extract_normal_vector()
        {
            if(normal_vector_queue.size() == normal_vector_queue_size && D_queue.size() == normal_vector_queue_size)
            {
                float sum_x = 0.0;
                float sum_y = 0.0;
                float sum_z = 0.0;
                float sum_d = 0.0;

                for(int k = 0; k<normal_vector_queue.size(); ++k)
                {
                    sum_x += normal_vector_queue.front().x;
                    sum_y += normal_vector_queue.front().y;
                    sum_z += normal_vector_queue.front().z;
                    sum_d += D_queue.front();
                }
                //다시 갱신.
                normal_vector.x = sum_x / normal_vector_queue.size();
                normal_vector.y = sum_y / normal_vector_queue.size();
                normal_vector.z = sum_z / normal_vector_queue.size();
                D = sum_d /D_queue.size();
                
                //최신의 4개의 data point를 가지고 평균을 내기때문
                normal_vector_queue.pop(); //맨 앞 원소 제거
                D_queue.pop();
            }
        }

        void projection_onto_plane()
        {
            Eigen::Vector4f coeffs;
            coeffs << normal_vector.x, normal_vector.y, normal_vector.z, -D;

            for(size_t i = 0; i < cloud_filtered->points.size(); ++i)
            {
                // projection이 수행되어야 하는 영역안의 points 추출 후, projection
                if(fabs(cloud_filtered->points[i].x) < x_limit && 
                    fabs(cloud_filtered->points[i].y) < y_limit && 
                    cloud_filtered->points[i].z < z_high_limit &&  
                    cloud_filtered->points[i].z > z_low_limit )
                {
                    pcl::PointXYZI projection, point;

                    projection.x = cloud_filtered->points[i].x;  
                    projection.y = cloud_filtered->points[i].y;
                    projection.z = (-1) * (normal_vector.x * cloud_filtered->points[i].x + normal_vector.y * cloud_filtered->points[i].y - D) / normal_vector.z;
                    //======//
                    if(cloud_filtered->points[i].z > -1.5){
                        point.x = cloud_filtered->points[i].x;  
                        point.y = cloud_filtered->points[i].y;
                        point.z = cloud_filtered->points[i].z;
                    }
                    //======//
                    filterd_scan.points.emplace_back(point);
                    projection.intensity = 2.0;
                    pcl_scan.points.emplace_back(projection);
                }
            }

            filterd_scan.width = static_cast<uint32_t>(filterd_scan.points.size());
            filterd_scan.height = 1;
            sensor_msgs::PointCloud2 projected_cloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
            pcl::toROSMsg(*n_ptr, projected_cloud);

            projected_cloud.header.frame_id = "velodyne";
            pub4.publish(projected_cloud);

            // ROS_INFO("filtered points : %ld", filterd_scan.points.size());
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub4;
        ros::Subscriber sub;
        QuadTree* QT;
        geometry_msgs::Point normal_vector; //법선벡터
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> scan; 
        pcl::PointCloud<pcl::PointXYZI> filterd_scan, pcl_scan;
        // 급격한 변화를 없애기 위해
        queue< geometry_msgs::Point > normal_vector_queue; 
        queue< float > D_queue;

        float D; // 평면의 방정식의 상수 값
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "clustering_final");
    totalBox.boxes.resize(0);
    totalBox.header.seq = 0;
    totalBox.header.stamp = ros::Time();
    totalBox.header.frame_id = "velodyne";

    Plane p;
    ros::spin();
    return 0;
}