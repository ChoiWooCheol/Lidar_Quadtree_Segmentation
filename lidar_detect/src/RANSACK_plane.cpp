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

/* user header */
#include "clustering/find_road_points.h"

using namespace pcl;
using namespace std;

/* RANSACK code */
class Plane
{
    public:
        Plane() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
        {   
            sub = nh.subscribe("/points_raw", 10000, &Plane::callback, this);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10000);
        }

        void callback(const sensor_msgs::PointCloud2Ptr &ptr)
        {
            sensor_msgs::PointCloud2 point_msg, filtered_msg;
        
            pcl::fromROSMsg(*ptr, scan);
            pcl::fromROSMsg(*ptr, filterd_scan);
            filterd_scan.clear();

            pcl::VoxelGrid<pcl::PointXYZ> vg;

            vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
            vg.setLeafSize(0.10f,0.10f,0.10f);//set the voxel grid size //10cm
            vg.filter(*cloud_filtered);//create the filtering object

            make_plane_RANSAC();
            projection_onto_plane();
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

            pcl::PointCloud<pcl::PointXYZI> projected_cloud_pcl;

            for(size_t i = 0; i < scan.points.size(); ++i)
            {
                // projection이 수행되어야 하는 영역안의 points 추출 후, projection
                if(fabs(scan.points[i].x) < x_limit && fabs(scan.points[i].y) < y_limit && scan.points[i].z < z_high_limit &&  scan.points[i].z > z_low_limit )
                {
                    pcl::PointXYZI projection, point;

                    projection.x = scan.points[i].x;  
                    projection.y = scan.points[i].y;
                    projection.z = (-1) * (normal_vector.x * scan.points[i].x + normal_vector.y * scan.points[i].y - D) / normal_vector.z;
                    projection.intensity = 2.0;
                    projected_cloud_pcl.push_back(projection);
                }
            }
            sensor_msgs::PointCloud2 projected_cloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(projected_cloud_pcl));
            pcl::toROSMsg(*n_ptr, projected_cloud);

            projected_cloud.header.frame_id = "velodyne";
            pub4.publish(projected_cloud);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub4;
        ros::Subscriber sub;
        geometry_msgs::Point normal_vector; //법선벡터
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> scan, filterd_scan; 
        // 급격한 변화를 없애기 위해
        queue< geometry_msgs::Point > normal_vector_queue; 
        queue< float > D_queue;

        float D; // 평면의 방정식의 상수 값
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RANSAC_plane");
    Plane p;
    ros::spin();
}