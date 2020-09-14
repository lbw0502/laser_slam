#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL)
            delete tf_;
    }

    // get raw laser data
    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
    {
        // read the laser data
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;

        int beamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * (beamNum - 1));

        // get angles and ranges
        // notice that the order is inversed
        std::vector<double> angles,ranges;
        for(int i = beamNum - 1; i >= 0; --i)
        {   
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angles[i];

            if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
                lidar_dist = 0.0;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }


        // convert the data to pcl::pointcloud for visuailization
        tf::Stamped<tf::Pose> visualPose;
        if(!getLaserPose(visualPose, startTime, tf_))
        {

            ROS_WARN("Not visualPose,Can not Calib");
            return ;
        }

        double visualYaw = tf::getYaw(visualPose.getRotation());

        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            // convert current point to odometry frame
            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned char r = 255, g = 0, b = 0;    //red color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        std::cout << std::endl;



        // calibration
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        // convert to pcl::pointcloud for visuailization
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);


            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // green color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        // visualization
         g_PointCloudView.showCloud(visual_cloud_.makeShared());
    }


    /**
     * @name getLaserPose()
     * @brief get laser pose under odometry frame at time stamp dt
     *        
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        
        // we take laser pose as robot pose
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt;

        // get the global pose of the robot
        try
        {
            if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        in this segmentation, the motion of robot is supposed to be uniform
     * @param frame_base_pose       reference frame
     * @param frame_start_pose      laser pose of first interpolation beam
     * @param frame_end_pose        laser pose of last interpolation beam
     * @param ranges                
     * @param angles                
     * @param startIndex            index of first interpolation beam in whole laser data
     * @param beam_number           number of beam for interpolation
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,      // T_odom_base
            tf::Stamped<tf::Pose> frame_start_pose,     // T_odom_start
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
       // get rotation and tranlation of laser pose at both interpolation side
       tf::Quaternion start_odom_q = frame_start_pose.getRotation();
       tf::Quaternion end_odom_q = frame_end_pose.getRotation();
       tf::Vector3 start_odom_origin(frame_start_pose.getOrigin().getX(),frame_start_pose.getOrigin().getY(),1);
       tf::Vector3 end_odom_origin(frame_end_pose.getOrigin().getX(),frame_end_pose.getOrigin().getY(),1);

       for(int i=startIndex; i<(startIndex+beam_number); i++){
           // use interpolation to get rotation and translation for each time
           tf::Vector3 mid_odom_origin = start_odom_origin.lerp(end_odom_origin, (i-startIndex)/(beam_number-1));
           tf::Quaternion mid_odom_q = start_odom_q.slerp(end_odom_q, (i-startIndex)/(beam_number - 1));
           tf::Transform T_odom_mid(mid_odom_q, mid_odom_origin);

            // laser point at mid time
           double x_mid = ranges[i] * cos(angles[i]);
           double y_mid = ranges[i] * sin(angles[i]);

           // convert every laser point back to base frame
           tf::Vector3 point_base = frame_base_pose.inverse() * T_odom_mid * tf::Vector3(x_mid, y_mid, 1);
            ranges[i] = sqrt(point_base[0] * point_base[0] + point_base[1] * point_base[1]);
            angles[i] = atan2(point_base[1], point_base[0]);
       }

    }



    /**
     * @name Lidar_Calibration()
     * @brief use linear intepolation to calibrate laser data
     *        segmentation period is 5ms
     * @param ranges
     * @param angle
     * @param startTime time stamp of first laser beam
     * @param endTime　time stamp of last laser beam
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        // beam number
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // segmentation period is 5ms
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / (beamNumber - 1); // interval for every two laser beam

        // start index for interpolation
        int start_index = 0;

        // get start time pose
        // after the calibration, all laser point will be calibrated to the frame where first beam is detected

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;

        // base pose is the laser pose for first laser beam
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            // segment the laser beam, duration for interpolation is interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                // get end pose for interpolation
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                // number of laser beam for interpolation
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                // update
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}


