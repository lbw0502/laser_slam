#include "tsdf_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        // last cell is excluded
        if (pointX == x1 && pointY == y1)
            continue;

        // save all
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

void SetMapParams(void)
{
    mapParams.width = 1000;
    mapParams.height = 1000;
    mapParams.resolution = 0.05;

    // log incremnet of hitted cell
    // will be needed by occupancy map
    mapParams.log_free = -1;
    mapParams.log_occ = 2;

    // max and min value of each cell
    mapParams.log_max = 100.0;
    mapParams.log_min = 0.0;

    mapParams.origin_x = 0.0;
    mapParams.origin_y = 0.0;

    // origin of map
    mapParams.offset_x = 500;
    mapParams.offset_y = 500;

    pMap = new unsigned char[mapParams.width * mapParams.height];

    // count model parameters
    // hit number of each cell
    pMapHits = new unsigned long[mapParams.width * mapParams.height];
    // miss number of each cell
    pMapMisses = new unsigned long[mapParams.width * mapParams.height];

    //TSDF map parameter
    pMapW = new unsigned long[mapParams.width * mapParams.height];
    pMapTSDF = new double[mapParams.width * mapParams.height];

    // initialization
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        pMap[i] = 50;
        pMapHits[i] = 0;
        pMapMisses[i] = 0;
        pMapW[i] = 0;
        pMapTSDF[i] = -1;
    }
}

// world coordinate -> map coordinate
GridIndex ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
    return linear_index;
}

bool isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

void DestoryMap()
{
    if (pMap != NULL)
        delete pMap;
}


void OccupanyMapping(std::vector<GeneralLaserScan> &scans, std::vector<Eigen::Vector3d> &robot_poses)
{
    std::cout << "mapping..." << std::endl;
    // read all laser data
    for (int i = 0; i < scans.size(); i++)
    {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        // robot coordinate in map
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

        for (int id = 0; id < scan.range_readings.size(); id++)
        {
            double dist = scan.range_readings[id];
            double angle = -scan.angle_readings[id]; // 激光雷达逆时针转，角度取反

            if (std::isinf(dist) || std::isnan(dist))
                continue;

            // world coodinate of hit point
            double theta = -robotPose(2); // anti-clock
            double laser_x = dist * cos(angle);
            double laser_y = dist * sin(angle);

            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);


            // compute each grid
            GridIndex hit = ConvertWorld2GridIndex(world_x, world_y);
            if(!isValidGridIndex(hit)){
                continue;
            }

            // update tsdf field
            const double truncation = 0.15;

            double laser_x_truncation = (dist + truncation) * cos(angle);
            double laser_y_truncation = (dist + truncation) * sin(angle);
            double world_x_truncation = cos(theta) * laser_x_truncation - sin(theta) * laser_y_truncation + robotPose(0);
            double world_y_truncation = sin(theta) * laser_x_truncation + cos(theta) * laser_y_truncation + robotPose(1);

            GridIndex idx_truncation = ConvertWorld2GridIndex(world_x_truncation, world_y_truncation);
            if(!isValidGridIndex(idx_truncation))
                idx_truncation = hit;

            std::vector<GridIndex> tsdf_idxs = TraceLine(robotIndex.x, robotIndex.y, idx_truncation.x, idx_truncation.y);

            for(auto idx : tsdf_idxs){
                int idx_linear = GridIndexToLinearIndex(idx);

                double disti = hypot(robotIndex.x-idx.x, robotIndex.y-idx.y) * mapParams.resolution;
                double sdfi = dist - disti;

                double tsdfi = std::max(-1.0, std::min(1.0, sdfi/truncation));
                
                pMapTSDF[idx_linear] = (pMapW[idx_linear] * pMapTSDF[idx_linear] + tsdfi) / (pMapW[idx_linear]+1);
                pMapW[idx_linear]++;

            }
        }
    }

    // search border along x-axis
    for(int y=0; y<mapParams.width; y++){
        for(int x=0; x<mapParams.height-1; x++){
            
            int curGridIndex = GridIndexToLinearIndex({x,y});
            int curGridIndex_x = GridIndexToLinearIndex({x+1,y});

            if(pMapTSDF[curGridIndex] * pMapTSDF[curGridIndex_x] <= 0){

                if(fabs(pMapTSDF[curGridIndex]) < fabs(pMapTSDF[curGridIndex_x]))
                    pMap[curGridIndex] = mapParams.log_max;
                else
                    pMap[curGridIndex_x] =  mapParams.log_max;

            }
            
        }
    }


    // update the whole map
    // search border along y-axis
    for(int x=0; x<mapParams.height; x++){
        for(int y=0; y<mapParams.width-1; y++){
            
            int curGridIndex = GridIndexToLinearIndex({x,y});
            int curGridIndex_y = GridIndexToLinearIndex({x,y+1});

            if(pMapTSDF[curGridIndex] * pMapTSDF[curGridIndex_y] <= 0){

                if(fabs(pMapTSDF[curGridIndex]) < fabs(pMapTSDF[curGridIndex_y]))
                    pMap[curGridIndex] = mapParams.log_max;
                else
                    pMap[curGridIndex_y] =  mapParams.log_max;

            }
            
        }
    }



    std::cout << "finished" << std::endl;
}

// publish map
void PublishMap(ros::Publisher &map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    int cnt0, cnt1, cnt2;
    cnt0 = cnt1 = cnt2 = 100;
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        if (pMap[i] == 50)
        {
            rosMap.data[i] = -1.0;
        }
        else
        {

            rosMap.data[i] = pMap[i];
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TSDFMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("tsdf_map", 1, true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "./src/data";

    std::string posePath = basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";


    ReadPoseInformation(posePath, robotPoses);

    ReadLaserScanInformation(anglePath,
                             scanPath,
                             generalLaserScans);


    SetMapParams();

    OccupanyMapping(generalLaserScans, robotPoses);

    PublishMap(mapPub);

    ros::spin();

    DestoryMap();

    std::cout << "Release Memory!!" << std::endl;
}
