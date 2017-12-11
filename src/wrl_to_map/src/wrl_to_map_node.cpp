#include <ros/ros.h>
#include <wrl_to_map/BresenhamAlgorithm.h>
#include <wrl_to_map/parserFileWRL.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char ** argv){

    ros::init(argc, argv, "wrl_to_map_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::vector<biorobotics::Polygon> polygons = biorobotics::parserFile("/opt/codigo/ros/catkin_ws_kinetic/src/wrl_to_map/wrl/random_1.wrl");

    ros::Publisher pubMap = nh.advertise<nav_msgs::OccupancyGrid>("wrl_to_map", 1);

    biorobotics::Vertex2 origin(-20.0, -20.0);
    float res = 0.05;
    float w = 40, h = 40;
    int sizeOccupancyMatrix = (int) (w / res * h / res);
    
    signed char * occupancyMatrix = new signed char[sizeOccupancyMatrix];
    int cols = (int) (w / res);
    int rows = (int) (h / res);
    for(int i = 0; i < cols; i++)
        for(int j = 0; j < rows; j++){
            occupancyMatrix[i + j * cols] = 0;
        }

    for(int i = 0 ; i < polygons.size(); i++){
        biorobotics::Polygon polygon = polygons[i];
        for(int j = 0; j < polygon.num_vertex; j++){
            biorobotics::Vertex2 v1 = polygon.vertex[j];
            biorobotics::Vertex2 v2;
            if(j < polygon.num_vertex - 1)
                v2 = polygon.vertex[j + 1];
            else
                v2 = polygon.vertex[0];
            int cellX1 = (int)((v1.x - origin.x) / res);
            int cellY1 = (int)((v1.y - origin.y) / res);
            int cellX2 = (int)((v2.x - origin.x) / res);
            int cellY2 = (int)((v2.y - origin.y) / res);
            bresenhamAlgorithm(cellX1, cellY1, cellX2, cellY2, occupancyMatrix, cols, rows, res);
        }
    }

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "/map";
    int seq = 0;
    grid.info.width = cols;
    grid.info.height = rows;
    grid.info.resolution = res;
    grid.info.origin.position.x = origin.x;
    grid.info.origin.position.y = origin.y;
    grid.data.insert(grid.data.begin(), occupancyMatrix, occupancyMatrix + sizeOccupancyMatrix);

    while(ros::ok()){

        grid.header.stamp = ros::Time();
        grid.header.seq = seq++;

        pubMap.publish(grid);

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
