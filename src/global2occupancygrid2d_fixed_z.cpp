#include "global2occupancygrid2d_fixed_z.h"

Global2OccupancyGrid2DFZ::Global2OccupancyGrid2DFZ(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->occupancygrid_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, buffersize);
}

void Global2OccupancyGrid2DFZ::setGlobalMap(global_map_cartesian &map, string world_fram_name)
{

    this->map2d_nx = map.map_nx;
    this->map2d_ny = map.map_ny;
    this->map2d_dx = map.map_dx;
    this->map2d_dy = map.map_dy;

    occupancy_grid.header.frame_id = world_fram_name;

    occupancy_grid.info.origin.position.x = -(map2d_dx*map2d_nx)/2;
    occupancy_grid.info.origin.position.y = -(map2d_dy*map2d_ny)/2;
    occupancy_grid.data.clear();

    occupancy_grid.info.resolution = map2d_dx;         // float32
    occupancy_grid.info.width      = map2d_nx;           // uint32
    occupancy_grid.info.height     = map2d_ny;           // uint32

    for (int i = 0; i < map2d_nx*map2d_ny; i++)
    {
        occupancy_grid.data.push_back(0);
    }
}

void Global2OccupancyGrid2DFZ::pub_occupancy_grid_2D_from_globalmap(global_map_cartesian &map, ros::Time stamp)
{

    occupancy_grid.header.stamp = stamp;

    //TODO Here: Get Occupancy from 0 to 100, need to think about right 2D projection. Max is only giving 50 and not smaller values
    //occupancy_grid.header.stamp = ros::Time::now();
    for(auto cellDouble:map.occupied_cell_percentage_idx_list)
    {
        int fixedZ = 6;
        auto cell = cellDouble.vec;
        // We need to think which fixed z slice is the best one TODO
        if (cell(2) == fixedZ){
            double occup3Dprob = (cellDouble.value)*100;
            occupancy_grid.data.at(cell(0)+cell(1)*map2d_nx)=occup3Dprob;
        }
    }

    this->occupancygrid_pub.publish(occupancy_grid);
}
