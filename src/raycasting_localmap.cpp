/*
 *  raycasting_localmap.cpp
 *
 *  author: Jumpei Arima
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


/*global variables*/
nav_msgs::OccupancyGrid local_map;
sensor_msgs::LaserScan scan;
bool received_localmap = false;

void index_to_point(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
  x = index%grid.info.width - grid.info.width/2.0;
  y = index/grid.info.width - grid.info.height/2.0;
}

int point_to_index(nav_msgs::OccupancyGrid grid, int x, int y)
{
  int x_ = x + grid.info.width/2.0;
  int y_ = y + grid.info.height/2.0;
  return  y_*grid.info.width + x_;
}

void index_to_meterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y)
{
  x = (index%grid.info.width - grid.info.width/2.0 + 0.5)*grid.info.resolution;
  y = (index/grid.info.width - grid.info.height/2.0 + 0.5)*grid.info.resolution;
}

void callback_localmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  local_map = *msg;
  received_localmap = true;
}

double map_calc_range(nav_msgs::OccupancyGrid map, double ox, double oy, double oa, double max_range)
{
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  int steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = (ox - map.info.origin.position.x) /map.info.resolution;
  y0 = (oy - map.info.origin.position.y) /map.info.resolution;

  x1 = (ox + max_range * cos(oa) -  map.info.origin.position.x) /map.info.resolution;
  y1 = (oy + max_range * sin(oa) -  map.info.origin.position.y) /map.info.resolution;

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;
  if(steep){
    tmp = x0;
    x0 = y0;
    y0 = tmp;
    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!((y >= 0) && (y < map.info.width) && (x >= 0) && (x < map.info.height))
        || map.data[(y + x *map.info.width)] != 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) *map.info.resolution;
  }
  else
  {
    if(!((x >= 0) && (x < map.info.width) && (y >= 0) && (y < map.info.height))
        || map.data[(x + y *map.info.width)] != 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) *map.info.resolution;
  }
  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!((y >= 0) && (y < map.info.width) && (x >= 0) && (x < map.info.height))
          || map.data[(y + x *map.info.width)] != 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
    }
    else
    {
      if(!((x >= 0) && (x < map.info.width) && (y >= 0) && (y < map.info.height))
          || map.data[(x + y *map.info.width)] != 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
    }
  }
  return max_range;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raycasting_localmap");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  /*sub*/
  ros::Subscriber sub_grid_lidar = nh.subscribe("/local_map", 1, callback_localmap);

  /*pub*/
  ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("/raycasting/scan",1);

  /*tf*/
  tf::TransformListener listener;
  tf::StampedTransform transform;

  /*variables*/
  sensor_msgs::LaserScan scan;

  /*initialization*/
  double num_scan_data;
  local_nh.getParam("range_size",num_scan_data);
  local_nh.getParam("scan_frame",scan.header.frame_id);
  local_nh.getParam("scan_angle_min",scan.angle_min);
  local_nh.getParam("scan_angle_max",scan.angle_max);
  local_nh.getParam("scan_time",scan.scan_time);
  local_nh.getParam("scan_range_min",scan.range_min);
  local_nh.getParam("scan_range_max",scan.range_max);
  scan.angle_increment=M_PI/num_scan_data;
  scan.time_increment=scan.scan_time/(num_scan_data*2.0);
  scan.ranges.resize(int(num_scan_data));

  /*loop*/
  ros::Rate loop_rate(20);

  while(ros::ok()){
    if(received_localmap && !local_map.data.empty()){
      try{
        listener.waitForTransform(local_map.header.frame_id,scan.header.frame_id,local_map.header.stamp,ros::Duration(1.0));
        listener.lookupTransform(local_map.header.frame_id, scan.header.frame_id,local_map.header.stamp,transform);
      }
      catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      double roll,pitch,yaw;
      tf::Matrix3x3(transform.getRotation()).getRPY(roll,pitch,yaw);

      for(int i = 0; i<num_scan_data; i++){
        double angle = yaw + i*scan.angle_increment + scan.angle_min;
        scan.ranges[i] = map_calc_range(local_map,0,0,angle, scan.range_max);
        scan.header.stamp = ros::Time::now();
      }
      pub_scan.publish(scan);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
