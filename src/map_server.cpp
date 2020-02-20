//ROS OMRON driver
//------------------


#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientRatioDrive.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"


#include <cmath>


ArMap arMap;
ArClientBase client;
ArTime start;

nav_msgs::MapMetaData meta_data_message_;
nav_msgs::GetMap::Response map_resp_;

int mapped = 0;

void handleGetMapName(ArNetPacket *packet)
{
  char buffer[512];

  packet->bufToStr(buffer, sizeof(buffer));
  printf("MapFile: %s\n", buffer);
}

void handleGetMap(ArNetPacket *packet)
{
  char buffer[10000];

  if (packet->getDataReadLength() == packet->getDataLength())
  {
    printf("Empty packet signifying end of map (for central forward)\n");
    return;
  }
  
  packet->bufToStr(buffer, sizeof(buffer));
  // if we got an end of line char instead of a line it means the map is over
  if (buffer[0] == '\0')
  {
    printf("First line \n %.*s", 20 , buffer);
    
    printf("Map took %g seconds\n", start.mSecSince() / 1000.0);
    arMap.parsingComplete();

    int n_points = arMap.getNumPoints();
    printf("Map has %d points\n", n_points);

    std::vector<ArPose> *point_list = new std::vector<ArPose> (n_points);
    point_list = arMap.getPoints();

    int res = arMap.getResolution();
    printf("Map has resolution of %dmm\n", res);

    ArPose minPose, maxPose;
    minPose = arMap.getMinPose();
    maxPose = arMap.getMaxPose();

    printf("Map has Min Coords of (%f %f) mm\n", minPose.getX(), minPose.getY() );
    printf("Map has Min Coords of (%f %f) mm\n", maxPose.getX(), maxPose.getY() );

    int gridX = (maxPose.getX() - minPose.getX())/res;
    int gridY = (maxPose.getY() - minPose.getY())/res;

    printf("Map has grid of (%d %d) mm\n", gridX, gridY);
    
    
    //client.disconnect();
    //exit(0);

    map_resp_.map.info.width = gridX;
    map_resp_.map.info.height = gridY;
    map_resp_.map.info.resolution = res/1000.0;

    map_resp_.map.info.origin.position.x = minPose.getX()/1000.0; //Y?
    map_resp_.map.info.origin.position.y = minPose.getY()/1000.0; //X?

    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = "map";
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
            map_resp_.map.info.width,
            map_resp_.map.info.height,
            map_resp_.map.info.resolution);
    meta_data_message_ = map_resp_.map.info;

    //Lets fill some datas
    map_resp_.map.data.resize(gridX * gridY);

    //Iterate through points and fill
    for(int i=0; i < point_list->size(); i++){
      int coord = gridX *  ((point_list->at(i).getY()-minPose.getY())/res) + (point_list->at(i).getX()-minPose.getX()) /res; //Y is flipped apparent
      printf("Point at Coord %d of %d\n", coord, gridX*gridY);
      map_resp_.map.data[coord] = 100;
    }

    mapped = 1;

  }


  else
  {

    //The header has changed but it still works with the old format for what we want. 
    char *header_location = strstr(buffer, "2D-Map-Ex4");
    if (header_location != NULL) /* Old header found */
    {
      buffer[6] = '\0'; //Cut the -EX4 off 
    }


    //printf("line '%s'\n", buffer);
    arMap.parseLine(buffer);
  }

}


int main(int argc, char **argv)
{

  ArGlobalFunctor1<ArNetPacket *> getMapNameCB(handleGetMapName);
  ArGlobalFunctor1<ArNetPacket *> getMapCB(handleGetMap);

  //Init ROS    
  ros::init(argc, argv, "omron");

  //make node handle  
  ros::NodeHandle n, np("~");

  //Aria
  Aria::init();
  
  //Create our client object. 
  ArClientBase client;

  //Set the magical protocol 
  client.enforceProtocolVersion("5MTX");

  //Settings
  ArArgumentBuilder args;
  //--------  
  //HOST
  args.addPlain("-host");
  std::string sparam;
  if (np.getParam("host", sparam))
  {
    args.addPlain(sparam.c_str());
  }
  else
  {
    args.addPlain("172.19.21.203");  //Default IP
  }
  
  //PORT
  args.addPlain("-p");
  if (np.getParam("port", sparam))
  {
    args.addPlain(sparam.c_str());
  }
  else
  {
    args.addPlain("7272");  //Default PORT
  }

  //USER
  args.addPlain("-u");
  if (np.getParam("user", sparam))
  {
    args.addPlain(sparam.c_str());
  }
  else
  {
    args.addPlain("steve");  //Default user
  }
  //NO PASSWD
  args.addPlain("-np");

  ArClientSimpleConnector clientConnector(&args);

  //Reard in args
  clientConnector.parseArgs();

  //Connect
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      ROS_ERROR("Server '%s' rejected connection, exiting\n", client.getHost());
    else
      ROS_ERROR("Could not connect to server '%s', exiting\n", client.getHost());
    exit(1);
  } 

  ROS_INFO("Connected to server.\n");

  //Setup the laser pub object
  //statusPub pub(&client, &n);

  
  client.addHandler("getMap", &getMapCB);
  client.addHandler("getMapName", &getMapNameCB);
  client.requestOnce("getMapName");
  start.setToNow();
  client.requestOnce("getMap");

  client.runAsync();

  while(!mapped){
    sleep(1);
  }

  ros::Publisher map_pub;
  ros::Publisher metadata_pub;

  // Latched publisher for metadata
  metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  metadata_pub.publish( meta_data_message_ );

  // Latched publisher for data
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_pub.publish( map_resp_.map );

  ros::spin();

  client.disconnect();
  Aria::exit(0);
}