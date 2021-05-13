//ROS OMRON driver
//------------------


#include <ros/ros.h>
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientRatioDrive.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>

class laserPub
{
public:
  //Constructor sets up the rostop
  laserPub(ArClientBase *client, ros::NodeHandle *nh, std::string name="Laser_1Current", std::string topic="/laser"); 
  //The laser callback is called when data is sent from the robot and publishes to ROS
  void laser_cb(ArNetPacket *packet);

protected:
  ros::Publisher laser_pub;
  ArClientBase *myClient;
  ros::NodeHandle *_nh;

  //Don't understand the ArFunctors but this is how they do it
  ArFunctor1C<laserPub, ArNetPacket *> myLaserCB;

  int seq = 0;

};

//Setup laserpub
laserPub::laserPub(ArClientBase *client, ros::NodeHandle *nh, std::string name, std::string topic) : myClient(client), _nh(nh), myLaserCB(this, &laserPub::laser_cb)
{
  laser_pub = _nh->advertise<sensor_msgs::PointCloud2>(topic, 1);
  myClient->addHandler(name.c_str(), &myLaserCB);
  myClient->request(name.c_str(), 200); //Assumes rate of 5hz, TODO could be param

  ROS_INFO("Setup Callback for %s publishing on %s",name.c_str(), topic.c_str());
}

void laserPub::laser_cb(ArNetPacket *packet)
{
  sensor_msgs::PointCloud2 myScan;
  
  myScan.header.stamp = ros::Time::now();
  myScan.header.seq = seq++;
  myScan.header.frame_id = "map";

  int x, y;
  int numReadings;
  int i;

  numReadings = packet->bufToByte4();


  //Setup the PointCloud2 MSG
  myScan.height = 1;
  myScan.width  = numReadings;
  myScan.fields.resize (3);
  myScan.fields[0].name = "x";
  myScan.fields[0].offset = 0;
  myScan.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  myScan.fields[0].count = 1;
  myScan.fields[1].name = "y";
  myScan.fields[1].offset = 4;
  myScan.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  myScan.fields[1].count = 1;
  myScan.fields[2].name = "z";
  myScan.fields[2].offset = 8;
  myScan.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  myScan.fields[2].count = 1;
  myScan.point_step = 12;
  myScan.row_step   = myScan.point_step * myScan.width;
  myScan.data.resize (myScan.row_step   * myScan.height);
  myScan.is_dense = false;

  if (numReadings == 0)
  {
    ROS_WARN("No readings for sensor %s\n\n", myClient->getName(packet));
    return;
  }

  for (i = 0; i < numReadings; i++)
  {
    x = packet->bufToByte4();
    y = packet->bufToByte4();

    float *pstep = (float*)&myScan.data[i * myScan.point_step];

    // Copy XYZ
    pstep[0] = x/1000.0;
    pstep[1] = y/1000.0;
    pstep[2] = 0;
  }

  laser_pub.publish(myScan);
}



int main(int argc, char **argv)
{
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
    args.addPlain("omron");  //Default user
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
  laserPub pub(&client, &n);

  //Setup the laser_low pub object
  laserPub pub_low(&client, &n, "Laser_2Current", "/laser_low");

  client.runAsync();

  ros::spin();

  client.disconnect();
  Aria::exit(0);
}