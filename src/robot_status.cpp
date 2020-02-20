//ROS OMRON driver
//------------------


#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientRatioDrive.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "geometry_msgs/PoseStamped.h"

#include <cmath>

class statusPub
{
public:
  //Constructor sets up the rostopic
  statusPub(ArClientBase *client, ros::NodeHandle *nh, std::string name="Pose", std::string topic="/pose"); 
  //The laser callback is called when data is sent from the robot and publishes to ROS
  void pose_cb(ArNetPacket *packet);
  void status_cb(ArNetPacket *packet);
  void simplePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

protected:

  ros::Publisher pose_pub, status_pub;
  ArClientBase *myClient;
  ros::NodeHandle *_nh;
  ros::Subscriber sub;
  //Don't understand the ArFunctors but this is how they do it
  ArFunctor1C<statusPub, ArNetPacket *> myPoseCB;
  ArFunctor1C<statusPub, ArNetPacket *> myStatusCB;

  int seq = 0;

};

//Setup setup callbac stuff
statusPub::statusPub(ArClientBase *client, ros::NodeHandle *nh, std::string name, std::string topic) : myClient(client), _nh(nh), myPoseCB(this, &statusPub::pose_cb)
{
  //Setup simple status publisher
  //TODO  
  //laser_pub = _nh->advertise<sensor_msgs::PointCloud2>(topic, 1);
  
  //Setup Callback for simple goal
  sub = _nh->subscribe("/move_base_simple/goal", 10, &statusPub::simplePoseCallback, this);
  ros::spinOnce;     

  myClient->addHandler("updateNumbers", &myPoseCB);
  myClient->request("updateNumbers", 50); //Seems if we request rate of 50 we get 10hz max

  ROS_INFO("Setup Callback for %s publishing on %s",name.c_str(), topic.c_str());
}

void statusPub::pose_cb(ArNetPacket *packet)
{
  // sensor_msgs::PointCloud2 myScan;
  
  // myScan.header.stamp = ros::Time::now();
  // myScan.header.seq = seq++;
  // myScan.header.frame_id = "base_link";

  int batVolt, x , y, theta, x_vel, y_vel, theta_vel, temp;

  batVolt = packet->bufToByte2();
  x = packet->bufToByte4();
  y = packet->bufToByte4();
  theta = packet->bufToByte2();
  x_vel = packet->bufToByte2();
  theta_vel = packet->bufToByte2();
  y_vel = packet->bufToByte2();
  temp = packet->bufToByte();

  static tf::TransformBroadcaster br; 
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x/1000.0, y/1000.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, angles::from_degrees(theta/1.0)); //TODO this is bit shit there is better numbers around
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

  //laser_pub.publish(myScan);
}

void statusPub::simplePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("I got a pose");


  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (optional 2-byte int)</td>

  int x = msg->pose.position.x*1000;
  int y = msg->pose.position.y*1000;
  //int theta = msg.pose. TODO

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte2ToBuf(0); //Theta
  myClient->requestOnce("gotoPose", &p);

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

  //Setup the status pub object
  statusPub pub(&client, &n);



  client.runAsync();

  ros::spin();

  client.disconnect();
  Aria::exit(0);
}