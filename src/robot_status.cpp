//ROS OMRON driver
//------------------


#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientRatioDrive.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <rv_msgs/SimpleRequest.h>
#include <std_srvs/Empty.h>
#include "rv_omron_driver/Omron.h"

#include <cmath>

class statusPub
{
public:
  //Constructor sets up the rostopic
  statusPub(ArClientBase *client, ros::NodeHandle *nh, std::string name="Pose", std::string topic="/pose"); 
  //The laser callback is called when data is sent from the robot and publishes to ROS
  void pose_cb(ArNetPacket *packet);
  void status_cb(ArNetPacket *packet);
  void dock_stats_cb(ArNetPacket *packet);
  void simplePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void LocaliseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void moveExecteCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);

  bool requestDock(rv_msgs::SimpleRequest::Request &req, rv_msgs::SimpleRequest::Response &res){
    res.result = true;
    myClient->requestOnce("dock");
    return true;
  } 

  //Public robot stats
  int robotMode = 0;
  int robotStatus = 0;
  int dock_status = 0;

  geometry_msgs::PoseStamped currentPose;

protected:

  ros::Publisher pose_pub, status_pub;
  ArClientBase *myClient;
  ros::NodeHandle *_nh;
  ros::Subscriber sub_goal, sub_initpose;


  //Don't understand the ArFunctors but this is how they do it
  ArFunctor1C<statusPub, ArNetPacket *> myPoseCB;
  ArFunctor1C<statusPub, ArNetPacket *> myStatusCB;
  ArFunctor1C<statusPub, ArNetPacket *> dockedStatusCB;

  int seq = 0;

  //Action Server
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; //Make a simple move to pose action server

  //feedback mesaages 
  move_base_msgs::MoveBaseFeedback feedback_; 
  move_base_msgs::MoveBaseResult result_;


  //Dock server
  ros::ServiceServer service;

  //Define callbacks
  void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void cmdVelWD(const ros::TimerEvent&);


  //Cmd Vel variables
  ros::Timer timer;
  ros::Subscriber cmdVelSub;
  uint8_t velCount;
  uint8_t prevVelCount;
  bool vel_valid;

};

//Setup setup callbac stuff
statusPub::statusPub(ArClientBase *client, ros::NodeHandle *nh, std::string name, std::string topic) : 
      myClient(client), _nh(nh), 
      myPoseCB(this, &statusPub::pose_cb), dockedStatusCB(this, &statusPub::dock_stats_cb),
      myStatusCB(this, &statusPub::status_cb),
      as_(*_nh, "move_base",  boost::bind(&statusPub::moveExecteCB, this, _1), false)
{
  //Setup simple status publisher
  //TODO  

  //Setup Callback for simple goal & localisation
  sub_goal = _nh->subscribe("/move_base_simple/goal", 10, &statusPub::simplePoseCallback, this);
  sub_initpose = _nh->subscribe("/initialpose", 10, &statusPub::LocaliseCallback, this);
  ros::spinOnce;     

  myClient->addHandler("updateNumbers", &myPoseCB);
  myClient->request("updateNumbers", 50); //Seems if we request rate of 50 we get 10hz max

  myClient->addHandler("updateStrings", &myStatusCB);
  myClient->request("updateStrings", -1); //request when changed

  myClient->addHandler("dockInfoChanged", &dockedStatusCB);
  myClient->requestOnce("dockInfoChanged");
  myClient->request("dockInfoChanged", -1);

  ROS_INFO("Setup Callback for %s publishing on %s",name.c_str(), topic.c_str());

  //Create the action server
  as_.start();

  //Setup 
  service = _nh->advertiseService("dock", &statusPub::requestDock, this);

  //Advertise Publisher
  status_pub = _nh->advertise<rv_omron_driver::Omron>("robot_status", 100);

  //Cmd vel
  cmdVelSub = _nh->subscribe("cmd_vel", 10, &statusPub::cmdVelCB, this); //register callback
  timer=_nh->createTimer(ros::Duration(0.2), &statusPub::cmdVelWD, this);
  velCount = 0;
  prevVelCount = 0;
  vel_valid = false;
}

void statusPub::pose_cb(ArNetPacket *packet)
{
  double batVolt, x , y, theta, x_vel, y_vel, theta_vel, temp;

  batVolt = ( (double) packet->bufToByte2() )/10.0;
  x = (double) packet->bufToByte4();
  y = (double) packet->bufToByte4();
  theta = (double) packet->bufToByte2();
  x_vel = (double) packet->bufToByte2();
  theta_vel = (double) packet->bufToByte2();
  y_vel = (double) packet->bufToByte2();
  temp = (double) packet->bufToByte();

  static tf::TransformBroadcaster br; 
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x/1000.0, y/1000.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, angles::from_degrees(theta/1.0)); //TODO this is bit shit there is better numbers around
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

  currentPose.header.frame_id = "base_link";
  currentPose.header.stamp = ros::Time::now();
  currentPose.pose.orientation.x = q.getX();
  currentPose.pose.orientation.y = q.getY();
  currentPose.pose.orientation.z = q.getZ();
  currentPose.pose.orientation.w = q.getW();
  currentPose.pose.position.x = x/1000.0;
  currentPose.pose.position.y = y/1000.0;
  currentPose.pose.position.z = 0;

  rv_omron_driver::Omron data;
  data.batteryPercentage = batVolt;
  data.dockStatus = dock_status;
  data.robotStatus = robotStatus;
  //publish the status data
  status_pub.publish(data);
}

void statusPub::status_cb(ArNetPacket *packet)
{
  char myStatus[256];
  char myMode[256];
  memset(myStatus, 0, sizeof(myStatus));
  memset(myMode, 0, sizeof(myMode));
  packet->bufToStr(myStatus, sizeof(myStatus));
  packet->bufToStr(myMode, sizeof(myMode));


  //Lets fill status using ActionLib Msgs - Goal Status
  char * pch;
  pch = strstr(myStatus, "Parking");  //When it is in an idle state the robot retruns parking
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::PENDING;
  }
  pch = strstr(myStatus, "Undocking");  //Undocking to move to the goal
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::ACTIVE;
  }
  pch = strstr(myStatus, "Going"); //Moving to goal
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::ACTIVE;
  }
  pch = strstr(myStatus, "Arrived"); //Got to goal
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::SUCCEEDED;
  }
  pch = strstr(myStatus, "Failed: Failed going to goal"); //Couldn't make it e.g. path blocked
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::ABORTED;
  }
  pch = strstr(myStatus, "Failed: Cannot find path"); //Couldnt find valid plan
  if (pch != NULL){
    robotStatus = actionlib_msgs::GoalStatus::REJECTED;
  }

  //ROS_INFO("Status %10s, code %d", myStatus, robotStatus);

}

void statusPub::dock_stats_cb(ArNetPacket *packet)
{
  int state = packet->bufToUByte();
  int forcedDock = packet->bufToUByte();
  int secondsToShutdown = packet->bufToUByte2();

  std::string stateStr;
  std::string forcedStr;

  if (state == 0)
    stateStr = "  Undocked";
  else if (state == 1)
    stateStr = "   Docking";
  else if (state == 2)
    stateStr = "   Docked";
  else if (state == 3)
    stateStr = "Undocking";
  else
    stateStr = "  Unknown";
  
  if (forcedDock == 0)
    forcedStr = "false";
  else if (forcedDock == 1)
    forcedStr = " true";
  else
    forcedStr = "unknown";

  //Store it
  this->dock_status = state;

  if (secondsToShutdown == 0)
    ROS_INFO("State: %s Forced: %s Shutdown: never", 
	       stateStr.c_str(), forcedStr.c_str());
  else
    ROS_INFO( "State: %s Forced: %s Shutdown: %d", 
	       stateStr.c_str(), forcedStr.c_str(), secondsToShutdown);
  
}

void statusPub::simplePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (optional 2-byte int)</td>

  int x = msg->pose.position.x*1000;
  int y = msg->pose.position.y*1000;

  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.orientation,orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int theta = angles::to_degrees(yaw);

  ROS_INFO("I got a pose at (%d, %d) Theta: %d", x, y, theta);

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte4ToBuf(theta); //Theta
  myClient->requestOnce("gotoPose", &p);

}

void statusPub::LocaliseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (4-byte int)</td>

  int x = msg->pose.pose.position.x*1000;
  int y = msg->pose.pose.position.y*1000;

  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.pose.orientation,orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int theta = angles::to_degrees(yaw);

  ROS_INFO("Set pose to (%d, %d) Theta: %d", x, y, theta);

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte4ToBuf(theta); //ThetaI got a pose at
  myClient->requestOnce("localizeToPose", &p);

}

void statusPub::moveExecteCB(const move_base_msgs::MoveBaseGoalConstPtr &goal){
  //TODO
  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (optional 2-byte int)</td>

  int x = goal->target_pose.pose.position.x*1000;
  int y = goal->target_pose.pose.position.y*1000;

  tf2::Quaternion orientation;
  tf2::fromMsg(goal->target_pose.pose.orientation,orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int theta = angles::to_degrees(yaw);

  ROS_INFO("I got a pose at (%d, %d) Theta: %d", x, y, theta);

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte4ToBuf(theta); //Theta
  myClient->requestOnce("gotoPose", &p);

  ros::Duration(0.5).sleep(); //Takes some time to switch states on robot

  //Provide feedback at 10hz
  ros::Rate r(10);

  bool running = true;
  bool success = true;
  while (running){
    
    //Check if pre-empted
    if (as_.isPreemptRequested() || !ros::ok()){
        ROS_INFO("%s: Preempted", "Move Base");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        running = false;

        //stop robot
        myClient->requestOnce("stop", NULL);
    }

    //Check if active 
    if (robotStatus == actionlib_msgs::GoalStatus::ACTIVE){
        ROS_INFO("%s: Moving to Goal", "Move Base");
    }
    if (robotStatus == actionlib_msgs::GoalStatus::SUCCEEDED){
        ROS_INFO("%s: Got to Goal", "Move Base");
        running = false;
        success = true;
        as_.setSucceeded(result_);
    }
    if (robotStatus == actionlib_msgs::GoalStatus::ABORTED || robotStatus == actionlib_msgs::GoalStatus::REJECTED){
        ROS_INFO("%s: Failed getting to Goal", "Move Base");
        running = false;
        success = false;
        as_.setAborted(result_);
    }

    feedback_.base_position = currentPose;
    as_.publishFeedback(feedback_);

    r.sleep();
  }
    


}

void statusPub::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg){
  velCount++;
  if (fabs(msg->linear.x) > 0.001 || fabs(msg->angular.z) > 0.001){
    ArNetPacket packet;
    vel_valid = true;
    packet.doubleToBuf(msg->linear.x);
    packet.doubleToBuf(msg->angular.z);
    packet.doubleToBuf(100); // this is an additional amount (percentage) that is applied to each of the trans,rot,lat velocities. 
    packet.doubleToBuf(0.0);
  //  if (myPrinting) printf("ArClientRatioDrive: Sending ratioDrive request\n");
    myClient->requestOnce("ratioDrive", &packet);
  }
  else {
    vel_valid = false;
  }

}

void statusPub::cmdVelWD(const ros::TimerEvent&){
    if (velCount - prevVelCount > 0){
      //Valid data
      prevVelCount = velCount;
    }
    else if (vel_valid == true){
      vel_valid = false;
      myClient->requestOnce("stop");
      ROS_WARN("Timeout on cmd_vel");
    }
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