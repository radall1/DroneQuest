/* Provides a trajectory for the vehicle to follow.

  EXAMPLE FILE; ONLY FOR SUPPORT.

  Whatever you do here, output a time-based continuous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <freyja_msgs/ReferenceState.h>

#define ROS_NODE_NAME "trajectory_provider"
typedef freyja_msgs::ReferenceState TrajRef;

#define DEG2RAD(D) ((D)*3.1415326/180.0)

// global decl for "start time". This can be reset by a callback
ros::Time init_time;

class DroneType {
  private:
    TrajRef ref_state;

  public:
      // Constructor
      DroneType(){
        ref_state.pn = 0.0;
        ref_state.pe = 0.0;
        ref_state.pd = -1.2;
        ref_state.vn = 0.0;
        ref_state.ve = 0.0;
        ref_state.vd = 0.0;
        ref_state.yaw = DEG2RAD(0.0);
        ref_state.an = 0.0;
        ref_state.ae = 0.0;
        ref_state.ad = 0.0;
      }
    
      // Setter methods
      void forward() {
        if (ref_state.pe < 2.8){
          ref_state.pe +=0.4;
          ref_state.pe = std::round(ref_state.pe * 100) / 100;
        } 
      }

      void backward() {
        if (ref_state.pe > -1.6){
            ref_state.pe -=0.4;
            ref_state.pe = std::round(ref_state.pe * 100) / 100;
        } 
      }

      void right() {
        if (ref_state.pn > -2.4){
          ref_state.pn -=0.4;
          ref_state.pn = std::round(ref_state.pn * 100) / 100;
        } 
      }

      void left() {
        if (ref_state.pn < 1.2){
          ref_state.pn +=0.4;
          ref_state.pn = std::round(ref_state.pn * 100) / 100;
        } 
      }

      // Attaches the time
      TrajRef return_state() {
        ref_state.header.stamp = ros::Time::now();
        return ref_state;     
      }
};


void timeResetCallback( const std_msgs::UInt8::ConstPtr &msg )
{
  if( msg -> data == 1 )
  {
    init_time = ros::Time::now();
    ROS_WARN( "%s: Time reset requested!", ROS_NODE_NAME );
  }
}


DroneType drone;

void gestureUpdate( const std_msgs::String::ConstPtr &msg )
{
  if (msg -> data == "Forward" ){
    drone.forward();
  } else if (msg -> data == "Right"){
    drone.right();
  } else if (msg -> data == "Left"){
    drone.left();
  } else if (msg -> data == "Backward"){
    drone.backward();
  } 
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, ROS_NODE_NAME);
  ros::NodeHandle nh, priv_nh("~");
  

  /* Publisher for trajectory */
  ros::Publisher traj_pub;
  traj_pub = nh.advertise <TrajRef> ( "/reference_state", 1, true );
  
  /* Create subscriber for resetting time -- restart the trajectory */
  ros::Subscriber time_reset_sub;
  time_reset_sub = nh.subscribe( "/reset_trajectory_time", 1, timeResetCallback );

  /* Create subscriber for receiving commands from the gesture unit */
  ros::Subscriber cameraStuff;
  cameraStuff = nh.subscribe( "/gesture/hand_sign", 1, gestureUpdate );
  
  std::string traj_type;
  priv_nh.param( "example_traj_type", traj_type, std::string("hover") );

  /* How fast should a trajectory update be made? */
  ros::Rate update_rate(50);
  init_time = ros::Time::now();

  while( ros::ok() ) {

    TrajRef ref_state;
    ref_state = drone.return_state();
    traj_pub.publish(ref_state);

    ros::spinOnce();
    update_rate.sleep();
  }

  return 0;
}
