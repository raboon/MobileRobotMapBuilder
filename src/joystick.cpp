#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Joy.h>


// Class Definition
class joystick {
public:
	joystick();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
    

protected:

	// Nodehandle for the joystick
	ros::NodeHandle m_nodeHandle;

	// Subscriber and Membervariable for the Laser
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

    ros::Subscriber m_robotposeSubscriber;
    ros::Subscriber joy_sub_;

	// Publisher und Membervariable für die Fahrbefehle
	ros::Publisher m_commandPublisher;
    //ros::Publisher vel_pub_;
	geometry_msgs::Twist m_roombaCommand;

private:
    int linear_, angular_;
    double l_scale_, a_scale_;
    bool isMoving;

};

// constructor
joystick::joystick() : linear_(1), angular_(2){

    m_nodeHandle.param("axis_linear", linear_, linear_);
    m_nodeHandle.param("axis_angular", angular_, angular_);
    m_nodeHandle.param("scale_angular", a_scale_, a_scale_);
    m_nodeHandle.param("scale_linear", l_scale_, l_scale_);

	//Knoten wird im root-Namespace angelegt
	ros::NodeHandle m_nodeHandle("/");

	// Initialisierung der Nachrichtenhandler
	m_laserSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &joystick::laserCallback, this);
    m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &joystick::poseCallback, this);

    joy_sub_ = m_nodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &joystick::joyCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
    //vel_pub_ = m_nodeHandle.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);

    isMoving=false;
}

// Callback for the Laser
void joystick::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) 
{
/*
	if( m_laserscan.get_ranges_size() < scanData->get_ranges_size())
		m_laserscan.set_ranges_size(scanData->get_ranges_size());
	for(unsigned int i = 0; i<scanData->get_ranges_size(); i++)
	{
		m_laserscan.ranges[i] = (scanData->ranges[i]>1.25)?1.25:scanData->ranges[i];
	}
*/

    m_laserscan.ranges = scanData->ranges;
}

void joystick::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    ROS_INFO("POS X= %f, Y= %f ,YAW = %f \n",msg->pose.pose.position.x,msg->pose.pose.position.y,(57.295645531)*tf::getYaw(msg->pose.pose.orientation));   
   
}

void joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
   {
      
      //turtlesim::Velocity vel;
      m_roombaCommand.angular.z = a_scale_*joy->axes[2];
      m_roombaCommand.linear.x = l_scale_*joy->axes[5];
    //  isMoving=true;
      // vel_pub_.publish(vel);
    //ROS_INFO("Linier: %f, Angular%f ",joy->axes[linear_],joy->axes[angular_]);   
     
   }
// Robot stops when an object is too close
void joystick::emergencyStop() 
{
	for(unsigned int i=0; i<m_laserscan.get_ranges_size(); i++)
	{
		if(m_laserscan.ranges[i] <=.45 && isMoving==true )
		{
            m_roombaCommand.linear.x = 0.0;            
			m_roombaCommand.angular.z = 0.0; 
            isMoving=false;           		
		}

	}
}

// Calculate the motion commands of the Roomba
// Many Magic Numbers that have emerged through trial and error and have no deeper meaning
void joystick::calculateCommand() 
{

	
}

// Hauptschleife
void joystick::mainLoop() {
	// Determines the number of loop iterations per second
	ros::Rate loop_rate(20);
    ROS_INFO(" hi ");
	// Loop stops if the node example gets a kill
	while (m_nodeHandle.ok())
	{
		//calculateCommand();
		//emergencyStop();

		//ROS_INFO(" Vorwaerts: %f - Drehung: %f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// Send the driving commands to the Roomba
		m_commandPublisher.publish(m_roombaCommand);

		// SpinOnce führt die Schleife einmal aus
		ros::spinOnce();
		// sleep stoppt den Prozess ~50ms, damit die looprate eingahlten wird
		loop_rate.sleep();
	}
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "joystick");

	ros::Time::init();

	ros::Duration(5).sleep();

	joystick w;

	// main loop
	w.mainLoop();

	return 0;
}



