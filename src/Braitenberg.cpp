#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>

// Class Definition
class Braitenberg {
public:
	Braitenberg();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
    

protected:

	// Nodehandle for the Braitenberg
	ros::NodeHandle m_nodeHandle;

	// Subscriber and Membervariable for the Laser
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

    ros::Subscriber m_robotposeSubscriber;

	// Publisher und Membervariable für die Fahrbefehle
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Braitenberg::Braitenberg() {
	//Knoten wird im root-Namespace angelegt
	ros::NodeHandle m_nodeHandle("/");

	// Initialisierung der Nachrichtenhandler
	m_laserSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Braitenberg::laserCallback, this);
    m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &Braitenberg::poseCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}

// Callback for the Laser
void Braitenberg::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
	if( m_laserscan.get_ranges_size() < scanData->get_ranges_size())
		m_laserscan.set_ranges_size(scanData->get_ranges_size());
	for(unsigned int i = 0; i<scanData->get_ranges_size(); i++)
	{
		m_laserscan.ranges[i] = (scanData->ranges[i]>2.25)?2.25:scanData->ranges[i];
	}
}

void Braitenberg::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    ROS_INFO("POS X= %f, Y= %f ,YAW = %f \n",msg->pose.pose.position.x,msg->pose.pose.position.y,(57.295645531)*tf::getYaw(msg->pose.pose.orientation));   
  
    
}

// Robot stops when an object is too close
void Braitenberg::emergencyStop() 
{
	for(unsigned int i=0; i<m_laserscan.get_ranges_size(); i++)
	{
		if( m_laserscan.ranges[i] <= 0.70 && m_laserscan.ranges[i] > 0.45) 
        {
			m_roombaCommand.linear.x = 0.2;                         
			
		}
		else if(m_laserscan.ranges[i] <=.45)
		{
			m_roombaCommand.linear.x = 0.0;            
			m_roombaCommand.angular.z = -0.8;            		
		}

	}
}

// Calculate the motion commands of the Roomba
// Many Magic Numbers that have emerged through trial and error and have no deeper meaning
void Braitenberg::calculateCommand() 
{

	double laser = 0.0, laserSumLeft = 0.0, laserSumRight = 0.0;
	if( m_laserscan.get_ranges_size() > 0 ) 
    {
        int noOfRaysFromLeft=45;
        float avgLeftDistance=m_laserscan.ranges[450],avgLeftDistanceDown=0,avgLeftDistanceUp=0;
        for(int i=0;i<noOfRaysFromLeft;i++)
            {
                avgLeftDistance+=m_laserscan.ranges[450-i]+m_laserscan.ranges[450+i];
                avgLeftDistanceUp+=m_laserscan.ranges[450-i];
                avgLeftDistanceDown+=m_laserscan.ranges[450+i];
            }

        avgLeftDistance=avgLeftDistance/((2*noOfRaysFromLeft)+1);
        avgLeftDistanceUp=avgLeftDistanceUp/noOfRaysFromLeft;
        avgLeftDistanceDown=avgLeftDistanceDown/noOfRaysFromLeft;

		ROS_DEBUG("Avarage Value: %f - Right: %f - Left: %f",avgLeftDistance, m_laserscan.ranges[337], m_laserscan.ranges[339]);
		m_roombaCommand.linear.x =.3;         
        
        m_roombaCommand.angular.z =(avgLeftDistanceUp-avgLeftDistanceDown)/2.0;

        if(avgLeftDistanceUp/avgLeftDistanceDown>=1.4 || avgLeftDistanceDown/avgLeftDistanceUp>=1.4)
            {
                m_roombaCommand.linear.x =.005;
                m_roombaCommand.linear.z =(m_roombaCommand.linear.z>0)?.9:-.9;
            }
        else if(avgLeftDistanceUp/avgLeftDistanceDown>=1.25 || avgLeftDistanceDown/avgLeftDistanceUp>=1.25)
            {
                m_roombaCommand.linear.x =.01;
                m_roombaCommand.linear.z =(m_roombaCommand.linear.z>0)?.5:-.5;
            }
         

	}
}

// Hauptschleife
void Braitenberg::mainLoop() {
	// Determines the number of loop iterations per second
	ros::Rate loop_rate(20);
    ROS_INFO(" hi ");
	// Loop stops if the node example gets a kill
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

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
	ros::init(argc, argv, "Braitenberg");

	ros::Time::init();

	ros::Duration(5).sleep();

	Braitenberg w;

	// main loop
	w.mainLoop();

	return 0;
}



