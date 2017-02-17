// Projektgruppe Multi Robotik Wintersemester 2010/2011
//
// wallfollower.cpp
// Author: Torsten Fiolka

// Includes für ROS und die benötigten Nachrichten zwischen den Knoten
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion>
#include <sensor_msgs/LaserScan.h>

// Klassendefinition
class Wallfollower 
{
public:
	Wallfollower();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void Wallfollower::poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();

protected:

	// Nodehandle für den Wallfollower
	ros::NodeHandle m_nodeHandle;

	// Subscriber und Membervariable für den Laser
	ros::Subscriber m_laserSubscriber;
	ros::Subscriber m_robotposeSubscriber;
	ros::Subscriber m_robotorientationSubscriber;
	
	sensor_msgs::LaserScan m_laserscan;

	// Publisher und Membervariable für die Fahrbefehle
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;
	

};

// Konstruktor
Wallfollower::Wallfollower() 
{
	//Knoten wird im root-Namespace angelegt
	ros::NodeHandle m_nodeHandle("/");

	// Initialisierung der Nachrichtenhandler
	m_laserSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Wallfollower::laserCallback, this);
	m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::Point>("point", 20, &Wallfollower::poseCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}

// Callback für den Laser
void Wallfollower::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData)
{
	if( m_laserscan.get_ranges_size() < scanData->get_ranges_size())
		m_laserscan.set_ranges_size(scanData->get_ranges_size());
	for(unsigned int i = 0; i<scanData->get_ranges_size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}


void Wallfollower::poseCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
	pose2d_t ret;

	ret.x = msg.transform.translation.x;
	ret.y = msg.transform.translation.y;

}

// Roboter stoppt, wenn ein Objekt zu nah dran ist
void Wallfollower::emergencyStop() 
{
	for(unsigned int i=0; i<m_laserscan.get_ranges_size(); i++)
	{
		if( m_laserscan.ranges[i] <= 0.15) {
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.1;
		}
	}
}

// Berechnung der Fahrbefehle des Roombas
// Viele Magic Numbers, die sind durch Ausprobieren entstanden und haben keinen tieferen Sinn
void Wallfollower::calculateCommand() 
{

	double laser = 0.0, laserSumForward = 0.0, laserSumRight = 0.0;
	if( m_laserscan.get_ranges_size() > 0 ) 
	{
		for(int i = 180; i<271; i+=5) 
		{
			laser=m_laserscan.ranges[i];
			if( laser > 1.0 ) {
				laser = 1.0;
			}
			laserSumForward += laser;
		}
		for(int i=75; i<135; i++ ) 
		{
			laser=m_laserscan.ranges[i];
			if( laser > 1.0 ) 
			{
				laser = 1.0;
			}
			laserSumRight += laser;
		}
		laserSumRight = laserSumRight / 60.0;
		laserSumForward = laserSumForward / 19.0;
		ROS_DEBUG("Gemittelte Werte rechts: %f - vorwaerts: %f", laserSumRight, laserSumForward);
		if( laserSumRight < 0.7 ) {
			m_roombaCommand.linear.x = laserSumRight / 2.0;
			m_roombaCommand.angular.z = 0.7 - laserSumRight;
		}
		else {
			m_roombaCommand.linear.x = laserSumRight / 3.0;
			m_roombaCommand.angular.z = (laserSumRight - 0.7) * -3.0;
		}
		if( laserSumForward < 1.0 ) {
			m_roombaCommand.linear.x = laserSumForward / 3.0;
			m_roombaCommand.angular.z = 1.0 - laserSumForward;
		}
	}
}

// Hauptschleife
void Wallfollower::mainLoop() {
	// Bestimmt die Anzahl der Schleifendurchläufe pro Sekunde
	ros::Rate loop_rate(20);

	// Schleife bricht ab, wenn der Knoten z.B. ein Kill bekommt
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

		ROS_INFO(" Vorwaerts: %f - Drehung: %f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// Schicke die Fahrbefehle an den Roomba
		m_commandPublisher.publish(m_roombaCommand);

		// SpinOnce führt die Schleife einmal aus
		ros::spinOnce();
        // sleep stoppt den Prozess ~50ms, damit die looprate eingahlten wird
		loop_rate.sleep();
	}
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "wallfollower");

	ros::Time::init();

	ros::Duration(5).sleep();

	Wallfollower w;

	// main loop
	w.mainLoop();

	return 0;
}
