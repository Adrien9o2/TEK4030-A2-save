#include <ros/ros.h>
#include <planar_robot_simulator/planar_robot_2dof.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


Eigen::Vector2d command(0.0,0.0);



void jointCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  command[0] = (msg.get())->data[0];
  command[1] = (msg.get())->data[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

  /* This is the object that simulates a pendulum */
  PlanarRobotSimulator::PlanarRobot2DOF sim;

  

	sensor_msgs::JointState msg;
	msg.name.push_back("joint_1");
	msg.name.push_back("joint_2");
	msg.position.push_back(0.0);
	msg.position.push_back(0.0);
	msg.velocity.push_back(0.0);
	msg.velocity.push_back(0.0);
  

  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1000);
  ros::Subscriber sub = nh.subscribe("joint_command", 1000, jointCommandCallback);


  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    /* This performs one step in the simulation */
    double dt = 1.0/100.0;
    //double u = 0.0;
    sim.step(dt, command);
    msg.header.stamp = ros::Time::now();

		Eigen::Vector2d q_m_pos = sim.getMotorPosition();
		msg.position[0] = q_m_pos(0);
		msg.position[1] = q_m_pos(1);

		Eigen::Vector2d q_m_vel = sim.getMotorVelocity();
		msg.velocity[0] = q_m_vel(0);
		msg.velocity[1] = q_m_vel(1);

    joint_state_pub.publish(msg);

    /* This creates a window that shows the pendulum */
    sim.draw();

    loop_rate.sleep();
  }

  return 0;
}
