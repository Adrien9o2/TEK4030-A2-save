#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <planar_robot_simulator/planar_robot_2dof.h>
#include <eigen3/Eigen/Eigen>

using namespace PlanarRobotSimulator::Parameters;


ros::Publisher* command_pub_global = NULL;
std_msgs::Float64MultiArray msgJointCommand;
Eigen::Vector2d w_n(2.0, 2.0);
Eigen::Vector2d ksi(2.0, 2.0);
Eigen::Vector2d k_m(1.0/k_v_1, 1.0/k_v_2);

Eigen::Vector2d e_i_m(0.0, 0.0);
Eigen::Vector2d T_p(1.0,1.0);

Eigen::Vector2d T_v((r_a_1*I_m_1)/(k_v_1*k_t_1),(r_a_1*I_m_1)/(k_v_2*k_t_2));
//Eigen::Vector2d K_p(1.0, 1.0);

Eigen::Vector2d K_p = 0.5*w_n.cwiseQuotient(ksi);
Eigen::Vector2d K_v = 2*(w_n.cwiseProduct(ksi)).cwiseQuotient(k_m);


Eigen::Vector2d q_m_set(0.5*M_PI*k_r_1, -0.25*M_PI*k_r_2);
Eigen::Vector2d q_m(0.0,0.0);
Eigen::Vector2d qd_m(0.0,0.0);
Eigen::Vector2d e_m(0.0,0.0);
bool FirstCallbackReached=false;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  q_m[0] = msg->position[0];
  q_m[1] = msg->position[1];
  qd_m[0] = msg->velocity[0];
  qd_m[1] = msg->velocity[1];

  e_m = q_m_set-q_m;
  //Eigen::Vector2d u = K_p.cwiseProduct(e_i_m+T_p.cwiseProduct(e_m));
  //Eigen::Vector2d u = T_p.cwiseProduct(e_m);
  Eigen::Vector2d u = K_v.cwiseProduct(K_p.cwiseProduct(e_i_m)-q_m+T_v.cwiseProduct(K_p.cwiseProduct(e_m)-qd_m));
  msgJointCommand.data = {u[0],u[1]};
  
   
 
  command_pub_global->publish(msgJointCommand);  
  FirstCallbackReached=true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  msgJointCommand.data.push_back(0.0);
  msgJointCommand.data.push_back(0.0);
  ros::Publisher command_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_command", 1000);
  command_pub_global = &command_pub;
  ros::Subscriber sub = nh.subscribe("joint_state", 1000, jointStateCallback);

  sleep(5);

  ros::Rate loop_rate(100);
  double dt = 1.0/100.0;

  while (ros::ok())
  {
    ros::spinOnce();
    e_i_m += dt*e_m;


    loop_rate.sleep();

  }

  return 0;
}
