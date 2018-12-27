#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <sstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>          // odom
#include <geometry_msgs/Twist.h>        // cmd_vel
#include <sensor_msgs/Range.h>         //sonars
#include <cereal_port/CerealPort.h>

// MAGABOT Dimensions

           
#ifndef NORMALIZE
    #define NORMALIZE(z) atan2(sin(z), cos(z))  // Normalize angle to domain -pi, pi 
#endif

double MAGABOT_WIDTH = 0.345;   //m. Between wheels
double WHEEL_RADIUS = 0.043937; //m
double TICKS_PER_TURN_L = 3900;//2450;
double TICKS_PER_TURN_R = 2450; //2230;
//MAGABOT data for odometry

double TWOPI  = 6.2831853070;
double PI = 3.1415926535;

const float K_MOTOR_RATIO = TICKS_PER_TURN_L * 0.00332 / ( PI * WHEEL_RADIUS);
ros::Publisher *odom_pub;
ros::Publisher *sonars_pub;
tf::TransformBroadcaster *odom_broadcaster;

cereal::CerealPort serial_port;

ros::Time current_time, last_time;
std::string base_frame_id;
std::string odom_frame_id;
double last_time_pub = 0.0;

bool signof (int n) { return n >= 0; }
bool confirm_communication = true;
int ID_Robot = 0;

double odometry_x = 0.0;
double odometry_y = 0.0;
double odometry_yaw = 0.0; 
double odometry_pitch = 0.0;
double odometry_roll = 0.0;
int right_encoder_prev = 0;
int left_encoder_prev = 0;
int n_inter = 0;
double dt = 0;
double vel_x = 0;
double vel_y = 0;
double vel_yaw = 0;
bool to_write = false;
double yaw_default = 0;
int left_encoder_count = 0;
int right_encoder_count = 0;
int left_encoder_acc = 0;
int right_encoder_acc = 0;
double x,y,th;
int n_iter = 0;
float correction = 0;
int lvel = 0;
int rvel = 0;



void drive(double linear_speed, double angular_speed) 
{
    ROS_INFO("LINEAR SPEED: %f", linear_speed);
    float ang = angular_speed * MAGABOT_WIDTH/2;
    int left_write = 0;
    int right_write = 0;
    if(linear_speed == 0 && angular_speed == 0)
    {
        left_write = 0;
        right_write = 0;
    }
    else
    {
        float l_ratio = K_MOTOR_RATIO* 0.5543808461*pow(fabs(linear_speed - ang),-0.1423547731);
        float r_ratio = K_MOTOR_RATIO* 0.5543808461*pow(fabs(linear_speed + ang),-0.1423547731);
    
        right_write = (int)((linear_speed + (angular_speed * MAGABOT_WIDTH / 2)) * r_ratio);
        left_write = (int)((linear_speed - (angular_speed * MAGABOT_WIDTH / 2)) * l_ratio);
    }
    //ROS_FATAL("[ref_values]: Vl = %d ; Vr = %d",left_write, right_write);

    if(left_write < 6 && left_write > 0)
    {
        left_write = 7;
    }

    if(left_write > -6 && left_write < 0)
    {
        left_write = -7;
    }
    if(right_write < 6 && right_write > 0)
    {
        right_write = 7;
    }

    if(right_write > -6 && right_write < 0)
    {
        right_write = -7;
    }

    
    lvel = left_write;
    rvel = right_write;
    
}
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
        //ROS_FATAL("[cmd_vel]: Vlinear = %f, Vangular = %f",cmd_vel->linear.x, cmd_vel->angular.z);
    drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

void publish_sonars(int sonars[5])
{
    sensor_msgs::Range son;
    
    for(int i=0;i<5;i++){
        son.header.stamp = ros::Time::now();
        std::string s = std::to_string(i);
        son.header.frame_id = "sonar"+s;
        son.radiation_type = 0;
        son.field_of_view = PI;
        son.min_range = 0;
        son.max_range = 2;
        float val = sonars[i]/100.0; 
        son.range = roundf(val * 100) / 100;
        sonars_pub->publish(son);
    }
}

void getSonars()
{
    char sonars_command[1];
    sonars_command[0] = char(0x83);
    serial_port.write(sonars_command,1);
    char sonars_response[10];
    serial_port.read(sonars_response, 10, 1000);
    int sonars[5];
    sonars[0] = ((int)(unsigned char)sonars_response[0] * 255 + (int)(unsigned char)sonars_response[1]);
    sonars[1] = ((int)(unsigned char)sonars_response[2] * 255 + (int)(unsigned char)sonars_response[3]);
    sonars[2] = ((int)(unsigned char)sonars_response[4] * 255 + (int)(unsigned char)sonars_response[5]);
    sonars[3] = ((int)(unsigned char)sonars_response[6] * 255 + (int)(unsigned char)sonars_response[7]);
    sonars[4] = ((int)(unsigned char)sonars_response[8] * 255 + (int)(unsigned char)sonars_response[9]);

    //ROS_INFO("SONARS:%d %d %d %d %d", sonars[0], sonars[1], sonars[2], sonars[3], sonars[4]);

    publish_sonars(sonars);
}

void getIR()
{
    char ir_command[1];
    ir_command[0] = char(0x49);
    serial_port.write(ir_command,1);
    char ir_response[6];
    serial_port.read(ir_response, 6, 1000);
    int ir0 = ((int)(unsigned char)ir_response[0]*255 + (int)(unsigned char)ir_response[1]);
    int ir1 = ((int)(unsigned char)ir_response[2]*255 + (int)(unsigned char)ir_response[3]);
    int ir2 = ((int)(unsigned char)ir_response[4]*255 + (int)(unsigned char)ir_response[5]);
    //ROS_FATAL("IRS:%d %d %d", ir0, ir1, ir2);

}

void getBattery()
{
    char bat_command[1];
    bat_command[0] = char(0x4B);
    serial_port.write(bat_command,1);
    char bat_response[2];
    serial_port.read(bat_response, 2, 1000);
    int bat_level = ((int)bat_response[0]*255 + (int)bat_response[1]);
    //ROS_FATAL("BATTERY LEVEL %d", bat_level);
}

bool getBumpers()
{
    bool _bump = false;
    char bumper_command[1];
    bumper_command[0] = char(0x66);
    serial_port.write(bumper_command,1);
    char bumper_response[2];
    serial_port.read(bumper_response, 2, 1000);
    if(bumper_response[0] == 1 || bumper_response[1] == 1)
    {
    _bump = true;
    //ROS_FATAL("BUMP");
    }
    
    return _bump;
    
}

void publish_odometry(int l_ticks, int r_ticks)
{
    double last_x = odometry_x;
    double last_y = odometry_y;
    double last_yaw = odometry_yaw;  
    last_time = current_time;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    bool publish_info = true;
    //ROS_FATAL("Num ticks acumulados: %i , %i", l_ticks, r_ticks);
    
    double dl = -(double)(l_ticks * 3.14 * 2 * WHEEL_RADIUS /TICKS_PER_TURN_L);
    double dr = (double)(r_ticks * 3.14 * 2 * WHEEL_RADIUS / TICKS_PER_TURN_R); 

    double dx = (dr + dl) /2.0;
    double dyaw = (double)((dr - dl)/MAGABOT_WIDTH);
    odometry_yaw += dyaw; 
    //odometry_yaw = -(ypr[0]);             //rad
    //double aux = odometry_yaw + angle;
    //odometry_yaw = atan2(sin(aux), cos(aux));                     //rad
    odometry_x += dx * cos((double) odometry_yaw);   //m
    odometry_y += dx * sin((double) odometry_yaw);  //m

    //double dt = (current_time - last_time).toSec();
    double vel_x = dx/dt;
    double vel_y = 0;
    double vel_yaw = (odometry_yaw - last_yaw)/dt;


// ******************************************************************************************       //first, we'll publish the transforms over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
            
    odom_trans.transform.translation.x = odometry_x;
    odom_trans.transform.translation.y = odometry_y;


    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_yaw);
    odom_broadcaster->sendTransform(odom_trans);
        
    // ******************************************************************************************
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
        
    //set the position
    odom.pose.pose.position.x = odometry_x;
    odom.pose.pose.position.y = odometry_y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_yaw);
//set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_yaw;

    odom_pub->publish(odom);
}

void setLEDS(int r, int g, int b){
    char led_command[4];
    led_command[0] = (char)(0x76);
    led_command[1] = (char)(r);
    led_command[2] = (char)(g);
    led_command[3] = (char)(b);
    serial_port.write(led_command,4);
}


void timerCallback(const ros::TimerEvent&)
{

    setLEDS(255,0,0);
    getSonars();

    //GET ODOMETRY  
    char tick_command[1];
    tick_command[0] = char(0x74);
    serial_port.write(tick_command,1);
    char tick_response[4];
    serial_port.read(tick_response, 4, 1000);
    int _l_ticks = ((int)tick_response[0]*256 + (int)tick_response[1]);
    int _r_ticks = ((int)tick_response[2]*256 + (int)tick_response[3]);
    //ROS_FATAL("ODOM:%d %d", _l_ticks, _r_ticks);  
    publish_odometry(_l_ticks, _r_ticks);
    
    getIR();    
    getBattery();   
    getBumpers();
    
    
    //WRITE VELOCITIES
    
    char vel_command[5];
    vel_command[0] = (char)(0x86);
    vel_command[1] = (char)((int)(abs(lvel)));
    if(lvel > 0)
        vel_command[2] = (char)(0);
    else
        vel_command[2] = (char)(1);

    vel_command[3] = (char)((int)(abs(rvel)));
    
    if(rvel > 0)
        vel_command[4] = (char)(0);
    else
        vel_command[4] = (char)(1); 
        
    serial_port.write(vel_command,5);
}

//receive cmds_vel from nav_stack

int main(int argc, char** argv){ 
  
    ros::init(argc, argv, "magabotnode");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    ros::NodeHandle pni("~");
    std::string port, portinertial;
    
    if (argc<2)
    {
        port="/dev/ttyACM0";
        //portinertial="/dev/ttyACM1";
        ROS_WARN("No Serial Port defined, defaulting to \"%s\"",port.c_str());
        ROS_WARN("Usage: \"rosrun [pkg] robot_node /serial_port\"");
    }
    else
    {
        port="/dev/ttyACM0";
        //portinertial="/dev/ttyACM1";
        ROS_INFO("Serial port: %s",port.c_str());
    }   
    


    pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    
    // ROS publishers and subscribers

    ros::Publisher odom_pub_ptr = n.advertise<nav_msgs::Odometry>("/odom", 500);
    ros::Publisher sonars_pub_ptr = n.advertise<sensor_msgs::Range>("/sonars", 500);
    tf::TransformBroadcaster odom_broadcaster_ptr;
    ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelReceived);
    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
    odom_pub = &odom_pub_ptr;
    odom_broadcaster = &odom_broadcaster_ptr;
    sonars_pub = &sonars_pub_ptr;
    
    
    // baud_rate and serial port:   
    int baudrate;
    pn.param<std::string>("port", port, port.c_str()); 
    pn.param("baudrate", baudrate, 9600); 

    // Open the serial port to the robot
    try
    { 
        serial_port.open((char*)port.c_str(), baudrate); 
    }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Robot -- Failed to open odometry serial port %s with error %s!",port.c_str(),e.what());
        ROS_BREAK();
    }
    ros::Duration(2.5).sleep(); 
    odometry_x = 0.;
    odometry_y = 0.;
    odometry_yaw = 0.;
   
    ros::spin(); //trigger callbacks and prevents exiting
    return(0);
}


