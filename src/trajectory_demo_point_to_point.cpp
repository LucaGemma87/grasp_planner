// Author(s): Luca Gemma

#include <string>
//#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>


// ROS headers
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <pcl/search/kdtree.h>
#include <pcl_ros/transforms.h>

#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/ObjectManaging.h"
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/CylinderFitting.h"
#include "visual_perception/Cylinder.h"
#include "visual_perception/SphereFitting.h"
#include "visual_perception/ConeFitting.h"
#include "visual_perception/ParallelepipedFitting.h"
#include "visual_perception/Sphere.h"
#include "visual_perception/Cone.h"
#include "grasp_planner/Trajectory.h"
#include "grasp_planner/GraspPlanning.h"
#include "grasp_planner/TrajectoryDemo.h"

// Moveit headers

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>





namespace grasp_planner {

class TrajectoryDemos 
{
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

private:
  //! The node handle
  ros::NodeHandle nh_;
 
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher graspplanning_pub_;

  ros::Publisher display_publisher;
  
  //! Service server for object detection
  ros::ServiceServer grasp_planning_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;



  //------------------ Callbacks -------------------

  
  //! Callback for service calls


  bool serviceCallback(TrajectoryDemo::Request &request, TrajectoryDemo::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TrajectoryDemos(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   graspplanning_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("grasp_planner_markers_out"), 10);

   grasp_planning_srv_ = nh_.advertiseService(nh_.resolveName("grasp_planning_srv"),    &TrajectoryDemos::serviceCallback, this);

   display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>(nh_.resolveName("/move_group/display_planned_path"), 1, true);
   
  
   
  }

  //! Empty stub
  ~TrajectoryDemos() {}

};


/*! call serviceCallback
 */
bool TrajectoryDemos::serviceCallback(TrajectoryDemo::Request &request, TrajectoryDemo::Response &response)
{ 
  int n_object;
  n_object=request.ob;
  double pi(3.1415);
  ROS_INFO("Listening for world_link_2_wrist_stamped");
  // geometry_msgs::PoseStamped pose_wr_msg=request.WristPose;
  // tf::Stamped<tf::Transform> pose_wr;
  // tf::poseStampedMsgToTF(pose_wr_msg,pose_wr);
  // tf::Matrix3x3 rotation_wrist_home = pose_wr.getBasis();
  // tf::Vector3 position_wrist_home = pose_wr.getOrigin();


  // //rotation_wrist_home = rotation_wrist_home.transpose();
  // tf::Quaternion orientation_wrist;
  // rotation_wrist_home.getRotation(orientation_wrist);
  // double roll_wrist, pitch_wrist, yaw_wrist;
  // rotation_wrist_home.getRPY(roll_wrist, pitch_wrist, yaw_wrist);
  // ROS_INFO("RPY_wrist = (%lf, %lf, %lf)", roll_wrist, pitch_wrist, yaw_wrist);
  tf::StampedTransform world_link_2_wrist_stamped;
  ros::Time now = ros::Time::now();
            
  listener_.waitForTransform("world_link", "left_arm_7_link", now, ros::Duration(4));
  listener_.lookupTransform("world_link", "left_arm_7_link", now, world_link_2_wrist_stamped);
  tf::Transform world_link_2_wrist;
  world_link_2_wrist.setOrigin(world_link_2_wrist_stamped.getOrigin());
  world_link_2_wrist.setBasis(world_link_2_wrist_stamped.getBasis());

  tf::Vector3 position_wrist_home = world_link_2_wrist.getOrigin();
  //tf::Vector3 position_wrist_home(-0.811268,-0.208724,0.6);
  tf::Quaternion quaternion_wrist_home = world_link_2_wrist.getRotation();

  tf::Transform wrist_pose;

  wrist_pose.setOrigin(position_wrist_home);
  wrist_pose.setRotation(quaternion_wrist_home);
  tf::Matrix3x3 rotation_wrist_home;
  rotation_wrist_home.setRotation(quaternion_wrist_home);
  double roll_wrist, pitch_wrist, yaw_wrist;
  rotation_wrist_home.getRPY(roll_wrist, pitch_wrist, yaw_wrist);
  ROS_INFO("RPY_wrist = (%f, %f, %f)", roll_wrist, pitch_wrist, yaw_wrist);
  ROS_INFO ("TrajectoryDemos:Success in receving wrist transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",quaternion_wrist_home[0],quaternion_wrist_home[1],quaternion_wrist_home[2],quaternion_wrist_home[3],position_wrist_home[0],position_wrist_home[1],position_wrist_home[2]);


  if(n_object==(int)1)
  {
    visual_perception::Cylinder cylinder;
    // cylinder = request.cylinder;
    tf::Transform rot_cyl;
    double min_x(-0.6);
    double max_x(-0.4);
    double min_y(-0.25);
    double max_y(0);

    srand(time(NULL));
    double X=(max_x-min_x)*((double)rand()/(double)RAND_MAX)+min_x;
    srand(time(NULL));
    double Y=(max_y-min_y)*((double)rand()/(double)RAND_MAX)+min_y;
    tf::Vector3 position_cyl(X,Y,0.155);
    //tf::Vector3 position_cyl(-0.611268,-0.208724,0.155);
    //tf::Vector3 position_cyl(0.000000, -1.129196,0.077);
    tf::Quaternion orientation_cyl(tf::createQuaternionFromRPY(0, 0, 0));
    rot_cyl.setRotation(orientation_cyl);    
    rot_cyl.setOrigin(position_cyl);

    

    cylinder.pose.pose.orientation.x=orientation_cyl[0];
    cylinder.pose.pose.orientation.y=orientation_cyl[1];
    cylinder.pose.pose.orientation.z=orientation_cyl[2];
    cylinder.pose.pose.orientation.w=orientation_cyl[3];

    cylinder.pose.pose.position.x=position_cyl[0]; 
    cylinder.pose.pose.position.y=position_cyl[1];
    cylinder.pose.pose.position.z=position_cyl[2];

    cylinder.h=(double)0.30;

    cylinder.r=(double)0.05;

    ROS_INFO("Created pose for cylinder"); 

    // Create the shape and the marker
    uint32_t cyl_shape = visualization_msgs::Marker::CYLINDER;
    visualization_msgs::Marker cyl_marker;
    // Set the marker type to CYLINDER
    cyl_marker.type = cyl_shape;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //cyl_marker.header.frame_id = "/camera_rgb_optical_frame";
    cyl_marker.header.frame_id = "/world_link";

    cyl_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    cyl_marker.ns = "trajectory_demo_node";

    cyl_marker.id = n_object;

    // Set the marker action.  Options are ADD and DELETE
    cyl_marker.action = visualization_msgs::Marker::ADD;

    cyl_marker.pose.position.x = cylinder.pose.pose.position.x;
    cyl_marker.pose.position.y = cylinder.pose.pose.position.y;
    cyl_marker.pose.position.z = cylinder.pose.pose.position.z;
    cyl_marker.pose.orientation.x = orientation_cyl[0];
    cyl_marker.pose.orientation.y = orientation_cyl[1];
    cyl_marker.pose.orientation.z = orientation_cyl[2];
    cyl_marker.pose.orientation.w = orientation_cyl[3];
           
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
    cyl_marker.scale.x = 2*fabs(cylinder.r);
    cyl_marker.scale.y = 2*fabs(cylinder.r);
    cyl_marker.scale.z = cylinder.h;

    // Set the color -- be sure to set alpha to something non-zero!
    cyl_marker.color.r = 0.0f;
    cyl_marker.color.g = 1.0f;
    cyl_marker.color.b = 0.0f;
    cyl_marker.color.a = 1.0;
    cyl_marker.lifetime = ros::Duration(100);
    graspplanning_pub_.publish(cyl_marker);

    ros::Time now = ros::Time::now();
    broadcaster_.sendTransform(tf::StampedTransform(rot_cyl, now, "world_link", "cylinder"));
          
    
    ROS_INFO("Starting wrist planning for cylinder"); 
    // decompose the cylinder data
    ROS_INFO ("TrajectoryDemos:Success in receving the cylinder transformation with 7 elements (Orient(x,y,z,w),Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",cylinder.pose.pose.orientation.x, cylinder.pose.pose.orientation.y, cylinder.pose.pose.orientation.z, cylinder.pose.pose.orientation.w,cylinder.pose.pose.position.x,  cylinder.pose.pose.position.y,  cylinder.pose.pose.position.z);
    tf::Transform cylinder_transform(tf::Quaternion(cylinder.pose.pose.orientation.x, cylinder.pose.pose.orientation.y, cylinder.pose.pose.orientation.z, cylinder.pose.pose.orientation.w), tf::Vector3( cylinder.pose.pose.position.x,  cylinder.pose.pose.position.y,  cylinder.pose.pose.position.z) );
    
    tf::Matrix3x3 rotation_cylinder = cylinder_transform.getBasis();
    tf::Vector3 position_cylinder = cylinder_transform.getOrigin();
    tf::Quaternion orientation_cylinder;
    rotation_cylinder.getRotation(orientation_cylinder);

    

    // check if the resulting orientation is correct
    if (rotation_cylinder.determinant() < 0.5 )
    {
     ROS_ERROR("The cylinder is bad posed");
     response.result = response.BAD_OBJECT;
     
    }
    else
    { 

      // Define wrist trasform to the cylinder
      // double roll_cyl, pitch_cyl, yaw_cyl;
      // rotation_cylinder.getRPY(roll_cyl, pitch_cyl, yaw_cyl);
      // ROS_INFO("RPY = (%lf, %lf, %lf)", roll_cyl, pitch_cyl, yaw_cyl);
     
      // //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_arm");

      // // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   

      ROS_INFO("Starting getting Trajectory Position");
      
      // Don't change this two value otherwise the compute_Carthesian_path doesn't find the solution 
      int int_traj =(int)6;
      double int_step = (double)6;
      
      while(int_traj<(int)7)
      {
      ROS_INFO("Try to find trajectory with %d points!!",int_traj);
      tf::Transform trajector[int_traj];
      tf::Vector3 position_traj[int_traj];
      tf::Quaternion orientation_traj[int_traj];
      tf::Vector3 position_wrist_traj[int_traj];
      tf::Quaternion quaternion_wrist_traj[int_traj];
      Trajectory traj[int_traj];
      Trajectory traj2[int_traj];
      std::string traj_name[int_traj];
      double Kp = (double)0.85;
      double Krot = (double)1;
      tf::Transform traj_in_wrist[int_traj];
      tf::StampedTransform traj_in_wrist_stamped[int_traj];
      double x_wrist_traj[int_traj],y_wrist_traj[int_traj],z_wrist_traj[int_traj];
      double Roll_angle_traj[int_traj],Pitch_angle_traj[int_traj],Yaw_angle_traj[int_traj];
      double Roll_angle_rot_step,Pitch_angle_rot_step,Yaw_angle_rot_step;
      ROS_INFO("Calculating alpha1");
      if ((position_cylinder[0] != position_wrist_home[0]) & (position_cylinder[1] != position_wrist_home[1]))
      {
       double alpha1=-(pow(position_cylinder[1]-position_wrist_home[1],4))/(position_cylinder[0]-position_wrist_home[0]); 
       double distance_y=position_cylinder[1]-position_wrist_home[1];
       ROS_INFO("Alpha1 = %f",alpha1);
       ROS_INFO("Calculated distance_y: %f",distance_y);
       double step_y=distance_y/int_step;
       ROS_INFO("Step_y = %f",step_y);
       
       if(position_cylinder[2] !=position_wrist_home[2])
       {
         // la follia non ha limiti !!!
         
         double sigma4=(double)6*pow(position_cylinder[1]*position_wrist_home[1],2);
          ROS_INFO("sigma4 = %f",sigma4);


         double sigma5=(double)6*pow(position_cylinder[0]*position_wrist_home[0],2);
         ROS_INFO("sigma5 = %f",sigma5);

         
         double sigma6=(double)4*pow(position_cylinder[1],3)*position_wrist_home[1];
         ROS_INFO("sigma6 = %f",sigma6);

         double sigma7=(double)4*position_cylinder[1]*pow(position_wrist_home[1],3);
         ROS_INFO("sigma7 = %f",sigma7);

         double sigma8=(double)4*pow(position_cylinder[0],3)*position_wrist_home[0];
         ROS_INFO("sigma8 = %f",sigma8);
         
         double sigma9=(double)4*position_cylinder[0]*pow(position_wrist_home[0],3);
         ROS_INFO("sigma9 = %f",sigma9);

         double sigma10=(double)2*pow(position_wrist_home[0]*position_wrist_home[1],2);
         ROS_INFO("sigma10 = %f",sigma10);
         
         double sigma11=(double)2*pow(position_wrist_home[0]*position_cylinder[1],2);
         ROS_INFO("sigma11 = %f",sigma11);

         double sigma12=(double)2*pow(position_cylinder[0]*position_wrist_home[1],2);
         ROS_INFO("sigma12 = %f",sigma12);

         double sigma13=(double)2*pow(position_cylinder[0]*position_cylinder[1],2);
         ROS_INFO("sigma13 = %f",sigma13);
         
         double sigma14=(double)4*pow(position_wrist_home[0],2)*position_cylinder[1]*position_wrist_home[1];
         ROS_INFO("sigma14 = %f",sigma14);

         double sigma15=(double)4*pow(position_cylinder[0],2)*position_cylinder[1]*position_wrist_home[1];
         ROS_INFO("sigma15 = %f",sigma15);

         double sigma16=(double)4*position_cylinder[0]*position_wrist_home[0]*pow(position_wrist_home[1],2);
         ROS_INFO("sigma16 = %f",sigma16);

         double sigma17=(double)4*position_cylinder[0]*position_wrist_home[0]*pow(position_cylinder[1],2);
         ROS_INFO("sigma17 = %f",sigma17);

         double sigma18=(double)8*position_cylinder[0]*position_wrist_home[0]*position_cylinder[1]*position_wrist_home[1];
         ROS_INFO("sigma18 = %f",sigma18);

         double sigma19=pow(position_cylinder[0],4)-sigma8+sigma5-sigma13+sigma15-sigma12-sigma9+sigma17-sigma18+sigma16+pow(position_wrist_home[0],4)-sigma11+sigma14-sigma10+pow(position_cylinder[1],4)-sigma6+sigma4-sigma7+pow(position_wrist_home[1],4)+pow(position_cylinder[2],2)-(double)2*position_cylinder[2]*position_wrist_home[2]+pow(position_wrist_home[2],2);
         ROS_INFO("sigma19 = %f",sigma19);

         double sigma20=pow(position_cylinder[0],4)-sigma8+sigma5+sigma13-sigma15+sigma12-sigma9-sigma17+sigma18-sigma16+pow(position_wrist_home[0],4)+sigma11-sigma14+sigma10+pow(position_cylinder[1],4)-sigma6+sigma4-sigma7+pow(position_wrist_home[1],4)+pow(position_cylinder[2],2)-(double)2*position_cylinder[2]*position_wrist_home[2]+pow(position_wrist_home[2],2);
         ROS_INFO("sigma20 = %f",sigma20);

         double sigma3=sqrt(sigma19*sigma20);
         ROS_INFO("sigma3 = %f",sigma3);

         double sigma1=(sigma5-sigma4-2*position_cylinder[2]*position_wrist_home[2]-sigma3-sigma9-sigma8+sigma7+sigma6+pow(position_cylinder[0],4)+pow(position_wrist_home[0],4)-pow(position_cylinder[1],4)-pow(position_wrist_home[1],4)+pow(position_cylinder[2],2)+pow(position_wrist_home[2],2))/((double)2*(position_cylinder[2]-position_wrist_home[2]));
         ROS_INFO("sigma1 = %f",sigma1);

         double alpha2=position_cylinder[2]-position_wrist_home[2]-sigma1;

         double beta2=sigma1;

         ROS_INFO("Alpha2 e beta2:%f , %f",alpha2,beta2);

         ROS_INFO("Start trajectory for generic position of the cylinder");
         ros::Time now = ros::Time::now();
         //tf::Quaternion quaternion_wrist_final(tf::createQuaternionFromRPY(0, 0, -pi/2));
         double Roll_wrist_end,Pitch_wrist_end,Yaw_wrist_end;
         double distance_x=position_cylinder[0]-position_wrist_home[0];
         double distance_y=position_cylinder[1]-position_wrist_home[1]; 
         double theta;
         if(abs(distance_x) > abs(distance_y))
         {
           theta = atan2(distance_x,distance_y);
         }
         else
         {
           theta = -atan2(distance_x,distance_y); 
         } 
         Roll_wrist_end=(double)(-pi/2+2*theta);
         //Roll_wrist_end=(double)(0);
         Pitch_wrist_end=(double)(0);
         Yaw_wrist_end=(double)(0);
         //Yaw_wrist_end=(double)(pi+2*theta);
         // Roll_wrist_end=(double)(-pi/2);
         // Pitch_wrist_end=(double)(-pi/2);
         // Yaw_wrist_end=(double)(-pi/2);

         Roll_angle_rot_step=(double)(Roll_wrist_end-roll_wrist)/(double)int_step;
         Pitch_angle_rot_step=(double)(Pitch_wrist_end-pitch_wrist)/(double)int_step;
         Yaw_angle_rot_step=(double)(Yaw_wrist_end-yaw_wrist)/(double)int_step;
        
        
         // cylinder in any position
         double step_y=distance_y/int_step;
         for (int i = 0; i < int_traj; ++i)
         {
           
           
           if (i==0)
           {
             Roll_angle_traj[i]=(double)roll_wrist-(double)Roll_angle_rot_step;
             Pitch_angle_traj[i]=(double)pitch_wrist-(double)Pitch_angle_rot_step;
             Yaw_angle_traj[i]=(double)yaw_wrist-(double)Yaw_angle_rot_step;
            }
            else
            {
             Roll_angle_traj[i]=(double)Roll_angle_traj[i-1]-(double)Roll_angle_rot_step;
             Pitch_angle_traj[i]=(double)Pitch_angle_traj[i-1]-(double)Pitch_angle_rot_step;
             Yaw_angle_traj[i]=(double)Yaw_angle_traj[i-1]-(double)Yaw_angle_rot_step;
            }

            ROS_INFO("Roll_angle_traj, Pitch_angle_traj[i],Yaw_angle_traj[i]:%f,%f,%f" ,Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]);
            tf::Quaternion quaternion_wrist_now(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));
            orientation_traj[i]=quaternion_wrist_now.normalize();
           ROS_INFO("quaternion_wrist_now:(%f,%f,%f,%f)",orientation_traj[0][i],orientation_traj[1][i],orientation_traj[2][i],orientation_traj[3][i]);
           //ROS_INFO("quaternion_wrist_now:(%f,%f,%f,%f)",orientation_traj[0],orientation_traj[1],orientation_traj[2],orientation_traj[3]);
           if(position_wrist_home[1]>position_cylinder[1])
           { 
             ROS_INFO("Wrist is on the right of cylinder");
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]-Kp*step_y;
              }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]-Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_cylinder[1],4)/alpha1+position_cylinder[0];
             z_wrist_traj[i]=pow(x_wrist_traj[i]-position_cylinder[0],4)/alpha2+pow(y_wrist_traj[i]-position_cylinder[1],4)/beta2+position_cylinder[2];
            }
          
            else
            { 
             
             ROS_INFO("Wrist is on the left of cylinder");
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]+Kp*step_y;
              }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]+Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_cylinder[1],4)/alpha1+position_cylinder[0];
             z_wrist_traj[i]=pow(x_wrist_traj[i]-position_cylinder[0],4)/alpha2+pow(y_wrist_traj[i]-position_cylinder[1],4)/beta2+position_cylinder[2];
            }
            ROS_INFO("x_wrist_traj,y_wrist_traj,z_wrist_traj:%f,%f,%f",x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]);
           traj[i].Traj.pose.position.x = x_wrist_traj[i];
           traj[i].Traj.pose.position.y = y_wrist_traj[i];
           traj[i].Traj.pose.position.z = z_wrist_traj[i];            
           traj[i].Traj.pose.orientation.x = orientation_traj[0][i];
           traj[i].Traj.pose.orientation.y =orientation_traj[1][i];
           traj[i].Traj.pose.orientation.z = orientation_traj[2][i];
           traj[i].Traj.pose.orientation.w = orientation_traj[3][i];
           traj[i].Traj.header.frame_id = "world_link";
           
           std::string traj_num("traj");
           std::ostringstream out;
           out << i;
           std::string n_traj;
           n_traj = out.str();
           traj_name[i] = traj_num+ n_traj;
            
           trajector[i].setOrigin(tf::Vector3(x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]));
           //trajector[i].setRotation(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));
           trajector[i].setRotation(orientation_traj[i]); 
           broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "world_link", traj_name[i]));
        
           ROS_INFO("Showed traj_number:%d",i);

          }         
        }
       else
       {
         ROS_INFO("Wrist is on the same z of the cylinder");
         ros::Time now = ros::Time::now();
         //tf::Quaternion quaternion_wrist_final(tf::createQuaternionFromRPY(0, 0, -pi/2));
         double Roll_wrist_end,Pitch_wrist_end,Yaw_wrist_end;
         Roll_wrist_end=(double)0;
         Pitch_wrist_end=(double)0;
         Yaw_wrist_end=(double)0;
         Roll_angle_rot_step=(Roll_wrist_end-roll_wrist)/int_step;
         Pitch_angle_rot_step=(Pitch_wrist_end-pitch_wrist)/int_step;
         Yaw_angle_rot_step=(Yaw_wrist_end-yaw_wrist)/int_step;
        
        
         // same z but x and y different for wrist and cylinder
         
         for (int i = 0; i < int_traj; ++i)
         {
           z_wrist_traj[i]=position_cylinder[2];
           //y_wrist_traj[i]=position_cylinder[1];
           if (i==0)
           {
             Roll_angle_traj[i]=roll_wrist+Roll_angle_rot_step;
             Pitch_angle_traj[i]=pitch_wrist+Pitch_angle_rot_step;
             Yaw_angle_traj[i]=yaw_wrist+Yaw_angle_rot_step;
            }
            else
            {
             Roll_angle_traj[i]=Roll_angle_traj[i-1]+Roll_angle_rot_step;
             Pitch_angle_traj[i]=Pitch_angle_traj[i-1]+Pitch_angle_rot_step;
             Yaw_angle_traj[i]=Yaw_angle_traj[i-1]+Yaw_angle_rot_step;
            }


            tf::Quaternion quaternion_wrist_now(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));
            orientation_traj[i]=quaternion_wrist_now;
          
           if(position_wrist_home[1]>position_cylinder[1])
           { 
             ROS_INFO("Wrist is on the right of cylinder");
             double distance_y=position_cylinder[1]-position_wrist_home[1];
             ROS_INFO("Calculated distance_y: %f",distance_y);
             double step_y=distance_y/int_step;
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]-Kp*step_y;
             }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]-Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_cylinder[1],4)/alpha1+position_cylinder[0];
            }
          
            else
            { 
             ROS_INFO("Wrist is on the left of the cylinder");
             double distance_y=position_cylinder[1]-position_wrist_home[1];
             ROS_INFO("Calculated distance_y: %f",distance_y);
             double step_y=distance_y/int_step;
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]+Kp*step_y;
              }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]+Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_cylinder[1],4)/alpha1+position_cylinder[0];
            }
          
           traj[i].Traj.pose.position.x = x_wrist_traj[i];
           traj[i].Traj.pose.position.y = y_wrist_traj[i];
           traj[i].Traj.pose.position.z = z_wrist_traj[i];            
           traj[i].Traj.pose.orientation.x = orientation_traj[0][i];
           traj[i].Traj.pose.orientation.y =orientation_traj[1][i];
           traj[i].Traj.pose.orientation.z = orientation_traj[2][i];
           traj[i].Traj.pose.orientation.w = orientation_traj[3][i];
           traj[i].Traj.header.frame_id = "world_link";

           std::string traj_name;
           std::string traj_num("traj");
           std::ostringstream out;
           out << i;
           std::string n_traj;
           n_traj = out.str();
           traj_name = traj_num+ n_traj;
            
           trajector[i].setOrigin(tf::Vector3(x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]));
           trajector[i].setRotation(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));  
           broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "world_link", traj_name));
          
        
           ROS_INFO("Showed traj_number:%d",i);
          } 
        }
      }
      else
      { 
       
       // same x and y for wrist and cylinder !!!
       ROS_INFO("Wrist is on or down the cylinder");
       ros::Time now = ros::Time::now();
        //tf::Quaternion quaternion_wrist_final(tf::createQuaternionFromRPY(0, 0, -pi/2));
        double Roll_wrist_end,Pitch_wrist_end,Yaw_wrist_end;
        Roll_wrist_end=(double)-pi;
        Pitch_wrist_end=(double)-pi;
        Yaw_wrist_end=(double)0;
        Roll_angle_rot_step=(Roll_wrist_end-roll_wrist)/int_step;
        Pitch_angle_rot_step=(Pitch_wrist_end-pitch_wrist)/int_step;
        Yaw_angle_rot_step=(Yaw_wrist_end-yaw_wrist)/int_step;
        

        for (int i = 0; i < int_traj; ++i)
        {
          z_wrist_traj[i]=position_cylinder[0];
          if (i==0)
          {
            Roll_angle_traj[i]=roll_wrist+Roll_angle_rot_step;
            Pitch_angle_traj[i]=pitch_wrist+Pitch_angle_rot_step;
            Yaw_angle_traj[i]=yaw_wrist+Yaw_angle_rot_step;
          }
          else
          {
            Roll_angle_traj[i]=Roll_angle_traj[i-1]+Roll_angle_rot_step;
            Pitch_angle_traj[i]=Pitch_angle_traj[i-1]+Pitch_angle_rot_step;
            Yaw_angle_traj[i]=Yaw_angle_traj[i-1]+Yaw_angle_rot_step;
          }


          tf::Quaternion quaternion_wrist_now(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));

          orientation_traj[i]=quaternion_wrist_now.normalized();
          

          if (position_wrist_home[2]>position_cylinder[2])
           { 
            ROS_INFO("Wrist is on the cylinder");
            double distance_z=position_wrist_home[2]-position_cylinder[2];
            ROS_INFO("Calculated distance_z: %f",distance_z);
            double step_z=distance_z/int_step;
            if(i==0) z_wrist_traj[i]=position_wrist_home[2]-Kp*step_z;
            else 
              {
                z_wrist_traj[i]=z_wrist_traj[i-1]-Kp*step_z;
              }
            
           }
          else
          { 
            ROS_INFO("Wrist is down the cylinder");
            double distance_z=position_cylinder[2]-position_wrist_home[2];
            ROS_INFO("Calculated distance_z: %f",distance_z);
            double step_z=distance_z/int_step;
            if(i==0) z_wrist_traj[i]=position_wrist_home[2]+Kp*step_z;
            else 
              {
                z_wrist_traj[i]=z_wrist_traj[i-1]+Kp*step_z;
              }
          }

         traj[i].Traj.pose.position.x = x_wrist_traj[i];
         traj[i].Traj.pose.position.y = y_wrist_traj[i];
         traj[i].Traj.pose.position.z = z_wrist_traj[i];            
         traj[i].Traj.pose.orientation.x = orientation_traj[0][i];
         traj[i].Traj.pose.orientation.y =orientation_traj[1][i];
         traj[i].Traj.pose.orientation.z = orientation_traj[2][i];
         traj[i].Traj.pose.orientation.w = orientation_traj[3][i];
         traj[i].Traj.header.frame_id = "world_link";
         std::string traj_name;
         std::string traj_num("traj");
         std::ostringstream out;
         out << i;
         std::string n_traj;
         n_traj = out.str();
         traj_name = traj_num+ n_traj;
            
         trajector[i].setOrigin(tf::Vector3(x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]));
         //trajector[i].setRotation(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));  
         trajector[i].setRotation(orientation_traj[i]); 
         broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "world_link", traj_name));
        
         ROS_INFO("Showed traj_number:%d",i); 

        }

      }  

      
  /////Getting Basic Information
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // // We can also print the name of the end-effector link for this group.
  ROS_INFO("EndEffector frame: %s", group.getEndEffectorLink().c_str());

  // // Cartesian Paths
  // // ^^^^^^^^^^^^^^^
  // // You can plan a cartesian path directly by specifying a list of waypoints 
  // // for the end-effector to go through. Note that we are starting 
  // // from the new start state above.  The initial pose (start state) does not
  // // need to be added to the waypoint list.
 //group.setStartStateToCurrentState();
  ROS_INFO ("TrajectoryDemos:Success in receving wrist transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",quaternion_wrist_home[0],quaternion_wrist_home[1],quaternion_wrist_home[2],quaternion_wrist_home[3],position_wrist_home[0],position_wrist_home[1],position_wrist_home[2]);
  geometry_msgs::Pose start_pose;
  start_pose.orientation.x = quaternion_wrist_home[0];
  start_pose.orientation.y = quaternion_wrist_home[1];
  start_pose.orientation.z = quaternion_wrist_home[2];
  start_pose.orientation.w = quaternion_wrist_home[3];

  start_pose.position.x = position_wrist_home[0];
  start_pose.position.y = position_wrist_home[1];
  start_pose.position.z = position_wrist_home[2];

  std::vector<geometry_msgs::Pose> waypoints[int_traj];
  geometry_msgs::Pose target_pose1[int_traj];// = start_pose;
  
 for (int i = 0; i < int_traj; ++i)//int_traj
  {
   ROS_INFO("Target pose1.x,Target pose1.y, Target pose1.z: %f,%f ,%f",traj[i].Traj.pose.position.x,traj[0].Traj.pose.position.y,traj[0].Traj.pose.position.z);
   ROS_INFO("traj[i].Traj.pose.orientation.x,traj[i].Traj.pose.orientation.y,traj[i].Traj.pose.orientation.z,traj[i].Traj.pose.orientation.w: %f,%f ,%f,%f",traj[i].Traj.pose.orientation.x,traj[i].Traj.pose.orientation.y,traj[i].Traj.pose.orientation.z,traj[i].Traj.pose.orientation.w);
   target_pose1[i].position.x = traj[i].Traj.pose.position.x;
   target_pose1[i].position.y = traj[i].Traj.pose.position.y;
   target_pose1[i].position.z = traj[i].Traj.pose.position.z;
   target_pose1[i].orientation.x = traj[i].Traj.pose.orientation.x;
   target_pose1[i].orientation.y = traj[i].Traj.pose.orientation.y;
   target_pose1[i].orientation.z = traj[i].Traj.pose.orientation.z;
   target_pose1[i].orientation.w = traj[i].Traj.pose.orientation.w;
   waypoints[i].push_back(target_pose1[i]);
  }
 

  // // We want the cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively
  // // disabling it.
  //group.setPlannerId("PRMstarkConfigDefault");
  //group.setPlannerId("RRTConnectkConfigDefault");
  double fraction[int_traj];
  double eef_step((double)0.50);
  double jump_threshold((double)0.000);
  double inc_treshold((double)0.1);
  double inc_eef_step((double)0.01);
  int int_jump_max(10);
  moveit_msgs::RobotTrajectory trajectory[int_traj];
  
  for (int i = 0; i < int_traj; ++i)
   {
     while((double)fraction[i]*100<(double)90)
     {  
       if (i==0) group.setStartState(*group.getCurrentState());
       
       jump_threshold=(double)0.00;//(inc_treshold*i);
       group.allowReplanning ("true");
       //group.setNumPlanningAttempts(10);
       //group.setPlannerId("RRTstarkConfigDefault");
       //group.setPlannerId("SBLkConfigDefault");
       //group.setPlannerId("ESTkConfigDefault");
       //group.setPlannerId("LBKPIECEkConfigDefault");
       //group.setPlannerId("BKPIECEkConfigDefault");
       //group.setPlannerId("KPIECEkConfigDefault");
       //group.setPlannerId("RRTkConfigDefault");
       //group.setPlannerId("RRTConnectkConfigDefault");
       //group.setPlannerId("TRRTkConfigDefault");
       //group.setPlannerId("PRMkConfigDefault");
        group.setPlannerId("PRMstarkConfigDefault");
        group.setPlanningTime((double)10);
        group.setGoalOrientationTolerance((double)0); 
        group.setGoalPositionTolerance((double)0.001); 
    
        //double 
       fraction[i] = group.computeCartesianPath(waypoints[i],
                                                eef_step,  // eef_step
                                                jump_threshold,   // jump_threshold
                                                trajectory[i]);

        ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction[i] * 100.0);    
       // /* Sleep to give Rviz time to visualize the plan. */
  
       //jump_threshold+=inc_treshold;
       //if((double)fraction*100>(double)90) break;
       //sleep(2.0);
        eef_step=eef_step-inc_eef_step;
        if(eef_step<(double)0.009)
        {
         ROS_ERROR("No solution for trajectory");
         break; 
        }
   
       if((double)fraction[i]>(double)0.90)
       {
       //   // std::ofstream name_file_ptr;
       //   // name_file_ptr.open("/home/luca/file_txt/cylinder_trajectory_demo.txt",std::ios::app);
      
       //   // name_file_ptr << int_step <<" ";
       //   // name_file_ptr << fraction <<" ";       
       //   // name_file_ptr << eef_step <<" ";
       //   // name_file_ptr << jump_threshold <<" ";

       //   // name_file_ptr << "\n";
       //   // name_file_ptr.close();  
       //   // ROS_INFO("Data wrote into the file cylinder_trajectory_demo.txt");
       ROS_DEBUG("Exit For::Find solution with eef_step and jump_threshold: %f %f",eef_step,jump_threshold);
        break;
      }
      
      if((double)fraction[int_traj-1]*100>(double)90) 
     {
       ROS_DEBUG("EXit While2::Find solution with eef_step and jump_threshold: %f %f",eef_step,jump_threshold);
       //break;
      }

      
    }  
     eef_step=(double)0.50; 
   
    response.result=response.SUCCESS;
    ROS_INFO("Starting sending response");
    //sleep(15.0);  
      for(int i =0;i<int_traj;i++)
      {
        response.traj[i] = traj[i];
      }
    ROS_INFO("TrajectoryDemo::Response Send");      
    
  } 
   
    
    // if((double)fraction[int_traj-1]*100>(double)90) 
    // {
    //   ROS_DEBUG("EXit While1::Find solution with eef_step and jump_threshold: %f %f",eef_step,jump_threshold);
    //   break;
    // }
    int_step+=(int)1;
    int_traj+=(double)1;
        
    }
   
  
  }

  }
  
  if(n_object==(int)2)
  {
    visual_perception::Sphere sphere;
    //sphere= request.sphere;
    ROS_INFO("Starting wrist planning for sphere"); 
  
    // // decompose the sphere data
    // ROS_INFO ("TrajectoryDemos:Success in receving the sphere transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z,sphere.pose.pose.orientation.w,sphere.pose.pose.position.x,sphere.pose.pose.position.y,sphere.pose.pose.position.z);
    // tf::Transform sphere_transform(tf::Quaternion(sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w), tf::Vector3( sphere.pose.pose.position.x,sphere.pose.pose.position.y,sphere.pose.pose.position.z) );
    
    // tf::Matrix3x3 rotation_sphere = sphere_transform.getBasis();
    // tf::Vector3 position_sphere = sphere_transform.getOrigin();
    // tf::Quaternion orientation_sphere;
    // rotation_sphere.getRotation(orientation_sphere);

    tf::Transform rot_sph;
    tf::Vector3 position_sph(-0.611268,-0.208724,0.155);
    //tf::Vector3 position_sph(0.000000, -1.129196,0.077);
    tf::Quaternion orientation_sph(tf::createQuaternionFromRPY(0, 0, 0));
    rot_sph.setRotation(orientation_sph);    
    rot_sph.setOrigin(position_sph);

    

    sphere.pose.pose.orientation.x=orientation_sph[0];
    sphere.pose.pose.orientation.y=orientation_sph[1];
    sphere.pose.pose.orientation.z=orientation_sph[2];
    sphere.pose.pose.orientation.w=orientation_sph[3];

    sphere.pose.pose.position.x=position_sph[0]; 
    sphere.pose.pose.position.y=position_sph[1];
    sphere.pose.pose.position.z=position_sph[2];

    

    sphere.r=(double)0.15;

    ROS_INFO("Created pose for cylinder"); 

    // Create the shape and the marker
    uint32_t sph_shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker sph_marker;
    // Set the marker type to CYLINDER
    sph_marker.type = sph_shape;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //cyl_marker.header.frame_id = "/camera_rgb_optical_frame";
    sph_marker.header.frame_id = "/world_link";

    sph_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    sph_marker.ns = "trajectory_demo_node";
    sph_marker.id = n_object;

    // Set the marker action.  Options are ADD and DELETE
    sph_marker.action = visualization_msgs::Marker::ADD;

    sph_marker.pose.position.x = sphere.pose.pose.position.x;
    sph_marker.pose.position.y = sphere.pose.pose.position.y;
    sph_marker.pose.position.z = sphere.pose.pose.position.z;
    sph_marker.pose.orientation.x = orientation_sph[0];
    sph_marker.pose.orientation.y = orientation_sph[1];
    sph_marker.pose.orientation.z = orientation_sph[2];
    sph_marker.pose.orientation.w = orientation_sph[3];
           
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
    sph_marker.scale.x = 2*fabs(sphere.r);
    sph_marker.scale.y = 2*fabs(sphere.r);
    sph_marker.scale.z = 2*fabs(sphere.r);

    // Set the color -- be sure to set alpha to something non-zero!
    sph_marker.color.r = 0.0f;
    sph_marker.color.g = 1.0f;
    sph_marker.color.b = 0.0f;
    sph_marker.color.a = 1.0;
    sph_marker.lifetime = ros::Duration(30);
    graspplanning_pub_.publish(sph_marker);

    ros::Time now = ros::Time::now();
    broadcaster_.sendTransform(tf::StampedTransform(rot_sph, now, "world_link", "sphere"));
          
    
    ROS_INFO("Starting wrist planning for sphere"); 
    // decompose the cylinder data
    ROS_INFO ("TrajectoryDemos:Success in receving the sphere transformation with 7 elements (Orient(x,y,z,w),Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w,sphere.pose.pose.position.x,  sphere.pose.pose.position.y,  sphere.pose.pose.position.z);
    tf::Transform sphere_transform(tf::Quaternion(sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w), tf::Vector3( sphere.pose.pose.position.x,  sphere.pose.pose.position.y,  sphere.pose.pose.position.z) );
    
    tf::Matrix3x3 rotation_sphere = sphere_transform.getBasis();
    tf::Vector3 position_sphere = sphere_transform.getOrigin();
    tf::Quaternion orientation_sphere;
    rotation_sphere.getRotation(orientation_sphere);

    

    // check if the resulting orientation is correct
    if (rotation_sphere.determinant() < 0.5 )
    {
     ROS_ERROR("The sphere is bad posed");
     response.result = response.BAD_OBJECT;
     
    }
    else
    { 

      // Define wrist trasform to the cylinder
      // double roll_cyl, pitch_cyl, yaw_cyl;
      // rotation_cylinder.getRPY(roll_cyl, pitch_cyl, yaw_cyl);
      // ROS_INFO("RPY = (%lf, %lf, %lf)", roll_cyl, pitch_cyl, yaw_cyl);
     
      // //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_arm");

      // // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   

        

      // moveit_msgs::CollisionObject collision_object;
      // collision_object.header.frame_id = group.getPlanningFrame();

        

      // /* The id of the object is used to identify it. */
      // collision_object.id = "cylinder_link";

      // /* Define a box to add to the world. */
      // shape_msgs::SolidPrimitive primitive;
      // primitive.type = primitive.CYLINDER;
      // primitive.dimensions.resize(3);
      // primitive.dimensions[0] =2*fabs(cylinder.r) ;
      // primitive.dimensions[1] =2*fabs(cylinder.r) ;
      // primitive.dimensions[2] =cylinder.h;

      //  //A pose for the box (specified relative to frame_id) 
      // //geometry_msgs::Pose _pose;
        
      // collision_object.primitives.push_back(primitive);
      // collision_object.primitive_poses.push_back(cylinder.pose.pose);
      // collision_object.operation = collision_object.ADD;

      //  std::vector<moveit_msgs::CollisionObject> collision_objects;  
      //  collision_objects.push_back(collision_object);  

      //  // Now, let's add the collision object into the world
      //  ROS_INFO("Add the cylinder into the world");  
      //  planning_scene_interface.addCollisionObjects(collision_objects);
  
       /* Sleep so we have time to see the object in RViz */
      //sleep(2.0);
    
    }
  }

  if(n_object==(int)3)
  {
    visual_perception::Cone  cone;
    //cone = request.cone;
    
    ROS_INFO("Starting wrist planning for cone"); 
  
    // decompose the cone data
    ROS_INFO ("TrajectoryDemos:Success in receving the cone transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",cone.pose.pose.orientation.x, cone.pose.pose.orientation.y, cone.pose.pose.orientation.z,cone.pose.pose.orientation.w,cone.pose.pose.position.x,cone.pose.pose.position.y,cone.pose.pose.position.z);
    tf::Transform cone_transform(tf::Quaternion(cone.pose.pose.orientation.x, cone.pose.pose.orientation.y, cone.pose.pose.orientation.z, cone.pose.pose.orientation.w), tf::Vector3( cone.pose.pose.position.x,cone.pose.pose.position.y,cone.pose.pose.position.z) );
    
    tf::Matrix3x3 rotation_cone = cone_transform.getBasis();
    tf::Vector3 position_cone = cone_transform.getOrigin();
    tf::Quaternion orientation_cone;
    rotation_cone.getRotation(orientation_cone);

    

    // check if the resulting orientation is correct
    if (rotation_cone.determinant() < 0.5 )
    {
     ROS_ERROR("The cone is bad posed");
     response.result = response.BAD_OBJECT;
     
    }
    else
    { 

      // Define wrist trasform to the cone
      double distance = 0.2;
      double threshold = 0.05;
      double pi = 3.14159;
      double roll_con, pitch_con, yaw_con;
      rotation_cone.getRPY(roll_con, pitch_con, yaw_con);
      ROS_INFO("RPY = (%lf, %lf, %lf)", roll_con, pitch_con, yaw_con);
      

      //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_arm");

      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

        

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = group.getPlanningFrame();

        

      /* The id of the object is used to identify it. */
      collision_object.id = "CONE";

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] =cone.h ;
      primitive.dimensions[1] =cone.h;
      primitive.dimensions[2] =cone.h;

      /* A pose for the box (specified relative to frame_id) */
      //geometry_msgs::Pose _pose;
        
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(cone.pose.pose);
      collision_object.operation = collision_object.ADD;

       std::vector<moveit_msgs::CollisionObject> collision_objects;  
       collision_objects.push_back(collision_object);  

       // Now, let's add the collision object into the world
       ROS_INFO("Add the cone into the world");  
       planning_scene_interface.addCollisionObjects(collision_objects);
  
       /* Sleep so we have time to see the object in RViz */
       sleep(2.0);



      /// continuare .....    

    }


  }

  if(n_object==(int)4)
  {
   visual_perception::Plane plane;

   //plane=request.plane;

   
   ROS_INFO("Starting wrist planning for plane"); 
  
    // decompose the plane data
    ROS_INFO ("TrajectoryDemos:Success in receving the plane transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",plane.pose.pose.orientation.x, plane.pose.pose.orientation.y, plane.pose.pose.orientation.z,plane.pose.pose.orientation.w,plane.pose.pose.position.x,plane.pose.pose.position.y,plane.pose.pose.position.z);
    tf::Transform plane_transform(tf::Quaternion(plane.pose.pose.orientation.x, plane.pose.pose.orientation.y, plane.pose.pose.orientation.z, plane.pose.pose.orientation.w), tf::Vector3( plane.pose.pose.position.x,plane.pose.pose.position.y,plane.pose.pose.position.z) );
    
    tf::Matrix3x3 rotation_plane = plane_transform.getBasis();
    tf::Vector3 position_plane = plane_transform.getOrigin();
    tf::Quaternion orientation_plane;
    rotation_plane.getRotation(orientation_plane);

    

    // check if the resulting orientation is correct
    if (rotation_plane.determinant() < 0.5 )
    {
     ROS_ERROR("The plane is bad posed");
     response.result = response.BAD_OBJECT;
     
    }
    else
    { 

      // Define wrist trasform to the plane
      double distance = 0.2;
      double threshold = 0.05;
      double pi = 3.14159;
      double roll_plan, pitch_plan, yaw_plan;
      rotation_plane.getRPY(roll_plan, pitch_plan, yaw_plan);
      ROS_INFO("RPY = (%lf, %lf, %lf)", roll_plan, pitch_plan, yaw_plan);


      /// continuare .....    

    } 
  }



} 

















}//namespace grasp_planner













int main(int argc, char **argv) 
{
  ros::init(argc, argv, "grasp_planning_node");
  ros::NodeHandle nh;
 
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  grasp_planner::TrajectoryDemos node(nh);

  ros::spin();
  return 0;
};