// Author(s): Luca Gemma

#include <string>
//#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>


// ROS headers
#include <ros/ros.h>


#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


// PCL headers

#include <pcl_ros/transforms.h>


#include "visual_perception/ObjectManaging.h"

#include "visual_perception/SphereFitting.h"

#include "visual_perception/Sphere.h"

#include "grasp_planner/Trajectory.h"
#include "grasp_planner/GraspPlanning.h"
#include "grasp_planner/TrajectoryDemo.h"
#include "grasp_planner/TrajectoryDemoSphere.h"

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


  bool serviceCallback(TrajectoryDemoSphere::Request &request, TrajectoryDemoSphere::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TrajectoryDemos(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   graspplanning_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("grasp_planner_markers_sphere_out"), 10);

   grasp_planning_srv_ = nh_.advertiseService(nh_.resolveName("grasp_planning_sphere_affordance_1_srv"),    &TrajectoryDemos::serviceCallback, this);

   display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>(nh_.resolveName("/move_group/display_planned_path"), 1, true);
   
  
   
  }

  //! Empty stub
  ~TrajectoryDemos() {}

};


/*! call serviceCallback
 */
bool TrajectoryDemos::serviceCallback(TrajectoryDemoSphere::Request &request, TrajectoryDemoSphere::Response &response)
{ 

  double pi(3.1415);
  double threshold_distance((double)0.015);
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
  ros::Time now0 = ros::Time::now();
            
  //listener_.waitForTransform("world_link","left_arm_7_link", now, ros::Duration(4));
  //listener_.lookupTransform("world_link","left_arm_7_link", now, world_link_2_wrist_stamped);

  listener_.waitForTransform("vito_anchor","left_hand_palm_dummy_link", now0, ros::Duration(4));
  listener_.lookupTransform("vito_anchor","left_hand_palm_dummy_link", now0, world_link_2_wrist_stamped);
  
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



  visual_perception::Sphere sphere;
  sphere= request.sphere;
  tf::Transform rot_sph(tf::Quaternion(sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w), tf::Vector3( sphere.pose.pose.position.x,  sphere.pose.pose.position.y,  sphere.pose.pose.position.z));;;
    

  ROS_INFO("Sphere received"); 

    // Create the shape and the marker
    uint32_t sph_shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker sph_marker;
    // Set the marker type to sphere
    sph_marker.type = sph_shape;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //sph_marker.header.frame_id = "/camera_rgb_optical_frame";
    sph_marker.header.frame_id = "/vito_anchor";

    sph_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    sph_marker.ns = "trajectory_demo_sphere_node";

    sph_marker.id =(int)1;

    // Set the marker action.  Options are ADD and DELETE
    sph_marker.action = visualization_msgs::Marker::ADD;

    sph_marker.pose.position.x = sphere.pose.pose.position.x;
    sph_marker.pose.position.y = sphere.pose.pose.position.y;
    sph_marker.pose.position.z = sphere.pose.pose.position.z;
    sph_marker.pose.orientation.x = sphere.pose.pose.orientation.x;
    sph_marker.pose.orientation.y = sphere.pose.pose.orientation.y;
    sph_marker.pose.orientation.z = sphere.pose.pose.orientation.z;
    sph_marker.pose.orientation.w = sphere.pose.pose.orientation.w;
           
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
    sph_marker.scale.x = 2*fabs(sphere.r);
    sph_marker.scale.y = 2*fabs(sphere.r);
    sph_marker.scale.z = 2*fabs(sphere.r);

    // Set the color -- be sure to set alpha to something non-zero!
    sph_marker.color.r = 0.0f;
    sph_marker.color.g = 0.0f;
    sph_marker.color.b = 1.0f;
    sph_marker.color.a = 1.0;
    sph_marker.lifetime = ros::Duration(20);
    graspplanning_pub_.publish(sph_marker);

    ros::Time now1 = ros::Time::now();
    broadcaster_.sendTransform(tf::StampedTransform(rot_sph, now1, "vito_anchor",  "sphere"));
          
    
    ROS_INFO("Starting wrist planning for sphere"); 
    // decompose the sphere data
    ROS_INFO("TrajectoryDemos:Success in receving the sphere transformation with 7 elements (Orient(x,y,z,w),Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f", sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w, sphere.pose.pose.position.x,  sphere.pose.pose.position.y,  sphere.pose.pose.position.z);
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

      // Define wrist trasform to the sphere
      // double roll_cyl, pitch_cyl, yaw_cyl;
      // sphere.getRPY(roll_cyl, pitch_cyl, yaw_cyl);
      // ROS_INFO("RPY = (%lf, %lf, %lf)", roll_cyl, pitch_cyl, yaw_cyl);
     
      // //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_hand_arm");

      // // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
      
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
     moveit_msgs::RobotTrajectory trajectory;
      ROS_INFO("Starting getting Trajectory Position");
      
      // (Value : 3 , 4,)  Don't change this two value otherwise the compute_Carthesian_path doesn't find the solution 
      int int_traj =(int)8;
      double int_step = (double)8;
      
      while(int_traj<(int)9)
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
   
      tf::Transform traj_in_wrist[int_traj];
      tf::StampedTransform traj_in_wrist_stamped[int_traj];
      double x_wrist_traj[int_traj],y_wrist_traj[int_traj],z_wrist_traj[int_traj];
      double Roll_angle_traj[int_traj],Pitch_angle_traj[int_traj],Yaw_angle_traj[int_traj];
      double Roll_angle_rot_step,Pitch_angle_rot_step,Yaw_angle_rot_step;
      double step_y;
      ROS_INFO("Calculating alpha1");
      if ((position_sphere[0] != position_wrist_home[0]) & (position_sphere[1] != position_wrist_home[1]))
      {
       double alpha1=-(pow(position_sphere[1]-position_wrist_home[1],4))/(position_sphere[0]-position_wrist_home[0]); 
       double distance_y=position_sphere[1]-position_wrist_home[1];
       double Kp = ((double)distance_y - (double)sphere.r -(double)threshold_distance)/(double)distance_y;
       double Krot = (double)0.85;
       ROS_INFO("Alpha1 = %f",alpha1);
       ROS_INFO("Calculated distance_y: %f",distance_y);
       step_y=(double)distance_y/(double)int_step;
       ROS_INFO("Step_y = %f",step_y);
       
       if(position_sphere[2] !=position_wrist_home[2])
       {
         // la follia non ha limiti !!!
         
         double sigma4=(double)6*pow(position_sphere[1]*position_wrist_home[1],2);
          ROS_INFO("sigma4 = %f",sigma4);


         double sigma5=(double)6*pow(position_sphere[0]*position_wrist_home[0],2);
         ROS_INFO("sigma5 = %f",sigma5);

         
         double sigma6=(double)4*pow(position_sphere[1],3)*position_wrist_home[1];
         ROS_INFO("sigma6 = %f",sigma6);

         double sigma7=(double)4*position_sphere[1]*pow(position_wrist_home[1],3);
         ROS_INFO("sigma7 = %f",sigma7);

         double sigma8=(double)4*pow(position_sphere[0],3)*position_wrist_home[0];
         ROS_INFO("sigma8 = %f",sigma8);
         
         double sigma9=(double)4*position_sphere[0]*pow(position_wrist_home[0],3);
         ROS_INFO("sigma9 = %f",sigma9);

         double sigma10=(double)2*pow(position_wrist_home[0]*position_wrist_home[1],2);
         ROS_INFO("sigma10 = %f",sigma10);
         
         double sigma11=(double)2*pow(position_wrist_home[0]*position_sphere[1],2);
         ROS_INFO("sigma11 = %f",sigma11);

         double sigma12=(double)2*pow(position_sphere[0]*position_wrist_home[1],2);
         ROS_INFO("sigma12 = %f",sigma12);

         double sigma13=(double)2*pow(position_sphere[0]*position_sphere[1],2);
         ROS_INFO("sigma13 = %f",sigma13);
         
         double sigma14=(double)4*pow(position_wrist_home[0],2)*position_sphere[1]*position_wrist_home[1];
         ROS_INFO("sigma14 = %f",sigma14);

         double sigma15=(double)4*pow(position_sphere[0],2)*position_sphere[1]*position_wrist_home[1];
         ROS_INFO("sigma15 = %f",sigma15);

         double sigma16=(double)4*position_sphere[0]*position_wrist_home[0]*pow(position_wrist_home[1],2);
         ROS_INFO("sigma16 = %f",sigma16);

         double sigma17=(double)4*position_sphere[0]*position_wrist_home[0]*pow(position_sphere[1],2);
         ROS_INFO("sigma17 = %f",sigma17);

         double sigma18=(double)8*position_sphere[0]*position_wrist_home[0]*position_sphere[1]*position_wrist_home[1];
         ROS_INFO("sigma18 = %f",sigma18);

         double sigma19=pow(position_sphere[0],4)-sigma8+sigma5-sigma13+sigma15-sigma12-sigma9+sigma17-sigma18+sigma16+pow(position_wrist_home[0],4)-sigma11+sigma14-sigma10+pow(position_sphere[1],4)-sigma6+sigma4-sigma7+pow(position_wrist_home[1],4)+pow(position_sphere[2],2)-(double)2*position_sphere[2]*position_wrist_home[2]+pow(position_wrist_home[2],2);
         ROS_INFO("sigma19 = %f",sigma19);

         double sigma20=pow(position_sphere[0],4)-sigma8+sigma5+sigma13-sigma15+sigma12-sigma9-sigma17+sigma18-sigma16+pow(position_wrist_home[0],4)+sigma11-sigma14+sigma10+pow(position_sphere[1],4)-sigma6+sigma4-sigma7+pow(position_wrist_home[1],4)+pow(position_sphere[2],2)-(double)2*position_sphere[2]*position_wrist_home[2]+pow(position_wrist_home[2],2);
         ROS_INFO("sigma20 = %f",sigma20);

         double sigma3=sqrt(sigma19*sigma20);
         ROS_INFO("sigma3 = %f",sigma3);

         double sigma1=(sigma5-sigma4-2*position_sphere[2]*position_wrist_home[2]-sigma3-sigma9-sigma8+sigma7+sigma6+pow(position_sphere[0],4)+pow(position_wrist_home[0],4)-pow(position_sphere[1],4)-pow(position_wrist_home[1],4)+pow(position_sphere[2],2)+pow(position_wrist_home[2],2))/((double)2*(position_sphere[2]-position_wrist_home[2]));
         ROS_INFO("sigma1 = %f",sigma1);

         double alpha2=position_sphere[2]-position_wrist_home[2]-sigma1;

         double beta2=sigma1;

         ROS_INFO("Alpha2 e beta2:%f , %f",alpha2,beta2);

         ROS_INFO("Start trajectory for generic position of the sphere");
         ros::Time now = ros::Time::now();
         //tf::Quaternion quaternion_wrist_final(tf::createQuaternionFromRPY(0, 0, -pi/2));
         double Roll_wrist_end,Pitch_wrist_end,Yaw_wrist_end;
         double distance_x=position_sphere[0]-position_wrist_home[0];
         double distance_y=position_sphere[1]-position_wrist_home[1]; 
         double theta;
         if(abs(distance_x) > abs(distance_y))
         {
           theta = atan2(distance_x,distance_y);
         }
         else
         {
           theta = -atan2(distance_x,distance_y); 
         } 
         //2*theta
         Roll_wrist_end=(double)(-pi/2+2*theta);
         //Roll_wrist_end=(double)(0);
         Pitch_wrist_end=(double)(-pi/2);
         Yaw_wrist_end=(double)(0);
         //Yaw_wrist_end=(double)(pi+2*theta);
         // Roll_wrist_end=(double)(-pi/2);
         // Pitch_wrist_end=(double)(-pi/2);
         // Yaw_wrist_end=(double)(-pi/2);

         Roll_angle_rot_step=(double)(Roll_wrist_end-roll_wrist)/(double)int_step;
         Pitch_angle_rot_step=(double)(Pitch_wrist_end-pitch_wrist)/(double)int_step;
         Yaw_angle_rot_step=(double)(Yaw_wrist_end-yaw_wrist)/(double)int_step;
        
        
         // sphere in any position
         double step_y=distance_y/int_step;
         for (int i = 0; i < int_traj; ++i)
         {
           
           
           if (i==0)
           {
             Roll_angle_traj[i]=(double)roll_wrist-(double)Krot*(double)Roll_angle_rot_step;
             Pitch_angle_traj[i]=(double)pitch_wrist-(double)Krot*(double)Pitch_angle_rot_step;
             Yaw_angle_traj[i]=(double)yaw_wrist-(double)Krot*(double)Yaw_angle_rot_step;
            }
            else
            {
             Roll_angle_traj[i]=(double)Roll_angle_traj[i-1]-(double)Krot*(double)Roll_angle_rot_step;
             Pitch_angle_traj[i]=(double)Pitch_angle_traj[i-1]-(double)Krot*(double)Pitch_angle_rot_step;
             Yaw_angle_traj[i]=(double)Yaw_angle_traj[i-1]-(double)Krot*(double)Yaw_angle_rot_step;
            }

            ROS_INFO("Roll_angle_traj, Pitch_angle_traj[i],Yaw_angle_traj[i]:%f,%f,%f" ,Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]);
            tf::Quaternion quaternion_wrist_now(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));
            orientation_traj[i]=quaternion_wrist_now.normalized();
           ROS_INFO("quaternion_wrist_now:(%f,%f,%f,%f)",orientation_traj[0][i],orientation_traj[1][i],orientation_traj[2][i],orientation_traj[3][i]);
           //ROS_INFO("quaternion_wrist_now:(%f,%f,%f,%f)",orientation_traj[0],orientation_traj[1],orientation_traj[2],orientation_traj[3]);
           if(position_wrist_home[1]>position_sphere[1])
           { 
             ROS_INFO("Wrist is on the right of sphere");
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]-Kp*step_y;
              }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]-Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_sphere[1],4)/alpha1+position_sphere[0];
             z_wrist_traj[i]=pow(x_wrist_traj[i]-position_sphere[0],4)/alpha2+pow(y_wrist_traj[i]-position_sphere[1],4)/beta2+position_sphere[2];
            }
          
            else
            { 
             
             ROS_INFO("Wrist is on the left of sphere");
             if(i==0) 
             {
               y_wrist_traj[i]=position_wrist_home[1]+Kp*step_y;
              }
             else 
             {
               y_wrist_traj[i]=y_wrist_traj[i-1]+Kp*step_y;
             }
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_sphere[1],4)/alpha1+position_sphere[0];
             z_wrist_traj[i]=pow(x_wrist_traj[i]-position_sphere[0],4)/alpha2+pow(y_wrist_traj[i]-position_sphere[1],4)/beta2+position_sphere[2];
            }
            ROS_INFO("x_wrist_traj,y_wrist_traj,z_wrist_traj:%f,%f,%f",x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]);
           traj[i].Traj.pose.position.x = x_wrist_traj[i];
           traj[i].Traj.pose.position.y = y_wrist_traj[i];
           traj[i].Traj.pose.position.z = z_wrist_traj[i];            
           traj[i].Traj.pose.orientation.x = orientation_traj[i][0];
           traj[i].Traj.pose.orientation.y =orientation_traj[i][1];
           traj[i].Traj.pose.orientation.z = orientation_traj[i][2];
           traj[i].Traj.pose.orientation.w = orientation_traj[i][3];
           traj[i].Traj.header.frame_id = "vito_anchor";
           
           std::string traj_num("traj");
           std::ostringstream out;
           out << i;
           std::string n_traj;
           n_traj = out.str();
           traj_name[i] = traj_num+ n_traj;
            
           trajector[i].setOrigin(tf::Vector3(x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]));
           //trajector[i].setRotation(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));
           trajector[i].setRotation(orientation_traj[i]); 
           broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "vito_anchor", traj_name[i]));
        
           ROS_INFO("Showed traj_number:%d",i);

          }         
        }
       else
       {
         ROS_INFO("Wrist is on the same z of the sphere");
         ros::Time now = ros::Time::now();
         //tf::Quaternion quaternion_wrist_final(tf::createQuaternionFromRPY(0, 0, -pi/2));
         double Roll_wrist_end,Pitch_wrist_end,Yaw_wrist_end;
         Roll_wrist_end=(double)0;
         Pitch_wrist_end=(double)0;
         Yaw_wrist_end=(double)0;
         Roll_angle_rot_step=(Roll_wrist_end-roll_wrist)/int_step;
         Pitch_angle_rot_step=(Pitch_wrist_end-pitch_wrist)/int_step;
         Yaw_angle_rot_step=(Yaw_wrist_end-yaw_wrist)/int_step;
        
        
         // same z but x and y different for wrist and sphere
         
         for (int i = 0; i < int_traj; ++i)
         {
           z_wrist_traj[i]=position_sphere[2];
           //y_wrist_traj[i]=position_sphere[1];
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
          
           if(position_wrist_home[1]>position_sphere[1])
           { 
             ROS_INFO("Wrist is on the right of sphere");
             double distance_y=position_sphere[1]-position_wrist_home[1];
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
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_sphere[1],4)/alpha1+position_sphere[0];
            }
          
            else
            { 
             ROS_INFO("Wrist is on the left of the sphere");
             double distance_y=position_sphere[1]-position_wrist_home[1];
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
             x_wrist_traj[i]=pow(y_wrist_traj[i]-position_sphere[1],4)/alpha1+position_sphere[0];
            }
          
           traj[i].Traj.pose.position.x = x_wrist_traj[i];
           traj[i].Traj.pose.position.y = y_wrist_traj[i];
           traj[i].Traj.pose.position.z = z_wrist_traj[i];            
           traj[i].Traj.pose.orientation.x = orientation_traj[i][0];
           traj[i].Traj.pose.orientation.y =orientation_traj[i][1];
           traj[i].Traj.pose.orientation.z = orientation_traj[i][2];
           traj[i].Traj.pose.orientation.w = orientation_traj[i][3];
           traj[i].Traj.header.frame_id = "vito_anchor";

           std::string traj_name;
           std::string traj_num("traj");
           std::ostringstream out;
           out << i;
           std::string n_traj;
           n_traj = out.str();
           traj_name = traj_num+ n_traj;
            
           trajector[i].setOrigin(tf::Vector3(x_wrist_traj[i],y_wrist_traj[i],z_wrist_traj[i]));
           trajector[i].setRotation(tf::createQuaternionFromRPY(Roll_angle_traj[i], Pitch_angle_traj[i],Yaw_angle_traj[i]));  
           broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "vito_anchor", traj_name));
          
        
           ROS_INFO("Showed traj_number:%d",i);
          } 
        }
      }
      else
      { 
       
       // same x and y for wrist and sphere !!!
       ROS_INFO("Wrist is on or down the sphere");
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
          z_wrist_traj[i]=position_sphere[0];
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
          

          if (position_wrist_home[2]>position_sphere[2])
           { 
            ROS_INFO("Wrist is on the sphere");
            double distance_z=position_wrist_home[2]-position_sphere[2];
            ROS_INFO("Calculated distance_z: %f",distance_z);
            double step_z=distance_z/int_step;
            double Kp = ((double)distance_z - (double)sphere.r -(double)threshold_distance/(double)4)/(double)distance_z;
            double Krot = (double)1;

            if(i==0) z_wrist_traj[i]=position_wrist_home[2]-Kp*step_z;
            else 
              {
                z_wrist_traj[i]=z_wrist_traj[i-1]-Kp*step_z;
              }
            
           }
          else
          { 
            ROS_INFO("Wrist is down the sphere");
            double distance_z=position_sphere[2]-position_wrist_home[2];
            ROS_INFO("Calculated distance_z: %f",distance_z);
            double Kp = ((double)distance_z - (double)sphere.r -(double)threshold_distance/(double)4)/(double)distance_z;
            double Krot = (double)1;
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
         traj[i].Traj.pose.orientation.x = orientation_traj[i][0];
         traj[i].Traj.pose.orientation.y =orientation_traj[i][1];
         traj[i].Traj.pose.orientation.z = orientation_traj[i][2];
         traj[i].Traj.pose.orientation.w = orientation_traj[i][3];
         traj[i].Traj.header.frame_id = "vito_anchor";
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
         broadcaster_.sendTransform(tf::StampedTransform(trajector[i], now, "vito_anchor", traj_name));
        
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

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose1;// = start_pose;
  
 for (int i = 0; i < int_traj; ++i)//int_traj
  {
   ROS_INFO("Target pose1.x,Target pose1.y, Target pose1.z: %f,%f ,%f",traj[i].Traj.pose.position.x,traj[0].Traj.pose.position.y,traj[0].Traj.pose.position.z);
   ROS_INFO("traj[i].Traj.pose.orientation.x,traj[i].Traj.pose.orientation.y,traj[i].Traj.pose.orientation.z,traj[i].Traj.pose.orientation.w: %f,%f ,%f,%f",traj[i].Traj.pose.orientation.x,traj[i].Traj.pose.orientation.y,traj[i].Traj.pose.orientation.z,traj[i].Traj.pose.orientation.w);
   target_pose1.position.x = traj[i].Traj.pose.position.x;
   target_pose1.position.y = traj[i].Traj.pose.position.y;
   target_pose1.position.z = traj[i].Traj.pose.position.z;
   target_pose1.orientation.x = traj[i].Traj.pose.orientation.x;
   target_pose1.orientation.y = traj[i].Traj.pose.orientation.y;
   target_pose1.orientation.z = traj[i].Traj.pose.orientation.z;
   target_pose1.orientation.w = traj[i].Traj.pose.orientation.w;
   waypoints.push_back(target_pose1);
  }
 

  // // We want the cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively
  // // disabling it.
  //group.setPlannerId("PRMstarkConfigDefault");
  //group.setPlannerId("RRTConnectkConfigDefault");
  double fraction((double)0);
  double eef_step;//((double)0.42591);
  eef_step=(double)2*step_y;
  double jump_threshold((double)0.000);
  double inc_treshold((double)0.1);
  double inc_eef_step((double)0.005);
  int int_jump_max(10);


  while((double)fraction*100<(double)95)
  {  
   for (int i = 0; i < int_jump_max; ++i)
   {
     jump_threshold=(inc_treshold*i);
     
     //double 
     fraction = group.computeCartesianPath(waypoints,
                                                eef_step,  // eef_step
                                                jump_threshold,   // jump_threshold
                                                trajectory);

     ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);    
     // /* Sleep to give Rviz time to visualize the plan. */
  
     //jump_threshold+=inc_treshold;
     //if((double)fraction*100>(double)90) break;
     //sleep(2.0);
  
   
     if((double)fraction>(double)0.95)
     {
     std::ofstream name_file_ptr;
     name_file_ptr.open("/home/pacman/Projects/LUCAGEMMA/file_txt/sphere_trajectory_demo.txt",std::ios::app);
      
     name_file_ptr << int_step <<" ";
     name_file_ptr << fraction <<" ";       
     name_file_ptr << eef_step <<" ";
     name_file_ptr << jump_threshold <<" ";

     name_file_ptr << "\n";
     name_file_ptr.close();  
     ROS_INFO("Data wrote into the file sphere_trajectory_demo.txt");
        

      ROS_DEBUG("Exit For::Find solution with eef_step and jump_threshold: %f %f",eef_step,jump_threshold);
     //   
     break;
     //  
    }
    //
  } 
    if((double)fraction*100>(double)95) 
    {
    ROS_DEBUG("EXit While2::Find solution with eef_step and jump_threshold: %f %f",eef_step,jump_threshold);
    response.result=response.SUCCESS;
    ROS_INFO("Starting sending response");
    //sleep(15.0);  
      for(int i =0;i<int_traj;i++)
      {
        response.traj[i] = traj[i];
      }
       response.moveit_trajectory= trajectory;
    ROS_INFO("TrajectoryDemo::Response Send");      
    break;
    }
    
    eef_step=eef_step-inc_eef_step;
   if(eef_step<(double)0.003)
    {
     ROS_ERROR("No solution for trajectory");
     response.result=response.OTHER_ERROR;
     break; 
    }
   }
  
    

   
    
 
    int_step+=(int)1;
    int_traj+=(double)1;

    }
   
  
  }


  
  
} 

















}//namespace grasp_planner













int main(int argc, char **argv) 
{
  ros::init(argc, argv, "grasp_planning_sphere_affordance_1_node");
  ros::NodeHandle nh;
 
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  grasp_planner::TrajectoryDemos node(nh);

  ros::spin();
  return 0;
};