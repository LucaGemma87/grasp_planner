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


// visual perception headers


#include "visual_perception/ObjectManaging.h"
#include "visual_perception/CylinderFitting.h"
#include "visual_perception/Cylinder.h"
#include "visual_perception/SphereFitting.h"
#include "visual_perception/ConeFitting.h"
#include "visual_perception/ParallelepipedFitting.h"
#include "visual_perception/Sphere.h"
#include "visual_perception/Cone.h"
// grasp planner headers
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

class TrajectoryManagerDemo 
{
  

private:
  //! The node handle
  ros::NodeHandle nh_;
 
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher trajectory_manager_demo_marker_pub_;

  
  //! Service server for object detection
  ros::ServiceServer trajectory_manager_demo_srv_;

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
  TrajectoryManagerDemo(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   trajectory_manager_demo_marker_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("trajectory_manager_demo_markers_out"), 10);

   trajectory_manager_demo_srv_ = nh_.advertiseService(nh_.resolveName("trajectory_manager_demo_srv"),    &TrajectoryManagerDemo::serviceCallback, this);

   
   
  
   
  }

  //! Empty stub
  ~TrajectoryManagerDemo() {}

};


/*! call serviceCallback
 */bool TrajectoryManagerDemo::serviceCallback(TrajectoryDemo::Request &request, TrajectoryDemo::Response &response)

{ 
  int n_object;
  n_object=request.ob;
  int int_traj =(int)4;
  Trajectory traj[int_traj];
  ros::NodeHandle nh;
  moveit_msgs::RobotTrajectory trajectory_affordance_1_cylinder;
  moveit_msgs::RobotTrajectory trajectory_affordance_1_sphere;
  moveit_msgs::RobotTrajectory trajectory_affordance_1_cone;
  moveit_msgs::RobotTrajectory trajectory_affordance_1_box;
  moveit_msgs::RobotTrajectory trajectory_affordance_2_cylinder;
  moveit_msgs::RobotTrajectory trajectory_affordance_2_sphere;
  moveit_msgs::RobotTrajectory trajectory_affordance_2_cone;
  moveit_msgs::RobotTrajectory trajectory_affordance_2_box;
  if(n_object==0)
  {
    ROS_INFO("Starting TrajectoryManagerDemo");
     

    std::string service_name1("grasp_planning_cylinder_affordance_1_srv");
    while ( !ros::service::waitForService(service_name1, ros::Duration().fromSec(3.0)) && nh.ok() )
    {
      ROS_INFO("Waiting for service %s...", service_name1.c_str());
    }
    if (!nh.ok()) exit(0);
    grasp_planner::TrajectoryDemo grasp_planning_cylinder_affordance_1_srv;
      
    grasp_planning_cylinder_affordance_1_srv.request.ob=(int)1;  

   ROS_INFO("grasp_planning_cylinder_affordance_1_srv  waiting for service on topic grasp_planning_cylinder_affordance_1_srv");
   if (!ros::service::call(service_name1, grasp_planning_cylinder_affordance_1_srv))
   {
      ROS_ERROR("Call to grasp planning service failed");
      exit(0);
    }
    else 
    {    
      if (grasp_planning_cylinder_affordance_1_srv.response.result != grasp_planning_cylinder_affordance_1_srv.response.SUCCESS)
      {
        ROS_ERROR("grasp planning service returned error %d", grasp_planning_cylinder_affordance_1_srv.response.result);
        //exit(0);
      }
      else
      {  
       ROS_INFO("Call to grasp_planning_cylinder_affordance_1_srv DONE !!!");
       grasp_planner::Trajectory traj[int_traj];
       for(int i =0;i<int_traj;i++)
        {
         traj[i] = grasp_planning_cylinder_affordance_1_srv.response.traj[i];
        }
        trajectory_affordance_1_cylinder=grasp_planning_cylinder_affordance_1_srv.response.moveit_trajectory;           
        ROS_INFO("grasp_planning_cylinder_affordance_1_srv find the Trajectory to the cylinder");
      }
    }
  

   sleep(1);


    std::string service_name2("grasp_planning_cylinder_affordance_2_srv");
    while ( !ros::service::waitForService(service_name2, ros::Duration().fromSec(3.0)) && nh.ok() )
    {
      ROS_INFO("Waiting for service %s...", service_name2.c_str());
    }
    if (!nh.ok()) exit(0);
    grasp_planner::TrajectoryDemo grasp_planning_cylinder_affordance_2_srv;
      
    grasp_planning_cylinder_affordance_2_srv.request.ob=(int)1;  

   ROS_INFO("grasp_planning_cylinder_affordance_2_srv  waiting for service on topic grasp_planning_cylinder_affordance_2_srv");
   if (!ros::service::call(service_name2, grasp_planning_cylinder_affordance_2_srv))
   {
      ROS_ERROR("Call to grasp planning service failed");
      exit(0);
    }
    else 
    {    
      if (grasp_planning_cylinder_affordance_2_srv.response.result != grasp_planning_cylinder_affordance_2_srv.response.SUCCESS)
      {
        ROS_ERROR("grasp planning service returned error %d", grasp_planning_cylinder_affordance_2_srv.response.result);
        //exit(0);
      }
      else
      {  
       ROS_INFO("Call to grasp_planning_cylinder_affordance_1_srv DONE !!!");
       grasp_planner::Trajectory traj[int_traj];
       for(int i =0;i<int_traj;i++)
       {
         traj[i] = grasp_planning_cylinder_affordance_2_srv.response.traj[i];
       }
       trajectory_affordance_2_cylinder=grasp_planning_cylinder_affordance_2_srv.response.moveit_trajectory;         
       ROS_INFO("grasp_planning_cylinder_affordance_2_srv find the Trajectory to the cylinder");
      }
    }
  

    sleep(1);

     std::string service_name3("grasp_planning_sphere_affordance_1_srv");
     while ( !ros::service::waitForService(service_name3, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name3.c_str());
      }
     if (!nh.ok()) exit(0);
     grasp_planner::TrajectoryDemo grasp_planning_sphere_affordance_1_srv;
      
     grasp_planning_sphere_affordance_1_srv.request.ob=(int)2;  

     ROS_INFO("grasp_planning_sphere_affordance_1_srv  waiting for service on topic grasp_plannin_sphere_affordance_1_srv");
     if (!ros::service::call(service_name3, grasp_planning_sphere_affordance_1_srv))
     {
       ROS_ERROR("Call to grasp planning service failed");
       exit(0);
      }
     else 
     {    
       if (grasp_planning_sphere_affordance_1_srv.response.result != grasp_planning_sphere_affordance_1_srv.response.SUCCESS)
       {
         ROS_ERROR("grasp planning service returned error %d", grasp_planning_sphere_affordance_1_srv.response.result);
         //exit(0);
        }
       else
       { 
         ROS_INFO("Call to grasp_planning_sphere_affordance_1_srv DONE !!!");
         grasp_planner::Trajectory traj[int_traj];
         for(int i =0;i<int_traj;i++)
         {
          traj[i] = grasp_planning_sphere_affordance_1_srv.response.traj[i];
         }
         trajectory_affordance_1_sphere=grasp_planning_sphere_affordance_1_srv.response.moveit_trajectory;           
         ROS_INFO("grasp_planning_sphere_affordance_1_srv find the Trajectory to the sphere");
        }
      }
  

     sleep(1);


      std::string service_name4("grasp_planning_sphere_affordance_2_srv");
      while ( !ros::service::waitForService(service_name4, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name4.c_str());
      }
     if (!nh.ok()) exit(0);
     grasp_planner::TrajectoryDemo grasp_planning_sphere_affordance_2_srv;
      
     grasp_planning_sphere_affordance_2_srv.request.ob=(int)2;  

     ROS_INFO("grasp_planning_sphere_affordance_2_srv  waiting for service on topic grasp_planning_sphere_affordance_2_srv");
     if (!ros::service::call(service_name4, grasp_planning_sphere_affordance_2_srv))
     {
       ROS_ERROR("Call to grasp planning service failed");
       exit(0);
      }
     else 
     {    
       if (grasp_planning_sphere_affordance_2_srv.response.result != grasp_planning_sphere_affordance_2_srv.response.SUCCESS)
       {
         ROS_ERROR("grasp planning service returned error %d", grasp_planning_sphere_affordance_2_srv.response.result);
         //exit(0);
        }
        else
        {  
         ROS_INFO("Call to grasp_planning_sphere_affordance_1_srv DONE !!!");
         grasp_planner::Trajectory traj[int_traj];
         for(int i =0;i<int_traj;i++)
          {
            traj[i] = grasp_planning_sphere_affordance_2_srv.response.traj[i];
          }  
         trajectory_affordance_2_sphere=grasp_planning_sphere_affordance_2_srv.response.moveit_trajectory;        
         ROS_INFO("grasp_planning_sphere_affordance_2_srv find the Trajectory to the sphere");
        }
      }  
  

     sleep(1);


      std::string service_name5("grasp_planning_cone_affordance_1_srv");
      while ( !ros::service::waitForService(service_name5, ros::Duration().fromSec(3.0)) && nh.ok() )
      {
       ROS_INFO("Waiting for service %s...", service_name5.c_str());
      } 
      if (!nh.ok()) exit(0);
      grasp_planner::TrajectoryDemo grasp_planning_cone_affordance_1_srv;
      
      grasp_planning_cone_affordance_1_srv.request.ob=(int)3;  

      ROS_INFO("grasp_planning_cone_affordance_1_srv  waiting for service on topic grasp_planning_cone_affordance_1_srv");
      if (!ros::service::call(service_name5, grasp_planning_cone_affordance_1_srv))
      {
        ROS_ERROR("Call to grasp planning service failed");
        exit(0);
      }
      else 
      {    
        if (grasp_planning_cone_affordance_1_srv.response.result != grasp_planning_cone_affordance_1_srv.response.SUCCESS)
        {
          ROS_ERROR("grasp planning service returned error %d", grasp_planning_cone_affordance_1_srv.response.result);
          //exit(0);
        }
       else
       {
         ROS_INFO("Call to grasp_planning_cone_affordance_1_srv DONE !!!");
         grasp_planner::Trajectory traj[int_traj];
         for(int i =0;i<int_traj;i++)
         {
           traj[i] = grasp_planning_cone_affordance_1_srv.response.traj[i];
          }
          trajectory_affordance_1_cone=grasp_planning_cone_affordance_1_srv.response.moveit_trajectory;          
         ROS_INFO("grasp_planning_cone_affordance_1_srv find the Trajectory to the cone");
        }
      }  
  

     sleep(1);


     std::string service_name6("grasp_planning_cone_affordance_2_srv");
     while ( !ros::service::waitForService(service_name6, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name6.c_str());
     }
     if (!nh.ok()) exit(0);
     grasp_planner::TrajectoryDemo grasp_planning_cone_affordance_2_srv;
      
     grasp_planning_cone_affordance_2_srv.request.ob=(int)3;  

     ROS_INFO("grasp_planning_cone_affordance_2_srv  waiting for service on topic grasp_planning_cone_affordance_2_srv");
     if (!ros::service::call(service_name2, grasp_planning_cone_affordance_2_srv))
     {
       ROS_ERROR("Call to grasp planning service failed");
       //exit(0);
      }
     else 
     {    
       if (grasp_planning_cone_affordance_2_srv.response.result != grasp_planning_cone_affordance_2_srv.response.SUCCESS)
       {
         ROS_ERROR("grasp planning service returned error %d", grasp_planning_cone_affordance_2_srv.response.result);
         //exit(0);
        }
        else
        { 
         ROS_INFO("Call to grasp_planning_cone_affordance_1_srv DONE !!!");
         grasp_planner::Trajectory traj[int_traj];
         for(int i =0;i<int_traj;i++)
        {
         traj[i] = grasp_planning_cone_affordance_2_srv.response.traj[i];
        } 
         trajectory_affordance_2_cone=grasp_planning_cone_affordance_2_srv.response.moveit_trajectory;         
         ROS_INFO("grasp_planning_cone_affordance_2_srv find the Trajectory to the cylinder");
        }
      }  
  

      sleep(1);

      std::string service_name7("grasp_planning_box_affordance_1_srv");
      while ( !ros::service::waitForService(service_name7, ros::Duration().fromSec(3.0)) && nh.ok() )
      {
        ROS_INFO("Waiting for service %s...", service_name7.c_str());
      }
      if (!nh.ok()) exit(0);
      grasp_planner::TrajectoryDemo grasp_planning_box_affordance_1_srv;
      
      grasp_planning_box_affordance_1_srv.request.ob=(int)4;  

      ROS_INFO("grasp_planning_box_affordance_1_srv  waiting for service on topic grasp_planning_box_affordance_1_srv");
      if (!ros::service::call(service_name7, grasp_planning_box_affordance_1_srv))
      {
       ROS_ERROR("Call to grasp planning service failed");
       exit(0);
      }
      else 
      {    
       if (grasp_planning_box_affordance_1_srv.response.result != grasp_planning_box_affordance_1_srv.response.SUCCESS)
       {
         ROS_ERROR("grasp planning service returned error %d", grasp_planning_box_affordance_1_srv.response.result);
         //exit(0);
        } 
        else
         { 
           ROS_INFO("Call to grasp_planning_box_affordance_1_srv DONE !!!");
           grasp_planner::Trajectory traj[int_traj];
           for(int i =0;i<int_traj;i++)
           {
             traj[i] = grasp_planning_box_affordance_1_srv.response.traj[i];
            }  
           trajectory_affordance_1_box=grasp_planning_box_affordance_1_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_box_affordance_1_srv find the Trajectory to the sphere");
          }
        } 

       sleep(1);


       std::string service_name8("grasp_planning_box_affordance_2_srv");
       while ( !ros::service::waitForService(service_name8, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name8.c_str());
        }
       if (!nh.ok()) exit(0);
       grasp_planner::TrajectoryDemo grasp_planning_box_affordance_2_srv;
      
       grasp_planning_box_affordance_2_srv.request.ob=(int)4;  

       ROS_INFO("grasp_planning_box_affordance_2_srv  waiting for service on topic grasp_planning_box_affordance_2_srv");
       if (!ros::service::call(service_name8, grasp_planning_box_affordance_2_srv))
       {
         ROS_ERROR("Call to grasp planning service failed");
         exit(0);
        }
        else 
        {    
          if (grasp_planning_box_affordance_2_srv.response.result != grasp_planning_box_affordance_2_srv.response.SUCCESS)
          {
            ROS_ERROR("grasp planning service returned error %d", grasp_planning_box_affordance_2_srv.response.result);
            //exit(0);
          } 
         else
         { 
           ROS_INFO("Call to grasp_planning_box_affordance_2_srv DONE !!!");
           grasp_planner::Trajectory traj[int_traj];
           for(int i =0;i<int_traj;i++)
           {
             traj[i] = grasp_planning_box_affordance_2_srv.response.traj[i];
            }  
           trajectory_affordance_2_box=grasp_planning_box_affordance_2_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_box_affordance_2_srv find the Trajectory to the sphere");
          }
        } 
      }
     ROS_INFO("Starting sending response");
     //sleep(15.0);  
     for(int i =0;i<int_traj;i++)
     {
       response.traj[i] = traj[i];
     }
     ROS_INFO("TrajectoryDemo::Response Send"); 
    }
}//namespace grasp_planner

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "trajectory_manager_demo_node");
  ros::NodeHandle nh;
 
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  grasp_planner::TrajectoryManagerDemo node(nh);

  ros::spin();
  return 0;
};  