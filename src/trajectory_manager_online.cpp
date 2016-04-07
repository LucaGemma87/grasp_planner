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
#include "grasp_planner/TrajectoryDemoOnline.h"
#include "grasp_planner/TrajectoryDemoCylinder.h"
#include "grasp_planner/TrajectoryDemoSphere.h"
#include "grasp_planner/TrajectoryDemoCone.h"
#include "grasp_planner/TrajectoryDemoPlane.h"

// Moveit headers

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// pacman interface
// #include <pacman/PaCMan/ROS.h>
// #include <pacman/PaCMan/Defs.h>



namespace grasp_planner {

class TrajectoryManager 
{
  

private:
  //! The node handle
  ros::NodeHandle nh_;
 
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher trajectory_manager_marker_pub_;

  
  //! Service server for object detection
  ros::ServiceServer trajectory_manager_srv_;

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


  bool serviceCallback(TrajectoryDemoOnline::Request &request, TrajectoryDemoOnline::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TrajectoryManager(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   trajectory_manager_marker_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("trajectory_manager_markers_out"), 10);

   trajectory_manager_srv_ = nh_.advertiseService(nh_.resolveName("trajectory_manager_online_srv"),    &TrajectoryManager::serviceCallback, this);

   
   
  
   
  }

  //! Empty stub
  ~TrajectoryManager() {}

};


/*! call serviceCallback
 */bool TrajectoryManager::serviceCallback(TrajectoryDemoOnline::Request &request, TrajectoryDemoOnline::Response &response)

{ 
  int count((int)0);
  int n_object;
  n_object=request.ob;
  int int_traj =(int)4;
  double height((double)0.50);
  Trajectory traj[int_traj];
  moveit_msgs::RobotTrajectory trajectory;
  int result((int)0);
  int iteration((int)2);
  int success((int)5);
  ros::NodeHandle nh;

  int affordance;

  ROS_INFO("Select the grasp affordances:");
  

  //std::cin >> affordance;

  ROS_INFO("Selected affordance %d",affordance); 
  /*! call serviceCallback for Cylinder                                                         */

  if(n_object==1)
  { 

    ROS_INFO("Starting TrajectoryManager");
    ROS_INFO("TrajectoryManager::Receiving cylinder");
    visual_perception::Cylinder cylinder;
      
     cylinder = request.cylinder;
       ROS_INFO ("TrajectoryDemosManager:Success in receving the cylinder transformation with 7 elements (Orient(x,y,z,w),Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",cylinder.pose.pose.orientation.x, cylinder.pose.pose.orientation.y, cylinder.pose.pose.orientation.z, cylinder.pose.pose.orientation.w,cylinder.pose.pose.position.x,  cylinder.pose.pose.position.y,  cylinder.pose.pose.position.z);
     
     ROS_INFO("Received cylinder"); 
     
     if(cylinder.pose.pose.position.z> height & cylinder.pose.pose.position.z< 0) 
     {
      response.result=response.BAD_OBJECT;
      ROS_INFO("Received bad cylinder");
      
     }
     
     if(cylinder.h>cylinder.r)
     {
      affordance=(int)1;
     }
     else
     {
      affordance=(int)2;
     }
     while (result!= success)
    {
     if(affordance==(int)1)
     { 

       ROS_INFO("Start servicecall for the cylinder grasp_planning_cylinder_affordance_1_srv");

       std::string service_name1("grasp_planning_cylinder_affordance_1_srv");
       while ( !ros::service::waitForService(service_name1, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name1.c_str());
        }
       if (!nh.ok()) exit(0);
       grasp_planner::TrajectoryDemoCylinder grasp_planning_cylinder_affordance_1_srv;
      
       grasp_planning_cylinder_affordance_1_srv.request.cylinder=cylinder;

       ROS_INFO("grasp_planning_cylinder_affordance_1_srv  waiting for service on topic grasp_planning_cylinder_affordance_1_srv");
       if (!ros::service::call(service_name1, grasp_planning_cylinder_affordance_1_srv))
       {
         ROS_ERROR("Call to grasp planning service failed");
         //exit(0);
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
           trajectory = grasp_planning_cylinder_affordance_1_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_cylinder_affordance_1_srv find the Trajectory to the cylinder");
           result=grasp_planning_cylinder_affordance_1_srv.response.result;
           break;
          }   
        }
        count=count+(int)1;
        if(count>(int)iteration) 
        { 
         //response.result = response.OTHER_ERROR;
         //break;
          affordance=(int)2;
        }
     }     //sleep(2);
     // if(affordance==(int)2)
     else
     {
        ROS_INFO("Start servicecall for the cylinder grasp_planning_cylinder_affordance_2_srv");

       std::string service_name2("grasp_planning_cylinder_affordance_2_srv");
       while ( !ros::service::waitForService(service_name2, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name2.c_str());
       }
       if (!nh.ok()) exit(0);
       grasp_planner::TrajectoryDemoCylinder grasp_planning_cylinder_affordance_2_srv;
      
       grasp_planning_cylinder_affordance_2_srv.request.cylinder=cylinder;

       ROS_INFO("grasp_planning_cylinder_affordance_2_srv  waiting for service on topic grasp_planning_cylinder_affordance_2_srv");
       if (!ros::service::call(service_name2, grasp_planning_cylinder_affordance_2_srv))
       {
         ROS_ERROR("Call to grasp planning service failed");
         //exit(0);
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
            ROS_INFO("Call to grasp_planning_cylinder_affordance_2_srv DONE !!!");
            grasp_planner::Trajectory traj[int_traj];
            for(int i =0;i<int_traj;i++)
            {
             traj[i] = grasp_planning_cylinder_affordance_2_srv.response.traj[i];
            }
            trajectory = grasp_planning_cylinder_affordance_2_srv.response.moveit_trajectory;        
            ROS_INFO("grasp_planning_cylinder_affordance_2_srv find the Trajectory to the cylinder");
            result=grasp_planning_cylinder_affordance_2_srv.response.result;
            break;
          }   
        } 
       count=count+(int)1;
       if(count>(int)iteration) 
       { 
         response.result = response.OTHER_ERROR;
         break;
       }
     }   
    }  
  }


  /*! call serviceCallback for Sphere                                                        */
  
  if(n_object==2)
  { 
     ROS_INFO("Starting TrajectoryManager");
     ROS_INFO("TrajectoryManager::Receiving sphere");
     visual_perception::Sphere sphere;
     
     sphere = request.sphere;
    
     ROS_INFO("Received sphere"); 

     if(sphere.pose.pose.position.z> height*2 & sphere.pose.pose.position.z< 0) 
     {
      response.result=response.BAD_OBJECT;
      ROS_INFO("Bad sphere");
      //break;
     }
     if(sphere.r>0.05)
     {
      affordance=(int)1;
     }
     else
     {
      affordance=(int)2;
     } 
    while (result!= success)
    {  
     
     if(affordance==1)
     {
       ROS_INFO("Start servicecall for the sphere grasp_planning_sphere_affordance_1_srv");

       std::string service_name3("grasp_planning_sphere_affordance_1_srv");
       while ( !ros::service::waitForService(service_name3, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name3.c_str());
        }
        if (!nh.ok()) exit(0);
        grasp_planner::TrajectoryDemoSphere grasp_planning_sphere_affordance_1_srv;
      
        grasp_planning_sphere_affordance_1_srv.request.sphere=sphere;

       ROS_INFO("grasp_planning_sphere_affordance_1_srv  waiting for service on topic grasp_planning_sphere_affordance_1_srv");
       if (!ros::service::call(service_name3, grasp_planning_sphere_affordance_1_srv))
       {
        ROS_ERROR("Call to grasp planning service failed");
        //exit(0);
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
           trajectory = grasp_planning_sphere_affordance_1_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_sphere_affordance_1_srv find the Trajectory to the sphere");
           result=grasp_planning_sphere_affordance_1_srv.response.result;
           break;
          }   
        }

       count=count+(int)1;
       if(count>(int)iteration) 
       { 
         response.result = response.OTHER_ERROR;
         break;
        }
      }
      
     //sleep(2);
     if(affordance==2)
     { 
       ROS_INFO("Start servicecall for the sphere grasp_planning_sphere_affordance_2_srv");

       std::string service_name4("grasp_planning_sphere_affordance_2_srv");
       while ( !ros::service::waitForService(service_name4, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name4.c_str());
       }
       if (!nh.ok()) exit(0);
       grasp_planner::TrajectoryDemoSphere grasp_planning_sphere_affordance_2_srv;
      
       grasp_planning_sphere_affordance_2_srv.request.sphere=sphere;

       ROS_INFO("grasp_planning_sphere_affordance_2_srv  waiting for service on topic grasp_planning_sphere_affordance_2_srv");
       if (!ros::service::call(service_name4, grasp_planning_sphere_affordance_2_srv))
       {
         ROS_ERROR("Call to grasp planning service failed");
         //exit(0);
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
           ROS_INFO("Call to grasp_planning_sphere_affordance_2_srv DONE !!!");
           grasp_planner::Trajectory traj[int_traj];
          for(int i =0;i<int_traj;i++)
           {
            traj[i] = grasp_planning_sphere_affordance_2_srv.response.traj[i];
           }
           trajectory = grasp_planning_sphere_affordance_2_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_sphere_affordance_2_srv find the Trajectory to the sphere");
           result=grasp_planning_sphere_affordance_2_srv.response.result;
           break;
          }   
         count=count+(int)1;
         if(count>(int)iteration)
         { 
           response.result = response.OTHER_ERROR;
           break;
          }  
        }
      }  
    }  
  } 

 /*! call serviceCallback for Cone                                                            */
  
  // if(n_object==3)
  // { 
  //   while (result!= success)
  //   {  
  //    ROS_INFO("Starting TrajectoryManager");
  //    ROS_INFO("TrajectoryManager::Receiving cone");
  //    visual_perception::Cone cone;
     
  //    cone = request.cone;

  //    ROS_INFO("Received cone"); 
      
  //    if(cone.pose.pose.position.z> height & cone.pose.pose.position.z< 0) 
  //    {
  //      response.result=response.BAD_OBJECT;
  //      ROS_INFO("Received cylinder");
  //      break;
  //    }
     
  //     if(affordance==1)
  //    {
  //    ROS_INFO("Start servicecall for the sphere grasp_planning_cone_affordance_1_srv");

  //    std::string service_name5("grasp_planning_cone_affordance_1_srv");
  //    while ( !ros::service::waitForService(service_name5, ros::Duration().fromSec(3.0)) && nh.ok() )
  //    {
  //      ROS_INFO("Waiting for service %s...", service_name5.c_str());
  //    }
  //     if (!nh.ok()) exit(0);
  //     grasp_planner::TrajectoryDemoCone grasp_planning_cone_affordance_1_srv;
      
  //     grasp_planning_cone_affordance_1_srv.request.cone=cone;

  //    ROS_INFO("grasp_planning_cone_affordance_1_srv  waiting for service on topic grasp_planning_cone_affordance_1_srv");
  //    if (!ros::service::call(service_name5, grasp_planning_cone_affordance_1_srv))
  //    {
  //      ROS_ERROR("Call to grasp planning service failed");
  //      //exit(0);
  //     }
  //    else 
  //    {    
  //      if (grasp_planning_cone_affordance_1_srv.response.result != grasp_planning_cone_affordance_1_srv.response.SUCCESS)
  //      {
  //        ROS_ERROR("grasp planning service returned error %d", grasp_planning_cone_affordance_1_srv.response.result);
  //        //exit(0);
  //      }
  //      else
  //      {  
  //        ROS_INFO("Call to grasp_planning_cone_affordance_1_srv DONE !!!");
  //        grasp_planner::Trajectory traj[int_traj];
  //        for(int i =0;i<int_traj;i++)
  //        {
  //          traj[i] = grasp_planning_cone_affordance_1_srv.response.traj[i];
  //         }
  //        trajectory = grasp_planning_cone_affordance_1_srv.response.moveit_trajectory;        
  //        ROS_INFO("grasp_planning_cone_affordance_1_srv find the Trajectory to the cone");
  //        result=grasp_planning_cone_affordance_1_srv.response.result;
  //        break;
  //       }   
  //     }
  //    } 
  //    //sleep(2);
  //     if(affordance==2)
  //    {
  //    ROS_INFO("Start servicecall for the cone grasp_planning_sphere_affordance_2_srv");

  //    std::string service_name6("grasp_planning_cone_affordance_2_srv");
  //    while ( !ros::service::waitForService(service_name6, ros::Duration().fromSec(3.0)) && nh.ok() )
  //    {
  //      ROS_INFO("Waiting for service %s...", service_name6.c_str());
  //    }
  //     if (!nh.ok()) exit(0);
  //     grasp_planner::TrajectoryDemoCone grasp_planning_cone_affordance_2_srv;
      
  //     grasp_planning_cone_affordance_2_srv.request.cone=cone;

  //    ROS_INFO("grasp_planning_cone_affordance_2_srv  waiting for service on topic grasp_planning_cone_affordance_2_srv");
  //    if (!ros::service::call(service_name6, grasp_planning_cone_affordance_2_srv))
  //    {
  //      ROS_ERROR("Call to grasp planning service failed");
  //      //exit(0);
  //     }
  //    else 
  //    {    
  //      if (grasp_planning_cone_affordance_2_srv.response.result != grasp_planning_cone_affordance_2_srv.response.SUCCESS)
  //      {
  //        ROS_ERROR("grasp planning service returned error %d", grasp_planning_cone_affordance_2_srv.response.result);
  //        //exit(0);
  //      }
  //      else
  //      {  
  //        ROS_INFO("Call to grasp_planning_cone_affordance_2_srv DONE !!!");
  //        grasp_planner::Trajectory traj[int_traj];
  //        for(int i =0;i<int_traj;i++)
  //        {
  //          traj[i] = grasp_planning_cone_affordance_2_srv.response.traj[i];
  //         }
  //        trajectory = grasp_planning_cone_affordance_2_srv.response.moveit_trajectory;        
  //        ROS_INFO("grasp_planning_cone_affordance_2_srv find the Trajectory to the cone");
  //        result=grasp_planning_cone_affordance_2_srv.response.result;
  //        break;
  //       }   
  //         count=count+(int)1;
  //    if(count>(int)iteration) 
  //      { 
  //     response.result = response.OTHER_ERROR;
  //     break;
  //      } 
  //     }
  
  //   } 
  // }
       
  // }



  /*! call serviceCallback for Box                                                               */
  
  if(n_object==4)
  { 

       ROS_INFO("Starting TrajectoryManager");
      ROS_INFO("TrajectoryManager::Receiving Box");
      visual_perception::Plane plane;
     
    plane = request.plane;
    
    ROS_INFO("Received plane"); 

    if(plane.pose.pose.position.z> height & plane.pose.pose.position.z< 0) 
    {
      response.result=response.BAD_OBJECT;
       ROS_INFO(" No Received plane");
       //break;
    }

    //  int affordance;

    // ROS_INFO("Select the grasp affordances:");
    // ROS_INFO("Press 1 for first affordance (lateral-object grasp)");
    // ROS_INFO("Press 2 for second affordance (up-object grasp)");

    // std::cin >> affordance;

    // ROS_INFO("Selected affordance %d",affordance); 

    if(plane.height>plane.width)
     {
      affordance=(int)1;
     }
     else
     {
      affordance=(int)2;
     }

        while (result!= success)
    { 
      if(affordance==1)
      {
        ROS_INFO("Starting TrajectoryManager");
        ROS_INFO("TrajectoryManager::Receiving Box");
        //visual_perception::Plane plane;
     
        //plane = request.plane;
    
        ROS_INFO("Received plane"); 

        ROS_INFO("Start servicecall for the box grasp_planning_box_affordance_1_srv");

        std::string service_name7("grasp_planning_box_affordance_1_srv");
        while ( !ros::service::waitForService(service_name7, ros::Duration().fromSec(3.0)) && nh.ok() )
       {
         ROS_INFO("Waiting for service %s...", service_name7.c_str());
       }
        if (!nh.ok()) exit(0);
        grasp_planner::TrajectoryDemoPlane grasp_planning_box_affordance_1_srv;
      
        grasp_planning_box_affordance_1_srv.request.plane=plane;

        ROS_INFO("grasp_planning_box_affordance_1_srv  waiting for service on topic grasp_planning_box_affordance_1_srv");
        if (!ros::service::call(service_name7, grasp_planning_box_affordance_1_srv))
       {
         ROS_ERROR("Call to grasp planning service failed");
        //exit(0);
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
           trajectory = grasp_planning_box_affordance_1_srv.response.moveit_trajectory;        
           ROS_INFO("grasp_planning_box_affordance_1_srv find the Trajectory to the box");
           result=grasp_planning_box_affordance_1_srv.response.result;
           break;
          }

          count=count+(int)1;
          if(count>(int)iteration)
         { 
           response.result = response.OTHER_ERROR;
           break;
          }     
        }
      }
      

      if(affordance==2)
      {  
       ROS_INFO("Start servicecall for the box grasp_planning_box_affordance_2_srv");

     std::string service_name8("grasp_planning_box_affordance_2_srv");
     while ( !ros::service::waitForService(service_name8, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name8.c_str());
     }
      if (!nh.ok()) exit(0);
      grasp_planner::TrajectoryDemoPlane grasp_planning_box_affordance_2_srv;
      
      grasp_planning_box_affordance_2_srv.request.plane=plane;

     ROS_INFO("grasp_planning_box_affordance_2_srv  waiting for service on topic grasp_planning_box_affordance_2_srv");
     if (!ros::service::call(service_name8, grasp_planning_box_affordance_2_srv))
     {
       ROS_ERROR("Call to grasp planning service failed");
       //exit(0);
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
         trajectory = grasp_planning_box_affordance_2_srv.response.moveit_trajectory;        
         ROS_INFO("grasp_planning_box_affordance_2_srv find the Trajectory to the box");
         result=grasp_planning_box_affordance_2_srv.response.result;
         break;
        }  

        count=count+(int)1;
         if(count>(int)iteration)
         { 
           response.result = response.OTHER_ERROR;
           break;
          }   
      }
    }  
   }

  } 


  
  //moveit::planning_interface::MoveGroup::Plan plan; 
  moveit::planning_interface::MoveGroup group("left_arm");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //moveit::robot_trajectory rt1; 

   //group.getCurrentState()->getRobotModel()

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(),"left_arm");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(),trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp((int)1000,(double)20.0);
  //rt.IterativeParabolicTimeParameterization((int)100,(double)0.001);
  double success2;
  success2 =iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  moveit_msgs::RobotTrajectory parametr_trajectory;
  rt.getRobotTrajectoryMsg(parametr_trajectory);
  std::cout << "trajectory input" << trajectory << std::endl;
  std::cout << "trajectory output" << parametr_trajectory << std::endl;
 
  //
  //std::cout << "iptp " << iptp << std::endl;
  //std::cout << "rt " << rt << std::endl;
  //plan.trajectory_=trajectory
  //

  // scrivere funzione che scrive position in points.txt con tabulazione dei dati
  // per farlo fare al robot:


  std::ofstream name_file_ptr;
  //name_file_ptr.open ("/home/pacman/Downloads/KukaAffordances/input_files/points.txt");
  name_file_ptr.open ("/home/pacman/Downloads/KukaAffordances/build/points.txt");
  
  //name_file_ptr2 << parametr_trajectory <<" ";
  for (int i=0; i<parametr_trajectory.joint_trajectory.points.size(); ++i) 
  { 
   for (int j=0; j<parametr_trajectory.joint_trajectory.points[0].positions.size(); ++j)          
   { 
   name_file_ptr << parametr_trajectory.joint_trajectory.points[i].positions[j] <<"\t";

   }
      name_file_ptr << "\n" ;
  }  

                   
  name_file_ptr << "\n";
  name_file_ptr.close();   

 ROS_INFO("Saved position of trajectory !!!");



  ROS_INFO("Starting sending response");
  //sleep(15.0);  
  for(int i =0;i<int_traj;i++)
  {
    response.traj[i] = traj[i];
  }
  response.moveit_trajectory= trajectory;
  response.result=response.SUCCESS;
  ROS_INFO("TrajectoryDemo::Response Send"); 
}
}//namespace grasp_planner

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "trajectory_manager_node");
  ros::NodeHandle nh;
 
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  grasp_planner::TrajectoryManager node(nh);

  ros::spin();
  return 0;
};  