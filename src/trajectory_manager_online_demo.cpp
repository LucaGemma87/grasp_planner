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
  //! The current marker being publishedvisual_perception::Plane plane;
    // cone = request.cone;
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
  TrajectoryManager(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   trajectory_manager_marker_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("trajectory_manager_markers_out"), 10);

   trajectory_manager_srv_ = nh_.advertiseService(nh_.resolveName("trajectory_manager_online_demo_srv"),    &TrajectoryManager::serviceCallback, this);

   
   
  
   
  }

  //! Empty stub
  ~TrajectoryManager() {}

};


/*! call serviceCallback
 */bool TrajectoryManager::serviceCallback(TrajectoryDemo::Request &request, TrajectoryDemo::Response &response)

{ 
  





  int count((int)0);
  int iteration((int)2);
 
  int n_object;
  n_object=request.ob;
  int int_traj =(int)10;
  Trajectory traj[int_traj];
  moveit_msgs::RobotTrajectory trajectory;
  int result((int)0);
  int success((int)5);
  ros::NodeHandle nh;
  
  if(n_object==1)
  {
  /*! call serviceCallback for Cylinder                                                         */
     
    visual_perception::Cylinder cylinder;
    tf::Transform rot_cyl;
    double min_x(-0.7);
    double max_x(-0.5);
    double min_y(-0.25);
    double max_y(-0.15);

    srand(time(NULL));
    double X=(max_x-min_x)*((double)rand()/(double)RAND_MAX)+min_x;
    srand(time(NULL));
    double Y=(max_y-min_y)*((double)rand()/(double)RAND_MAX)+min_y;
    tf::Vector3 position_cyl(X,Y,0.155);
    //tf::Vector3 position_cyl(-0.611268,-0.208724,0.155);
    //tf::Vector3 position_cyl(0.000000, -1.129196,0.077);
    tf::Quaternion orientation_cyl(tf::createQuaternionFromRPY(0, 0, 0));
    ROS_INFO("Orientation cyl(%f,%f,%f,%f)",orientation_cyl[0],orientation_cyl[1],orientation_cyl[2],orientation_cyl[3]);
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
  //if(n_object==1)


  //{ 
 
    int affordance;

    ROS_INFO("Select the grasp affordances:");
    ROS_INFO("Press 1 for first affordance (lateral-object grasp)");
    ROS_INFO("Press 2 for second affordance (up-object grasp)");

    std::cin >> affordance;

    ROS_INFO("Selected affordance %d",affordance); 



    while (result!= success)
    {
     if(affordance==1)
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
         response.result = response.OTHER_ERROR;
         break;
        }
      }     //sleep(2);
      if(affordance==2)
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
    visual_perception::Sphere sphere;
    //  sphere= request.sphere;
    tf::Transform rot_sph;
    double min_x1(-0.7);
    double max_x1(-0.5);
    double min_y1(-0.25);
    double max_y1(-0.15);

    srand(time(NULL));
    double X1=(max_x1-min_x1)*((double)rand()/(double)RAND_MAX)+min_x1;
    srand(time(NULL));
    double Y1=(max_y1-min_y1)*((double)rand()/(double)RAND_MAX)+min_y1;
    tf::Vector3 position_sph(X1,Y1,0.125);
    //tf::Vector3 position_cyl(-0.611268,-0.208724,0.155);
    //tf::Vector3 position_cyl(0.000000, -1.129196,0.077);
    tf::Quaternion orientation_sph(tf::createQuaternionFromRPY(0, 0, 0));
    ROS_INFO("Orientation sph(%f,%f,%f,%f)",orientation_sph[0],orientation_sph[1],orientation_sph[2],orientation_sph[3]);
    rot_sph.setRotation(orientation_sph);    
    rot_sph.setOrigin(position_sph);

    

    sphere.pose.pose.orientation.x=orientation_sph[0];
    sphere.pose.pose.orientation.y=orientation_sph[1];
    sphere.pose.pose.orientation.z=orientation_sph[2];
    sphere.pose.pose.orientation.w=orientation_sph[3];

    sphere.pose.pose.position.x=position_sph[0]; 
    sphere.pose.pose.position.y=position_sph[1];
    sphere.pose.pose.position.z=position_sph[2];



    sphere.r=(double)0.125;

    ROS_INFO("Created pose for sphere"); 

    int affordance;

    ROS_INFO("Select the grasp affordances:");
    ROS_INFO("Press 1 for first affordance (lateral-object grasp)");
    ROS_INFO("Press 2 for second affordance (up-object grasp)");

    std::cin >> affordance;

    ROS_INFO("Selected affordance %d",affordance); 


  
  
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
    

  //   if(n_object==3)
  // { 

  //   visual_perception::Cone cone;
  //   // cone = request.cone;
  //   tf::Transform rot_con;
  //   double min_x2(-0.75);
  //   double max_x2(-0.55);
  //   double min_y2(-0.25);
  //   double max_y2(0);

  //   srand(time(NULL));
  //   double X2=(max_x2-min_x2)*((double)rand()/(double)RAND_MAX)+min_x2;
  //   srand(time(NULL));
  //   double Y2=(max_y2-min_y2)*((double)rand()/(double)RAND_MAX)+min_y2;
  //   tf::Vector3 position_con(X2,Y2,0.125);
  //   //tf::Vector3 position_cyl(-0.611268,-0.208724,0.155);
  //   //tf::Vector3 position_cyl(0.000000, -1.129196,0.077);
  //   tf::Quaternion orientation_con(tf::createQuaternionFromRPY(0, 0, 0));
  //   ROS_INFO("Orientation con(%f,%f,%f,%f)",orientation_con[0],orientation_con[1],orientation_con[2],orientation_con[3]);
  //   rot_con.setRotation(orientation_con);    
  //   rot_con.setOrigin(position_con);

    

  //   cone.pose.pose.orientation.x=orientation_con[0];
  //   cone.pose.pose.orientation.y=orientation_con[1];
  //   cone.pose.pose.orientation.z=orientation_con[2];
  //   cone.pose.pose.orientation.w=orientation_con[3];

  //   cone.pose.pose.position.x=position_con[0]; 
  //   cone.pose.pose.position.y=position_con[1];
  //   cone.pose.pose.position.z=position_con[2];


  //   cone.angle=(double)0.2;
  //   cone.h=(double)0.25;

  //   ROS_INFO("Created pose for cone"); 

  //   int affordance;

  //   ROS_INFO("Select the grasp affordances:");
  //   ROS_INFO("Press 1 for first affordance (lateral-object grasp)");
  //   ROS_INFO("Press 2 for second affordance (up-object grasp)");

  //   std::cin >> affordance;

  //   ROS_INFO("Selected affordance %d",affordance); 






  
  //   while (result!= success)
  //   { 

  //    if(affordance==1)
  //    { 
  //      ROS_INFO("Starting TrajectoryManager");
  //      ROS_INFO("TrajectoryManager::Receiving cone");
  //      //visual_perception::Cone cone;
     
  //      //cone = request.cone;
    
  //      ROS_INFO("Received cone"); 

  //      ROS_INFO("Start servicecall for the sphere grasp_planning_cone_affordance_1_srv");

  //      std::string service_name5("grasp_planning_cone_affordance_1_srv");
  //      while ( !ros::service::waitForService(service_name5, ros::Duration().fromSec(3.0)) && nh.ok() )
  //      {
  //        ROS_INFO("Waiting for service %s...", service_name5.c_str());
  //       }
  //      if (!nh.ok()) exit(0);
  //      grasp_planner::TrajectoryDemoCone grasp_planning_cone_affordance_1_srv;
      
  //      grasp_planning_cone_affordance_1_srv.request.cone=cone;

  //      ROS_INFO("grasp_planning_cone_affordance_1_srv  waiting for service on topic grasp_planning_cone_affordance_1_srv");
  //      if (!ros::service::call(service_name5, grasp_planning_cone_affordance_1_srv))
  //      {
  //        ROS_ERROR("Call to grasp planning service failed");
  //        //exit(0);
  //       }
  //      else 
  //      {    
  //        if (grasp_planning_cone_affordance_1_srv.response.result != grasp_planning_cone_affordance_1_srv.response.SUCCESS)
  //        {
  //          ROS_ERROR("grasp planning service returned error %d", grasp_planning_cone_affordance_1_srv.response.result);
  //          //exit(0);
  //         }
  //         else
  //        {  
  //          ROS_INFO("Call to grasp_planning_cone_affordance_1_srv DONE !!!");
  //          grasp_planner::Trajectory traj[int_traj];
  //          for(int i =0;i<int_traj;i++)
  //          {
  //           traj[i] = grasp_planning_cone_affordance_1_srv.response.traj[i];
  //         }
  //        trajectory = grasp_planning_cone_affordance_1_srv.response.moveit_trajectory;        
  //        ROS_INFO("grasp_planning_cone_affordance_1_srv find the Trajectory to the cone");
  //        result=grasp_planning_cone_affordance_1_srv.response.result;
  //        break;
  //       }   
  //     }



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
  //     }
  //   }  
  // }



 //  /*! call serviceCallback for Box                                                               */
   if(n_object==4)
   {
    visual_perception::Plane plane;
    // cone = request.cone;
    tf::Transform rot_plan;
    double min_x4(-0.7);
    double max_x4(-0.5);
    double min_y4(-0.25);
    double max_y4(-0.15);

    srand(time(NULL));
    double X4=(max_x4-min_x4)*((double)rand()/(double)RAND_MAX)+min_x4;
    srand(time(NULL));
    double Y4=(max_y4-min_y4)*((double)rand()/(double)RAND_MAX)+min_y4;
    tf::Vector3 position_plan(X4,Y4,0.15);
    //tf::Vector3 position_cyl(-0.611268,-0.208724,0.155);
    //tf::Vector3 position_cyl(0.000000, -1.129196,0.077);
    tf::Quaternion orientation_plan(tf::createQuaternionFromRPY(0, 0, 0));
    ROS_INFO("Orientation plan(%f,%f,%f,%f)",orientation_plan[0],orientation_plan[1],orientation_plan[2],orientation_plan[3]);
    rot_plan.setRotation(orientation_plan);    
    rot_plan.setOrigin(position_plan);

    

    plane.pose.pose.orientation.x=orientation_plan[0];
    plane.pose.pose.orientation.y=orientation_plan[1];
    plane.pose.pose.orientation.z=orientation_plan[2];
    plane.pose.pose.orientation.w=orientation_plan[3];

    plane.pose.pose.position.x=position_plan[0]; 
    plane.pose.pose.position.y=position_plan[1];
    plane.pose.pose.position.z=position_plan[2];
    
    plane.x_min=(double)0.2;
    plane.x_max=(double)0.45;
    plane.y_min=(double)0.1;
    plane.y_max=(double)0.5;
    plane.z_max=(double)0.2;
    plane.z_min=(double)0.5;

    double height(plane.x_max-plane.x_min);
    double width(plane.y_max-plane.y_min);
    double depth(plane.z_max-plane.z_min);


    ROS_INFO("Created pose for plane"); 



    int affordance;

    ROS_INFO("Select the grasp affordances:");
    ROS_INFO("Press 1 for first affordance (lateral-object grasp)");
    ROS_INFO("Press 2 for second affordance (up-object grasp)");

    std::cin >> affordance;

    ROS_INFO("Selected affordance %d",affordance); 
 
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










  ROS_INFO("Starting sending response");

  //moveit::planning_interface::MoveGroup group("left_arm");


  //sleep(15.0);  
  for(int i =0;i<int_traj;i++)
  {
    response.traj[i] = traj[i];
  }
  response.moveit_trajectory= trajectory;
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
} ;