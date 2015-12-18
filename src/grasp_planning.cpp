// Author(s): Luca Gemma

#include <string>

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

// Moveit headers

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>





namespace grasp_planner {

class GraspPlanner 
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


  bool serviceCallback(GraspPlanning::Request &request, GraspPlanning::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  GraspPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   graspplanning_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("grasp_planner_markers_out"), 10);

   grasp_planning_srv_ = nh_.advertiseService(nh_.resolveName("grasp_planning_srv"),    &GraspPlanner::serviceCallback, this);

   display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>(nh_.resolveName("/move_group/display_planned_path"), 1, true);
   
  
   
  }

  //! Empty stub
  ~GraspPlanner() {}

};


/*! call serviceCallback
 */
bool GraspPlanner::serviceCallback(GraspPlanning::Request &request, GraspPlanning::Response &response)
{ 
  int n_object;
  n_object=request.ob;
  geometry_msgs::PoseStamped pose_wr_msg=request.WristPose;
  tf::Stamped<tf::Transform> pose_wr;
  tf::poseStampedMsgToTF(pose_wr_msg,pose_wr);
  tf::Matrix3x3 rotation_wrist_home = pose_wr.getBasis();
  tf::Vector3 position_wrist_home = pose_wr.getOrigin();


  //rotation_wrist_home = rotation_wrist_home.transpose();
  tf::Quaternion orientation_wrist;
  rotation_wrist_home.getRotation(orientation_wrist);
  double roll_wrist, pitch_wrist, yaw_wrist;
  rotation_wrist_home.getRPY(roll_wrist, pitch_wrist, yaw_wrist);
  ROS_INFO("RPY_wrist = (%lf, %lf, %lf)", roll_wrist, pitch_wrist, yaw_wrist);


  if(n_object==(int)1)
  {
    visual_perception::Cylinder cylinder;
    cylinder = request.cylinder;
    
    ROS_INFO("Starting wrist planning for cylinder"); 
  
    // decompose the cylinder data
    ROS_INFO ("GraspPlanner:Success in receving the cylinder transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",cylinder.pose.pose.orientation.x, cylinder.pose.pose.orientation.y, cylinder.pose.pose.orientation.z, cylinder.pose.pose.orientation.w,cylinder.pose.pose.position.x,  cylinder.pose.pose.position.y,  cylinder.pose.pose.position.z);
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
     
      //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_arm");

      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
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

      //  A pose for the box (specified relative to frame_id) 
      // //geometry_msgs::Pose _pose;
        
      // collision_object.primitives.push_back(primitive);
      // collision_object.primitive_poses.push_back(cylinder.pose.pose);
      // collision_object.operation = collision_object.ADD;

      //  std::vector<moveit_msgs::CollisionObject> collision_objects;  
      //  collision_objects.push_back(collision_object);  

      //  // Now, let's add the collision object into the world
      //  ROS_INFO("Add the cylinder into the world");  
      //  planning_scene_interface.addCollisionObjects(collision_objects);
  
      //  /* Sleep so we have time to see the object in RViz */
      // sleep(2.0);


      ROS_INFO("Starting getting Trajectory Position");
      int int_traj =20;
      tf::Transform trajector[int_traj];
      tf::Vector3 position_traj[int_traj];
      tf::Quaternion orientation_traj[int_traj];
      
      ROS_INFO("Calculate alpha_2 and beta_2");
      double alpha=8;
      double beta;
      double distance_x=position_cylinder[0]-position_wrist_home[0];
      ROS_INFO("Calculated distance_x: %f",distance_x);
      double distance_y=position_cylinder[1]-position_wrist_home[1];
      ROS_INFO("Calculated distance_y: %f",distance_y);
      
      double int_step = (double)10;

      double step_x=distance_x/int_step;
      double step_y=distance_y/int_step;

      ROS_INFO("Step_x= %f",step_x);
      ROS_INFO("Step_y = %f",step_y);
      double x_wrist_traj[10],y_wrist_traj[10],z_wrist_traj[10];


      double sigma_1=pow(position_cylinder[0]-position_wrist_home[0],4)*alpha*position_cylinder[2];

      if ((sigma_1 !=(double)0) & (position_cylinder[1] != position_wrist_home[1]))
       {
        ROS_INFO("Calculate beta");

         beta=-pow((position_cylinder[1]-position_wrist_home[1]),4)/(position_cylinder[2]+(pow((position_cylinder[1]-position_wrist_home[1]),4))/alpha);

         ROS_INFO("Beta = %f",beta);

         for (int i = 0; i < int_step; ++i)
         {
           // x_wrist_traj[i]=;
           // y_wrist_traj[i]=;
           // z_wrist_traj[i]= ;
         }



       } 
       else
       {
        ROS_INFO("Failed to calculate beta");
        ROS_INFO("Proceding to linear trajectory");
        double distance_z=position_cylinder[2]-position_wrist_home[2];
        ROS_INFO("Calculated distance_z: %f",distance_z);
        



       }
      
      // int count = 12;
      // int int_grasp =0;
      // int tresh_z=20;
      // int j=0;    

      // double z_grasp[1000];
      // double x_grasp[1000];
      // double y_grasp[1000];

  

      // inserire un altro for per la min_distance max_distance
    
      //  for (int k = 0; k < tresh_z; ++k)
      //  {
      //    for (int i = 0; i < count; ++i)
      //     {
      //       x_grasp[j]= ((double)min_distance+(double)cylinder.r)* cos((double)i/(double)count*(double)2*pi); 
      //       y_grasp[j]= ((double)min_distance+(double)cylinder.r)* sin((double)i/(double)count*(double)2*pi);
      //       z_grasp[jn_ob]= -(double)cylinder.h/(double)2 + ((double)k/(double)tresh_z)*(double)cylinder.h;
      //       ROS_INFO("Genetated x_grasp y_grasp z_grasp: %f,%f,%f",x_grasp[j], y_grasp[j], z_grasp[j]);
      //       int_grasp+=1;
      //       j+=1;

      //     }
      //   }
      // ROS_INFO("Genetated grasp: %f",(double)int_grasp);

      // ROS_INFO("Genetated random phi: %f", z_grasp);
      // x_grasp= distance * sin(theta*2*pi)*cos(phi*2*pi);
      // ROS_INFO("Genetated random x_grasp: %f", x_grasp);
      // y_grasp= distance * sin(theta*2*pi)*sin(phi*2*pi);
      //ROS_INFO("Genetated random y_grasp: %f", y_grasp);

      //x_grasp=(distance+cylinder.r)*cos(theta*2*pi);
      //  ROS_INFO("Genetated random x_grasp: %f", x_grasp);
      //  y_grasp=(distance+cylinder.r)*sin(theta*2*pi);
      //  ROS_INFO("Genetated random y_grasp: %f", y_grasp);

      // tf::Transform rot[1000];
      // tf::Transform grasp[1000];
      // tf::Vector3 position_grasp[1000];
      // ros::Time now = ros::Time::now();
      // for (int g = 0; g < (int)int_grasp; ++g)
      // {
      //   rot[g].setOrigin(tf::Vector3(x_grasp[g],y_grasp[g],z_grasp[g]));
      //   rot[g].setRotation(tf::createQuaternionFromRPY(0, 0, 0));     
      //   grasp[g]=rot[g]*cylinder_transform;
      //   position_grasp[g] = grasp[g].getOrigin();
      
      //  //  if(position_grasp[2][i]>threshold)
      //  // {
      //    std::string grasp_name;
      //    std::string grasp_num("grasp");
      //    std::ostringstream out;
      //    out << g;
      //    std::string n_ob;
      //    n_ob = out.str();
      //    grasp_name = grasp_num+ n_ob;
         
      //    broadcaster_.sendTransform(tf::StampedTransform(grasp[g], now, "world_link", grasp_name));
      //   //}   
      //   ROS_INFO("Showed grasp n: %d",g); 
      //   //sleep(1.0); 
      // }
      
 
// // Getting Basic Information
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
  ROS_INFO ("GraspPlanner:Success in receving wrist transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",orientation_wrist[0],orientation_wrist[1],orientation_wrist[2],orientation_wrist[3],position_wrist_home[0],position_wrist_home[1],position_wrist_home[2]);
  geometry_msgs::Pose start_pose;
  start_pose.orientation.x = orientation_wrist[0];
  start_pose.orientation.y = orientation_wrist[1];
  start_pose.orientation.z = orientation_wrist[2];
  start_pose.orientation.w = orientation_wrist[3];

  start_pose.position.x = position_wrist_home[0];
  start_pose.position.y = position_wrist_home[1];
  start_pose.position.z = position_wrist_home[2];

  

  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::Pose target_pose1;// = start_pose;
  // target_pose1.position.x -= 0.4;
  // target_pose1.position.y += 0.7;
  // target_pose1.position.z -= 0.7;
  // target_pose1.orientation.x = orientation_wrist[0];
  // target_pose1.orientation.y = orientation_wrist[1];
  // target_pose1.orientation.z = orientation_wrist[2];
  // target_pose1.orientation.w = orientation_wrist[3];
  //waypoints.push_back(target_pose1);  // avanti e giù
  target_pose1.position.x = position_cylinder[0];
  target_pose1.position.z = position_cylinder[1];
  target_pose1.position.y = position_cylinder[2];
  target_pose1.orientation.x = orientation_cylinder[0];
  target_pose1.orientation.y = orientation_cylinder[1];
  target_pose1.orientation.z = orientation_cylinder[2];
  target_pose1.orientation.w = orientation_cylinder[3];
  waypoints.push_back(target_pose1);  // sinistra
  // target_pose1.orientation.x = orientation_wrist[0]-0.5;
  // target_pose1.orientation.y = orientation_wrist[1]-0.5;
  // target_pose1.orientation.z = orientation_wrist[2]-0.5;
  // target_pose1.orientation.w = orientation_wrist[3]-0.5;

  // target_pose1.position.z -= 0.2;
  // target_pose1.position.y += 0.2;
  // target_pose1.position.x -= 0.2;
  // waypoints.push_back(target_pose1);  // back to start

  // // We want the cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively
  // // disabling it.
  //group.setPlannerId("PRMstarkConfigDefault");
  group.setPlanningTime((double)20);
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                                0.01,  // eef_step
                                                0.001,   // jump_threshold
                                                trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);    
  // /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);
  response.result=response.SUCCESS;
  ROS_INFO("Starting sending response");
     Trajectory traj[1000];
      for(int i =0;i<1000;i++)
      {
        response.traj[i] = traj[i];
      }
        


   } 
  }
  
  if(n_object==(int)2)
  {
    visual_perception::Sphere sphere;
    sphere= request.sphere;
    ROS_INFO("Starting wrist planning for sphere"); 
  
    // decompose the sphere data
    ROS_INFO ("GraspPlanner:Success in receving the sphere transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z,sphere.pose.pose.orientation.w,sphere.pose.pose.position.x,sphere.pose.pose.position.y,sphere.pose.pose.position.z);
    tf::Transform sphere_transform(tf::Quaternion(sphere.pose.pose.orientation.x, sphere.pose.pose.orientation.y, sphere.pose.pose.orientation.z, sphere.pose.pose.orientation.w), tf::Vector3( sphere.pose.pose.position.x,sphere.pose.pose.position.y,sphere.pose.pose.position.z) );
    
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
      double distance = 0.2;
      double threshold = 0.05;
      double pi = 3.14159;
      double roll_sph, pitch_sph, yaw_sph;
      rotation_sphere.getRPY(roll_sph, pitch_sph, yaw_sph);
      ROS_INFO("RPY = (%lf, %lf, %lf)", roll_sph, pitch_sph, yaw_sph);
      
      //moveit::move_group_interface::move_group group('left_arm');
      moveit::planning_interface::MoveGroup group("left_arm");

      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

        

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = group.getPlanningFrame();

        

      /* The id of the object is used to identify it. */
      collision_object.id = "sphere";

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] =2*fabs(sphere.r) ;
      primitive.dimensions[1] =2*fabs(sphere.r) ;
      primitive.dimensions[2] =2*fabs(sphere.r);

      /* A pose for the box (specified relative to frame_id) */
      //geometry_msgs::Pose _pose;
        
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(sphere.pose.pose);
      collision_object.operation = collision_object.ADD;

       std::vector<moveit_msgs::CollisionObject> collision_objects;  
       collision_objects.push_back(collision_object);  

       // Now, let's add the collision object into the world
       ROS_INFO("Add the sphere into the world");  
       planning_scene_interface.addCollisionObjects(collision_objects);
  
       /* Sleep so we have time to see the object in RViz */
       sleep(2.0);
      /// continuare .....    

    }
  }

  if(n_object==(int)3)
  {
    visual_perception::Cone  cone;
    cone = request.cone;
    
    ROS_INFO("Starting wrist planning for cone"); 
  
    // decompose the cone data
    ROS_INFO ("GraspPlanner:Success in receving the cone transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",cone.pose.pose.orientation.x, cone.pose.pose.orientation.y, cone.pose.pose.orientation.z,cone.pose.pose.orientation.w,cone.pose.pose.position.x,cone.pose.pose.position.y,cone.pose.pose.position.z);
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

   plane=request.plane;

   
   ROS_INFO("Starting wrist planning for plane"); 
  
    // decompose the plane data
    ROS_INFO ("GraspPlanner:Success in receving the plane transformation with 7 elements (Orient(x,y,z,w) ,Pos(x,y,z)): %f, %f ,%f, %f, %f, %f, %f",plane.pose.pose.orientation.x, plane.pose.pose.orientation.y, plane.pose.pose.orientation.z,plane.pose.pose.orientation.w,plane.pose.pose.position.x,plane.pose.pose.position.y,plane.pose.pose.position.z);
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

  grasp_planner::GraspPlanner node(nh);

  ros::spin();
  return 0;
};