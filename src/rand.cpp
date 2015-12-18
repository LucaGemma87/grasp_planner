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