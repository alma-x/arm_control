#include <signal.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "ros/service.h"
#include "arm_control/aruco_service.h"
#include "arm_control/UserInterface.h"
#include "arm_control/collision_object_srv.h"
#include "arm_control/float_return_srv.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <ctime>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <fstream>
#include "control_msgs/GripperCommandActionGoal.h"
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include<list>

using namespace std;
using namespace geometry_msgs;
using namespace moveit;
using namespace planning_interface;
using namespace moveit_msgs;
using namespace Eigen;


bool action_gripper(string input){


  moveit_msgs::MoveGroupActionResult msg;
  moveit_msgs::MoveGroupActionResultConstPtr msg_pointer;
  std_msgs::String msg_to_pub;
  ros::Duration d;
  ROS_INFO("TRYING TO SET GRIPPER AS:");
  cout<<input<<endl;

  d.sec=6;


  if(input==posizione_gripper){
    ROS_INFO("Gripper already in the correct position(STATE MACHINE)");
  }


  msg_to_pub.data=input;
  pub_gripper.publish(msg_to_pub);
  msg_pointer=(ros::topic::waitForMessage<moveit_msgs::MoveGroupActionResult>("/move_group/result",d));
  if(msg_pointer==NULL){

    ROS_INFO("NO MESSAGES RECEIVED");

    actionlib_msgs::GoalID msg_traj_cancel;
    msg_traj_cancel.id="";
    pub_traj_cancel.publish(msg_traj_cancel);
    return false;

  }
  msg=*msg_pointer;

  if(msg.status.text=="Requested path and goal constraints are already met.")
  {
    ROS_INFO("Gripper already in the correct position");
    return true;
  }
  if(msg.status.text=="Solution was found and executed."){

    ROS_INFO("Gripper solution found and executed");
    posizione_gripper=input;
    update_gripper_collision_box(input);
    return true;

  }if(msg.status.text=="No motion plan found. No execution attempted."){

    ROS_INFO("No solution found for the gripper movement");
    return false;

  }
  //ROS_INFO("size: %d",msg.result.planned_trajectory.joint_trajectory.joint_names.size());
  if(msg.result.planned_trajectory.joint_trajectory.joint_names.size()>0){
    if(msg.result.planned_trajectory.joint_trajectory.joint_names[0]=="finger_joint"){

      double final_value_gripper=-20;
      final_value_gripper=msg.result.planned_trajectory.joint_trajectory.points[msg.result.planned_trajectory.joint_trajectory.points.size()-1].positions[0];

      if(final_value_gripper>-0.2 && final_value_gripper<0.2){
        if(input=="open"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;

          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=0.2 && final_value_gripper<0.85){
        if(input=="semi_open"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;

          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=0.85 && final_value_gripper<1.18){
        if(input=="semi_close"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;
          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=1.18){
        if(input=="close"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;
          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }

      ROS_INFO("finger joint letto, final value: %f",final_value_gripper);
    }

    ROS_INFO("finger joint non letto");
  }

  ROS_INFO("RESULT MESSAGE NON ANALIZZATO CORRETTAMENTE");

  return false;

}


bool move_aruco_to_center_of_camera(double percentual_zoom){
  arm_control::aruco_serviceResponse msg_from_bridge=bridge_service(str_md_rd,"");

  if(msg_from_bridge.aruco_found){
    ROS_INFO("ERC: centering camera");
    Pose pose_robot=robot->getCurrentPose().pose;

    Pose pose_robot_target,pose_finalpos,pose_aruco,pose_camera;
    Affine3d T_0_tool,T_0_camera,T_camera_aruco,T_camera_aruco_modified,T_0_aruco,T_0_camera_gazebo;
    tf::Pose pose_robot_tf,pose_target_tf,pose_finalpos_tf,pose_aruco_tf,pose_camera_tf;

    tf::poseMsgToTF(pose_robot,pose_robot_tf);
    tf::poseTFToEigen(pose_robot_tf,T_0_tool);
    //In questo punto ho T_0_tool

    T_0_camera=T_0_tool*T_tool_camera;
    tf::poseEigenToTF(T_0_camera,pose_camera_tf);
    tf::poseTFToMsg(pose_camera_tf,pose_camera);
    //In questo punto ho T_0_camera


    T_0_camera_gazebo=T_0_camera*T_camera_camera_gazebo;
    //In questo punto ho T_0_camera_gazebo


    Vector3d translation_camera_aruco(msg_from_bridge.x,msg_from_bridge.y,msg_from_bridge.z);
    Matrix3d rotation_camera_aruco;
    Vector3d xct(msg_from_bridge.vector[0],msg_from_bridge.vector[1],msg_from_bridge.vector[2]),yct(msg_from_bridge.vector[3],msg_from_bridge.vector[4],msg_from_bridge.vector[5]),zct(msg_from_bridge.vector[6],msg_from_bridge.vector[7],msg_from_bridge.vector[8]);
    rotation_camera_aruco.row(0)=xct;
    rotation_camera_aruco.row(1)=yct;
    rotation_camera_aruco.row(2)=zct;
    T_camera_aruco.translation()=translation_camera_aruco;
    T_camera_aruco.linear()=rotation_camera_aruco;

    T_0_aruco=T_0_camera_gazebo*T_camera_aruco;
    tf::poseEigenToTF(T_0_aruco,pose_aruco_tf);
    tf::poseTFToMsg(pose_aruco_tf,pose_aruco);
    //In questo punto ho T_0_aruco



    Affine3d T_0_tool_modified,T_tool_ee,T_0_ee_modified,T_0_camera_modified,T_aruco_camera_modified,T_my_rotation,T_my_translation;
    Pose pose_ee_modified,pose_camera_modified;
    tf::Pose pose_ee_modified_tf,pose_cam_mod_tf;



    //calcolo vettore che collega 0_camera_gazebo con aruco
    Vector3d vettore_gazebo_aruco(0,0,0);
    vettore_gazebo_aruco.x()=T_0_aruco.translation().x()-T_0_camera_gazebo.translation().x();
    vettore_gazebo_aruco.y()=T_0_aruco.translation().y()-T_0_camera_gazebo.translation().y();
    vettore_gazebo_aruco.z()=T_0_aruco.translation().z()-T_0_camera_gazebo.translation().z();
    Matrix3d rotation_my_rot;
    Vector3d x_my_rot(1,0,0),y_my_rot(0,1,0),z_my_rot(0,0,0),trans_my_rot(0,0,0);
    rotation_my_rot.col(0)=x_my_rot;
    rotation_my_rot.col(1)=y_my_rot;
    rotation_my_rot.col(2)=vettore_gazebo_aruco;
    T_my_rotation.translation()=trans_my_rot;
    T_my_rotation.linear()=rotation_my_rot;

    Affine3d T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_aruco_modified_orizz,T_all_rotativo;


    //float roll = 0, pitch = 0, yaw = atan(T_camera_aruco.translation().y()/T_camera_aruco.translation().x());
    float roll = 0, pitch = 0, yaw = atan(vettore_gazebo_aruco.y()/vettore_gazebo_aruco.x());
    //
    //

    Quaterniond q;
    q = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_allineamento_orizzontale=q.toRotationMatrix();

    T_all_orizz.translation().x()=0;
    T_all_orizz.translation().y()=0;
    T_all_orizz.translation().z()=0;
    T_all_orizz.linear()=rot_allineamento_orizzontale;


    //Applicando la matrice sopra a T_0_camera_gazebo, ottengo che l'asse blu è esattamente sopra all'aruco. se rotazione su asse verde allora perfetto
    // mi serve la nuova distanza tra la camera e l'aruco sull'asse rosso

    T_0_camera_gazebo_modified_orizz=T_0_camera_gazebo;
    T_0_camera_gazebo_modified_orizz.linear()=rot_allineamento_orizzontale;
    T_camera_aruco_modified_orizz=T_0_camera_gazebo_modified_orizz.inverse()*T_0_aruco;

    double dissallineamento_verticale=M_PI/2 - atan(T_camera_aruco_modified_orizz.translation().z()/T_camera_aruco_modified_orizz.translation().x());
    double sq_dist=sqrt(vettore_gazebo_aruco.x()*vettore_gazebo_aruco.x() +vettore_gazebo_aruco.y()*vettore_gazebo_aruco.y() + vettore_gazebo_aruco.z()*vettore_gazebo_aruco.z());


    roll = 0, pitch = dissallineamento_verticale, yaw = 0;
    Quaterniond q_all_vert;
    q_all_vert = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_all_vert=q_all_vert.toRotationMatrix();

    T_all_vert.translation().x()=0;
    T_all_vert.translation().y()=0;
    T_all_vert.translation().z()=0;
    T_all_vert.linear()=rot_all_vert;


    roll = 0, pitch = 0, yaw = -M_PI/2;
    Quaterniond q_all_rotativo;
    q_all_rotativo = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_all_rotativo=q_all_rotativo.toRotationMatrix();

    T_all_rotativo.translation().x()=0;
    T_all_rotativo.translation().y()=0;
    T_all_rotativo.translation().z()=0;
    T_all_rotativo.linear()=rot_all_rotativo;




    T_0_camera_gazebo_modified=T_0_camera_gazebo;
    T_0_camera_gazebo_modified.linear()=rot_allineamento_orizzontale;
    T_0_camera_gazebo_modified=T_0_camera_gazebo_modified*T_all_vert*T_all_rotativo;

    //ZOOM
    //T_0_camera*T_camera_aruco_mod=T_0_aruco
    T_camera_aruco_modified=T_0_camera_gazebo_modified.inverse()*T_0_aruco;
    double distance_camera_aruco_z=T_camera_aruco_modified.translation().z();

    Affine3d T_zoom;
    T_zoom.linear().setIdentity();
    T_zoom.translation().z()=percentual_zoom*distance_camera_aruco_z/100;
    T_0_camera_gazebo_modified=T_0_camera_gazebo_modified*T_zoom;




    T_0_camera_modified=T_0_camera_gazebo_modified*T_camera_camera_gazebo.inverse();
    tf::poseEigenToTF(T_0_camera_modified,pose_cam_mod_tf);
    tf::poseTFToMsg(pose_cam_mod_tf,pose_camera_modified);


    T_0_tool_modified=T_0_camera_modified*T_tool_camera.inverse();


    pose_ee_modified=homo_to_pose(T_0_tool_modified);
    bool debug=false;
    if(debug){
      cout<<"Translation camera aruco modified orizz:"<<endl<<T_camera_aruco_modified_orizz.translation();

      cout<<"Rotation with rpy:"<<endl<<T_0_camera_gazebo_modified.linear();
      cout<<"Translation with rpy:"<<endl<<T_0_camera_gazebo_modified.translation();


      cout<<"Disallineamento verticale:"<<dissallineamento_verticale<<endl;
      cout<<"ee pose:"<<endl;
      stampa_Pose(pose_robot);

      cout<<"camera pose:"<<endl;
      stampa_Pose(pose_camera);

      cout<<"camera_modified pose:"<<endl;
      stampa_Pose(pose_camera_modified);

      cout<<"pose ee_modified:"<<endl;

      stampa_Pose(pose_ee_modified);

      cout<<"MY_TRANSLATION translation:"<<endl<<T_my_translation.translation()<<endl;

      cout<<"MY_TRANSLATION rotation:"<<endl<<T_my_translation.linear()<<endl;

      cout<<"MY_ROTATION translation:"<<endl<<T_my_rotation.translation()<<endl;

      cout<<"MY_ROTATION rotation:"<<endl<<T_my_rotation.linear()<<endl;

      cout<<"T_0_camera translation:"<<endl<<T_0_camera.translation()<<endl;

      cout<<"T_0_camera rotation:"<<endl<<T_0_camera.linear()<<endl;

      cout<<"T_0_camera_modified translation:"<<endl<<T_0_camera_modified.translation()<<endl;

      cout<<"T_0_camera_modified rotation:"<<endl<<T_0_camera_modified.linear()<<endl;


      cout<<"T_camera_aruco translation:"<<endl<<T_camera_aruco.translation()<<endl;

      cout<<"T_camera_aruco rotation:"<<endl<<T_camera_aruco.linear()<<endl;


      cout<<"T_camera_aruco modified translation:"<<endl<<T_camera_aruco_modified.translation()<<endl;

      cout<<"T_camera_aruco modified rotation:"<<endl<<T_camera_aruco_modified.linear()<<endl;

      cout<<"T_camera_aruco modified MANUAL INVERSE translation:"<<endl<<T_aruco_camera_modified.translation()<<endl;

      cout<<"T_camera_aruco modified MANUAL INVERSE rotation:"<<endl<<T_aruco_camera_modified.linear()<<endl;

      cout<<"T_camera_aruco modified INVERSE translation:"<<endl<<T_camera_aruco_modified.inverse().translation()<<endl;

      cout<<"T_camera_aruco modified INVERSE rotation:"<<endl<<T_camera_aruco_modified.inverse().linear()<<endl;

    }


//      move_to_pose(pose_ee_modified,true);
//      return true;
    return move_to_pose_cartesian(pose_ee_modified);

  }
  else{
    ROS_INFO("No aruco detected, no return??");
    return false;
  }

}
bool centra_aruco_nella_camera(int ID_aruco, double percentuale_zoom){

  if(!Aruco_values[ID_aruco].valid){
    ROS_INFO("NON POSSEGGO QUELL ARUCO");
    return false;
  }

  Affine3d T_0_aruco,T_0_tool,T_0_camera_gazebo,T_camera_gazebo_aruco,T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_gazebo_aruco_modified_orizz,T_all_rotativo,T_0_tool_modified_oriz;
  Vector3d vettore_gazebo_aruco(0,0,0);
  Matrix3d rot_allineamento_orizzontale,rot_all_vert,rot_all_rotativo;
  float roll_oriz, pitch_oriz, yaw_oriz,roll_vert,pitch_vert,yaw_vert,roll_rot,pitch_rot,yaw_rot;
  T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
  T_0_aruco=pose_to_homo(Aruco_values[ID_aruco].pose);
  T_0_camera_gazebo=T_0_tool*T_tool_camera_gazebo;

  //T_0_aruco=T_0_camera*T_camera_aruco

  T_camera_gazebo_aruco=T_0_camera_gazebo.inverse()*T_0_aruco;
  Affine3d T_tool_aruco;
  Affine3d T_ok;
  T_tool_aruco=T_0_tool.inverse()*T_0_aruco;

  stampa_homo_named(T_tool_aruco,"T_tool_aruco");
  float theta1,theta2;

  //con servizio python------------------
  //ros::NodeHandle node_handle;
  //ros::ServiceClient client1;
  //client1 = node_handle.serviceClient<arm_control::float_return_srv>("/solve_equation_serv");
  //arm_control::float_return_srv msg;
  //msg.request.num1=T_tool_aruco.translation().x();
  //msg.request.num2=T_tool_aruco.translation().y();
  //client1.call(msg);
  //--------------------------

  float Xa=T_tool_aruco.translation().x();
  float Ya=T_tool_aruco.translation().y();
  theta1=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) + sqrt((25000000*pow(Xa,2)*pow(cos(pi/18),2) - 190969*pow(cos(pi/18),2) + 25000000*pow(Ya,2)*pow(cos(pi/18),2) + 25000000*pow(Xa,2)*pow(sin(pi/18),2) + 25000000*pow(Ya,2)*pow(sin(pi/18),2))))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)));
  theta2=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) - sqrt((25000000*pow(Xa,2)*pow(cos(pi/18),2) - 190969*pow(cos(pi/18),2) + 25000000*pow(Ya,2)*pow(cos(pi/18),2) + 25000000*pow(Xa,2)*pow(sin(pi/18),2) + 25000000*pow(Ya,2)*pow(sin(pi/18),2))))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)));
  printf("res1: %f \n",theta1);
  printf("res2: %f \n",theta2);

  Affine3d T_finale;

{
  Matrix3d rot_ok;
  rot_ok=from_rpy_to_rotational_matrix(0,0,theta1);
  T_ok.linear()=rot_ok;

  T_ok.translation().x()=0;
  T_ok.translation().y()=0;
  T_ok.translation().z()=0;

  Affine3d T_0_tool_vert,T,T_tool_aruco_vert;
  T_0_tool_vert=T_0_tool*T_ok;

  T_tool_aruco_vert=T_0_tool_vert.inverse()*T_0_aruco;


  //move_to_pose_optimized(homo_to_pose(T_0_tool_vert));
  //sleep(2);

  double phi=0;
  phi=-(M_PI/2-atan2(T_tool_aruco_vert.translation().x(),T_tool_aruco_vert.translation().z()));



  Affine3d T_orizz;
  Matrix3d rot_orizz;
  rot_orizz=from_rpy_to_rotational_matrix(0,phi,0);
  T_orizz.linear()=rot_orizz;

  T_orizz.translation().x()=0;
  T_orizz.translation().y()=0;
  T_orizz.translation().z()=0;

  Affine3d T_0_tool_vert_orizz,T_tool_aruco_theta1;
  T_0_tool_vert_orizz=T_0_tool_vert*T_orizz;

  T_tool_aruco_theta1=T_0_tool_vert_orizz.inverse()*T_0_aruco;
  stampa_homo_named(T_tool_aruco_theta1,"T_tool_aruco_theta1");
  if(T_tool_aruco_theta1.translation().x()>0){
    T_finale=T_0_tool_vert_orizz;
  }
}
{

    Matrix3d rot_ok;
    rot_ok=from_rpy_to_rotational_matrix(0,0,theta2);
    T_ok.linear()=rot_ok;

    T_ok.translation().x()=0;
    T_ok.translation().y()=0;
    T_ok.translation().z()=0;

    Affine3d T_0_tool_vert,T,T_tool_aruco_vert;
    T_0_tool_vert=T_0_tool*T_ok;

    T_tool_aruco_vert=T_0_tool_vert.inverse()*T_0_aruco;


    //move_to_pose_optimized(homo_to_pose(T_0_tool_vert));
    //sleep(2);

    double phi=0;
    phi=-(M_PI/2-atan2(T_tool_aruco_vert.translation().x(),T_tool_aruco_vert.translation().z()));



    Affine3d T_orizz;
    Matrix3d rot_orizz;
    rot_orizz=from_rpy_to_rotational_matrix(0,phi,0);
    T_orizz.linear()=rot_orizz;

    T_orizz.translation().x()=0;
    T_orizz.translation().y()=0;
    T_orizz.translation().z()=0;

    Affine3d T_0_tool_vert_orizz,T_tool_aruco_theta2;
    T_0_tool_vert_orizz=T_0_tool_vert*T_orizz;

    T_tool_aruco_theta2=T_0_tool_vert_orizz.inverse()*T_0_aruco;
    stampa_homo_named(T_tool_aruco_theta2,"T_tool_aruco_theta1");
    if(T_tool_aruco_theta2.translation().x()>0){
      T_finale=T_0_tool_vert_orizz;
    }
}

  move_to_pose_optimized(homo_to_pose(T_finale));

//  stampa_homo_named(T_tool_camera_gazebo,"T_tool_camera_gazebo");
//  stampa_homo_named(T_0_aruco,"T_0_aruco");
//  stampa_homo_named(T_0_camera_gazebo,"T_0_camera_gazebo");
//  stampa_homo_named(T_0_tool,"T_0_tool");
//  stampa_homo_named(T_camera_gazebo_aruco,"T_camera_gazebo_aruco");
//  stampa_homo_named(T_ok,"T_ok");

  return true;
}
bool zoom_camera_to_aruco(int ID_aruco,double valore){
  if(!Aruco_values[ID_aruco].valid){
    ROS_INFO("NON POSSEGGO QUELL ARUCO");
    return false;
  }
  cambia_aruco(to_string(ID_aruco));
  if(!aruco_individuato()){
    centra_aruco_nella_camera(ID_aruco, 0);
  }


  Affine3d T_0_aruco,T_0_tool,T_0_camera_gazebo,T_camera_gazebo_aruco,T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_gazebo_aruco_modified_orizz,T_all_rotativo,T_0_tool_modified_oriz,T_0_tool_zoom;
  Vector3d vettore_gazebo_aruco(0,0,0);
  Matrix3d rot_allineamento_orizzontale,rot_all_vert,rot_all_rotativo;
  float roll_oriz, pitch_oriz, yaw_oriz,roll_vert,pitch_vert,yaw_vert,roll_rot,pitch_rot,yaw_rot;
  T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
  T_0_aruco=pose_to_homo(Aruco_values[ID_aruco].pose);
  T_0_camera_gazebo=T_0_tool*T_tool_camera_gazebo;

  //T_0_aruco=T_0_camera*T_camera_aruco

  T_camera_gazebo_aruco=T_0_camera_gazebo*T_0_aruco.inverse();
  float distanza_su_asse_focale=T_camera_gazebo_aruco.translation().z();

  Affine3d T_ok;
  T_ok.translation().z()=distanza_su_asse_focale*valore/100;
  T_ok.translation().x()=0;
  T_ok.translation().y()=0;
  T_ok.linear()=from_rpy_to_rotational_matrix(0,0,0);

  T_0_camera_gazebo=T_0_camera_gazebo*T_ok;

  T_0_tool_zoom=T_0_camera_gazebo*T_tool_camera_gazebo.inverse();

  move_to_pose_optimized(homo_to_pose(T_0_tool_zoom));

}

//Panels
/*bool solleva_coperchio(){

  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }

  ROS_INFO("ARUCO TROVATO, POSSO ANDARE A PRENDERE LA COVER");
  Pose p_0_aruco=Aruco_values[ID_INSPECTION_WINDOW_COVER].pose;
  vector<double> joint_group_positions=pos_joint_iniziale;

  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);

  move_to_joints(joint_group_positions);

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE ALLA COVER");

  Affine3d T_aruco_final01,T_aruco_final,T_aruco_final02,T_0_final,T_0_final01,T_0_final02, T_0_aruco;
  Pose pose_final01,pose_final02,pose_final;
  T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);


  T_aruco_final.translation().x()=0.0499;//0.05
  T_aruco_final.translation().y()=0.0249;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final.translation().z()=0.18;//0.015
  T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final=T_0_aruco*T_aruco_final;
  pose_final=homo_to_pose(T_0_final);




  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);
  joint_group_positions[1]=grad_to_rad(-90);
  joint_group_positions[2]=grad_to_rad(87.4);
  joint_group_positions[3]=grad_to_rad(-78);
  joint_group_positions[4]=grad_to_rad(-90);
  joint_group_positions[5]=grad_to_rad(-120);

  if(!move_to_joints(joint_group_positions)){
    return false;
  }


  pose_final01=robot->getCurrentPose().pose;
  pose_final01.position.z=pose_final.position.z;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final01)){
      return false;
  }


  pose_final02=robot->getCurrentPose().pose;
  pose_final02.orientation=pose_final.orientation;
  //stampa_Pose(pose_final02);


  if(!move_to_pose_optimized(pose_final02)){
      return false;
  }

  //stampa_Pose(pose_final);
  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER)){
    T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);
    T_0_final=T_0_aruco*T_aruco_final;
    pose_final=homo_to_pose(T_0_final);
  }

  //scendi per afferrare
  Affine3d T_aruco_final_avvicinato,T_0_final_avvicinato;
  Pose pose_final_avvicinato;

  T_aruco_final_avvicinato.translation().x()=0.0499;//0.05
  T_aruco_final_avvicinato.translation().y()=0.0249;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final_avvicinato.translation().z()=0.1495;//132+35/2=149.5
  T_aruco_final_avvicinato.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final_avvicinato=T_0_aruco*T_aruco_final_avvicinato;
  pose_final_avvicinato=homo_to_pose(T_0_final_avvicinato);


  if(!move_to_pose_optimized(pose_final_avvicinato)){
      return false;
  }

  action_gripper("semi_close");

  ROS_INFO("IL coperchio e' stato afferrato, lo sollevo e lo appoggio nel punto corretto");

  //mi alzo

  if(!move_to_pose_optimized(pose_final)){
      return false;
  }


  ROS_INFO("FUNCTION COMPLETE");
  return true;
}
*/
bool solleva_imu(){
  action_gripper("open");

  if(!Aruco_values[ID_IMU_MODULE].valid){
    if(!esplora_cerca_IMU())
      return false;
  }


  PosizioniBase(str_pos_iniziale);

  Affine3d T_aruco_finalpos,T_0_finalpos_pregrasp,T_0_aruco_IMU_Module;
  Pose pose_final_grasp,pose_final01,pose_final02,pose_final03,pose_final025;
  //calcolo pose

  {
  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=0;
  T_aruco_finalpos.translation().z()=0.1018;//128-(1.2+25)
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI/2,0,0);

  T_0_aruco_IMU_Module=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);
  T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

  pose_final_grasp=homo_to_pose(T_0_finalpos_pregrasp);
}


  double angolo_disallineamento_imu=atan2(pose_final_grasp.position.y,pose_final_grasp.position.x);

  vector<double> joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=angolo_disallineamento_imu;
  move_to_joints(joint_group_positions);

  vector<double> save_joint0=joint_group_positions;

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE A PRENDERE IMU");

  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=angolo_disallineamento_imu;
  joint_group_positions[1]=grad_to_rad(-90);
  joint_group_positions[2]=grad_to_rad(87.4);
  joint_group_positions[3]=grad_to_rad(-78);
  joint_group_positions[4]=grad_to_rad(-90);
  joint_group_positions[5]=grad_to_rad(-120);
  move_to_joints(joint_group_positions);

  vector<double> save_joint1=joint_group_positions;

  //sistemo orientamento
  pose_final01=robot->getCurrentPose().pose;
  pose_final01.orientation=pose_final_grasp.orientation;
  if(!move_to_pose_cartesian(pose_final01)){
    if(!move_to_pose(pose_final01,true)){
      return false;
    }
  }


  //sistemo x,y
  pose_final02=robot->getCurrentPose().pose;
  pose_final02.position.x=pose_final_grasp.position.x;
  pose_final02.position.y=pose_final_grasp.position.y;
  if(!move_to_pose_cartesian(pose_final02)){
    if(!move_to_pose(pose_final02,true)){
      return false;
    }
  }
  if(aggiorna_aruco_selezionato(ID_IMU_MODULE)){
    T_0_aruco_IMU_Module=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);
    T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

    pose_final_grasp=homo_to_pose(T_0_finalpos_pregrasp);
  }


  remove_box("imu_module");

  //mi avvicino alla corretta z
  pose_final025=robot->getCurrentPose().pose;
  pose_final025.position.z=pose_final_grasp.position.z+0.08;
  if(!move_to_pose_optimized(pose_final025)){
      return false;
  }



  //mi avvicino alla corretta z
  pose_final03=robot->getCurrentPose().pose;
  pose_final03.position.z=pose_final_grasp.position.z+0.03;
  //stampa_Pose(pose_final02);
  if(!move_to_pose_cartesian(pose_final03)){
    if(!move_to_pose(pose_final03,true)){
      return false;
    }
  }



//  if(gazebo_bool) {
//    string nome;
//    boost::thread pick_thread(pick, "imu_module",debug[2]);
//  }
    action_gripper("semi_open");

  //Aggiungo collisione imu attached to robot
  {
      Affine3d T_tool_center_of_box,T_0_tool,T_0_imu,T_imu_to_center_of_box,T_0_center_of_box;
      T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
      T_0_imu=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);

      T_imu_to_center_of_box.translation().x()=0;
      T_imu_to_center_of_box.translation().y()=0;
      T_imu_to_center_of_box.translation().z()=-(0.0012+0.025);
      T_imu_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);

      T_0_center_of_box=T_0_imu*T_imu_to_center_of_box;

      T_tool_center_of_box=T_0_tool.inverse()*T_0_center_of_box;



      PoseStamped box_pose;
      float box_size[3];
      string box_name="imu_module";
      box_pose.header.frame_id="ee_link";
      box_size[0]=0.0512;
      box_size[1]=0.165;
      box_size[2]=0.05;


      Pose pose_center_of_box=homo_to_pose(T_tool_center_of_box);
      box_pose.pose=pose_center_of_box;
      SetPoseOrientationRPY(&box_pose.pose,0,0,0);

      collision_boxes[box_name].name=box_name;
      collision_boxes[box_name].pose=box_pose;
      collision_boxes[box_name].size[0]=box_size[0];
      collision_boxes[box_name].size[1]=box_size[1];
      collision_boxes[box_name].size[2]=box_size[2];
      add_and_attach_box(collision_boxes[box_name]);
  }

  //IMU PRESO, MI ALZO

  Pose pose_final_alzato=robot->getCurrentPose().pose;
  pose_final_alzato.position.z+=0.2;

  if(!move_to_pose_optimized(pose_final_alzato)){
    return false;
  }

  return true;
}
bool go_and_attach_imu(double angolo){

    Affine3d T_aruco_final,T_aruco_pre_final,T_0_final,T_0_pre_final;
    Pose pose_final,pose_pre_final;


    T_aruco_pre_final.translation().x()=0.065;
    T_aruco_pre_final.translation().y()=-0.2;
    T_aruco_pre_final.translation().z()=0.2;
    T_aruco_pre_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0)*from_rpy_to_rotational_matrix(grad_to_rad(-90)-grad_to_rad(angolo),0,0);


    T_aruco_final.translation().x()=0.065;
    T_aruco_final.translation().y()=-0.2;
    T_aruco_final.translation().z()=0.15;
    T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0)*from_rpy_to_rotational_matrix(grad_to_rad(-90)-grad_to_rad(angolo),0,0);


    T_0_final=pose_to_homo(Aruco_values[ID_IMU_DESTINATION_PLANE].pose)*T_aruco_final;
    T_0_pre_final=pose_to_homo(Aruco_values[ID_IMU_DESTINATION_PLANE].pose)*T_aruco_pre_final;

    pose_final=homo_to_pose(T_0_final);
    pose_pre_final=homo_to_pose(T_0_pre_final);

    if(!allineamento_w_z_x_y(pose_pre_final)){
      if(!move_to_pose_optimized(pose_pre_final))
        return false;
    }

    remove_box("left_panel");
    remove_and_detach_box(collision_boxes["imu_module"]);
    if(!move_to_pose_optimized(pose_final))
      return false;

    action_gripper("open");
    if(gazebo_bool)
      picked=false;

    //Mi allontano tornando a dove ero prima

    if(!move_to_pose_optimized(pose_pre_final))
      return false;

    add_box(collision_boxes["left_panel"]);
    PosizioniBase(str_pos_iniziale);




}
bool solleva_coperchio(){

  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }

  ROS_INFO("ARUCO TROVATO, POSSO ANDARE A PRENDERE LA COVER");
  Pose p_0_aruco=Aruco_values[ID_INSPECTION_WINDOW_COVER].pose;
  vector<double> joint_group_positions=pos_joint_iniziale_cam_alta;

  move_to_joints(joint_group_positions);

  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);

  move_to_joints(joint_group_positions);

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE ALLA COVER");

  Affine3d T_aruco_final01,T_aruco_final,T_aruco_final02,T_0_final,T_0_final01,T_0_final02, T_0_aruco;
  Pose pose_final01,pose_final02,pose_final03,pose_final;
  T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);




  T_aruco_final.translation().x()=0.0499;//0.05
  T_aruco_final.translation().y()=0.0249-(0.096+0.0125+0.03);//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final.translation().z()=0.017 + 0.05;//0.015
  T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
  T_0_final=T_0_aruco*T_aruco_final;
  pose_final=homo_to_pose(T_0_final);



  pose_final01=robot->getCurrentPose().pose;
  pose_final01.orientation=pose_final.orientation;
  //stampa_Pose(pose_final02);


  if(!move_to_pose_optimized(pose_final01)){
      return false;
  }

  pose_final02=robot->getCurrentPose().pose;
  pose_final02.position.z=pose_final.position.z;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final02)){
      return false;
  }

  pose_final03=robot->getCurrentPose().pose;
  pose_final03.position.x=pose_final.position.x;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final03)){
      return false;
  }


  //stampa_Pose(pose_final);
  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
//  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER)){
//    T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);
//    T_0_final=T_0_aruco*T_aruco_final;
//    pose_final=homo_to_pose(T_0_final);
//  }

  //scendi per afferrare
  Affine3d T_aruco_final_avvicinato,T_0_final_avvicinato;
  Pose pose_final_avvicinato;

  T_aruco_final_avvicinato.translation().x()=0.0499;//0.05
  T_aruco_final_avvicinato.translation().y()=0.0249-(0.096+0.0125+0.03);//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final_avvicinato.translation().z()=0.017;//132+35/2=149.5
  T_aruco_final_avvicinato.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
  T_0_final_avvicinato=T_0_aruco*T_aruco_final_avvicinato;
  pose_final_avvicinato=homo_to_pose(T_0_final_avvicinato);

  remove_box("inspection_box_base_coperchio");
  remove_box("inspection_box_pomello_coperchio");
  sleep(3);
  if(!move_to_pose_optimized(pose_final_avvicinato)){
      return false;
  }

  if(!gazebo_bool)
    action_gripper("semi_close");

  ROS_INFO("IL coperchio e' stato afferrato, lo sollevo e lo appoggio nel punto corretto");


  //aggiungo la collisione del coperchio solidale
  {
    string box_name_coperchio="inspection_box_base_coperchio";
    string box_name_pomello="inspection_box_pomello_coperchio";
    Affine3d T_0_pomello,T_0_coperchio,T_0_tool,T_tool_coperchio,T_tool_pomello;
    Pose pose_pomello_wrt_ee,pose_coperchio_wrt_ee;//pose pomello with respect to end effector.....
    T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
    T_0_pomello=pose_to_homo(collision_boxes[box_name_pomello].pose.pose);
    T_0_coperchio=pose_to_homo(collision_boxes[box_name_coperchio].pose.pose);
    T_tool_coperchio=T_0_tool.inverse()*T_0_coperchio;
    T_tool_pomello=T_0_tool.inverse()*T_0_pomello;

    pose_pomello_wrt_ee=homo_to_pose(T_tool_pomello);
    pose_coperchio_wrt_ee=homo_to_pose(T_tool_coperchio);

    collision_boxes[box_name_coperchio].pose.header.frame_id="ee_link";
    collision_boxes[box_name_pomello].pose.header.frame_id="ee_link";

    collision_boxes[box_name_coperchio].pose.pose=pose_coperchio_wrt_ee;
    collision_boxes[box_name_pomello].pose.pose=pose_pomello_wrt_ee;

    add_and_attach_box(collision_boxes[box_name_coperchio]);
    add_and_attach_box(collision_boxes[box_name_pomello]);

  }



  //mi alzo

  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
  //VADO AD APPOGGIARE COPERCHIO


  Affine3d T_aruco_final_storage,T_aruco_final_storage_vicino,T_0_aruco_storage,T_0_final_storage,T_0_final_storage_vicino;
  Pose pose_final_storage,pose1,pose2,pose_saved1,pose_saved2,pose_saved3,pose_final_storage_vicino;
  T_0_aruco_storage=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);


  T_aruco_final_storage.translation().x()=0.075;
  T_aruco_final_storage.translation().y()=-0.232;
  T_aruco_final_storage.translation().z()=0.1;
  //T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);

  T_0_final_storage=T_0_aruco_storage*T_aruco_final_storage;
  pose_final_storage=homo_to_pose(T_0_final_storage);
/*
  pose_saved1=robot->getCurrentPose().pose;
  pose1=robot->getCurrentPose().pose;
  pose1.position.x=pose_final_storage.position.x;
  pose1.position.y=pose_final_storage.position.y;
  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  pose2=robot->getCurrentPose().pose;
  pose2.orientation=pose_final_storage.orientation;
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      ROS_INFO("MOVIMENTO FALLITO");
      return false;
    }
  }

  //movimento pre finale
  if(!move_to_pose_cartesian(pose_final_storage)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }
  */

  if(!allineamento_x_y_w_z(pose_final_storage)){
    if(!move_to_pose_optimized(pose_final_storage))
      return false;
  }

  action_gripper("open");
  remove_and_detach_box(collision_boxes["inspection_box_base_coperchio"]);
  remove_and_detach_box(collision_boxes["inspection_box_pomello_coperchio"]);

  ROS_INFO("FUNCTION COMPLETE");
  return true;
}
bool action_aruco_button(string ID_str){
  stringstream ss;
  int ID_int;

  ss<<ID_str;
  ss >> ID_int;

  if(!Aruco_values[ID_int].valid){//se non è mai stato trovato l'aruco di interesse

    if(!esplorazione_middle_panel_per_trovare_aruco(ID_str)){//qui inizia a cercare l'aruco
        ROS_INFO("ARUCO NON TROVATO, ID:%d",ID_int);
      return false;
    }

  }
  ROS_INFO("Aruco trovato, vado a premere il pulsante");


  PosizioniBase(str_pos_iniziale_cam_alta);
  action_gripper("close");



  Affine3d T_aruco_finalpos,T_0_finalpos_pregrasp, T_0_aruco;

  T_0_aruco=pose_to_homo(Aruco_values[ID_int].pose);


  //stampa_Pose(homo_to_pose(T_0_aruco));


  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=-0.055; //-0.055 per la gara
  T_aruco_finalpos.translation().z()=0.20; //23+135+12,5+sicurezza=170,5mm + sicurezza=0.1705 metri + 0.03=0.20
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0);//*from_rpy_to_rotational_matrix(M_PI,0,0);


  T_0_finalpos_pregrasp=T_0_aruco*T_aruco_finalpos;
  //In questo punto ho T_0_final

  Pose pose_final_pose_pregrasp=homo_to_pose(T_0_finalpos_pregrasp);

  //AVVICINAMENTO
  Pose pose1,pose2,pose3;
  {
    pose1=robot->getCurrentPose().pose;
    pose1.orientation=pose_final_pose_pregrasp.orientation;
    if(!move_to_pose_optimized(pose1)){
        return false;
    }

    pose2=robot->getCurrentPose().pose;
    pose2.position.x=pose_final_pose_pregrasp.position.x;
    pose2.position.y=pose_final_pose_pregrasp.position.y;
    if(!move_to_pose_optimized(pose2)){
        return false;
    }

    pose3=robot->getCurrentPose().pose;
    pose3.position.z=pose_final_pose_pregrasp.position.z;
    if(!move_to_pose_optimized(pose3)){
        return false;
    }


  }



  remove_box("button_"+ID_str);
  remove_box("middle_panel");
  //remove_box("gripper_box");


  if(!move_to_pose_optimized(pose_final_pose_pregrasp)){
      return false;
  }




  //POSIZIONE PER PREMERE PULSANTE
  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=-0.055; //-0.055 per la gara
  T_aruco_finalpos.translation().z()=0.16;//0.18 È GIUSTO //17+135+12,5=164.5 mm=0.164m    - in teoria 1mm di spessore aruco=0.163
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0);//*from_rpy_to_rotational_matrix(M_PI,0,0);


  Affine3d T_0_finalpos_premuto=T_0_aruco*T_aruco_finalpos;
  //In questo punto ho T_0_final

  Pose pose_final_pose_premuto=homo_to_pose(T_0_finalpos_premuto);

  ros::NodeHandle n;
  n.setParam("freno_a_mano_buttons",true);

  move_to_pose_optimized(pose_final_pose_premuto);
  sleep(2);


  n.setParam("freno_a_mano_buttons",false);

  //RITORNO INDIETRO-------------------


  if(!move_to_pose_optimized(pose_final_pose_pregrasp)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  add_box(collision_boxes["button_"+ID_str]);
  add_box(collision_boxes["middle_panel"]);
  //add_and_attach_box(collision_boxes["gripper_box"]);

  if(!move_to_pose_optimized(pose3)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  if(!move_to_pose_optimized(pose2)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  if(!move_to_pose_optimized(pose1)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }


  PosizioniBase(str_pos_iniziale_cam_alta);

  //add_box(collision_boxes["button_"+ID_str]);
  action_gripper("open");

  return true;
}
bool right_panel(){
  action_gripper("open");

  //Se necessario fa le esplorazioni
  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid){
    if(!esplora_inspection_cover_storage())
      return false;
  }
  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }


  if(!solleva_coperchio()) return false;

  Affine3d T_aruco_final_storage,T_aruco_final_storage_vicino,T_0_aruco_storage,T_0_final_storage,T_0_final_storage_vicino;
  Pose pose_final_storage,pose1,pose2,pose_saved1,pose_saved2,pose_saved3,pose_final_storage_vicino;
  T_0_aruco_storage=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);


  T_aruco_final_storage.translation().x()=0;
  T_aruco_final_storage.translation().y()=0;
  T_aruco_final_storage.translation().z()=0.2;
  T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final_storage=T_0_aruco_storage*T_aruco_final_storage;
  pose_final_storage=homo_to_pose(T_0_final_storage);

  pose_saved1=robot->getCurrentPose().pose;
  pose1=robot->getCurrentPose().pose;
  pose1.position.x=pose_final_storage.position.x;
  pose1.position.y=pose_final_storage.position.y;
  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  pose2=robot->getCurrentPose().pose;
  pose2.orientation=pose_final_storage.orientation;
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      ROS_INFO("MOVIMENTO FALLITO");
      return false;
    }
  }

  //movimento pre finale
  if(!move_to_pose_cartesian(pose_final_storage)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  if(!gazebo_bool){
    T_aruco_final_storage_vicino.translation().x()=0;
    T_aruco_final_storage_vicino.translation().y()=0;
    T_aruco_final_storage_vicino.translation().z()=0.05;
    T_aruco_final_storage_vicino.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
    T_0_final_storage_vicino=T_0_aruco_storage*T_aruco_final_storage_vicino;
    pose_final_storage_vicino=homo_to_pose(T_0_final_storage_vicino);

    if(!move_to_pose_cartesian(pose_final_storage_vicino)){
      if(!move_to_pose(pose_final_storage_vicino,true)){
        return false;
      }
    }
  }
  action_gripper("open");
  //TORNIAMO INDIETRO
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      return false;
    }
  }

  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  if(!move_to_pose_cartesian(pose_saved1)){
    if(!move_to_pose(pose_saved1,true)){
      return false;
    }
  }

  int cont=0;
  int aruco_nascosto_trovato=-1;
  arm_control::aruco_serviceResponse msg=bridge_service(str_md_rd,"");
  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true){
      cont++;
      if(i>0 && i<10){
        ROS_INFO("ARUCO NASCOSTO TROVATO, ID: %d",i);
        aruco_nascosto_trovato=i;
      }
    }
}
  if(aruco_nascosto_trovato!=-1){
    //DEVE PRIMA RITORNARE INDIETRO E POI PREMERE IL PULSANTE. MANCA LA PARTE IN CUI TORNA INDIETRO
    action_aruco_button(to_string(aruco_nascosto_trovato));

  }
  else{
    ROS_INFO("Vedo %d elementi ma nessun elemento corrispondente ai pulsanti da premere",cont);
    //devo cambiare posizione
  }
  return true;


{
//  Affine_valid T_0_aruco_valid_panel=homo_0_aruco_elaration();
//  if(! T_0_aruco_valid_panel.valid){
//    //Devo cambiare posizione

//    return false;
//  }



//  //ARUCO 13 TROVATO


//  Affine3d T_aruco_finalpos_panel,T_0_finalpos_panel, T_0_aruco_panel;
//  Pose pose_final_pose_panel;
//  T_0_aruco_panel=T_0_aruco_valid_panel.homo_matrix;


//  T_aruco_finalpos_panel.translation().x()=0.05;//0.05
//  T_aruco_finalpos_panel.translation().y()=-0.22;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
//  T_aruco_finalpos_panel.translation().z()=0.015;//0.015
//  //T_aruco_finalpos_panel.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(-M_PI/2,0,0);
//  T_aruco_finalpos_panel.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);

//  //CONOSCO LA MIA POSIZIONE PER RAGGIUNGERE IL BLOCCO
//  //CERCO DI AVVICINARMI
//  vector<double> joint_group_positions;
//  double alpha=atan2(pose_final_pose_panel.position.y,pose_final_pose_panel.position.x);
//  joint_group_positions=pos_joint_iniziale_cam_alta;
//  joint_group_positions[0] = alpha;  // radians
//  robot->setJointValueTarget(joint_group_positions);
//  success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//  robot->move();


//  //Ora dovrei essere orientato correttamente, ora mi posiziono con la corretta 'z'


//  Pose p2=robot->getCurrentPose().pose;
//  p2.position.z=pose_final_pose_panel.position.z;
//  if(!move_to_pose_cartesian(p2)){
//    move_to_pose(p2,true);
//  }


//  //ORA POSSO FINALMENTE ANDARE AL INSPECTION WINDOW


//  if(!move_to_pose_cartesian(pose_final_pose_panel)){
//    move_to_pose(pose_final_pose_panel,true);
//  }


//  //MI AVVICINO ALL INSPECTION

//  T_aruco_finalpos_panel.translation().y()=-0.107;//0.025 - (0.132)=-0.107
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);

//  sleep(1);
//  action_gripper("semi_close");

//  //AFFERRO


//  T_aruco_finalpos_panel.translation().z()=0.06;
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);


//  T_aruco_finalpos_panel.translation().y()=-0.22;
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);


//  //PRE RAGGIUNGIMENTO INSPECTION COVER STORAGE

//  {
//    vector<double> joint_group_positions=pos_joint_iniziale;
//    joint_group_positions[0] = joint_group_positions[0]- grad_to_rad(35);  // radians
//    joint_group_positions[3] = grad_to_rad(70);  // radians
//    joint_group_positions[5] = grad_to_rad(0);  // radians
//    robot->setJointValueTarget(joint_group_positions);
//    success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//    robot->move();
//  }
///*
//  bridge_service(str_md_next_aruco,"14");
//  sleep(2);
//  ruota_e_cerca_aruco();
//  sleep(2);//ATTENTO UNO SLEEP, CREDO SIA UTILE PERCHÈ SENNÒ IL BLOCCO SULLA ROTAZIONE INFLUISCE SUL PROSSIMO MOVIMENTO


//  Affine_valid T_0_aruco_valid_ground=homo_0_aruco_elaration();
//  if(! T_0_aruco_valid_ground.valid){
//    //Devo cambiare posizione

//    return false;
//  }


//  Affine3d T_aruco_finalpos_ground,T_0_finalpos_ground, T_0_aruco_ground;
//  T_0_aruco_ground=T_0_aruco_valid_ground.homo_matrix;
//*/
//  Affine3d T_aruco_finalpos_ground,T_0_finalpos_ground, T_0_aruco_ground;
//  Pose pose_final_pose_ground;
//  T_0_aruco_ground=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);

//  T_aruco_finalpos_ground.translation().x()=0;
//  T_aruco_finalpos_ground.translation().y()=0;
//  T_aruco_finalpos_ground.translation().z()=0.2;
//  T_aruco_finalpos_ground.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
//  T_0_finalpos_ground=T_0_aruco_ground*T_aruco_finalpos_ground;
//  pose_final_pose_ground=homo_to_pose(T_0_finalpos_ground);
//  stampa_Pose(pose_final_pose_ground);
//  stampa_homo(T_aruco_finalpos_ground);

//  if(!move_to_pose_cartesian(pose_final_pose_ground)){
//    move_to_pose(pose_final_pose_ground,true);
//  }


//  action_gripper("open");


//  PosizioniBase(str_pos_iniziale);
//  action_gripper("open");



//  return true;
}

}
bool go_and_recognize_id_in_inspection_window(){

  PosizioniBase(str_pos_iniziale_cam_alta);
  action_gripper("open");

  Affine3d T_0_scatola,T_scatola_final,T_0_final;
  T_0_scatola=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);

  T_scatola_final.translation().x()=0.05;
  T_scatola_final.translation().y()=-0.07;
  T_scatola_final.translation().z()=0.07;
  T_scatola_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0);

  T_0_final=T_0_scatola*T_scatola_final;

  remove_box("inspection_box_base_coperchio");
  remove_box("inspection_box_pomello_coperchio");
  if(!move_to_pose_optimized(homo_to_pose(T_0_final))){

    return false;

  }


  int cont=0;
  int aruco_nascosto_trovato=-1;
  arm_control::aruco_serviceResponse msg=bridge_service(str_md_rd,"");
  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true){
      cont++;
      if(i>0 && i<10){
        ROS_INFO("ARUCO NASCOSTO TROVATO, ID: %d",i);
        aruco_nascosto_trovato=i;
      }
    }
}
  if(aruco_nascosto_trovato!=-1){
    //DEVE PRIMA RITORNARE INDIETRO E POI PREMERE IL PULSANTE. MANCA LA PARTE IN CUI TORNA INDIETRO

    ros::NodeHandle n;
    n.setParam("hidden_id",to_string(aruco_nascosto_trovato));
  }
  else{
    ROS_INFO("Vedo %d elementi ma nessun elemento corrispondente ai pulsanti da premere",cont);
    //devo cambiare posizione
  }
  return true;

}

