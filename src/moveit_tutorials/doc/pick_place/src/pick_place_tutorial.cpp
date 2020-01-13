
// ROS
#include <ros/ros.h>
#include <cstring>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

std::string tableFilePath = "file:///home/elif/robotic_project/src/moveit_tutorials/doc/pick_place/src/TableSquareLegs_stl.stl";

std::string cupFilePath = "file:///home/elif/robotic_project/src/moveit_tutorials/doc/pick_place/src/Lowpoly_Cup.stl";

std::string burgerFilePath = "file:///home/elif/robotic_project/src/moveit_tutorials/doc/pick_place/src/burger.stl";

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, int choice)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  if(choice ==1)
    grasps[0].grasp_pose.pose.position.y = -0.2;
  else
      grasps[0].grasp_pose.pose.position.y = 0.3;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;


  openGripper(grasps[0].pre_grasp_posture);

  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");

  if(choice==1)
    move_group.pick("object", grasps);
  else
    move_group.pick("object2", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group, int choice)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  float xValue = 0.0;
  float zValue = 0.4;
  if (choice != 1)
  {
    xValue = 0.3;
  }

  place_location[0].place_pose.pose.position.x = xValue;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = zValue;

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("table2");
  if (choice == 1)
    group.place("object", place_location);
  else
    group.place("object2", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  using namespace Eigen; 

  Vector3d b1(0.2, 0.2, 0.35);
  Vector3d b2(0.0001, 0.0001, 0.0001);
  Vector3d b3(0.002, 0.002, 0.002);
  moveit_msgs::CollisionObject co1;
  moveit_msgs::CollisionObject co;
    
  shape_msgs::Mesh mesh1;
  shapes::ShapeMsg mesh_msg1;
  shape_msgs::Mesh mesh2;
  shapes::ShapeMsg mesh_msg2; 

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.7;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;

  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.7;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;

  collision_objects[1].operation = collision_objects[1].ADD;

// object

    co.id = "object";
    shapes::Mesh* m1 = shapes::createMeshFromResource(burgerFilePath,b3); 
    ROS_INFO("mesh loaded");

    shapes::constructMsgFromShape(m1, mesh_msg1);
    mesh1 = boost::get<shape_msgs::Mesh>(mesh_msg1);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh1;
    co.header.frame_id = "panda_link0";
    co.mesh_poses[0].position.x = 0.5;
    co.mesh_poses[0].position.y = -0.2;
    co.mesh_poses[0].position.z = 0.375;
    
    co.mesh_poses[0].orientation.w= 0.0; 
    co.mesh_poses[0].orientation.x= 0.0; 
    co.mesh_poses[0].orientation.y= 0.0;
    co.mesh_poses[0].orientation.z= 0.0;   

    co.meshes.push_back(mesh1);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    collision_objects.push_back(co);

// object2

    co1.id = "object2";
    shapes::Mesh* m2 = shapes::createMeshFromResource(cupFilePath,b2); 
    ROS_INFO("mesh loaded");

    shapes::constructMsgFromShape(m2, mesh_msg2);
    mesh2 = boost::get<shape_msgs::Mesh>(mesh_msg2);

    co1.meshes.resize(1);
    co1.mesh_poses.resize(1);
    co1.meshes[0] = mesh2;
    co1.header.frame_id = "panda_link0";
    co1.mesh_poses[0].position.x = 0.5;
    co1.mesh_poses[0].position.y = 0.3;
    co1.mesh_poses[0].position.z = 0.375;
    
    co1.mesh_poses[0].orientation.w= 0.0; 
    co1.mesh_poses[0].orientation.x= 0.0; 
    co1.mesh_poses[0].orientation.y= 0.0;
    co1.mesh_poses[0].orientation.z= 0.0;   

    co1.meshes.push_back(mesh2);
    co1.mesh_poses.push_back(co1.mesh_poses[0]);
    co1.operation = co1.ADD;
    collision_objects.push_back(co1);

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int isItemValid(std::string urun)
{
  if(urun == "hamburger")
    return 1;
  else if(urun == "gazoz")
    return 2;
  else if (urun == "cik")
    return 3;
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  std::string urun;
  int choice = 0;
  while(true){
  do{
     std::cout<<"Solda taraftaki urun icin hamburger, sag taraftaki urun icin gazoz yaziniz"<<std::endl << "Cikmak için cik yaziniz" << std::endl;
     std::cin>>urun;
     choice = isItemValid(urun);
     if(!choice)
        std::cout<<"Urun bulunamamistir"<<std::endl;
  }while(!choice);

  if(choice==3)
        break;
  else
    pick(group, choice);

  ros::WallDuration(1.0).sleep();

  place(group,choice);
  }

  std::cout<<"Ros kapanisi bekleniyor"<<std::endl;
  ros::waitForShutdown();
  return 0;
}

