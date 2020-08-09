/* Author: Akash 
Publish collision objects in planning scene */

#include <moveit/move_group_interface/move_group_interface.h>      
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/console.h>
#include <ros/console.h>

#include "tf/LinearMath/Transform.h"

#include "octomap_msgs/conversions.h"

#define NUM_OF_OBSTACLES (3)   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_planning_scene");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();

    ros::Publisher planning_scene_diff_publisher_ = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    /*          Code for adding collision object in planning scene from STL file       */
    moveit_msgs::CollisionObject co_msu_object;
    co_msu_object.meshes.resize(NUM_OF_OBSTACLES);        //Resize by number of STL meshes to be added to planning scene
    co_msu_object.mesh_poses.resize(NUM_OF_OBSTACLES);

    shapes::Mesh* msu_object = shapes::createMeshFromResource("package://put_to_wall_manipulation/meshes/objects/Octagon_Structure_V1.stl"); 
    shape_msgs::Mesh msu_object_mesh ;
    shapes::ShapeMsg msu_object_mesh_msg ;  
    shapes::constructMsgFromShape(msu_object, msu_object_mesh_msg) ;    
    msu_object_mesh = boost::get<shape_msgs::Mesh>(msu_object_mesh_msg) ;  

    shapes::Mesh* base_cover_object = shapes::createMeshFromResource("package://put_to_wall_manipulation/meshes/objects/base_cover.stl"); 
    shape_msgs::Mesh base_cover_object_mesh ;
    shapes::ShapeMsg base_cover_object_mesh_msg ;  
    shapes::constructMsgFromShape(base_cover_object, base_cover_object_mesh_msg) ;    
    base_cover_object_mesh = boost::get<shape_msgs::Mesh>(base_cover_object_mesh_msg) ;  

    shapes::Mesh* robot_base_object = shapes::createMeshFromResource("package://put_to_wall_manipulation/meshes/objects/robot_base.stl"); 
    shape_msgs::Mesh robot_base_object_mesh ;
    shapes::ShapeMsg robot_base_object_mesh_msg ;  
    shapes::constructMsgFromShape(robot_base_object, robot_base_object_mesh_msg) ;    
    robot_base_object_mesh = boost::get<shape_msgs::Mesh>(robot_base_object_mesh_msg) ;  

    co_msu_object.meshes[0] = msu_object_mesh ;   
    co_msu_object.meshes[1] = robot_base_object_mesh ;                     
    co_msu_object.meshes[2] = base_cover_object_mesh ;     
    
    co_msu_object.header.frame_id = "/world" ;
    co_msu_object.id = "Put_to_Wall_Structure" ;   
    
    //This breakout of co-ordinates is due to origin 
    //of MSU cad being at center of its body and all the 
    //calculations are being made from center of planning scene   

    co_msu_object.mesh_poses[0].position.x = 0.0 ;
    co_msu_object.mesh_poses[0].position.y = 0.0 ;
    co_msu_object.mesh_poses[0].position.z = 0.0 ;  
    co_msu_object.mesh_poses[0].orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0) ;  

    co_msu_object.mesh_poses[1].position.x = 0.0 ;
    co_msu_object.mesh_poses[1].position.y = 0.0 ;
    co_msu_object.mesh_poses[1].position.z = 1.22 ;  
    co_msu_object.mesh_poses[1].orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0) ;  

    co_msu_object.mesh_poses[2].position.x = 0.0 ;
    co_msu_object.mesh_poses[2].position.y = 0.0 ;
    co_msu_object.mesh_poses[2].position.z = 0.3 ;  
    co_msu_object.mesh_poses[2].orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0) ;  

    co_msu_object.operation = co_msu_object.ADD;

    //ALTERNATE WAY OF ADDING COLLISON OBJECTS TO PLANNING SCENE
    moveit_msgs::PlanningScene planning_scene_add;
    planning_scene_add.world.collision_objects.push_back(co_msu_object);
    planning_scene_add.is_diff = true;
     

    //planning_scene_diff_publisher_.publish(planning_scene_add);
    moveit_msgs::ApplyPlanningScene srv1;
    srv1.request.scene = planning_scene_add;
    planning_scene_diff_client.call(srv1);

    ROS_INFO("Put to Wall structure published to planning scene");

    moveit_msgs::GetPlanningScene planning_srv_name;
    planning_srv_name.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    ros::ServiceClient client_get_scene_ = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    client_get_scene_.waitForExistence();

    bool msu_in_world = false;
    
    while(msu_in_world==false)
    {
        ROS_INFO("waiting for objects to appear in planning scene");
        if (client_get_scene_.call(planning_srv_name))
        {
            for (int i = 0; i < (int)planning_srv_name.response.scene.world.collision_objects.size(); ++i)
            {
                if (planning_srv_name.response.scene.world.collision_objects[i].id == co_msu_object.id){
                       msu_in_world = true;
                }       
            }
          }
    }

    ROS_INFO("Collision object Put to Wall structure added in planning scene");  

    ros::shutdown();
    return 0;
}
