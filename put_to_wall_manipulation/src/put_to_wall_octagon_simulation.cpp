#include <moveit/move_group_interface/move_group_interface.h>     
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetPlanningScene.h>

static const std::string PLANNING_GROUP = "manipulator";     

#define EXECUTE_PATHS

#define NUM_OF_POINTS (120)

#define OCTAGON_DIMENSIONS_MIN_X (1.2)
#define OCTAGON_DIMENSIONS_MIN_X_UPPER_BIN (1.0)
#define OCTAGON_DIMENSIONS_MAX_X (1.0)
#define OCTAGON_X_INCREAMENTS (0.2)
#define VERTICAL_BINS_IN_OCTAGON (3)
#define HORIZONTAL_BINS_IN_OCTAGON (2)

#define OCTAGON_DIMENSIONS_MIN_Y (0.55 + 0.3)
#define OCTAGON_DIMENSIONS_MAX_Y (0.8)
#define OCTAGON_Y_INCREAMENTS (0.55)

#define OCTAGON_DIMENSIONS_MIN_Z (0.18)
#define OCTAGON_Z_INCREAMENTS (0.60)      //0.15 Added for center of put bin
#define DROP_POINT_ABOVE_CHUTE (0.20)

#define INTER_WALL_SEPERATION (0.1)

#define UPPER_BINS_RANGE (1.6)
#define LOWER_BINS_RANGE (0.8)

#define NUMBER_OF_GRASP_POSES (1)

#define GRASP_POSES_X_AXIS_MIN (-0.7)
#define GRASP_POSES_X_AXIS_MAX (0.7)
#define GRASP_POSES_X_AXIS_RESOLUTION (0.1)

#define GRASP_POSES_Y_AXIS_MIN (-0.7)
#define GRASP_POSES_Y_AXIS_MAX (0.7)
#define GRASP_POSES_Y_AXIS_RESOLUTION (0.1)

#define GRASP_POSES_Z_AXIS (0.1)
#define GRASP_POSES_PITCH_RESOLUTION (1.57/5)

#define UPPER_BINS_DROP_PITCH (-0.52)
#define LOWER_BINS_DROP_PITCH (0.52)

#define PRE_GRASP_POSE_OFFSET (0.05)

#define BARCODE_SCAN_POINT_Z_AXIS (0.6)

#define PRE_DROP_POSE_X_OFFSET (OCTAGON_DIMENSIONS_MIN_X/3)
#define PRE_DROP_POSE_Z_OFFSET (0.2)

std::vector<double> robot_barcode_point_to_drop_point_execution_time ,robot_drop_point_to_barcode_point_execution_time ;
double barcode_point_to_drop_point_total_robot_execution_time = 0 , drop_point_to_barcode_point_total_robot_execution_time = 0;
int barcode_point_to_drop_point_failed_plans = 0 , barcode_point_to_drop_point_successful_plans = 0;
int drop_point_to_barcode_point_failed_plans = 0 , drop_point_to_barcode_point_successful_plans = 0;

std::vector<double> robot_barcode_point_to_grasp_point_execution_time ,robot_grasp_point_to_barcode_point_execution_time ;
double barcode_point_to_grasp_point_total_robot_execution_time = 0 , grasp_point_to_barcode_point_total_robot_execution_time = 0;
int barcode_point_to_grasp_point_failed_plans = 0 , barcode_point_to_grasp_point_successful_plans = 0;
int grasp_point_to_barcode_point_failed_plans = 0 , grasp_point_to_barcode_point_successful_plans = 0;

void 
computeOctagonWallPoses(geometry_msgs::PoseArray &single_plane_points,double yaw)
{
    for(int points_cnt = 0 ; points_cnt < (VERTICAL_BINS_IN_OCTAGON*HORIZONTAL_BINS_IN_OCTAGON) ; points_cnt++)
    {
        geometry_msgs::Pose wall_pose;
        wall_pose.position.x = single_plane_points.poses[points_cnt].position.x ;
        wall_pose.position.y = single_plane_points.poses[points_cnt].position.y ;
        wall_pose.position.z = single_plane_points.poses[points_cnt].position.z ;
        wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw); 
        tf::Matrix3x3 rot(tf::Quaternion(wall_pose.orientation.x,wall_pose.orientation.y,wall_pose.orientation.z,wall_pose.orientation.w));
        tf::Vector3 point(wall_pose.position.x,wall_pose.position.y,wall_pose.position.z);
        tf::Vector3 point_rot = rot * point;
        wall_pose.position.x = point_rot.x();
        wall_pose.position.y = point_rot.y();
        wall_pose.position.z = point_rot.z();
        if(wall_pose.position.z <= LOWER_BINS_RANGE)
        {
            wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,LOWER_BINS_DROP_PITCH,yaw); 
        }
        else if (wall_pose.position.z >= UPPER_BINS_RANGE)
        {
            wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,UPPER_BINS_DROP_PITCH,yaw); 
        }
        else
        {
            wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,UPPER_BINS_DROP_PITCH,yaw); 
        }

        single_plane_points.poses.push_back(wall_pose);
    }
}

void 
generateOctagonPoses(geometry_msgs::PoseArray &octagonal_poses)
{
    geometry_msgs::PoseArray single_plane_points;

    //Compute single face points first then translate and rotate points in octagonal shape
    for(int points_z_axis_cnt = 0 ; points_z_axis_cnt < VERTICAL_BINS_IN_OCTAGON ; points_z_axis_cnt++)
    {
        for(int points_y_axis_cnt = 0 ; points_y_axis_cnt < HORIZONTAL_BINS_IN_OCTAGON ; points_y_axis_cnt++)
        {
            geometry_msgs::Pose wall_pose;
            wall_pose.position.x = OCTAGON_DIMENSIONS_MIN_X;
            wall_pose.position.y = OCTAGON_Y_INCREAMENTS + (points_y_axis_cnt * OCTAGON_Y_INCREAMENTS) - OCTAGON_DIMENSIONS_MIN_Y ;
            wall_pose.position.z = OCTAGON_Z_INCREAMENTS + (points_z_axis_cnt * OCTAGON_Z_INCREAMENTS) ;
            if(wall_pose.position.z <= LOWER_BINS_RANGE)
            {
                wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,LOWER_BINS_DROP_PITCH,0.0); 
            }
            else if (wall_pose.position.z >= UPPER_BINS_RANGE)
            {
                wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,UPPER_BINS_DROP_PITCH,0.0); 
            }
            else
            {
                wall_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,UPPER_BINS_DROP_PITCH,0.0); 
            }
            wall_pose.position.z += DROP_POINT_ABOVE_CHUTE;
            single_plane_points.poses.push_back(wall_pose);
        }
    }

    computeOctagonWallPoses(single_plane_points,0.78);
    computeOctagonWallPoses(single_plane_points,1.57);
    computeOctagonWallPoses(single_plane_points,2.34);
    computeOctagonWallPoses(single_plane_points,3.14);
    computeOctagonWallPoses(single_plane_points,-0.78);
    computeOctagonWallPoses(single_plane_points,-1.57);
    computeOctagonWallPoses(single_plane_points,-2.34);

    for(int pose_cnt = 0 ; pose_cnt < single_plane_points.poses.size() ; pose_cnt++)
    {
        octagonal_poses.poses.push_back(single_plane_points.poses[pose_cnt]);
    }
}

void 
generateGraspPoses(geometry_msgs::PoseArray &grasp_poses,double min_pitch,double max_pitch)
{
    for(double x = GRASP_POSES_X_AXIS_MIN ; x <= GRASP_POSES_X_AXIS_MAX ;)
    {
        for(double y = GRASP_POSES_Y_AXIS_MIN ; y <= GRASP_POSES_Y_AXIS_MAX ;)
        {
            geometry_msgs::Pose grasp_pose;
            grasp_pose.position.x = x;
            grasp_pose.position.y = y;
            grasp_pose.position.z = GRASP_POSES_Z_AXIS;
            grasp_pose.orientation.x = -0.498991;
            grasp_pose.orientation.y = 0.498697;
            grasp_pose.orientation.z = 0.500991;
            grasp_pose.orientation.w = 0.501316;
            // for(double pitch_poses = min_pitch ; pitch_poses <= max_pitch ;)
            // {
                // grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,pitch_poses,0.0);
            grasp_poses.poses.push_back(grasp_pose); 
                // pitch_poses += GRASP_POSES_PITCH_RESOLUTION;
            // }
            y += GRASP_POSES_Y_AXIS_RESOLUTION;
        }
        x += GRASP_POSES_X_AXIS_RESOLUTION;
    }
}

std::string
addGraspObject(moveit::planning_interface::MoveGroupInterface &move_group,geometry_msgs::Pose robot_pose,
       ros::ServiceClient &client_get_planning_scene,ros::ServiceClient &planning_scene_diff_client)
{
    moveit_msgs::GetPlanningScene planning_srv_name;
    moveit_msgs::PlanningScene planning_scene_add;
    moveit_msgs::ApplyPlanningScene srv1;

    planning_srv_name.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "grasp_object";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.05;

    geometry_msgs::Pose box_pose;
    box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    box_pose.position = robot_pose.position;
    box_pose.position.z -= 0.06;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    bool grasp_object_in_world = false;
    
    while(grasp_object_in_world==false)
    {
        ROS_INFO("Searching for grasp object to appear in planning scene");
        if (client_get_planning_scene.call(planning_srv_name))
        {
            for (int i = 0; i < (int)planning_srv_name.response.scene.world.collision_objects.size(); ++i)
            {
                if (planning_srv_name.response.scene.world.collision_objects[i].id == collision_object.id)
                {
                    ROS_INFO("Got grasp object in planning scene.Removing!");
                    grasp_object_in_world = true;

                    collision_object.operation = collision_object.REMOVE;

                    planning_scene_add.world.collision_objects.clear();
                    planning_scene_add.world.collision_objects.push_back(collision_object);
                    planning_scene_add.is_diff = true;

                    srv1.request.scene = planning_scene_add;
                    planning_scene_diff_client.call(srv1);
                    sleep(2.0);
                    break ;
                }       
            }
            break;
        }
    }

    collision_object.operation = collision_object.ADD;

    planning_scene_add.world.collision_objects.clear();
    planning_scene_add.world.collision_objects.push_back(collision_object);
    planning_scene_add.is_diff = true;

    srv1.request.scene = planning_scene_add;
    planning_scene_diff_client.call(srv1);
    sleep(2.0);

    ROS_INFO("Added grasp object in planning scene!");

    return collision_object.id;
}

void 
visualizeSuccessfulMarkerPoints(std::string marker_array_name,int marker_id,int marker_action,geometry_msgs::Pose marker_pose,
    visualization_msgs::MarkerArray &marker)
{
    marker.markers[marker_id].header.frame_id = "world";
    marker.markers[marker_id].header.stamp = ros::Time();
    marker.markers[marker_id].ns = marker_array_name;
    marker.markers[marker_id].id = marker_id;
    marker.markers[marker_id].type = visualization_msgs::Marker::SPHERE;
    marker.markers[marker_id].action = marker_action;
    marker.markers[marker_id].pose = marker_pose;
    marker.markers[marker_id].scale.x = 0.08;
    marker.markers[marker_id].scale.y = 0.08;
    marker.markers[marker_id].scale.z = 0.08;
    marker.markers[marker_id].color.a = 1.0;
    marker.markers[marker_id].color.r = 0.0;
    marker.markers[marker_id].color.g = 0.0;
    marker.markers[marker_id].color.b = 1.0;
}

void 
visualizeFailedMarkerPoints(std::string marker_array_name,int marker_id,int marker_action,geometry_msgs::Pose marker_pose,
    visualization_msgs::MarkerArray &marker)
{
    marker.markers[marker_id].header.frame_id = "world";
    marker.markers[marker_id].header.stamp = ros::Time();
    marker.markers[marker_id].ns = marker_array_name;
    marker.markers[marker_id].id = marker_id;
    marker.markers[marker_id].type = visualization_msgs::Marker::SPHERE;
    marker.markers[marker_id].action = marker_action;
    marker.markers[marker_id].pose = marker_pose;
    marker.markers[marker_id].scale.x = 0.08;
    marker.markers[marker_id].scale.y = 0.08;
    marker.markers[marker_id].scale.z = 0.08;
    marker.markers[marker_id].color.a = 1.0;
    marker.markers[marker_id].color.r = 1.0;
    marker.markers[marker_id].color.g = 0.0;
    marker.markers[marker_id].color.b = 0.0;
}

moveit_msgs::RobotTrajectory 
Stich_trajectories(robot_state::RobotState current_robot_state,moveit::planning_interface::MoveGroupInterface& move_group,
                    moveit_msgs::RobotTrajectory traj1,moveit_msgs::RobotTrajectory traj2)
{
    moveit_msgs::RobotTrajectory final_trajectory ; 
    robot_state::RobotState prev_state(*move_group.getCurrentState()) ;

    const double gp_j1=traj1.joint_trajectory.points.front().positions[0];
    const double gp_j2=traj1.joint_trajectory.points.front().positions[1];
    const double gp_j3=traj1.joint_trajectory.points.front().positions[2];
    const double gp_j4=traj1.joint_trajectory.points.front().positions[3];
    const double gp_j5=traj1.joint_trajectory.points.front().positions[4];
    const double gp_j6=traj1.joint_trajectory.points.front().positions[5];
    
    prev_state.setJointPositions("shoulder_joint",&gp_j1);
    prev_state.setJointPositions("upperArm_joint",&gp_j2);
    prev_state.setJointPositions("foreArm_joint",&gp_j3);
    prev_state.setJointPositions("wrist1_joint",&gp_j4);
    prev_state.setJointPositions("wrist2_joint",&gp_j5);
    prev_state.setJointPositions("wrist3_joint",&gp_j6);

    robot_trajectory::RobotTrajectory single_trajectory(move_group.getCurrentState()->getRobotModel(), move_group.getName()) ;
    single_trajectory.setRobotTrajectoryMsg(current_robot_state, traj2);

    robot_trajectory::RobotTrajectory combined_trajectory(move_group.getCurrentState()->getRobotModel(), move_group.getName()) ;
    combined_trajectory.setRobotTrajectoryMsg(prev_state, traj1);
    combined_trajectory.append(single_trajectory,0.1);          
    combined_trajectory.getRobotTrajectoryMsg(final_trajectory);

    return final_trajectory ;
}

moveit_msgs::RobotTrajectory
computeTimeStamps(moveit::planning_interface::MoveGroupInterface& move_group,moveit_msgs::RobotTrajectory source_traj)
{
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);

    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), source_traj);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool suc = iptp.computeTimeStamps(rt,1.0,1.0);

    ROS_INFO("Compute-Time stamp %s",suc?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(source_traj);

    return source_traj;
}

moveit_msgs::RobotTrajectory 
reverseTrajectory(moveit_msgs::RobotTrajectory source_trajectory,moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::RobotTrajectory reversed_trajectory;

    reversed_trajectory.joint_trajectory.joint_names.resize(source_trajectory.joint_trajectory.joint_names.size());
    reversed_trajectory.joint_trajectory.points.resize(source_trajectory.joint_trajectory.points.size());

    for(int joint_names_cnt = 0 ; joint_names_cnt < source_trajectory.joint_trajectory.joint_names.size() ; joint_names_cnt++)
    {
        reversed_trajectory.joint_trajectory.joint_names[joint_names_cnt] = source_trajectory.joint_trajectory.joint_names[joint_names_cnt];
    }

    for(int traj_points = 0 ; traj_points < source_trajectory.joint_trajectory.points.size() ; traj_points++)
    {
        reversed_trajectory.joint_trajectory.points[traj_points] = source_trajectory.joint_trajectory.points[
                                                                    (source_trajectory.joint_trajectory.points.size() - 1) - traj_points];
    }

    return computeTimeStamps(move_group,reversed_trajectory);
}


bool 
planAndMoveToGraspPose(moveit::planning_interface::MoveGroupInterface& move_group,geometry_msgs::Pose grasp_pose,int plan_number)
{
    //Compute cartesian from pre-grasp to grasp pose
//     moveit_msgs::RobotTrajectory cartesian_trajectory;
//     moveit_msgs::MoveItErrorCodes cartesian_path_error_code;
//     std::vector<geometry_msgs::Pose> cartesian_path_waypoints;

//     geometry_msgs::Pose pre_grasp_pose;
//     pre_grasp_pose = grasp_pose;
//     pre_grasp_pose.position.z = BARCODE_SCAN_POINT_Z_AXIS;

//     cartesian_path_waypoints.push_back(pre_grasp_pose);
//     cartesian_path_waypoints.push_back(grasp_pose);

//     double cartesian_fraction = move_group.computeCartesianPath(cartesian_path_waypoints,
//                                                     0.05,  // eef_step
//                                                     3.0,   // jump_threshold            Set it always above 7.14 since avg change between joint space is 0.006
//                                                     cartesian_trajectory, //Trajectories generated
//                                                     false,  //Collision are allowed
//                                                     &cartesian_path_error_code); 

//     ROS_INFO("Cartesian path computed :%f percent Error code:%d",cartesian_fraction*100,cartesian_path_error_code);

//     if(cartesian_fraction > 0.9)
//     {
        
//         cartesian_trajectory = computeTimeStamps(move_group,cartesian_trajectory);

//         moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan ;
//         cartesian_plan.trajectory_ = cartesian_trajectory;
        
//         moveit_msgs::RobotState current_start_state;
//         robot_state::robotStateToRobotStateMsg(*move_group.getCurrentState(),current_start_state);
//         cartesian_plan.start_state_ = current_start_state;

// #ifdef EXECUTE_PATHS
//         move_group.execute(cartesian_plan);
// #endif
//         robot_barcode_point_to_grasp_point_execution_time.push_back(cartesian_plan.trajectory_.joint_trajectory.points
//             [cartesian_plan.trajectory_.joint_trajectory.points.size()-1].time_from_start.toSec() - 
//             cartesian_plan.trajectory_.joint_trajectory.points[0].time_from_start.toSec());
//         barcode_point_to_grasp_point_successful_plans += 1;
//         ROS_INFO("/barcode point to grasp point Planning number %d succeded with exec time : %f",plan_number,
//             robot_barcode_point_to_grasp_point_execution_time.back());

//         return true;
//     }
//     else
//     {
//         ROS_ERROR("Cartesian path failed");
//         barcode_point_to_grasp_point_failed_plans += 1;
//         return false;
//     }
    return true;
}

bool 
planAndMoveToBarcodePose(moveit::planning_interface::MoveGroupInterface& move_group,geometry_msgs::Pose barcode_pose,int plan_number)
{
//     moveit_msgs::RobotTrajectory cartesian_trajectory;
//     moveit_msgs::MoveItErrorCodes cartesian_path_error_code;
//     std::vector<geometry_msgs::Pose> cartesian_path_waypoints;
//     cartesian_path_waypoints.push_back(barcode_pose);

//     double cartesian_fraction = move_group.computeCartesianPath(cartesian_path_waypoints,
//                                                     0.05,  // eef_step
//                                                     3.0,   // jump_threshold            Set it always above 7.14 since avg change between joint space is 0.006
//                                                     cartesian_trajectory, //Trajectories generated
//                                                     false,  //Collision are allowed
//                                                     &cartesian_path_error_code); 
//     ROS_INFO("Cartesian path computed :%f percent Error code:%d",cartesian_fraction*100,cartesian_path_error_code);

//     if(cartesian_fraction > 0.9)
//     {
//         moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan ;
//         cartesian_plan.trajectory_ = cartesian_trajectory;
        
//         moveit_msgs::RobotState current_start_state;
//         robot_state::robotStateToRobotStateMsg(*move_group.getCurrentState(),current_start_state);
//         cartesian_plan.start_state_ = current_start_state;
// #ifdef EXECUTE_PATHS
//         move_group.execute(cartesian_plan);
// #endif
//         robot_grasp_point_to_barcode_point_execution_time.push_back(cartesian_plan.trajectory_.joint_trajectory.points
//         [cartesian_plan.trajectory_.joint_trajectory.points.size()-1].time_from_start.toSec() - 
//         cartesian_plan.trajectory_.joint_trajectory.points[0].time_from_start.toSec());
//         grasp_point_to_barcode_point_successful_plans += 1;
//         ROS_INFO("Grasp point to Barcode point Planning number %d succeded with exec time : %f",plan_number,
//                 robot_grasp_point_to_barcode_point_execution_time.back());
//         return true;
//     }
//     else
//     {
//         ROS_ERROR("Grasp pose to barcode pose plan failed");
//         grasp_point_to_barcode_point_failed_plans += 1;
//         return false;
//     }
    return true;
}

bool 
requestIK(geometry_msgs::Pose target_pose,ros::ServiceClient& compute_ik_client,moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::GetPositionIK::Request IK_service_request;
    moveit_msgs::GetPositionIK::Response IK_service_response; 

    IK_service_request.ik_request.group_name = PLANNING_GROUP ;
    IK_service_request.ik_request.pose_stamped.header.frame_id = move_group.getPlanningFrame() ;
    IK_service_request.ik_request.pose_stamped.pose = target_pose ;

    robot_state::RobotState current_robot_state(*move_group.getCurrentState());
    moveit_msgs::RobotState current_robot_state_msg;
    robotStateToRobotStateMsg(current_robot_state,current_robot_state_msg,true);
    
    IK_service_request.ik_request.robot_state = current_robot_state_msg ;

    IK_service_request.ik_request.avoid_collisions = true ;                  
    
    if(compute_ik_client.call(IK_service_request, IK_service_response))
    {
        if(IK_service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("IK Computation passed");
            return true ;
        }
        else
        {
            ROS_ERROR("IK Computation failed with error code : %d",IK_service_response.error_code.val);
            return false ;
        }
    }
    else
    {
        ROS_ERROR("Call to IK Service failed.Check weather TracIK library is loaded or not");
        return false ;
    }
}

bool goToDropPose(moveit::planning_interface::MoveGroupInterface& move_group,geometry_msgs::Pose drop_pose,int plan_num,
                moveit_msgs::RobotTrajectory &drop_trajectory,moveit_msgs::RobotTrajectory pre_drop_align_pose_trajectory)
{
    moveit_msgs::MoveItErrorCodes wall_plan_succ;
    moveit::planning_interface::MoveGroupInterface::Plan wall_plan ;
    moveit_msgs::RobotTrajectory pre_drop_trajectory;

    tf::Vector3 offset(PRE_DROP_POSE_X_OFFSET, 0.0, PRE_DROP_POSE_Z_OFFSET);

    // Since tf2::Matrix3x3 doesn't have a Mat * Vec function, we do it manually
    tf::Matrix3x3 drop_pose_mat = tf::Matrix3x3(tf::Quaternion(drop_pose.orientation.x,drop_pose.orientation.y,drop_pose.orientation.z
                                    ,drop_pose.orientation.w));
    geometry_msgs::Pose pre_drop_pose;

    pre_drop_pose.position.x = drop_pose.position.x - drop_pose_mat.getRow(0).dot(offset);
    pre_drop_pose.position.y = drop_pose.position.y - drop_pose_mat.getRow(1).dot(offset);
    pre_drop_pose.position.z = drop_pose.position.z - drop_pose_mat.getRow(2).dot(offset);
    pre_drop_pose.orientation = drop_pose.orientation;

    move_group.setPoseTarget(pre_drop_pose);

    wall_plan_succ = move_group.plan(wall_plan);

    if(wall_plan_succ.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        pre_drop_trajectory = wall_plan.trajectory_;

        ROS_INFO("Drop pose orientation achievement time : %f",pre_drop_trajectory.joint_trajectory.points
                                    [pre_drop_trajectory.joint_trajectory.points.size()-1].time_from_start.toSec() - 
                                    pre_drop_trajectory.joint_trajectory.points[0].time_from_start.toSec());

        move_group.execute(wall_plan);

        pre_drop_trajectory = Stich_trajectories(*move_group.getCurrentState(),move_group,pre_drop_align_pose_trajectory,pre_drop_trajectory);

        move_group.setPoseTarget(drop_pose);

        wall_plan_succ = move_group.plan(wall_plan);

        // moveit_msgs::RobotTrajectory cartesian_trajectory;
        // moveit_msgs::MoveItErrorCodes cartesian_path_error_code;
        // std::vector<geometry_msgs::Pose> cartesian_path_waypoints;
        // cartesian_path_waypoints.push_back(drop_pose);

        // double cartesian_fraction = move_group.computeCartesianPath(cartesian_path_waypoints,
        //                                             0.1,  // eef_step
        //                                             10.0,   // jump_threshold          
        //                                             cartesian_trajectory, //Trajectories generated
        //                                             true,  //Collision are not allowed
        //                                             &cartesian_path_error_code); 
        // ROS_INFO("Cartesian path computed :%f percent Error code:%d",cartesian_fraction*100,cartesian_path_error_code);

        // if(cartesian_fraction == 1.0)
        // {
        //     wall_plan.trajectory_ = cartesian_trajectory;

        //     ROS_INFO("Drop pose cartesian path trans time : %f",cartesian_trajectory.joint_trajectory.points
        //                             [cartesian_trajectory.joint_trajectory.points.size()-1].time_from_start.toSec() - 
        //                             cartesian_trajectory.joint_trajectory.points[0].time_from_start.toSec());
            
        //     moveit_msgs::RobotState current_start_state;
        //     robot_state::robotStateToRobotStateMsg(*move_group.getCurrentState(),current_start_state);
        //     wall_plan.start_state_ = current_start_state;

        //     wall_plan_succ.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        // }
        // else
        // {
        //     ROS_INFO("Cartesian path failed!Retrying with joint space planner");

        //     move_group.setPlannerId("RRTConnect");
        //     move_group.setPoseTarget(drop_pose);

        //     wall_plan_succ = move_group.plan(wall_plan);

        //     move_group.setPlannerId("PRMstar");
        // }

        if(wall_plan_succ.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            move_group.execute(wall_plan);

            drop_trajectory = Stich_trajectories(*move_group.getCurrentState(),move_group,pre_drop_trajectory,wall_plan.trajectory_);

            drop_trajectory = computeTimeStamps(move_group,drop_trajectory);

            robot_barcode_point_to_drop_point_execution_time.push_back(drop_trajectory.joint_trajectory.points
                                    [drop_trajectory.joint_trajectory.points.size()-1].time_from_start.toSec() - 
                                    drop_trajectory.joint_trajectory.points[0].time_from_start.toSec());
            
            barcode_point_to_drop_point_successful_plans += 1;
            
            ROS_INFO("Barcode point to drop point Planning number %d succeded with exec time : %f",plan_num,robot_barcode_point_to_drop_point_execution_time.back());


            return true;
        }   
        else
        {
            barcode_point_to_drop_point_failed_plans += 1;
            return false;
        }     

    }
    else
    {
        barcode_point_to_drop_point_failed_plans += 1;
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"put_to_wall_simulation");
    ros::NodeHandle node_handle ;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::ServiceClient compute_ik_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    ros::ServiceClient compute_fk_client = node_handle.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    ros::ServiceClient client_get_planning_scene = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    client_get_planning_scene.waitForExistence();

    ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();

    ////////////////////////////////////////////////////////////////////
    ///////////////// MoveGroup Initialization /////////////////////////
    ////////////////////////////////////////////////////////////////////

    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setPlannerId("PRMstar");  

    move_group.setNumPlanningAttempts(10);

    move_group.setPoseReferenceFrame( move_group.getPlanningFrame());

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);
    move_group.setGoalTolerance(0.001);
    move_group.setPlanningTime(10);

    // move_group.setWorkspace(-1.3,1.3,-1.3,1.3,0.0,2.5);

    //////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////
    ///////////////// Generate drop and grasp poses //////////////////////
    //////////////////////////////////////////////////////////////////////

    geometry_msgs::PoseArray pose_array;
    geometry_msgs::PoseArray grasp_poses;

    //Generate octagonal put poses
    generateOctagonPoses(pose_array);

    geometry_msgs::Pose barcode_scan_pose;

    barcode_scan_pose.position.x = 0.71;
    barcode_scan_pose.position.y = 0.17;
    barcode_scan_pose.position.z = 0.72;
    barcode_scan_pose.orientation.x = -0.73;
    barcode_scan_pose.orientation.y = 0.06;
    barcode_scan_pose.orientation.z = 0.66;
    barcode_scan_pose.orientation.w = 0.10;

    std::vector<double> barcode_pose_joint_angles;
    barcode_pose_joint_angles.push_back(-3.139055);
    barcode_pose_joint_angles.push_back(-0.822926);
    barcode_pose_joint_angles.push_back(-0.621118);
    barcode_pose_joint_angles.push_back(5.146556);
    barcode_pose_joint_angles.push_back(-1.621760);
    barcode_pose_joint_angles.push_back(-3.385229);

    geometry_msgs::Pose test_grasp_pose;
    test_grasp_pose.position.x = 0.71;
    test_grasp_pose.position.y = 0.17;
    test_grasp_pose.position.z = 0.1;
    test_grasp_pose.orientation.x = -0.73;
    test_grasp_pose.orientation.y = 0.06;
    test_grasp_pose.orientation.z = 0.66;
    test_grasp_pose.orientation.w = 0.10;

    //Generate floor test grasp poses within cone per point
    // generateGraspPoses(grasp_poses,1.57,1.57);

    grasp_poses.poses.push_back(test_grasp_pose);

    //////////////////////////////////////////////////////////////////////////

    visualization_msgs::MarkerArray marker, grasp_poses_marker;
    
    marker.markers.resize(pose_array.poses.size());
    
    grasp_poses_marker.markers.resize(grasp_poses.poses.size());

    for(int pose_cnt = 0 ; pose_cnt < pose_array.poses.size(); pose_cnt++)
    {
        visualizeSuccessfulMarkerPoints("wall_put_points",pose_cnt,visualization_msgs::Marker::ADD,pose_array.poses[pose_cnt],marker);
    }
    vis_pub.publish(marker);

    moveit::planning_interface::MoveGroupInterface::Plan wall_plan ;
    std::string collision_object_id;

    moveit_msgs::MoveItErrorCodes wall_plan_succ;

    for(int grasp_pose_cnt = 0 ; grasp_pose_cnt < grasp_poses.poses.size(); grasp_pose_cnt++)
    {
        move_group.setJointValueTarget(barcode_pose_joint_angles);

        wall_plan_succ = move_group.move();

        if(wall_plan_succ.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("Robot on barcode scan pose x : %f y : %f z : %f",grasp_poses.poses[grasp_pose_cnt].position.x,
                grasp_poses.poses[grasp_pose_cnt].position.y,grasp_poses.poses[grasp_pose_cnt].position.z);

            std::vector<std::string> touch_links;
            touch_links.push_back("vacuum_gripper_eef_link");

            for(int i = 0 ; i < pose_array.poses.size() ; i++)
            {
                if(planAndMoveToGraspPose(move_group,grasp_poses.poses[0],i))
                {
                    if(planAndMoveToBarcodePose(move_group,barcode_scan_pose,i))
                    {
                        // collision_object_id.clear();
                        // collision_object_id = addGraspObject(move_group,barcode_scan_pose,
                        //                                     client_get_planning_scene,planning_scene_diff_client);

                        // ROS_INFO("Attaching grasp object to robot");
                        
                        // move_group.attachObject(collision_object_id,"vacuum_gripper_eef_link");
                        
                        // ROS_INFO("Attached grasp object to robot");

                        if(requestIK(pose_array.poses[i],compute_ik_client,move_group))
                        {
                            std::vector<double> pre_drop_pose_joint_angles;
                            tf::Quaternion pre_drop_pose_quat(pose_array.poses[i].orientation.x,
                                                            pose_array.poses[i].orientation.y,
                                                            pose_array.poses[i].orientation.z,
                                                            pose_array.poses[i].orientation.w);
                            tf::Matrix3x3 pre_drop_pose_mat(pre_drop_pose_quat);
                            double roll, pitch, yaw;
                            pre_drop_pose_mat.getRPY(roll, pitch, yaw);
                            double zero_yaw_base_joint_value = 3.14;
                            if((yaw <= 0.80) && (yaw >= 0.76))
                            {
                                pre_drop_pose_joint_angles.push_back(zero_yaw_base_joint_value - 0.78);
                            }
                            else if((yaw <= 1.59) && (yaw >= 1.55))
                            {
                                pre_drop_pose_joint_angles.push_back(zero_yaw_base_joint_value - 1.57);
                            }
                            else if((yaw <= 2.36) && (yaw >= 2.32))
                            {
                                pre_drop_pose_joint_angles.push_back(zero_yaw_base_joint_value - 2.34);
                            }
                            else if((yaw <= 3.16) && (yaw >= 3.12))
                            {
                                pre_drop_pose_joint_angles.push_back(zero_yaw_base_joint_value - 3.14);
                            }
                            else if((yaw >= -0.80) && (yaw <= -0.76))
                            {
                                pre_drop_pose_joint_angles.push_back(-1*zero_yaw_base_joint_value + 0.78);
                            }
                            else if((yaw >= -1.59) && (yaw <= -1.55))
                            {
                                pre_drop_pose_joint_angles.push_back(-1*zero_yaw_base_joint_value + 1.57);
                            }
                            else if((yaw >= -2.36) && (yaw <= -2.32))
                            {
                                pre_drop_pose_joint_angles.push_back(-1*zero_yaw_base_joint_value + 2.34);
                            }
                            else
                            {
                                pre_drop_pose_joint_angles.push_back(-1*zero_yaw_base_joint_value);
                            }

                            pre_drop_pose_joint_angles.push_back(-0.822926);
                            pre_drop_pose_joint_angles.push_back(-0.621118);
                            pre_drop_pose_joint_angles.push_back(5.146556);
                            pre_drop_pose_joint_angles.push_back(-1.621760);
                            pre_drop_pose_joint_angles.push_back(-3.385229);

                            moveit_msgs::GetPositionFK::Request FK_service_request;
                            moveit_msgs::GetPositionFK::Response FK_service_response; 
                            std::vector<std::string> link_names;
                            link_names.push_back(move_group.getEndEffectorLink());

                            FK_service_request.fk_link_names = link_names ;

                            moveit_msgs::RobotState current_robot_state_msg;
                            current_robot_state_msg.joint_state.name.push_back("shoulder_joint");
                            current_robot_state_msg.joint_state.name.push_back("upperArm_joint");
                            current_robot_state_msg.joint_state.name.push_back("foreArm_joint");
                            current_robot_state_msg.joint_state.name.push_back("wrist1_joint");
                            current_robot_state_msg.joint_state.name.push_back("wrist2_joint");
                            current_robot_state_msg.joint_state.name.push_back("wrist3_joint");

                            current_robot_state_msg.joint_state.position = pre_drop_pose_joint_angles ;
                            current_robot_state_msg.is_diff = true;
                            
                            FK_service_request.robot_state = current_robot_state_msg ;

                            geometry_msgs::PoseStamped pre_drop_pose;
                            
                            if(compute_fk_client.call(FK_service_request, FK_service_response))
                            {
                                if(FK_service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                                {
                                    ROS_INFO("FK Computation passed");
                                    pre_drop_pose = FK_service_response.pose_stamped[0];
                                }
                                else
                                {
                                    ROS_ERROR("IK Computation failed with error code : %d",FK_service_response.error_code.val);
                                    break;
                                }
                            }
                            else
                            {
                                ROS_ERROR("Call to FK Service failed.Check weather TracIK library is loaded or not");
                                break;
                            }

                            moveit_msgs::RobotTrajectory pre_drop_trajectory;

                            move_group.setPlannerId("RRTConnect");

                            // move_group.setJointValueTarget(pre_drop_pose_joint_angles);
                            move_group.setPoseTarget(pre_drop_pose.pose);

                            wall_plan_succ = move_group.plan(wall_plan);

                            move_group.setPlannerId("PRMstar");

                            if(wall_plan_succ.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                            {
                                pre_drop_trajectory = wall_plan.trajectory_;

                                move_group.execute(wall_plan);

                                if(pose_array.poses[i].position.z <= LOWER_BINS_RANGE)
                                {

                                    move_group.setPoseTarget(pose_array.poses[i]);

                                    wall_plan_succ = move_group.plan(wall_plan);

                                    if(wall_plan_succ.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                                    {
                                        move_group.execute(wall_plan);

                                        wall_plan.trajectory_ = Stich_trajectories(*move_group.getCurrentState(),move_group,pre_drop_trajectory,
                                                                                    wall_plan.trajectory_);

                                        robot_barcode_point_to_drop_point_execution_time.push_back(wall_plan.trajectory_.joint_trajectory.points
                                            [wall_plan.trajectory_.joint_trajectory.points.size()-1].time_from_start.toSec() - 
                                            wall_plan.trajectory_.joint_trajectory.points[0].time_from_start.toSec());
                                        barcode_point_to_drop_point_successful_plans += 1;
                                        ROS_INFO("Barcode point to drop point Planning number %d succeded with exec time : %f",i,robot_barcode_point_to_drop_point_execution_time.back());

                                        // move_group.detachObject(collision_object_id); 

                                        // move_group.clearPathConstraints();

                                        moveit_msgs::RobotState current_start_state;
                                        robot_state::robotStateToRobotStateMsg(*move_group.getCurrentState(),current_start_state);

                                        wall_plan.trajectory_ = reverseTrajectory(wall_plan.trajectory_,move_group);
                                        wall_plan.start_state_ = current_start_state;
                                        wall_plan_succ.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

                                        robot_drop_point_to_barcode_point_execution_time.push_back(wall_plan.trajectory_.joint_trajectory.points
                                        [wall_plan.trajectory_.joint_trajectory.points.size()-1].time_from_start.toSec() - 
                                        wall_plan.trajectory_.joint_trajectory.points[0].time_from_start.toSec());

                                        drop_point_to_barcode_point_successful_plans += 1;
                                        ROS_INFO("Drop point to Barcode point Planning number %d succeded with exec time : %f",i,robot_drop_point_to_barcode_point_execution_time.back());

                                        move_group.execute(wall_plan);
                                    }
                                    else
                                    {
                                        visualizeFailedMarkerPoints("wall_put_points",i,visualization_msgs::Marker::MODIFY,pose_array.poses[i],marker);
                                        vis_pub.publish(marker);

                                        ROS_ERROR("Planning number %d failed",i);
                                        barcode_point_to_drop_point_failed_plans += 1 ;
                                    }
                                }
                                else
                                {
                                    if(goToDropPose(move_group,pose_array.poses[i],i,wall_plan.trajectory_,pre_drop_trajectory))
                                    {
                                        // move_group.detachObject(collision_object_id); 

                                        // move_group.clearPathConstraints();

                                        // move_group.setJointValueTarget(barcode_pose_joint_angles);

                                        // wall_plan_succ = move_group.plan(wall_plan);

                                        moveit_msgs::RobotState current_start_state;
                                        robot_state::robotStateToRobotStateMsg(*move_group.getCurrentState(),current_start_state);

                                        wall_plan.trajectory_ = reverseTrajectory(wall_plan.trajectory_,move_group);
                                        wall_plan.start_state_ = current_start_state;
                                        wall_plan_succ.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

                                        robot_drop_point_to_barcode_point_execution_time.push_back(wall_plan.trajectory_.joint_trajectory.points
                                        [wall_plan.trajectory_.joint_trajectory.points.size()-1].time_from_start.toSec() - 
                                        wall_plan.trajectory_.joint_trajectory.points[0].time_from_start.toSec());

                                        drop_point_to_barcode_point_successful_plans += 1;
                                        ROS_INFO("Drop point to Barcode point Planning number %d succeded with exec time : %f",i,robot_drop_point_to_barcode_point_execution_time.back());

                                        move_group.execute(wall_plan);
                                    }
                                    else
                                    {
                                        // move_group.detachObject(collision_object_id);

                                        visualizeFailedMarkerPoints("wall_put_points",i,visualization_msgs::Marker::MODIFY,pose_array.poses[i],marker);
                                        vis_pub.publish(marker);

                                        move_group.setJointValueTarget(barcode_pose_joint_angles);

                                        move_group.move();
                                    }
                                }
                            }
                        }
                        else
                        {
                            // move_group.detachObject(collision_object_id);

                            visualizeFailedMarkerPoints("wall_put_points",i,visualization_msgs::Marker::MODIFY,pose_array.poses[i],marker);
                            vis_pub.publish(marker);

                            ROS_ERROR("No IK Found for specified pose number : %d",i);
                            barcode_point_to_drop_point_failed_plans += 1 ;
                        }
                        ros::spinOnce();
                    }
                }
            }

        // move_group.detachObject(collision_object_id); 
        // ROS_INFO("Dettached grasp object from robot");
        }
        else
        {
            // visualize_failed_marker_points("grasp_points",grasp_pose_cnt,visualization_msgs::Marker::MODIFY,
            //                         pose_array.poses[grasp_pose_cnt],grasp_poses_marker);
            // vis_pub.publish(grasp_poses_marker);
        }
    }

    for(int j = 0 ; j < robot_barcode_point_to_drop_point_execution_time.size() ; j++)
    {
        barcode_point_to_drop_point_total_robot_execution_time += robot_barcode_point_to_drop_point_execution_time[j];
    }

    ROS_INFO("Average Robot execution time Barcode point to drop point : %f",barcode_point_to_drop_point_total_robot_execution_time/barcode_point_to_drop_point_successful_plans);
    ROS_INFO("Total plans failed Barcode point to drop point : %d Plans success : %d",barcode_point_to_drop_point_failed_plans,barcode_point_to_drop_point_successful_plans);

    for(int j = 0 ; j < robot_drop_point_to_barcode_point_execution_time.size() ; j++)
    {
        drop_point_to_barcode_point_total_robot_execution_time += robot_drop_point_to_barcode_point_execution_time[j];
    }

    ROS_INFO("Average Robot execution time Drop point to Barcode point : %f",drop_point_to_barcode_point_total_robot_execution_time/drop_point_to_barcode_point_successful_plans);
    ROS_INFO("Total plans failed Drop point to Barcode point : %d Plans success : %d",drop_point_to_barcode_point_failed_plans,drop_point_to_barcode_point_successful_plans);

    for(int j = 0 ; j < robot_barcode_point_to_grasp_point_execution_time.size() ; j++)
    {
        barcode_point_to_grasp_point_total_robot_execution_time += robot_barcode_point_to_grasp_point_execution_time[j];
    }

    ROS_INFO("Average Robot execution time Barcode point to grasp point : %f",barcode_point_to_grasp_point_total_robot_execution_time/barcode_point_to_grasp_point_successful_plans);
    ROS_INFO("Total plans failed Barcode point to grasp point : %d Plans success : %d",barcode_point_to_grasp_point_failed_plans,barcode_point_to_grasp_point_successful_plans);

    for(int j = 0 ; j < robot_grasp_point_to_barcode_point_execution_time.size() ; j++)
    {
        grasp_point_to_barcode_point_total_robot_execution_time += robot_grasp_point_to_barcode_point_execution_time[j];
    }

    ROS_INFO("Average Robot execution time grasp point to barcode point : %f",grasp_point_to_barcode_point_total_robot_execution_time/grasp_point_to_barcode_point_successful_plans);
    ROS_INFO("Total plans failed grasp point to barcode point : %d Plans success : %d",grasp_point_to_barcode_point_failed_plans,grasp_point_to_barcode_point_successful_plans);

    ROS_INFO("Total average robot execution time : %f",((barcode_point_to_drop_point_total_robot_execution_time/barcode_point_to_drop_point_successful_plans)
        +(drop_point_to_barcode_point_total_robot_execution_time/drop_point_to_barcode_point_successful_plans)
        +(barcode_point_to_grasp_point_total_robot_execution_time/barcode_point_to_grasp_point_successful_plans)
        +(grasp_point_to_barcode_point_total_robot_execution_time/grasp_point_to_barcode_point_successful_plans)));
    ros::shutdown();
    
    return 0;
}
