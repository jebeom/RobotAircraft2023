/**
 * 2022 로봇항공기경진대회 초급부문
 * 건국대학교 메카트론
 * VTOL 경로 비행 코드
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/TimeReference.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <stdio.h>

#include "algebra.h"
#include "config.h"
#include "control.h"
#include "frame.h"
#include "global_def.h"
#include "guidance.h"
#include "mission.h"
// #include "log.h"

int current_flight_mode = MC;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pose;
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
}

sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    global_pose = *msg;
}

mavros_msgs::ExtendedState current_extended_state;
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    current_extended_state = *msg;
}

sensor_msgs::TimeReference time_reference;
unsigned long timestamp;
void time_ref_cb(const sensor_msgs::TimeReference::ConstPtr &msg)
{
    time_reference = *msg;
    timestamp = time_reference.time_ref.toNSec()/1000;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rohang22");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pose_cb);
    ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 1, extended_state_cb);
    ros::Subscriber time_reference_sub = nh.subscribe<sensor_msgs::TimeReference>("mavros/time_reference", 1, time_ref_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient vtol_transition_client = nh.serviceClient<mavros_msgs::CommandVtolTransition>("mavros/cmd/vtol_transition");

    ros::Publisher set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_FREQUENCY_HZ);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL tol_cmd;

    mavros_msgs::CommandVtolTransition vtol_state;
    vtol_state.request.state = MC;
    
    geometry_msgs::PoseStamped pose;
    pose.pose = local_pose.pose;
    set_heading(pose, get_current_heading(local_pose));

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        set_pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    std::vector<double> home_local = {local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z};
    std::vector<double> home_global = {global_pose.latitude, global_pose.longitude, global_pose.altitude};
    if(FRAME_MODE == LLH){
        ROS_INFO("global: %f, %f, %f", home_global[0], home_global[1], home_global[2]);
        ROS_INFO("local: %f, %f, %f", home_local[0], home_local[1], home_local[2]);
        mission_llh2enu(WPT, home_global);
        mission_calib_local(WPT, home_local);
    }

    
    int process = 0;
    int i = 0;
    
    bool hold_flag = false;
    bool hold_flag2 = false;
    bool active_once_flag = false;

    float step = MC_SPEED;
    double h_err = MC_HORIZONTAL_ERROR;
    double v_err = MC_VERTICAL_ERROR;
    double a_err = HEADING_ERROR;

    std::vector<double> start;
    std::vector<double> end;
    std::vector<double> center;
    std::vector<double> local;
    std::vector<double> set = {0, 0, 0};
    double heading;

    std::vector<double> vec_i;
    std::vector<double> vec_f;
    std::vector<double> vec_c;

    ROS_INFO("Mission Start");
    create_file(FILE_PATH);
    save_timestamp(FILE_PATH, 1, timestamp); // Start, Takeoff and move to BASE

    while(ros::ok()){
        // default setpoint signal
        update_posestamp_header(pose);
        set_position(pose, {local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z});

        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(1.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(1.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        if(current_state.armed && current_state.mode == "OFFBOARD"){ 
            switch(process)
            {
            case 0:
                // i=0; WPT#0 BASE
                set_position(pose, WPT[i]);
                set_heading(pose, get_current_heading(local_pose));
                set = WPT[i];

                if(is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err)){
                    if(hold_flag == false && hold(1))
                        hold_flag = true;
                    if(hold_flag){
                        start = {WPT[i][0], WPT[i][1]};
                        end = {WPT[i+1][0], WPT[i+1][1]};
                        heading = get_angle(start, end);
                        set_heading(pose, heading);

                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(hold_flag2 == false && hold(HOLD_TIME))
                                hold_flag2 = true;
                            if(hold_flag2){
                                // ROS_INFO("Take off Complete");
                                ROS_INFO("move to WPT1..");
                                process++; // 다음 단계
                                i++; // 미션 웨이포인트 인덱스 1 높임

                                hold_flag = false;
                                hold_flag2 = false;

                                save_timestamp(FILE_PATH, 2, timestamp); // move to WPT1
                            }
                        }
                    }
                    
                }
                break;

            case 1: // WPT1 and Hover
                // i=1; WPT#1 MC
                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);

                if( is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err) || is_increase_dist(remain_dist(local_pose, WPT[i])) ){
                    set_position(pose, WPT[i]);

                    if(hold_flag == false && hold(1))
                        hold_flag = true;
                    if(hold_flag){
                        start = {WPT[i][0], WPT[i][1]};
                        end = {WPT[i+1][0], WPT[i+1][1]};
                        heading = get_angle(start, end);
                        set_heading(pose, heading);
                        
                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(active_once_flag == false){
                                active_once_flag = true;
                                save_timestamp(FILE_PATH, 3, timestamp); // arrived at WPT1
                            }
                            if(hold_flag2 == false && hold(HOLD_TIME))
                                hold_flag2 = true;
                            if(hold_flag2){
                                ROS_INFO("Arrived at WPT1..");
                                process++;
                                i++;

                                hold_flag = false;
                                hold_flag2 = false;
                                active_once_flag = false;

                                save_timestamp(FILE_PATH, 4, timestamp); // move to WPT2
                            }
                        }
                    }  
                }
                break;

            case 2: // Transition to FW
            #ifndef QUAD_MODE
                vtol_state.request.state = FW;
                if(vtol_transition_client.call(vtol_state)){
                    current_flight_mode = FW;
                    step = FW_SPEED;
                    h_err = FW_HORIZONTAL_ERROR;
                    v_err = FW_VERTICAL_ERROR;

                    ROS_INFO("transition FW");
                    process++;
                }
                set_position(pose, WPT[i]);
                break;
            #else
                process++; //pass
            #endif

            case 3: // to WPT2
                // ROS_INFO("fly to WPT2..");
                // i=2; WPT#2 MC

                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);

                if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
                    save_timestamp(FILE_PATH, 5, timestamp); // arrived at WPT2

                    ROS_INFO("Arrived at WPT2..");
                    process++;
                    i++;

                    save_timestamp(FILE_PATH, 6, timestamp); // move to WPT3
                }
                break;

            case 4: // to WPT3
                // ROS_INFO("fly to WPT3..");
                // i=3; WPT#3 MC
                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);

                if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
                    save_timestamp(FILE_PATH, 7, timestamp); // arrived at WPT3
                    
                    ROS_INFO("Arrived at WPT3..");
                    process++;
                    i++;

                    save_timestamp(FILE_PATH, 8, timestamp); // move to WPT4
                }
                break;

            case 5: // Circle
                // i=4; circle and WPT#4

                // // 미션 바뀌기 전; 2~3번 wpt 경로에 수직으로 원 반경만큼 떨어진 곳이 원 중심
                // start = {WPT[i-2][0], WPT[i-2][1]}; // 2번
                // end = {WPT[i-1][0], WPT[i-1][1]};   // 3번
                // local = {local_pose.pose.position.x, local_pose.pose.position.y};
                // center = eminus(end, start);
                // center = {-center[1], center[0]}; // 외적의 결과; 시계방향회전에서는 서로 부호가 바뀜
                // center = mult_const(center, 1/norm(center));
                // center = mult_const(center, MISSION_RADIUS); 
                // center = eplus(center, end);

                // 미션 바뀐 후; 3~4번 WPT 사이가 원 중심
                start = {WPT[i-1][0], WPT[i-1][1]}; // 3번
                end = {WPT[i][0], WPT[i][1]};       // 4번
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                center = mult_const(eplus(start, end), 0.5);
                //center = {0, 0};

                set = circle_guidance(center, MISSION_RADIUS, CIRCLE_DIRECTION, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], local_pose.pose.position.z + CLIME_RATE};
                if(set[2] > WPT[i][2] + 1){ // 목표고도에 근접하면 상승률이 낮아지므로 목표고도 자체를 조금 더 높게 설정
                    set[2] = WPT[i][2] + 1;
                }
                set_position(pose, set);
                
                if(hold_flag == false && hold(5)){ // 3번 경로점을 충분히 지나갈 때까지 대기
                    hold_flag = true;
                }
                if(is_arrived_hori(local_pose, WPT[i-1], 20) && hold_flag == true){ // 원을 그려야 하므로 한 바퀴 돌아 다시 3번 웨이포인트를 통과하는지 판별, 정확도는 중요치 않으니 오차범위를 다소 크게 지정
                    hold_flag2 = true;
                }

                if(hold_flag2 == true){ // 원 탈출할 준비가 됨
                    if(is_arrived_hori(local_pose, WPT[i], 3*h_err) && is_arrived_verti(local_pose, WPT[i], v_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){ // 4번 경로점 통과
                        save_timestamp(FILE_PATH, 9, timestamp); // arrived at WPT4

                        process++;
                        i++;
                        hold_flag = false;
                        hold_flag2 = false; 

                        save_timestamp(FILE_PATH, 10, timestamp); // move to WPT5
                    }
                    
                }
                break;

            case 6: // 
                process++; //pass

            case 7: // to WPT5
                // ROS_INFO("fly to WPT5..");
                // i=5; WPT#5 MC
                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);

                if(remain_dist(local_pose, WPT[i]) < BACK_TRANSITION_DIST){
                    ROS_INFO("Back Transition Start.");
                    process++;
                }
                break;

            case 8: // Transition to MC
            #ifndef QUAD_MODE
                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                heading = get_angle(start, end);
                set_heading(pose, heading);
                // set_heading(pose, get_current_heading(local_pose));
                
                vtol_state.request.state = MC;
                if(vtol_transition_client.call(vtol_state)){
                    current_flight_mode = MC;
                    step = MC_SPEED;
                    h_err = MC_HORIZONTAL_ERROR;
                    v_err = MC_VERTICAL_ERROR;

                    ROS_INFO("transition to MC");
                    set_position(pose, WPT[i]);
                    process++;
                }
                
                break;
            #else
                process++; //pass
            #endif

            case 9: // to WPT5 and Hover
                // break;
                // i=5; WPT#5 MC

                // 경로 유도 안함
                // start = {WPT[i-1][0], WPT[i-1][1]};
                // end = {WPT[i][0], WPT[i][1]};
                // local = {local_pose.pose.position.x, local_pose.pose.position.y};
                // set = line_guidance(start, end, local, step);
                // set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                // set_position(pose, set);

                set_position(pose, WPT[i]);

                if( is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err) ){ // || is_increase_dist(remain_dist(local_pose, WPT[i]))
                    set_position(pose, WPT[i]);

                    if(hold_flag == false && hold(2))
                        hold_flag = true;
                    if(hold_flag){
                        start = {WPT[i][0], WPT[i][1]};
                        end = {WPT[0][0], WPT[0][1]};
                        heading = get_angle(start, end);
                        set_heading(pose, heading);
                        
                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(active_once_flag == false){
                                active_once_flag = true;
                                save_timestamp(FILE_PATH, 11, timestamp); // arrived at WPT5
                            }
                            
                            
                            if(hold_flag2 == false && hold(HOLD_TIME))
                                hold_flag2 = true;
                            if(hold_flag2){
                                ROS_INFO("Arrived at WPT5..");
                                process++;
                                i++;

                                hold_flag = false;
                                hold_flag2 = false;
                                active_once_flag == false;

                                save_timestamp(FILE_PATH, 12, timestamp); // move to BASE
                            }
                        }
                    }  
                }
                break;

            case 10: // to Base and Hover
                // i=5
                // ROS_INFO("move to Base..");
                start = {WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);
                
                if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
                    set_position(pose, WPT[i]);

                    if(hold_flag == false && hold(1))
                        hold_flag = true;
                    if(hold_flag){
                        heading = get_current_heading(local_pose);
                        set_heading(pose, heading);
                        
                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(hold_flag2 == false && hold(HOLD_TIME))
                                hold_flag2 = true;
                            if(hold_flag2){
                                ROS_INFO("Arrived at BASE..");
                                process++;

                                hold_flag = false;
                                hold_flag2 = false;

                                save_timestamp(FILE_PATH, 13, timestamp); // arrived at BASE and Landing
                            }
                        }
                    }  
                }
                break;

            case 11: // Land
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if(set_mode_client.call(offb_set_mode) &&
                   offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Auto Land enabled");
                    process++;
                }
                break;
            }

            // // Update my pose
            // set_pose_pub.publish(pose);
        }
        // Update my pose
        set_pose_pub.publish(pose);

        // double param;
        // nh.setParam("mavros/conn/system_time_rate", 2.0);
        // nh.getParam("mavros/conn/system_time_rate", param);
        

        ROS_INFO("Current: %.2f, %.2f %.2f", local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z);
        ROS_INFO("Setpoint: %.2f, %.2f, %.2f", set[0], set[1], set[2]);
        ROS_INFO("Remain Dist: %f", remain_dist(local_pose, WPT[i]));
        ROS_INFO("\n");

        ros::spinOnce();
        rate.sleep();
    }
}
