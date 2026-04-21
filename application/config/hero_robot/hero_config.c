#include "hero_config.h"

Robot_Config_s *HeroConfigInit(void) 
{
    static Robot_Config_s hero_config =  {
        .chassis_param = {
            .chassis_motor_id[MOTOR_LF] = 1,    //1,4,3,2
            .chassis_motor_id[MOTOR_RF] = 4,
            .chassis_motor_id[MOTOR_RB] = 3,
            .chassis_motor_id[MOTOR_LB] = 2,

            .chassis_motor_id[MOTOR_UP_LF] = 6,   //左前舵轮
            .chassis_motor_id[MOTOR_UP_RF] = 4,
            .chassis_motor_id[MOTOR_UP_LB] = 1,
            .chassis_motor_id[MOTOR_UP_RB] = 3,        
            .wheel_type = STEER_WHEEL,
            .wheel_measure = {
                .center_gimbal_offset_x = 0.0,
                .center_gimbal_offset_y = 0.0,
                .wheel_base = 406.46,
                .track_width = 361.53,
                .radius_wheel = 75,
                .reduction_ratio_wheel = 19.0,
            },
            //轮向电机配置
            .chassis_motor_config = {
                .can_init_config.can_handle = &hcan1,
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 5.0,
                        .Ki = 1.0,
                        .Kd = 0.0, 
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 12000,
                    },
                    .current_PID = {
                        .Kp = 0.5, 
                        .Ki = 0.0,   
                        .Kd = 0.0,
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 15000,
                    },
                },   
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                },
                .motor_type = M3508,             
            },
            //舵向电机配置
            .rudder_motor_config = {
                .can_init_config.can_handle = &hcan1,
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 10.0,
                        .Ki = 1.0,
                        .Kd = 0.0, 
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 12000,
                    },
                    .angle_PID = {
                        .Kp = 45.0, 
                        .Ki = 5.0,   
                        .Kd = 0.2,
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 15000,
                    },
                },   
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                },
                .motor_type = GM6020,             
            }
         },
        //云台配置
        .gimbal_param = {
            .gimbal_type = SINGLE_GIMBAL,
             .gimbal_offset = {
                    .yaw_offset = 2777,
                    .pitch_min_angle = -18.0f,
                    .pitch_max_angle = 20.0f,
                },
            //yaw轴也走can1，id5避免和舵向冲突
            .yaw_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 2,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10.0,
                        .Ki = 0.0, 
                        .Kd = 0.5, 
                        .DeadBand = 0.1,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 500,
                    },
                    .speed_PID = {
                        .Kp = 200.0, 
                        .Ki = 150.0, 
                        .Kd = 0.0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 3000,
                        .MaxOut = 20000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                },
                .motor_type = GM6020
            },        
            //拨弹上面电机走can2
            .pitch_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 0x7f,
                    .ext_id = 0x7f,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 0.8, // 10
                        .Ki = 0,
                        .Kd = 0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 20,
                    },
                    .speed_PID = {
                        .Kp = 0.2,  // 50
                        .Ki = 0, // 350
                        .Kd = 0,   // 0
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 2500,
                        .MaxOut = 20,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
                .motor_type = XMCY
            }                
        },

        .shoot_param = {
            //英雄双发射机构
            .friction_motor_id[MOTOR_LF] = 2,
            .friction_motor_id[MOTOR_RF] = 3,
            // .friction_motor_id[MOTOR_LB] = 1,
            // .friction_motor_id[MOTOR_RB] = 4,

            .shoot_type = DUAL_SHOOT,
            .friction_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                },
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 20.0, 
                        .Ki = 0.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                    .current_PID = {
                        .Kp = 1.0, 
                        .Ki = 0.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
                .motor_type = M3508
            },

            .loader_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 1,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                        .Kp = 10.0, 
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .MaxOut = 200, 
                    },
                    .speed_PID = {
                        .Kp = 10.0, 
                        .Ki = 1.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 8000,
                        .MaxOut = 10000,
                    },
                    .current_PID = {
                        .Kp = 0.4, 
                        .Ki = 0.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 5000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED, 
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                },
                .motor_type = M3508 // 英雄使用M3508
            }
        }
    };

    return &hero_config;
} 





