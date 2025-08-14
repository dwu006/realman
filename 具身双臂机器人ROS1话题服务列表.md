| 节点名称                    | 分类      | Topic名称                                        | 使用msg类型                                        | 功能                                    |
| --------------------------- | --------- | ------------------------------------------------ | -------------------------------------------------- | --------------------------------------- |
| rm_driver（驱动机器人节点） |           |                                                  |                                                    |                                         |
| 左臂                        | subscribe | /l_arm/rm_driver/Arm_Analog_Output               | const dual_arm_msgs::Arm_Analog_Output msg         | 设置机械臂模拟 IO 输出状态              |
|                             | subscribe | /l_arm/rm_driver/Arm_Digital_Output              | const dual_arm_msgs::Arm_Digital_Output msg        | 设置机械臂数字 IO 输出状态              |
|                             | subscribe | /l_arm/rm_driver/Arm_JointTeach                  | const dual_arm_msgs::Joint_Teach msg               | 关节示教                                |
|                             | subscribe | /l_arm/rm_driver/Arm_OrtTeach                    | const dual_arm_msgs::Ort_Teach msg                 | 姿态示教                                |
|                             | subscribe | /l_arm/rm_driver/Arm_PosTeach                    | const dual_arm_msgs::Pos_Teach msg                 | 位置示教                                |
|                             | subscribe | /l_arm/rm_driver/Arm_StopTeach                   | const dual_arm_msgs::Stop_Teach msg                | 示教停止                                |
|                             | subscribe | /l_arm/rm_driver/ChangeToolName_Cmd              | const dual_arm_msgs::ChangeTool_Name msg           | 切换当前工具坐标系                      |
|                             | subscribe | /l_arm/rm_driver/ChangeWorkFrame_Cmd             | const dual_arm_msgs::ChangeWorkFrame_Name msg      | 切换当前工作坐标系                      |
|                             | subscribe | /l_arm/rm_driver/ClearForceData_Cmd              | const std_msgs::Empty msg                          | 清空六维力数据                          |
|                             | subscribe | /l_arm/rm_driver/Clear_System_Err                | const std_msgs::Empty msg                          | 清除系统错误                            |
|                             | subscribe | /l_arm/rm_driver/Emergency_Stop                  | const std_msgs::Empty msg                          | 轨迹急停                                |
|                             | subscribe | /l_arm/rm_driver/ForcePositionMoveJiont_Cmd      | const dual_arm_msgs::Force_Position_Move_Joint msg | 透传力位混合控制补偿(弧度)              |
|                             | subscribe | /l_arm/rm_driver/ForcePositionMovePose_Cmd       | const dual_arm_msgs::Force_Position_Move_Pose msg  | 透传力位混合控制补偿(位姿)              |
|                             | subscribe | /l_arm/rm_driver/GetArmJoint_Cmd                 | const std_msgs::Empty msg                          | 查询机械臂关节角度                      |
|                             | subscribe | /l_arm/rm_driver/GetArmStateTimerSwitch          | const std_msgs::Bool msg                           | 选择开启udp数据上报                     |
|                             | subscribe | /l_arm/rm_driver/GetArmState_Cmd                 | const dual_arm_msgs::GetArmState_Command msg       | 查询机械臂状态（弧度+四元数）           |
|                             | subscribe | /l_arm/rm_driver/GetCurrentArmState              | const std_msgs::Empty msg                          | 查询当前机械臂状态（角度+欧拉角）       |
|                             | subscribe | /l_arm/rm_driver/GetCurrentJointCurrent          | const std_msgs::Empty msg                          | 获取机械臂关节当前电流                  |
|                             | subscribe | /l_arm/rm_driver/GetOneForce_Cmd                 | const std_msgs::Empty msg                          | 查询一维力数据指令                      |
|                             | subscribe | /l_arm/rm_driver/GetSixForce                     | const std_msgs::Empty msg                          | 查询六维力数据                          |
|                             | subscribe | /l_arm/rm_driver/GetSixForce_Cmd                 | const std_msgs::Empty msg                          | 查询六维力数据                          |
|                             | subscribe | /l_arm/rm_driver/GetTotalWorkFrame               | const std_msgs::Empty msg                          | 查询当前工作坐标系                      |
|                             | subscribe | /l_arm/rm_driver/Get_Arm_Software_Version        | const std_msgs::Empty msg                          | 读取软件版本号                          |
|                             | subscribe | /l_arm/rm_driver/Get_Realtime_Push               | const std_msgs::Empty msg                          | 查询 UDP 机械臂状态主动上报配置         |
|                             | subscribe | /l_arm/rm_driver/Gripper_Pick                    | dual_arm_msgs::Gripper_Pick msg                    | 设置夹爪力控夹取                        |
|                             | subscribe | /l_arm/rm_driver/Gripper_Pick_On                 | dual_arm_msgs::Gripper_Pick msg                    | 设置夹爪持续力控夹取                    |
|                             | subscribe | /l_arm/rm_driver/Gripper_Set                     | dual_arm_msgs::Gripper_Set msg                     | 设置夹爪行程量                          |
|                             | subscribe | /l_arm/rm_driver/Hand_SetAngle                   | const dual_arm_msgs::Hand_Angle msg                | 设置灵巧手角度                          |
|                             | subscribe | /l_arm/rm_driver/Hand_SetForce                   | const dual_arm_msgs::Hand_Force msg                | 设置灵巧手关节力阈值                    |
|                             | subscribe | /l_arm/rm_driver/Hand_SetPosture                 | const dual_arm_msgs::Hand_Posture msg              | 设置灵巧手手势序号                      |
|                             | subscribe | /l_arm/rm_driver/Hand_SetSeq                     | const dual_arm_msgs::Hand_Seq msg                  | 设置灵巧手动作序列                      |
|                             | subscribe | /l_arm/rm_driver/Hand_SetSpeed                   | const dual_arm_msgs::Hand_Speed msg                | 设置灵巧手关节速度                      |
|                             | subscribe | /l_arm/rm_driver/IO_Update                       | const dual_arm_msgs::IO_Update msg                 | 设置机械臂数字 IO 输出状态              |
|                             | subscribe | /l_arm/rm_driver/JointPos                        | const dual_arm_msgs::JointPos msg                  | 关节角度 CANFD 透传                     |
|                             | subscribe | /l_arm/rm_driver/Joint_Enable                    | const dual_arm_msgs::Joint_Enable msg              | 设置关节使能状                          |
|                             | subscribe | /l_arm/rm_driver/Lift_GetState                   | const std_msgs::Empty msg                          | 获取升降机构状态                        |
|                             | subscribe | /l_arm/rm_driver/Lift_SetHeight                  | const dual_arm_msgs::Lift_Height msg               | 升降机构位置闭环控制                    |
|                             | subscribe | /l_arm/rm_driver/Lift_SetSpeed                   | const dual_arm_msgs::Lift_Speed msg                | 升降机构速度开环控制                    |
|                             | subscribe | /l_arm/rm_driver/ManualSetForcePose_Cmd          | const dual_arm_msgs::Manual_Set_Force_Pose msg     | 手动标定六维力数据                      |
|                             | subscribe | /l_arm/rm_driver/MoveC_Cmd                       | const dual_arm_msgs::MoveC msg                     | 笛卡尔空间圆弧运动                      |
|                             | subscribe | /l_arm/rm_driver/MoveJ_Cmd                       | const dual_arm_msgs::MoveJ msg                     | 关节空间运动                            |
|                             | subscribe | /l_arm/rm_driver/MoveJ_P_Cmd                     | const dual_arm_msgs::MoveJ_P msg                   | 关节空间规划到目标位姿                  |
|                             | subscribe | /l_arm/rm_driver/MoveL_Cmd                       | const dual_arm_msgs::MoveL msg                     | 笛卡尔空间直线运动                      |
|                             | subscribe | /l_arm/rm_driver/MoveP_Fd_Cmd                    | const dual_arm_msgs::CarteFdPose msg               | 位姿 CANFD 透传                         |
|                             | subscribe | /l_arm/rm_driver/SetArmPower                     | const std_msgs::Byte msg                           | 控制机械臂上电断电                      |
|                             | subscribe | /l_arm/rm_driver/SetForcePosition_Cmd            | const dual_arm_msgs::Set_Force_Position msg        | 力位混合控制                            |
|                             | subscribe | /l_arm/rm_driver/SetForceSensor_Cmd              | const std_msgs::Empty msg                          | 自动设置六维力重心参数                  |
|                             | subscribe | /l_arm/rm_driver/SetJointStep                    | const dual_arm_msgs::Joint_Step msg                | 关节步进                                |
|                             | subscribe | /l_arm/rm_driver/SetToolVoltage                  | const std_msgs::Byte msg                           | 设置工具端电压                          |
|                             | subscribe | /l_arm/rm_driver/Set_Realtime_Push               | const dual_arm_msgs::Set_Realtime_Push msg         | 设置 UDP 机械臂状态主动上报配置         |
|                             | subscribe | /l_arm/rm_driver/StartForcePositionMove_Cmd      | const std_msgs::Empty msg                          | 开启透传力位混合控制补偿模式            |
|                             | subscribe | /l_arm/rm_driver/StartMultiDragTeach_Cmd         | const dual_arm_msgs::Start_Multi_Drag_Teach msg    | 开启复合拖动示教                        |
|                             | subscribe | /l_arm/rm_driver/StopDragTeach_Cmd               | const std_msgs::Empty msg                          | 拖动示教停止                            |
|                             | subscribe | /l_arm/rm_driver/StopForcePositionMove_Cmd       | const std_msgs::Empty msg                          | 开启透传力位混合控制补偿模式            |
|                             | subscribe | /l_arm/rm_driver/StopForcePosition_Cmd           | const std_msgs::Empty msg                          | 结束力位混合控制                        |
|                             | subscribe | /l_arm/rm_driver/StopSetForceSensor_Cmd          | const std_msgs::Empty msg                          | 停止标定力传感器重心                    |
|                             | subscribe | /l_arm/rm_driver/Tool_Analog_Output              | const dual_arm_msgs::Tool_Analog_Output msg        | 设置末端工具模拟IO信号输出              |
|                             | subscribe | /l_arm/rm_driver/Tool_Digital_Output             | const dual_arm_msgs::Tool_Digital_Output msg       | 设置工具端数字 IO 输出状态              |
|                             |           |                                                  |                                                    |                                         |
|                             |           |                                                  |                                                    |                                         |
|                             | publish   | /l_arm/joint_states                              | sensor_msgs::JointState                            | 查询机械臂弧度数据返回结果              |
|                             | publish   | /l_arm/rm_driver/ArmCurrentState                 | dual_arm_msgs::ArmState                            | 查询机械臂状态（弧度+四元数）返回结果   |
|                             | publish   | /l_arm/rm_driver/ArmError                        | std_msgs::UInt16                                   | 查询机械臂错误返回结果                  |
|                             | publish   | /l_arm/rm_driver/Arm_Current_State               | dual_arm_msgs::Arm_Current_State                   | 查询机械臂状态（角度+欧拉角）返回结果   |
|                             | publish   | /l_arm/rm_driver/Arm_IO_State                    | dual_arm_msgs::Arm_IO_State                        | 获取所有 IO 输入状态返回结果            |
|                             | publish   | /l_arm/rm_driver/ChangeTool_State                | dual_arm_msgs::ChangeTool_State                    | 查询切换当前工具坐标系返回结果          |
|                             | publish   | /l_arm/rm_driver/ChangeWorkFrame_State           | dual_arm_msgs::ChangeWorkFrame_State               | 切换当前工作坐标系返回结果              |
|                             | publish   | /l_arm/rm_driver/ClearForceData_Result           | std_msgs::Bool                                     | 清空六维力数据返回结果                  |
|                             | publish   | /l_arm/rm_driver/ForceSensorSet_Result           | std_msgs::Bool                                     | 自动设置六维力重心参数返回结果          |
|                             | publish   | /l_arm/rm_driver/Force_Position_Move_Result      | std_msgs::Bool                                     | 透传力位混合控制返回结果                |
|                             | publish   | /l_arm/rm_driver/Force_Position_State            | dual_arm_msgs::Force_Position_State                | 透传力位混合控制状态                    |
|                             | publish   | /l_arm/rm_driver/Get_Arm_Software_Version_Result | dual_arm_msgs::Arm_Software_Version                | 读取软件版本号返回结果                  |
|                             | publish   | /l_arm/rm_driver/Get_Realtime_Push_Result        | dual_arm_msgs::Set_Realtime_Push                   | 查询 UDP 机械臂状态主动上报配置返回结果 |
|                             | publish   | /l_arm/rm_driver/JointErrorCode                  | dual_arm_msgs::Manual_Set_Force_Pose               | 关节错误返回结果                        |
|                             | publish   | /l_arm/rm_driver/Joint_Clear_Err_Result          | std_msgs::Bool                                     | 清除关节错误代码返回结果                |
|                             | publish   | /l_arm/rm_driver/Joint_Current                   | dual_arm_msgs::Joint_Current                       | 获取机械臂关节当前电流 返回结果         |
|                             | publish   | /l_arm/rm_driver/Joint_En_State_Result           | std_msgs::Bool                                     | 设置关节使能状态返回结果                |
|                             | publish   | /l_arm/rm_driver/LiftState                       | dual_arm_msgs::LiftState                           | 获取升降机构状态返回结果                |
|                             | publish   | /l_arm/rm_driver/Plan_State                      | dual_arm_msgs::Plan_State                          | 关节空间运动返回结果                    |
|                             | publish   | /l_arm/rm_driver/Pose_State                      | geometry_msgs::Pose                                | 位姿信息返回结果                        |
|                             | publish   | /l_arm/rm_driver/SetForcePosition_Result         | std_msgs::Bool                                     | 力位混合控制返回结果                    |
|                             | publish   | /l_arm/rm_driver/SetJointTeach_Result            | std_msgs::Bool                                     | 关节示教返回结果                        |
|                             | publish   | /l_arm/rm_driver/SetOrtTeach_Result              | std_msgs::Bool                                     | 姿态示教返回结果                        |
|                             | publish   | /l_arm/rm_driver/SetPosTeach_Result              | std_msgs::Bool                                     | 位置示教返回结果                        |
|                             | publish   | /l_arm/rm_driver/SetStopTeach_Result             | std_msgs::Bool                                     | 示教停止返回结果                        |
|                             | publish   | /l_arm/rm_driver/Set_AO_State_Result             | std_msgs::Bool                                     | 设置IO输出状态返回结果                  |
|                             | publish   | /l_arm/rm_driver/Set_Arm_Power_Result            | std_msgs::Bool                                     | 控制机械臂上电断电返回结果              |
|                             | publish   | /l_arm/rm_driver/Set_Arm_Stop_Result             | std_msgs::Bool                                     | 轨迹急停返回结果                        |
|                             | publish   | /l_arm/rm_driver/Set_DO_State_Result             | std_msgs::Bool                                     | 设置机械臂数字 IO 输出状态返回结果      |
|                             | publish   | /l_arm/rm_driver/Set_Gripper_Result              | std_msgs::Bool                                     | 设置夹爪持续力控夹取返回结果            |
|                             | publish   | /l_arm/rm_driver/Set_Hand_Angle_Result           | std_msgs::Bool                                     | 设置灵巧手各自由度角度返回结果          |
|                             | publish   | /l_arm/rm_driver/Set_Hand_Force_Result           | std_msgs::Bool                                     | 设置灵巧手力阈值返回结果                |
|                             | publish   | /l_arm/rm_driver/Set_Hand_Posture_Result         | std_msgs::Bool                                     | 设置灵巧手手势序号返回结果              |
|                             | publish   | /l_arm/rm_driver/Set_Hand_Seq_Result             | std_msgs::Bool                                     | 设置灵巧手动作序列返回结果              |
|                             | publish   | /l_arm/rm_driver/Set_Hand_Speed_Result           | std_msgs::Bool                                     | 设置灵巧手速度返回结果                  |
|                             | publish   | /l_arm/rm_driver/Set_Lift_Speed_Result           | std_msgs::Bool                                     | 升降机构速度开环控制返回结果            |
|                             | publish   | /l_arm/rm_driver/Set_Realtime_Push_Result        | std_msgs::Bool                                     | 设置 UDP 机械臂状态主动上报配置返回结果 |
|                             | publish   | /l_arm/rm_driver/Set_Tool_DO_State_Result        | std_msgs::Bool                                     | 设置工具端数字 IO 输出状态返回结果      |
|                             | publish   | /l_arm/rm_driver/Set_Tool_Voltage_Result         | std_msgs::Bool                                     | 设置工具端电压返回结果                  |
|                             | publish   | /l_arm/rm_driver/SixZeroForce                    | dual_arm_msgs::Six_Force                           | 查询六维力数据返回结果                  |
|                             | publish   | /l_arm/rm_driver/StartForcePositionMove_Result   | std_msgs::Bool                                     | 开启透传力位混合控制补偿模式返回结果    |
|                             | publish   | /l_arm/rm_driver/StartMultiDragTeach_Result      | std_msgs::Bool                                     | 开启复合拖动示教返回结果                |
|                             | publish   | /l_arm/rm_driver/StopDragTeach_Result            | std_msgs::Bool                                     | 拖动示教结束返回结果                    |
|                             | publish   | /l_arm/rm_driver/StopForcePositionMove_Result    | std_msgs::Bool                                     | 开启透传力位混合控制补偿模式返回结果    |
|                             | publish   | /l_arm/rm_driver/StopForcePosition_Result        | std_msgs::Bool                                     | 关闭力位混合控制补偿模式结果            |
|                             | publish   | /l_arm/rm_driver/StopSetForceSensor_Result       | std_msgs::Bool                                     | 停止标定力传感器重心返回结果            |
|                             | publish   | /l_arm/rm_driver/SysError                        | std_msgs::UInt16                                   | 系统错误                                |
|                             | publish   | /l_arm/rm_driver/System_En_State_Result          | std_msgs::Bool                                     | 清除系统错误返回结果                    |
|                             | publish   | /l_arm/rm_driver/ToolZeroForce                   | dual_arm_msgs::Six_Force                           | 工具坐标系下的传感器数据返回结果        |
|                             | publish   | /l_arm/rm_driver/Tool_IO_State                   | dual_arm_msgs::Tool_IO_State                       | 获取工具端数字 IO 状态返回结果          |
|                             | publish   | /l_arm/rm_driver/UdpSixForce                     | dual_arm_msgs::Six_Force                           | udp上报六维力数据                       |
|                             | publish   | /l_arm/rm_driver/UdpSixZeroForce                 | dual_arm_msgs::Six_Force                           | 当前六维力传感器系统外受力数据          |
|                             | publish   | /l_arm/rm_driver/Udp_Coordinate                  | std_msgs::UInt16                                   | 系统外受力数据参考坐标系                |
|                             | publish   | /l_arm/rm_driver/Udp_Pose_Euler                  | dual_arm_msgs::Arm_Pose_Euler                      | udp位姿数据（欧拉角）上报               |
|                             | publish   | /l_arm/rm_driver/WorkZeroForce                   | dual_arm_msgs::Six_Force                           | 工作坐标系下的传感器数据                |
|                             |           |                                                  |                                                    |                                         |
| 右臂                        | subscribe | /r_arm/rm_driver/Arm_Analog_Output               | const dual_arm_msgs::Arm_Analog_Output msg         | 设置机械臂模拟 IO 输出状态              |
|                             | subscribe | /r_arm/rm_driver/Arm_Digital_Output              | const dual_arm_msgs::Arm_Digital_Output msg        | 设置机械臂数字 IO 输出状态              |
|                             | subscribe | /r_arm/rm_driver/Arm_JointTeach                  | const dual_arm_msgs::Joint_Teach msg               | 关节示教                                |
|                             | subscribe | /r_arm/rm_driver/Arm_OrtTeach                    | const dual_arm_msgs::Ort_Teach msg                 | 姿态示教                                |
|                             | subscribe | /r_arm/rm_driver/Arm_PosTeach                    | const dual_arm_msgs::Pos_Teach msg                 | 位置示教                                |
|                             | subscribe | /r_arm/rm_driver/Arm_StopTeach                   | const dual_arm_msgs::Stop_Teach msg                | 示教停止                                |
|                             | subscribe | /r_arm/rm_driver/ChangeToolName_Cmd              | const dual_arm_msgs::ChangeTool_Name msg           | 切换当前工具坐标系                      |
|                             | subscribe | /r_arm/rm_driver/ChangeWorkFrame_Cmd             | const dual_arm_msgs::ChangeWorkFrame_Name msg      | 切换当前工作坐标系                      |
|                             | subscribe | /r_arm/rm_driver/ClearForceData_Cmd              | const std_msgs::Empty msg                          | 清空六维力数据                          |
|                             | subscribe | /r_arm/rm_driver/Clear_System_Err                | const std_msgs::Empty msg                          | 清除系统错误                            |
|                             | subscribe | /r_arm/rm_driver/Emergency_Stop                  | const std_msgs::Empty msg                          | 轨迹急停                                |
|                             | subscribe | /r_arm/rm_driver/ForcePositionMoveJiont_Cmd      | const dual_arm_msgs::Force_Position_Move_Joint msg | 透传力位混合控制补偿(弧度)              |
|                             | subscribe | /r_arm/rm_driver/ForcePositionMovePose_Cmd       | const dual_arm_msgs::Force_Position_Move_Pose msg  | 透传力位混合控制补偿(位姿)              |
|                             | subscribe | /r_arm/rm_driver/GetArmJoint_Cmd                 | const std_msgs::Empty msg                          | 查询机械臂关节角度                      |
|                             | subscribe | /r_arm/rm_driver/GetArmStateTimerSwitch          | const std_msgs::Bool msg                           | 选择开启udp数据上报                     |
|                             | subscribe | /r_arm/rm_driver/GetArmState_Cmd                 | const dual_arm_msgs::GetArmState_Command msg       | 查询机械臂状态（弧度+四元数）           |
|                             | subscribe | /r_arm/rm_driver/GetCurrentArmState              | const std_msgs::Empty msg                          | 查询当前机械臂状态（角度+欧拉角）       |
|                             | subscribe | /r_arm/rm_driver/GetCurrentJointCurrent          | const std_msgs::Empty msg                          | 获取机械臂关节当前电流                  |
|                             | subscribe | /r_arm/rm_driver/GetOneForce_Cmd                 | const std_msgs::Empty msg                          | 查询一维力数据指令                      |
|                             | subscribe | /r_arm/rm_driver/GetSixForce                     | const std_msgs::Empty msg                          | 查询六维力数据                          |
|                             | subscribe | /r_arm/rm_driver/GetSixForce_Cmd                 | const std_msgs::Empty msg                          | 查询六维力数据                          |
|                             | subscribe | /r_arm/rm_driver/GetTotalWorkFrame               | const std_msgs::Empty msg                          | 查询当前工作坐标系                      |
|                             | subscribe | /r_arm/rm_driver/Get_Arm_Software_Version        | const std_msgs::Empty msg                          | 读取软件版本号                          |
|                             | subscribe | /r_arm/rm_driver/Get_Realtime_Push               | const std_msgs::Empty msg                          | 查询 UDP 机械臂状态主动上报配置         |
|                             | subscribe | /r_arm/rm_driver/Gripper_Pick                    | dual_arm_msgs::Gripper_Pick msg                    | 设置夹爪力控夹取                        |
|                             | subscribe | /r_arm/rm_driver/Gripper_Pick_On                 | dual_arm_msgs::Gripper_Pick msg                    | 设置夹爪持续力控夹取                    |
|                             | subscribe | /r_arm/rm_driver/Gripper_Set                     | dual_arm_msgs::Gripper_Set msg                     | 设置夹爪行程量                          |
|                             | subscribe | /r_arm/rm_driver/Hand_SetAngle                   | const dual_arm_msgs::Hand_Angle msg                | 设置灵巧手角度                          |
|                             | subscribe | /r_arm/rm_driver/Hand_SetForce                   | const dual_arm_msgs::Hand_Force msg                | 设置灵巧手关节力阈值                    |
|                             | subscribe | /r_arm/rm_driver/Hand_SetPosture                 | const dual_arm_msgs::Hand_Posture msg              | 设置灵巧手手势序号                      |
|                             | subscribe | /r_arm/rm_driver/Hand_SetSeq                     | const dual_arm_msgs::Hand_Seq msg                  | 设置灵巧手动作序列                      |
|                             | subscribe | /r_arm/rm_driver/Hand_SetSpeed                   | const dual_arm_msgs::Hand_Speed msg                | 设置灵巧手关节速度                      |
|                             | subscribe | /r_arm/rm_driver/IO_Update                       | const dual_arm_msgs::IO_Update msg                 | 设置机械臂数字 IO 输出状态              |
|                             | subscribe | /r_arm/rm_driver/JointPos                        | const dual_arm_msgs::JointPos msg                  | 关节角度 CANFD 透传                     |
|                             | subscribe | /r_arm/rm_driver/Joint_Enable                    | const dual_arm_msgs::Joint_Enable msg              | 设置关节使能状                          |
|                             | subscribe | /r_arm/rm_driver/Lift_GetState                   | const std_msgs::Empty msg                          | 获取升降机构状态                        |
|                             | subscribe | /r_arm/rm_driver/Lift_SetHeight                  | const dual_arm_msgs::Lift_Height msg               | 升降机构位置闭环控制                    |
|                             | subscribe | /r_arm/rm_driver/Lift_SetSpeed                   | const dual_arm_msgs::Lift_Speed msg                | 升降机构速度开环控制                    |
|                             | subscribe | /r_arm/rm_driver/ManualSetForcePose_Cmd          | const dual_arm_msgs::Manual_Set_Force_Pose msg     | 手动标定六维力数据                      |
|                             | subscribe | /r_arm/rm_driver/MoveC_Cmd                       | const dual_arm_msgs::MoveC msg                     | 笛卡尔空间圆弧运动                      |
|                             | subscribe | /r_arm/rm_driver/MoveJ_Cmd                       | const dual_arm_msgs::MoveJ msg                     | 关节空间运动                            |
|                             | subscribe | /r_arm/rm_driver/MoveJ_P_Cmd                     | const dual_arm_msgs::MoveJ_P msg                   | 关节空间规划到目标位姿                  |
|                             | subscribe | /r_arm/rm_driver/MoveL_Cmd                       | const dual_arm_msgs::MoveL msg                     | 笛卡尔空间直线运动                      |
|                             | subscribe | /r_arm/rm_driver/MoveP_Fd_Cmd                    | const dual_arm_msgs::CarteFdPose msg               | 位姿 CANFD 透传                         |
|                             | subscribe | /r_arm/rm_driver/SetArmPower                     | const std_msgs::Byte msg                           | 控制机械臂上电断电                      |
|                             | subscribe | /r_arm/rm_driver/SetForcePosition_Cmd            | const dual_arm_msgs::Set_Force_Position msg        | 力位混合控制                            |
|                             | subscribe | /r_arm/rm_driver/SetForceSensor_Cmd              | const std_msgs::Empty msg                          | 自动设置六维力重心参数                  |
|                             | subscribe | /r_arm/rm_driver/SetJointStep                    | const dual_arm_msgs::Joint_Step msg                | 关节步进                                |
|                             | subscribe | /r_arm/rm_driver/SetToolVoltage                  | const std_msgs::Byte msg                           | 设置工具端电压                          |
|                             | subscribe | /r_arm/rm_driver/Set_Realtime_Push               | const dual_arm_msgs::Set_Realtime_Push msg         | 设置 UDP 机械臂状态主动上报配置         |
|                             | subscribe | /r_arm/rm_driver/StartForcePositionMove_Cmd      | const std_msgs::Empty msg                          | 开启透传力位混合控制补偿模式            |
|                             | subscribe | /r_arm/rm_driver/StartMultiDragTeach_Cmd         | const dual_arm_msgs::Start_Multi_Drag_Teach msg    | 开启复合拖动示教                        |
|                             | subscribe | /r_arm/rm_driver/StopDragTeach_Cmd               | const std_msgs::Empty msg                          | 拖动示教停止                            |
|                             | subscribe | /r_arm/rm_driver/StopForcePositionMove_Cmd       | const std_msgs::Empty msg                          | 开启透传力位混合控制补偿模式            |
|                             | subscribe | /r_arm/rm_driver/StopForcePosition_Cmd           | const std_msgs::Empty msg                          | 结束力位混合控制                        |
|                             | subscribe | /r_arm/rm_driver/StopSetForceSensor_Cmd          | const std_msgs::Empty msg                          | 停止标定力传感器重心                    |
|                             | subscribe | /r_arm/rm_driver/Tool_Analog_Output              | const dual_arm_msgs::Tool_Analog_Output msg        | 设置末端工具模拟IO信号输出              |
|                             | subscribe | /r_arm/rm_driver/Tool_Digital_Output             | const dual_arm_msgs::Tool_Digital_Output msg       | 设置工具端数字 IO 输出状态              |
|                             |           |                                                  |                                                    |                                         |
|                             |           |                                                  |                                                    |                                         |
|                             | publish   | /r_arm/joint_states                              | sensor_msgs::JointState                            | 查询机械臂弧度数据返回结果              |
|                             | publish   | /r_arm/rm_driver/ArmCurrentState                 | dual_arm_msgs::ArmState                            | 查询机械臂状态（弧度+四元数）返回结果   |
|                             | publish   | /r_arm/rm_driver/ArmError                        | std_msgs::UInt16                                   | 查询机械臂错误返回结果                  |
|                             | publish   | /r_arm/rm_driver/Arm_Current_State               | dual_arm_msgs::Arm_Current_State                   | 查询机械臂状态（角度+欧拉角）返回结果   |
|                             | publish   | /r_arm/rm_driver/Arm_IO_State                    | dual_arm_msgs::Arm_IO_State                        | 获取所有 IO 输入状态返回结果            |
|                             | publish   | /r_arm/rm_driver/ChangeTool_State                | dual_arm_msgs::ChangeTool_State                    | 查询切换当前工具坐标系返回结果          |
|                             | publish   | /r_arm/rm_driver/ChangeWorkFrame_State           | dual_arm_msgs::ChangeWorkFrame_State               | 切换当前工作坐标系返回结果              |
|                             | publish   | /r_arm/rm_driver/ClearForceData_Result           | std_msgs::Bool                                     | 清空六维力数据返回结果                  |
|                             | publish   | /r_arm/rm_driver/ForceSensorSet_Result           | std_msgs::Bool                                     | 自动设置六维力重心参数返回结果          |
|                             | publish   | /r_arm/rm_driver/Force_Position_Move_Result      | std_msgs::Bool                                     | 透传力位混合控制返回结果                |
|                             | publish   | /r_arm/rm_driver/Force_Position_State            | dual_arm_msgs::Force_Position_State                | 透传力位混合控制状态                    |
|                             | publish   | /r_arm/rm_driver/Get_Arm_Software_Version_Result | dual_arm_msgs::Arm_Software_Version                | 读取软件版本号返回结果                  |
|                             | publish   | /r_arm/rm_driver/Get_Realtime_Push_Result        | dual_arm_msgs::Set_Realtime_Push                   | 查询 UDP 机械臂状态主动上报配置返回结果 |
|                             | publish   | /r_arm/rm_driver/JointErrorCode                  | dual_arm_msgs::Manual_Set_Force_Pose               | 关节错误返回结果                        |
|                             | publish   | /r_arm/rm_driver/Joint_Clear_Err_Result          | std_msgs::Bool                                     | 清除关节错误代码返回结果                |
|                             | publish   | /r_arm/rm_driver/Joint_Current                   | dual_arm_msgs::Joint_Current                       | 获取机械臂关节当前电流 返回结果         |
|                             | publish   | /r_arm/rm_driver/Joint_En_State_Result           | std_msgs::Bool                                     | 设置关节使能状态返回结果                |
|                             | publish   | /r_arm/rm_driver/LiftState                       | dual_arm_msgs::LiftState                           | 获取升降机构状态返回结果                |
|                             | publish   | /r_arm/rm_driver/Plan_State                      | dual_arm_msgs::Plan_State                          | 关节空间运动返回结果                    |
|                             | publish   | /r_arm/rm_driver/Pose_State                      | geometry_msgs::Pose                                | 位姿信息返回结果                        |
|                             | publish   | /r_arm/rm_driver/SetForcePosition_Result         | std_msgs::Bool                                     | 力位混合控制返回结果                    |
|                             | publish   | /r_arm/rm_driver/SetJointTeach_Result            | std_msgs::Bool                                     | 关节示教返回结果                        |
|                             | publish   | /r_arm/rm_driver/SetOrtTeach_Result              | std_msgs::Bool                                     | 姿态示教返回结果                        |
|                             | publish   | /r_arm/rm_driver/SetPosTeach_Result              | std_msgs::Bool                                     | 位置示教返回结果                        |
|                             | publish   | /r_arm/rm_driver/SetStopTeach_Result             | std_msgs::Bool                                     | 示教停止返回结果                        |
|                             | publish   | /r_arm/rm_driver/Set_AO_State_Result             | std_msgs::Bool                                     | 设置IO输出状态返回结果                  |
|                             | publish   | /r_arm/rm_driver/Set_Arm_Power_Result            | std_msgs::Bool                                     | 控制机械臂上电断电返回结果              |
|                             | publish   | /r_arm/rm_driver/Set_Arm_Stop_Result             | std_msgs::Bool                                     | 轨迹急停返回结果                        |
|                             | publish   | /r_arm/rm_driver/Set_DO_State_Result             | std_msgs::Bool                                     | 设置机械臂数字 IO 输出状态返回结果      |
|                             | publish   | /r_arm/rm_driver/Set_Gripper_Result              | std_msgs::Bool                                     | 设置夹爪持续力控夹取返回结果            |
|                             | publish   | /r_arm/rm_driver/Set_Hand_Angle_Result           | std_msgs::Bool                                     | 设置灵巧手各自由度角度返回结果          |
|                             | publish   | /r_arm/rm_driver/Set_Hand_Force_Result           | std_msgs::Bool                                     | 设置灵巧手力阈值返回结果                |
|                             | publish   | /r_arm/rm_driver/Set_Hand_Posture_Result         | std_msgs::Bool                                     | 设置灵巧手手势序号返回结果              |
|                             | publish   | /r_arm/rm_driver/Set_Hand_Seq_Result             | std_msgs::Bool                                     | 设置灵巧手动作序列返回结果              |
|                             | publish   | /r_arm/rm_driver/Set_Hand_Speed_Result           | std_msgs::Bool                                     | 设置灵巧手速度返回结果                  |
|                             | publish   | /r_arm/rm_driver/Set_Lift_Speed_Result           | std_msgs::Bool                                     | 升降机构速度开环控制返回结果            |
|                             | publish   | /r_arm/rm_driver/Set_Realtime_Push_Result        | std_msgs::Bool                                     | 设置 UDP 机械臂状态主动上报配置返回结果 |
|                             | publish   | /r_arm/rm_driver/Set_Tool_DO_State_Result        | std_msgs::Bool                                     | 设置工具端数字 IO 输出状态返回结果      |
|                             | publish   | /r_arm/rm_driver/Set_Tool_Voltage_Result         | std_msgs::Bool                                     | 设置工具端电压返回结果                  |
|                             | publish   | /r_arm/rm_driver/SixZeroForce                    | dual_arm_msgs::Six_Force                           | 查询六维力数据返回结果                  |
|                             | publish   | /r_arm/rm_driver/StartForcePositionMove_Result   | std_msgs::Bool                                     | 开启透传力位混合控制补偿模式返回结果    |
|                             | publish   | /r_arm/rm_driver/StartMultiDragTeach_Result      | std_msgs::Bool                                     | 开启复合拖动示教返回结果                |
|                             | publish   | /r_arm/rm_driver/StopDragTeach_Result            | std_msgs::Bool                                     | 拖动示教结束返回结果                    |
|                             | publish   | /r_arm/rm_driver/StopForcePositionMove_Result    | std_msgs::Bool                                     | 开启透传力位混合控制补偿模式返回结果    |
|                             | publish   | /r_arm/rm_driver/StopForcePosition_Result        | std_msgs::Bool                                     | 关闭力位混合控制补偿模式结果            |
|                             | publish   | /r_arm/rm_driver/StopSetForceSensor_Result       | std_msgs::Bool                                     | 停止标定力传感器重心返回结果            |
|                             | publish   | /r_arm/rm_driver/SysError                        | std_msgs::UInt16                                   | 系统错误                                |
|                             | publish   | /r_arm/rm_driver/System_En_State_Result          | std_msgs::Bool                                     | 清除系统错误返回结果                    |
|                             | publish   | /r_arm/rm_driver/ToolZeroForce                   | dual_arm_msgs::Six_Force                           | 工具坐标系下的传感器数据返回结果        |
|                             | publish   | /r_arm/rm_driver/Tool_IO_State                   | dual_arm_msgs::Tool_IO_State                       | 获取工具端数字 IO 状态返回结果          |
|                             | publish   | /r_arm/rm_driver/UdpSixForce                     | dual_arm_msgs::Six_Force                           | udp上报六维力数据                       |
|                             | publish   | /r_arm/rm_driver/UdpSixZeroForce                 | dual_arm_msgs::Six_Force                           | 当前六维力传感器系统外受力数据          |
|                             | publish   | /r_arm/rm_driver/Udp_Coordinate                  | std_msgs::UInt16                                   | 系统外受力数据参考坐标系                |
|                             | publish   | /r_arm/rm_driver/Udp_Pose_Euler                  | dual_arm_msgs::Arm_Pose_Euler                      | udp位姿数据（欧拉角）上报               |
|                             | publish   | /r_arm/rm_driver/WorkZeroForce                   | dual_arm_msgs::Six_Force                           | 工作坐标系下的传感器数据                |
|                             |           |                                                  |                                                    |                                         |
| camera_show（相机节点）     | publish   | /camera0/image_raw                               | sensor_msgs::Image                                 | 用来发布前usb相机的颜色帧               |
|                             | publish   | /camera1/image_raw                               | sensor_msgs::Image                                 | 用来发布后usb相机的颜色帧               |
|                             | publish   | /camera_0/color/image_raw                        | sensor_msgs::Image                                 | 用来发布前第一个D435相机的颜色帧        |
|                             | publish   | /camera_0/depth/image_raw                        | sensor_msgs::Image                                 | 用来发布前第一个D435相机的深度帧        |
|                             | publish   | /camera_1/color/image_raw                        | sensor_msgs::Image                                 | 用来发布前第二个D435相机的颜色帧        |
|                             | publish   | /camera_1/depth/image_raw                        | sensor_msgs::Image                                 | 用来发布前第二个D435相机的深度帧        |
|                             | publish   | /camera_2/color/image_raw                        | sensor_msgs::Image                                 | 用来发布前第三个D435相机的颜色帧        |
|                             | publish   | /camera_2/depth/image_raw                        | sensor_msgs::Image                                 | 用来发布前第三个D435相机的深度帧        |