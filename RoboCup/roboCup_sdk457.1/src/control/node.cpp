#include "control/node.h"

BT::NodeStatus camToPosition::tick()
{
    double joint0_angle, joint1_angle, duration;
    getInput("joint0_angle", joint0_angle);
    getInput("joint1_angle", joint1_angle);
    getInput("duration", duration);

    if(!firstRun)
    {
        initAngle<<_interface->servoState->msg_.states()[0].q(),_interface->servoState->msg_.states()[1].q();
        targetAngle.Zero(2);
        targetAngle<<joint0_angle,joint1_angle;
        firstRun = true;
    }

    percent += (float)1 / duration;
    percent = percent > 1 ? 1 : percent;

    int desired0_position = (1 - percent) * initAngle(0) + percent * targetAngle(0);
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    int desired1_position = (1 - percent) * initAngle(1) + percent * targetAngle(1);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = desired1_position;
    _interface->servoCmd->unlockAndPublish();

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus camFindBall::tick()
{
    using Clock = std::chrono::steady_clock;
    // —— 可调参数（角度单位：度 / 角速度单位：弧度每秒）——
    constexpr float YAW_MIN = -50.0f, YAW_MAX = 50.0f;     // 舵机限位（与你的 config 相符）
    constexpr float PIT_MIN = -20.0f, PIT_MAX = 85.0f;
    constexpr float LOCAL_A0_Y = 10.0f, LOCAL_A0_P = 8.0f; // 刚丢球时的局部扫幅
    constexpr float LOCAL_GROW_Y = 25.0f, LOCAL_GROW_P = 16.0f; // 扫幅增长率（°/s）
    constexpr float GLOBAL_MAX_Y = 45.0f, GLOBAL_MAX_P = 25.0f; // 全局最大扫幅
    constexpr float OMEGA = 1.6f;           // 基本扫描角频（rad/s）
    constexpr float FREQ_RATIO = 1.7f;      // 纵向频率比，非整数避免重复轨迹
    constexpr float PHASE_DRIFT = 0.6f;     // 相位漂移速度（rad/s），填补死角
    constexpr float RECENT_LOST_T = 1.0f;   // 丢球 1s 内按“局部搜索”，之后全局
    constexpr float BASE_VYAW = 0.25f;      // 底盘配合转的最大角速度（rad/s）
    constexpr float BASE_HELP_EDGE = 40.0f; // 相机 yaw 超过此角度时带动底盘
    constexpr int   SEEN_OK = 2, LOST_OK = 5; // 去抖阈值（连续帧）

    // —— 跨 tick 的状态 ——（函数静态局部，避免改头文件）
    static bool inited = false;
    static float yaw_center = 0.0f, pit_center = 0.0f; // 搜索中心（刚丢球时的角度）
    static float yaw_last = 0.0f, pit_last = 0.0f;     // 记录当前输出，用于平滑
    static float phi = 0.0f;                            // Lissajous 相位
    static int seen_cnt = 0, lost_cnt = 0;
    static Clock::time_point t0 = Clock::now(), last = Clock::now();
    static Clock::time_point last_seen_tp = Clock::now();
    if (!inited) { // 第一次进入时初始化
        yaw_center = _interface->servoState->msg_.states()[0].q();
        pit_center = _interface->servoState->msg_.states()[1].q();
        yaw_last = yaw_center; 
        pit_last = pit_center;
        t0 = last = last_seen_tp = Clock::now();
        inited = true;
    }

    // —— 检测去抖 —— 
    if (_interface->ballDetected) {
        seen_cnt++; lost_cnt = 0;
        last_seen_tp = Clock::now();
    } else {
        lost_cnt++; seen_cnt = 0;
    }
    const bool ball_stable_seen = (seen_cnt >= SEEN_OK);
    const bool ball_stable_lost = (lost_cnt >= LOST_OK);

    if (ball_stable_seen) {
        // 看到球：重置搜索器，直接成功返回
        //（让下一次“丢球”从当前角度开始局部搜索）
        yaw_center = _interface->servoState->msg_.states()[0].q();
        pit_center = _interface->servoState->msg_.states()[1].q();
        yaw_last = yaw_center;
        pit_last = pit_center;
        phi = 0.0f;
        _interface->locoClient.Move(0, 0, 0); // 看到球就不再空转底盘
        return BT::NodeStatus::SUCCESS;
    }

    // —— 时间步长 —— 
    Clock::time_point now = Clock::now();
    float dt = std::chrono::duration<float>(now - last).count();
    dt = std::clamp(dt, 0.0f, 0.05f); // 防止异常大步长
    last = now;

    // —— 选用局部/全局搜索策略 —— 
    float since_lost = std::chrono::duration<float>(now - last_seen_tp).count();
    float Ay, Ap;        // 扫描幅度（度）
    float yc, pc;        // 中心
    if (since_lost < RECENT_LOST_T) {
        // 局部螺旋：围绕最后看见的位置逐步放大
        float g = std::clamp(since_lost / RECENT_LOST_T, 0.0f, 1.0f);
        Ay = std::min(LOCAL_A0_Y + LOCAL_GROW_Y * since_lost, GLOBAL_MAX_Y);
        Ap = std::min(LOCAL_A0_P + LOCAL_GROW_P * since_lost, GLOBAL_MAX_P);
        yc = yaw_center; pc = pit_center;
    } else {
        // 全局 Lissajous：横向大扫，纵向不同频率 + 相位漂移，覆盖无死角
        Ay = GLOBAL_MAX_Y;
        Ap = GLOBAL_MAX_P;
        yc = 0.0f;       // 回到全局中心（可改成偏下略俯视：如 -5 度）
        pc = -5.0f;
    }

    // —— 生成目标角：Lissajous + 速度成形（sin^3 提升中区速度感）——
    static float t = 0.0f;
    t += OMEGA * dt;
    phi += PHASE_DRIFT * dt;

    auto sin3 = [](float x){ float s = std::sin(x); return s*s*s; };

    float yaw_cmd = yc + Ay * sin3(t);
    float pit_cmd = pc + Ap * std::sin(FREQ_RATIO * t + phi);

    // —— 限幅 & 平滑 —— 
    yaw_cmd = std::clamp(yaw_cmd, YAW_MIN, YAW_MAX);
    pit_cmd = std::clamp(pit_cmd, PIT_MIN, PIT_MAX);

    // 给一点一阶平滑，避免抖：限制单步最大变化（°/tick）
    const float MAX_STEP = 6.0f; // 每 tick 最多改 6°
    float dy = std::clamp(yaw_cmd - yaw_last, -MAX_STEP, MAX_STEP);
    float dp = std::clamp(pit_cmd - pit_last, -MAX_STEP, MAX_STEP);
    yaw_last += dy;
    pit_last += dp;

    // —— 下发相机舵机 —— 
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q()    = yaw_last;
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q()    = pit_last;
    _interface->servoCmd->unlockAndPublish();

    // —— 底盘配合：只有当相机 yaw 接近边缘时才轻微带动底盘 —— 
    float vyaw = 0.0f;
    if (std::fabs(yaw_last) > BASE_HELP_EDGE) {
        vyaw = BASE_VYAW * ((yaw_last > 0) ? 1.0f : -1.0f);
    }
    // 如果还处在“刚丢球且未去抖稳定”，就暂不转底盘，避免和别的节点打架
    if (!ball_stable_lost) vyaw = 0.0f;

    _interface->locoClient.Move(0.0, 0.0, vyaw);

    return BT::NodeStatus::SUCCESS;
}




BT::NodeStatus robotTrackPelvis::tick()
{
    if (!_interface->ballDetected)
    {
        _interface->locoClient.Move(0, 0, 0);
        return BT::NodeStatus::SUCCESS;
    }
    if ((abs(_interface->ballYawToPelvis)<=0.20)&&(abs(_interface->ballPositionInPelvis(0)) <= 0.55)&& (abs(_interface->ballPositionInPelvis(1)) <= 0.3))
    {
        _interface->locoClient.Move(0, 0, 0);
    }
    else
    {
        double vx = _interface->ballPositionInPelvis(0);
        double vy = _interface->ballPositionInPelvis(1);

        const double linearFactorSlope = 3.0;     
        const double linearFactorOffset = 3.0;     
        double linearFactor = 1.0 / (1.0 + exp(
            linearFactorSlope * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - linearFactorOffset
        ));

        vx *= linearFactor;
        vy *= linearFactor;
        vx = saturation(vx, Vec2<double>(-0.8,1.0));
        vy = saturation(vy, Vec2<double>(-0.8,1.0));
        double vyaw = _interface->ballYawToPelvis;
        vyaw = saturation(vyaw, Vec2<double>(-0.5,0.5));

        _interface->locoClient.Move(vx, vy,vyaw);
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus robotTrackField::tick()
{
    double vx_chase = _interface->ballPositionInPelvis(0);
    double vy_chase = _interface->ballPositionInPelvis(1);

    double linearFactor = 1 / (1 + exp(3 * (_interface->ball_range_selected * fabs(_interface->ballYawToPelvis)) - 3));
    vx_chase *= linearFactor;
    vy_chase *= linearFactor;

    
    vx_chase = saturation(vx_chase, Vec2<double>(-0.6,0.6));
    vy_chase = saturation(vy_chase, Vec2<double>(-0.6,0.6));

    double vyaw_chase= _interface->ballYawToPelvis;
    vyaw_chase = saturation(vyaw_chase, Vec2<double>(-0.5,0.5));
 
    Vec2<double> vec_goal_ball_field;
    vec_goal_ball_field(0) = 4.5 - _interface->ballPositionInField(0);
    vec_goal_ball_field(1) = 0 - _interface->ballPositionInField(1);

    double angle_goal_ball_field = atan2(vec_goal_ball_field(1),vec_goal_ball_field(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);

    double angle_robot_ball_field = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double deltaDir = angle_goal_ball_field - angle_robot_ball_field;

    double dir = deltaDir > 0 ? -1.0 : 1.0;

    double s = 0.4;
    double r = 0.8;

    double ballYawToPelvis = _interface->ballYawToPelvis;
    double vtheta = (ballYawToPelvis - dir * s) / r;

    double vx_adjust = 0, vy_adjust = 0;
    vx_adjust = -s * dir * sin(ballYawToPelvis);
    vy_adjust = s * dir * cos(ballYawToPelvis);
    vy_adjust = saturation(vy_adjust, Vec2<double>(-0.4,0.4));
    // std::cout<<"debug \n"<<std::endl;
    // std::cout<<"vy: "<<vy<<std::endl;
    // std::cout<<"deltaDir: "<<deltaDir<<std::endl;
    // std::cout<<"angle_goal_ball_field: "<<rad2deg(angle_goal_ball_field)<<std::endl;
    // std::cout<<"angle_robot_ball_field: "<<rad2deg(angle_robot_ball_field)<<std::endl;
    double vyaw_adjust = vtheta;
    vyaw_adjust = saturation(vtheta, Vec2<double>(-0.4,0.4));



    double d_switch = 1.5; // 切换距离，可根据需求调整
    double ballRange = _interface->ball_range_selected;
    double w_chase = std::clamp(ballRange / d_switch, 0.0, 1.0);
    double w_orbit = 1.0 - w_chase;
    // std::cout<<std::endl;
    // std::cout<<"debug d_switch: "<<d_switch<<std::endl;
    // std::cout<<"debug w_orbit: "<<w_orbit<<std::endl;
    // std::cout<<"ballRange: "<<ballRange<<std::endl;
    // std::cout<<"deltaDir: "<<deltaDir<<std::endl;
    // std::cout<<"vec_goal_ball_field(1): "<<vec_goal_ball_field(1)<<std::endl;

    double vx = w_chase * vx_chase + w_orbit * vx_adjust;
    double vy = w_chase * vy_chase + w_orbit * vy_adjust;
    double vyaw = w_chase * vyaw_chase + w_orbit * vyaw_adjust;

    _interface->locoClient.Move(vx, vy,vyaw);

    // if((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55)&& (abs(_interface->ballPositionInPelvis(1)) <= 0.3)&&(fabs(deltaDir) < 0.20))
    // {
    //     // _interface->locoClient.Move(0, 0, 0);
    //     _interface->trackDone = true;
    // }
    // else 
    // {
        
    //     _interface->locoClient.Move(vx, vy,vyaw);
    //     _interface->trackDone = false;
    // }


    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus kick::tick()
{
    Vec2<double> leftGoalField;
    leftGoalField(0) = 4.5;
    leftGoalField(1) = 2.6 / 2;

    Vec2<double> rightGoalField;
    rightGoalField(0) = 4.5;
    rightGoalField(1) = -2.6 / 2;

    double margin = 0.3;

    Vec2<double> vecBallLeftGoalField;
    vecBallLeftGoalField(0) = leftGoalField(0) - _interface->ballPositionInField(0);
    vecBallLeftGoalField(1) = leftGoalField(1) - margin - _interface->ballPositionInField(1);
    double angleballLeftGoalField = atan2(vecBallLeftGoalField(1),vecBallLeftGoalField(0));

    Vec2<double> vecBallRightGoalField;
    vecBallRightGoalField(0) = rightGoalField(0) - _interface->ballPositionInField(0);
    vecBallRightGoalField(1) = rightGoalField(1) + margin - _interface->ballPositionInField(1);
    double angleballRightGoalField = atan2(vecBallRightGoalField(1),vecBallRightGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    double angleRobotBallField = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    if((angleRobotBallField<angleballLeftGoalField) && (angleRobotBallField>angleballRightGoalField))
    {
        std::cout<<"[node::kick] The shooting angle looks good "<<std::endl;
    }
    else
    {
        std::cout<<"[node::kick] The shooting angle doesn’t look good "<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    double ballYawToPelvis = _interface->ballYawToPelvis;
    double p = 0.6;
    double vx = 0,vy = 0;
    vx = p * cos(ballYawToPelvis);
    vy = p * sin(ballYawToPelvis);

    vx = saturation(vx, Vec2<double>(-0.6,0.6));
    vy = saturation(vy, Vec2<double>(-0.6,0.6));

    double vyaw = _interface->ballYawToPelvis;
    vyaw = saturation(vyaw, Vec2<double>(-0.6,0.6));
    
    if(_interface->ballPositionInField(0)>=4.5)
    {
            _interface->locoClient.Move(0,0,0);
    }
    else
    {
            _interface->locoClient.Move(vx,vy,0);
    }
    
   return BT::NodeStatus::SUCCESS;
}



BT::NodeStatus camTrackBall::tick()
{
    if (!_interface->ballDetected)
    {   
        if (!is_recovering_) {
            target_yaw_ = 0.0f;
            target_pitch_ = 0.0f;
            is_recovering_ = true;
        }

        // 获取当前舵机角度
        float current_yaw = _interface->servoState->msg_.states()[0].q();
        float current_pitch = _interface->servoState->msg_.states()[1].q();

        // 计算渐进步长 (基于时间步长dt，假设dt≈16ms/60Hz)
        float dt = 0.016; 
        float max_delta = recovery_speed_ * dt;

        // Yaw轴渐进逼近
        if (fabs(current_yaw - target_yaw_) > max_delta) {
            current_yaw += (current_yaw > target_yaw_) ? -max_delta : max_delta;
        } else {
            current_yaw = target_yaw_;
        }

        // Pitch轴同理
        if (fabs(current_pitch - target_pitch_) > max_delta) {
            current_pitch += (current_pitch > target_pitch_) ? -max_delta : max_delta;
        } else {
            current_pitch = target_pitch_;
        }

        // 检查归零是否完成
        if (current_yaw == target_yaw_ && current_pitch == target_pitch_) {
            is_recovering_ = false; // 归零完成
        }

        // 发送渐进角度指令
        _interface->servoCmd->msg_.cmds()[0].q() = current_yaw;
        _interface->servoCmd->msg_.cmds()[1].q() = current_pitch;
        _interface->servoCmd->unlockAndPublish();
        return BT::NodeStatus::SUCCESS;
        
    }
    
    is_recovering_ = false; 
    float fov_x = _interface->ball_offset_fov(0);
    float fov_y = _interface->ball_offset_fov(1);

    yaw_angle_add = fov_x * 0.6;
    pitch_angle_add = fov_y * 0.6;
    // printf("yawadd：%3.f\n",yaw_angle_add);
    // printf("pitchadd：%3.f\n",pitch_angle_add);
    const float MAX_ANGULAR_RATE = 10;
    const float CONTROL_DT = 0.02;
    const float max_delta = MAX_ANGULAR_RATE * CONTROL_DT;

    float target_yaw = _interface->servoState->msg_.states()[0].q()-yaw_angle_add;
    float target_pitch = _interface->servoState->msg_.states()[1].q()+ pitch_angle_add;

    float delta_yaw = target_yaw - last_yaw_;
    delta_yaw = std::clamp(delta_yaw, -max_delta, max_delta);
    float smooth_yaw = last_yaw_ + delta_yaw;

    float delta_pitch = target_pitch - last_pitch_;
    delta_pitch = std::clamp(delta_pitch, -max_delta, max_delta);
    float smooth_pitch = last_pitch_ + delta_pitch;

    last_yaw_ = smooth_yaw;
    last_pitch_ = smooth_pitch;
    
    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q() = smooth_yaw;
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q() = smooth_pitch;
    _interface->servoCmd->unlockAndPublish();
    // printf("control_yaw:%.2f\n",_interface->servoCmd->msg_.cmds()[0].q());
    // printf("control_pitch:%.2f\n",_interface->servoCmd->msg_.cmds()[1].q());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus playerDecision::tick()
{   
    std::string decision;
    bool goalSignal;
    if(_interface->ballPositionInField(0)>4.5) // Here, we simply treat the situation where the ball’s x-direction distance in the Field coordinate system is greater than 4.5 as a goal signal.
    {
        goalSignal = true;
    }
    else
    {
        goalSignal = false;
    }

    bool enableCamFindBallNode;
    if(!_interface->ballDetected)
    {
        enableCamFindBallNode = true;
    }
    else 
    {
        enableCamFindBallNode = false;
    }

    Vec2<double> goalField;
    goalField(0) = 4.5;
    goalField(1) = 0;
    bool enableRobotTrackBallNode;
    bool enableRobotTrackFieldNode;
    Vec2<double> vecBallGoalField;
    vecBallGoalField(0) = goalField(0) - _interface->ballPositionInField(0);
    vecBallGoalField(1) = goalField(1) - _interface->ballPositionInField(1);
    double angleBallGoalField = atan2(vecBallGoalField(1),vecBallGoalField(0));

    Vec2<double> vecPelvisBallField;
    vecPelvisBallField(0) = _interface->ballPositionInField(0) - _interface->homoMatPelvisToField(0,2);
    vecPelvisBallField(1) = _interface->ballPositionInField(1) - _interface->homoMatPelvisToField(1,2);
    double anglePlevisBallfield = atan2(vecPelvisBallField(1),vecPelvisBallField(0));

    double biasAngle = angleBallGoalField - anglePlevisBallfield;
    if(goalSignal|| ((abs(_interface->ballYawToPelvis)<=0.20) && (abs(_interface->ballPositionInPelvis(0)) <= 0.55) && (abs(_interface->ballPositionInPelvis(1)) <= 0.3) && (fabs(biasAngle) < 0.20) ))
    {
       enableRobotTrackFieldNode = false;
    }
    else 
    {
       enableRobotTrackFieldNode = true;
    }
    // if(!_interface->start_signal || _interface->pause_signal){
    //     enableRobotTrackBallNode = false;
    // }else{
    //     enableRobotTrackBallNode = true;
    // }

    // if(enableRobotTrackBallNode)
    // {
    //     decision = "camTrackBall";
    // }
    if(enableCamFindBallNode)
    {
        decision = "camFindBall";
    }
    else if(enableRobotTrackFieldNode)
    {
        decision = "robotTrackField";
    }
    else if(!goalSignal)
    {
        decision = "kick";
    }
    else
    {
        decision = "stop";
        _interface->locoClient.Move(0, 0, 0);
    }
    
    std::cout<<"debug decision: "<<std::endl;
    std::cout<<decision<<std::endl;

    setOutput("decision", decision);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus WristControl::tick()
{
    getInput("wrist_angle", target_angle);
    if(!hasSubWristState)
    {
        init_angle = _interface->lowState->msg_.motor_state()[JointIndex::kWaistYaw].q();
        std::cout<<"debug init_angle: "<<std::endl;
        std::cout<<init_angle<<std::endl;
        hasSubWristState = true;
        weight = 1.0;
        _interface->armCmd->msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
        _interface->armCmd->unlockAndPublish();
    }
    _percent_1 += (float)1 / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;

    float desired_position =  (1 - _percent_1) * init_angle + _percent_1 * target_angle;

    std::cout<<"debug desired_position: "<<std::endl;
    std::cout<<desired_position<<std::endl;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).q() = desired_position;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kp() = kp;
    _interface->armCmd->msg_.motor_cmd().at(JointIndex::kWaistYaw).kd() = kd;
    _interface->armCmd->unlockAndPublish();
    // _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    // _interface->servoCmd->msg_.cmds()[0].q() = desired0_position;

    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Speak::tick()
{
    int32_t ret;
    std::string context;
    getInput("A Nice Shot, isn't it?", context);
    uint8_t volume;
    ret = _interface->audioClient.GetVolume(volume);
    std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
    ret = _interface->audioClient.SetVolume(100);
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    std::cout<<"debug text: "<<context<<std::endl;
    ret = _interface->audioClient.TtsMaker(context,
                        0);  // Auto play
    std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

    return BT::NodeStatus::SUCCESS;
}




BT::NodeStatus AbnormalCondition::tick()
{
    B2G_RotMat = _interface->rotMatPelvisToGlobal;
    G2B_RotMat = B2G_RotMat.transpose();
    Vec3<double> projected_gravity_body,projected_gravity_world;
    projected_gravity_world<<0,0,-1;
    projected_gravity_body = G2B_RotMat * projected_gravity_world;
    std::cout<<"[AbnormalCondition::tick]"<<std::endl;
    // std::cout<<"[AbnormalCondition::tick] debug projected_gravity_body: "<<std::endl;
    // std::cout<< projected_gravity_body <<std::endl;

    if(abs(projected_gravity_body(0))>=0.4 || abs(projected_gravity_body(1))>=0.4)
    {   
        std::cout<<"enter abnormal condition: "<<std::endl;
        if(projected_gravity_body(1)>0)
        {
            _interface->servoCmd->msg_.cmds()[0].mode() = 1;
            _interface->servoCmd->msg_.cmds()[0].q() = -50;
            _interface->servoCmd->msg_.cmds()[1].mode() = 1;
            _interface->servoCmd->msg_.cmds()[1].q() = 10;
            _interface->servoCmd->unlockAndPublish();
        }

        if(projected_gravity_body(1)<0)
        {
            _interface->servoCmd->msg_.cmds()[0].mode() = 1;
            _interface->servoCmd->msg_.cmds()[0].q() = 50;
            _interface->servoCmd->msg_.cmds()[1].mode() = 1;
            _interface->servoCmd->msg_.cmds()[1].q() = 10;
            _interface->servoCmd->unlockAndPublish();
        }
      
    }
    else
    {
        _interface->servoCmd->msg_.cmds()[0].mode() = 1;
        _interface->servoCmd->msg_.cmds()[0].q() = 0;
        _interface->servoCmd->msg_.cmds()[1].mode() = 1;
        _interface->servoCmd->msg_.cmds()[1].q() = 0;
        _interface->servoCmd->unlockAndPublish();

    }
    return BT::NodeStatus::SUCCESS;
}
