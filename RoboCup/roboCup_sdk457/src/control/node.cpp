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

    // —— 可调参数（结合你现有限位：yaw ±50°, pitch [-20°,85°]）——
    constexpr float YAW_MIN = -50.0f, YAW_MAX = 50.0f;
    constexpr float PIT_MIN = -20.0f, PIT_MAX = 85.0f;

    // 去抖
    constexpr int   SEEN_OK = 2, LOST_OK = 5;

    // “突然到身后”判定与动作
    constexpr float EDGE_TRIG = 38.0f;       // 上次看到时 yaw 靠边
    constexpr float LOST_EDGE_WINDOW = 0.6f; // 丢失 < 0.6s 视为“突丢”
    constexpr float BODY_TURN_ANGLE = float(M_PI); // 约 180°
    constexpr float BODY_TURN_SPEED = 0.55f;       // rad/s
    constexpr float BODY_TURN_MAX_T = 2.8f;        // 超时兜底（s）

    // 长时未见 → 环扫
    constexpr float SWEEP_ENTER_T = 2.0f;   // s
    constexpr float SWEEP_OMEGA   = 0.35f;  // 环扫底盘角速度（rad/s）

    // 常规边缘辅助
    constexpr float BASE_HELP_EDGE = 40.0f; // 相机 yaw 接近边缘才带动底盘
    constexpr float BASE_VYAW      = 0.25f; // 辅助角速度（rad/s）

    // —— 底盘状态机（仅在函数内部保存）——
    enum class Mode { Normal, BodyTurn, BaseSweep };
    static bool inited = false;
    static Mode mode = Mode::Normal;

    static int seen_cnt = 0, lost_cnt = 0;
    static float last_seen_yaw = 0.0f;
    static Clock::time_point tp_last = Clock::now();
    static Clock::time_point tp_seen = Clock::now();

    // BodyTurn 控制量
    static int   turn_dir = 0;                 // -1 左 / +1 右
    static float turn_remain = 0.0f;
    static Clock::time_point tp_turn_start = Clock::now();

    // 环扫方向交替
    static int sweep_dir = +1;

    auto now = Clock::now();
    if (!inited) { tp_last = now; tp_seen = now; inited = true; }

    // —— 去抖统计（先做，以便后面任意分支都可用）——
    if (_interface->ballDetected) { seen_cnt++; lost_cnt = 0; }
    else                           { lost_cnt++; seen_cnt = 0; }
    bool ball_stable_seen = (seen_cnt >= SEEN_OK);
    bool ball_stable_lost = (lost_cnt >= LOST_OK);

    float dt = std::chrono::duration<float>(now - tp_last).count();
    dt = std::clamp(dt, 0.0f, 0.05f);
    tp_last = now;

    // —— 原版：未见球则准备插补器；见球则复位并 SUCCESS —— 
    if (!_interface->ballDetected)
    {
        if (firstRun)
        {
            initAngle << _interface->servoState->msg_.states()[0].q(),
                         _interface->servoState->msg_.states()[1].q();

            Vec2f initAngleVec(
                _interface->servoState->msg_.states()[0].q(),
                _interface->servoState->msg_.states()[1].q()
            );

            interpolator.reset(initAngleVec);
            for (const auto& [target, duration] : predefinedPhases)
            {
                interpolator.addPhase(target, duration);
            }
            firstRun = false;
        }
    }
    else
    {
        // —— 看到了球：记录信息并复位搜索器，立刻成功返回 —— 
        tp_seen = now;
        last_seen_yaw = _interface->servoState->msg_.states()[0].q();
        mode = Mode::Normal;
        turn_dir = 0; turn_remain = 0.0f;

        firstRun = true;
        interpolator = MultiStageInterpolator();
        _interface->locoClient.Move(0, 0, 0); // 停止底盘空转
        return BT::NodeStatus::SUCCESS;
    }

    // —— 原版：相机角度由插补器产生 —— 
    interpolator.interpolate(targetAngle);

    // 安全限幅（避免预设相位把舵机顶到死角）
    targetAngle(0) = std::clamp(targetAngle(0), YAW_MIN, YAW_MAX);
    targetAngle(1) = std::clamp(targetAngle(1), PIT_MIN, PIT_MAX);

    _interface->servoCmd->msg_.cmds()[0].mode() = 1;
    _interface->servoCmd->msg_.cmds()[0].q()    = targetAngle(0);
    _interface->servoCmd->msg_.cmds()[1].mode() = 1;
    _interface->servoCmd->msg_.cmds()[1].q()    = targetAngle(1);
    _interface->servoCmd->unlockAndPublish();

    // —— 新增：底盘角速度策略（不改变你的头部扫描轨迹）——
    float vyaw = 0.0f;
    float since_seen = std::chrono::duration<float>(now - tp_seen).count();

    // 1) “突丢转身”触发：刚丢且上次看到 yaw 在边缘
    if (since_seen < LOST_EDGE_WINDOW &&
        std::fabs(last_seen_yaw) > EDGE_TRIG &&
        mode != Mode::BodyTurn)
    {
        mode = Mode::BodyTurn;
        turn_dir = (last_seen_yaw > 0.0f) ? +1 : -1;
        turn_remain = BODY_TURN_ANGLE;
        tp_turn_start = now;
    }

    // 2) 长时未见 → 环扫（只在非转身状态下进入）
    if (mode != Mode::BodyTurn && since_seen > SWEEP_ENTER_T)
    {
        if (mode != Mode::BaseSweep) // 刚切入时换个方向
            sweep_dir = (sweep_dir == +1) ? -1 : +1;
        mode = Mode::BaseSweep;
    }

    // 按状态机输出底盘角速度
    switch (mode)
    {
        case Mode::BodyTurn:
        {
            vyaw = turn_dir * BODY_TURN_SPEED;
            turn_remain -= std::fabs(vyaw) * dt;
            bool timeout = std::chrono::duration<float>(now - tp_turn_start).count() > BODY_TURN_MAX_T;
            if (turn_remain <= 0.0f || timeout) mode = Mode::Normal; // 转完回正常
            break;
        }
        case Mode::BaseSweep:
        {
            vyaw = sweep_dir * SWEEP_OMEGA;
            break;
        }
        case Mode::Normal:
        default:
        {
            // 仅当相机 yaw 接近边缘且“稳定丢失”时，给一点辅助转动
            if (ball_stable_lost && std::fabs(targetAngle(0)) > BASE_HELP_EDGE)
                vyaw = (targetAngle(0) > 0.0f) ? +BASE_VYAW : -BASE_VYAW;
            break;
        }
    }

    _interface->locoClient.Move(0, 0, vyaw);
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
