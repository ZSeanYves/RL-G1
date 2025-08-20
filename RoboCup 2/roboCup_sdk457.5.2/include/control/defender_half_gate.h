#pragma once
struct Twist { double vx, vy, wz; }; 

class DefenderHalfGate {
public:
    explicit DefenderHalfGate(double half_margin = 0.05)
    : half_margin_(half_margin) {}

    // 裁切速度：禁止继续向 +x 推进；若已越线，只允许向 -x 回来
    inline void clipVelocity(double robot_x_in_field, Twist& cmd) const {
        if (robot_x_in_field >= -half_margin_ && cmd.vx > 0.0) cmd.vx = 0.0;
        if (robot_x_in_field > 0.0 && cmd.vx > 0.0)            cmd.vx = 0.0;
    }

    // 裁切目标点
    inline void clipTargetX(double& target_x_in_field) const {
        if (target_x_in_field > -half_margin_) target_x_in_field = -half_margin_;
    }

private:
    double half_margin_;   // 中线安全边：0.03~0.07m 皆可
};
