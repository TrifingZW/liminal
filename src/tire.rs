//! 轮胎冲量模型：由车轮运动、侧偏角与法向载荷计算纵向与侧向力。

/// 轮胎力模型参数。
#[derive(Debug, Clone, Copy)]
pub struct TireModel {
    /// 参考载荷下的峰值附着系数。
    pub friction_coefficient: f32,
    /// 侧向刚度，每单位侧偏角（弧度）。
    pub lateral_stiffness: f32,
    /// 载荷敏感度：`μ = μ₀·(F_N/F_ref)^(-k)`；非正值关闭。
    pub load_sensitivity: f32,
    /// 载荷敏感的参考法向载荷（N），须为正才启用载荷敏感。
    pub reference_load: f32,
    /// 滚动阻力系数（无量纲）；非正值关闭。
    pub rolling_resistance: f32,
}

/// 轮胎坐标系下的合力与车轮反作用。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TireForces {
    /// 纵向力：正滑移速度（车轮转速高于地面速度）产生正向力。
    pub longitudinal: f32,
    /// 侧向力：正侧偏角产生负向力。
    pub lateral: f32,
    /// 摩擦对车轮的反作用力矩（N·m），不含滚动阻力。
    pub wheel_torque: f32,
}

/// 车轮运动状态：轮胎冲量解的输入。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelMotion {
    /// 车轮角速度（rad/s）。
    pub angular_velocity: f32,
    /// 接触点纵向速度（m/s）。
    pub ground_speed: f32,
    /// 车轮滚动半径（m），须为正。
    pub radius: f32,
    /// 车轮转动惯量（kg·m²），须为正。
    pub inertia: f32,
}

impl Default for TireModel {
    fn default() -> Self {
        Self {
            friction_coefficient: 0.0,
            lateral_stiffness: 0.0,
            load_sensitivity: 0.0,
            reference_load: 4_000.0,
            rolling_resistance: 0.0,
        }
    }
}

impl TireModel {
    /// 创建固定附着的侧向刚度模型，载荷敏感与滚动阻力关闭。
    pub fn new(friction_coefficient: f32, lateral_stiffness: f32) -> Self {
        Self {
            friction_coefficient,
            lateral_stiffness,
            ..Self::default()
        }
    }

    /// 冲量解：纵向由本步内使轮面线速度对齐地面速度的冲量折算，侧向由侧偏刚度，
    /// 合力按摩擦圆裁剪，并叠加滚动阻力。返回轮胎坐标系力（N）与车轮反作用力矩（N·m）。
    ///
    /// 纵向等效刚度为 `J/R²/dt`，与摩擦圆上限共同决定纵向力，无需额外刚度参数；
    /// 冲量按目标速度对齐计算，低速时不会跨零震荡。
    pub fn impulse_forces(
        &self,
        motion: WheelMotion,
        slip_angle: f32,
        normal_force: f32,
        dt: f32,
    ) -> TireForces {
        let max_force = self.friction_limit(normal_force);
        let valid = dt.is_finite()
            && dt > 0.0
            && motion.radius.is_finite()
            && motion.radius > 0.0
            && motion.inertia.is_finite()
            && motion.inertia > 0.0
            && motion.angular_velocity.is_finite()
            && motion.ground_speed.is_finite()
            && slip_angle.is_finite();
        if !valid || max_force <= 0.0 {
            return TireForces::default();
        }

        // 1) 纵向冲量：本步内使轮面线速度对齐地面速度所需冲量，再折算为力
        let slip_speed = motion.angular_velocity * motion.radius - motion.ground_speed;
        if !slip_speed.is_finite() {
            return TireForces::default();
        }
        let impulse = motion.inertia * slip_speed / (motion.radius * motion.radius);
        let friction_longitudinal = (impulse / dt).clamp(-max_force, max_force);

        // 2) 侧向力：侧偏刚度模型
        let lateral = finite_or_zero(-self.lateral_stiffness * slip_angle);

        // 3) 摩擦圆裁剪；车轮只承受摩擦力矩，滚动阻力由地面偏置法向力承担
        let (friction_longitudinal, lateral) =
            clip_to_friction_circle(friction_longitudinal, lateral, max_force);

        // 4) 滚动阻力：逆向地面速度，不产生车轮反作用力矩
        let rolling_force = self.rolling_force(motion.ground_speed, normal_force, max_force);

        TireForces {
            longitudinal: friction_longitudinal + rolling_force,
            lateral,
            wheel_torque: -friction_longitudinal * motion.radius,
        }
    }

    /// 载荷敏感后的摩擦圆上限（N）。
    fn friction_limit(&self, normal_force: f32) -> f32 {
        let load = finite_or_zero(normal_force).max(0.0);
        let base = self.friction_coefficient;
        if load <= 0.0 || !base.is_finite() || base <= 0.0 {
            return 0.0;
        }

        let sensitivity = self.load_sensitivity;
        let reference = self.reference_load;
        if !sensitivity.is_finite()
            || sensitivity <= 0.0
            || !reference.is_finite()
            || reference <= 0.0
        {
            return base * load;
        }

        // 1) 载荷比钳制到 [0.25, 4]，避免极端载荷下的数值发散
        // 2) k 钳制到 [0, 1]，载荷敏感度超出该区间没有物理意义
        let ratio = (load / reference).clamp(0.25, 4.0);
        let exponent = sensitivity.min(1.0);
        base * ratio.powf(-exponent) * load
    }

    /// 滚动阻力（N）：逆向地面速度，钳制到摩擦圆上限。
    fn rolling_force(&self, ground_speed: f32, normal_force: f32, max_force: f32) -> f32 {
        let coefficient = self.rolling_resistance;
        let load = finite_or_zero(normal_force).max(0.0);
        if !coefficient.is_finite() || coefficient <= 0.0 || load <= 0.0 {
            return 0.0;
        }
        if ground_speed.abs() <= f32::EPSILON {
            return 0.0;
        }

        let magnitude = (coefficient * load).min(max_force);
        if !magnitude.is_finite() || magnitude <= 0.0 {
            return 0.0;
        }
        -magnitude * ground_speed.signum()
    }
}

/// 将非有限值归零，避免 NaN 沿求解链传播。
fn finite_or_zero(value: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

/// 按摩擦圆上限等比缩放合力方向；上限非正时输出为零。
fn clip_to_friction_circle(longitudinal: f32, lateral: f32, max_force: f32) -> (f32, f32) {
    if !max_force.is_finite() || max_force <= 0.0 {
        return (0.0, 0.0);
    }

    let magnitude = longitudinal.hypot(lateral);
    if !magnitude.is_finite() {
        return (0.0, 0.0);
    }
    if magnitude <= max_force || magnitude == 0.0 {
        return (longitudinal, lateral);
    }

    let scale = max_force / magnitude;
    (longitudinal * scale, lateral * scale)
}

#[cfg(test)]
mod tests {
    use super::{TireForces, TireModel, WheelMotion};

    fn model() -> TireModel {
        TireModel::new(1.0, 100.0)
    }

    fn motion(angular_velocity: f32, ground_speed: f32) -> WheelMotion {
        WheelMotion {
            angular_velocity,
            ground_speed,
            radius: 0.5,
            inertia: 1.0,
        }
    }

    #[test]
    fn impulse_forces_are_zero_at_rest() {
        let forces = model().impulse_forces(motion(0.0, 0.0), 0.0, 500.0, 0.01);
        assert_eq!(forces.longitudinal, 0.0);
        assert_eq!(forces.lateral, 0.0);
        assert_eq!(forces.wheel_torque, 0.0);
    }

    #[test]
    fn impulse_forces_saturate_at_friction_circle() {
        // J=1, R=0.5, ω=10 → 冲量 20 N·s，dt=0.01 → 2000 N；μF_N=500 → 裁剪到 500。
        let forces = model().impulse_forces(motion(10.0, 0.0), 0.0, 500.0, 0.01);
        assert!((forces.longitudinal - 500.0).abs() < 1e-3);
        assert!((forces.wheel_torque + 250.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_are_linear_below_limit() {
        // J=1, R=0.5, ω=0, v=1 → 冲量 -4 N·s，dt=0.01 → -400 N；上限 1000 不裁剪。
        let forces = model().impulse_forces(motion(0.0, 1.0), 0.0, 1_000.0, 0.01);
        assert!((forces.longitudinal + 400.0).abs() < 1e-3);
        assert!((forces.wheel_torque - 200.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_combine_with_lateral_clip() {
        let forces = model().impulse_forces(motion(10.0, 0.0), 1.0, 500.0, 0.01);
        let magnitude = forces.longitudinal.hypot(forces.lateral);
        assert!((magnitude - 500.0).abs() < 1e-3);
    }

    #[test]
    fn negative_normal_force_yields_no_force() {
        let forces = model().impulse_forces(motion(10.0, 0.0), 0.5, -10.0, 0.01);
        assert_eq!(forces, TireForces::default());
    }

    #[test]
    fn non_finite_slip_angle_yields_no_force() {
        let forces = model().impulse_forces(motion(10.0, 0.0), f32::NAN, 500.0, 0.01);
        assert_eq!(forces, TireForces::default());
    }

    #[test]
    fn load_sensitivity_scales_friction_at_reference_and_high_load() {
        let model = TireModel {
            load_sensitivity: 0.1,
            reference_load: 1_000.0,
            ..model()
        };
        // ω=0、v=10、J=1、R=0.5、dt=0.01 → 原始纵向力 -4000 N，摩擦圆饱和。
        let reference = model.impulse_forces(motion(0.0, 10.0), 0.0, 1_000.0, 0.01).longitudinal;
        let high = model.impulse_forces(motion(0.0, 10.0), 0.0, 4_000.0, 0.01).longitudinal;

        assert!((reference + 1_000.0).abs() < 1e-2);
        let expected = -4_000.0 * 4.0f32.powf(-0.1);
        assert!((high - expected).abs() < 1e-1);
        assert!(high > -4_000.0);
    }

    #[test]
    fn invalid_reference_load_disables_load_sensitivity() {
        let model = TireModel {
            load_sensitivity: 0.5,
            reference_load: 0.0,
            ..model()
        };

        let forces = model.impulse_forces(motion(0.0, 10.0), 0.0, 2_000.0, 0.01);
        assert!((forces.longitudinal + 2_000.0).abs() < 1e-3);
    }

    #[test]
    fn rolling_resistance_opposes_ground_speed_without_wheel_reaction() {
        let model = TireModel {
            rolling_resistance: 0.02,
            ..model()
        };
        // 自由滚动：ωR = v = 5 m/s，纵向摩擦力为零，仅剩滚动阻力。
        let forward = model.impulse_forces(motion(10.0, 5.0), 0.0, 1_000.0, 0.01);
        assert!((forward.longitudinal + 20.0).abs() < 1e-3);
        assert_eq!(forward.wheel_torque, 0.0);

        let backward = model.impulse_forces(motion(-10.0, -5.0), 0.0, 1_000.0, 0.01);
        assert!((backward.longitudinal - 20.0).abs() < 1e-3);
        assert_eq!(backward.wheel_torque, 0.0);
    }

    #[test]
    fn rolling_resistance_is_zero_when_stationary_or_disabled() {
        let mut tire = model();
        tire.rolling_resistance = 0.02;
        let stationary = tire.impulse_forces(motion(0.0, 0.0), 0.0, 1_000.0, 0.01);
        assert_eq!(stationary.longitudinal, 0.0);

        tire.rolling_resistance = 0.0;
        let rolling = tire.impulse_forces(motion(10.0, 5.0), 0.0, 1_000.0, 0.01);
        assert_eq!(rolling.longitudinal, 0.0);
    }

    #[test]
    fn impulse_forces_reject_invalid_step_or_geometry() {
        let model = model();
        assert_eq!(model.impulse_forces(motion(10.0, 0.0), 0.0, 500.0, 0.0).longitudinal, 0.0);
        assert_eq!(model.impulse_forces(motion(10.0, 0.0), 0.0, 500.0, -0.01).longitudinal, 0.0);

        let no_radius = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.0,
            inertia: 1.0,
        };
        assert_eq!(model.impulse_forces(no_radius, 0.0, 500.0, 0.01).longitudinal, 0.0);

        let no_inertia = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.5,
            inertia: 0.0,
        };
        assert_eq!(model.impulse_forces(no_inertia, 0.0, 500.0, 0.01).longitudinal, 0.0);
    }
}
