//! 轮胎模型：由车轮运动、侧偏角与法向载荷计算纵向/侧向力界或冲量解。

/// 低速侧向阻尼的参考速度（m/s）。
/// 低于该速度时，侧向力从侧偏刚度平滑过渡到速度阻尼，提供静止侧向保持。
pub const LOW_SPEED_REFERENCE: f32 = 1.0;

/// 轮胎模型参数。
///
/// 纵向与侧向使用独立附着系数，合力受摩擦椭圆约束；`static_friction_scale`
/// 给出低速/静止工况的附着放大倍率，供适配层在静摩擦与动摩擦之间过渡。
#[derive(Debug, Clone, Copy)]
pub struct TireModel {
    /// 纵向峰值附着系数。
    pub friction_coefficient: f32,
    /// 侧向峰值附着系数。
    pub lateral_friction_coefficient: f32,
    /// 侧向刚度，每单位侧偏角（弧度）。
    pub lateral_stiffness: f32,
    /// 载荷敏感度：`μ = μ₀·(F_N/F_ref)^(-k)`；非正值关闭。
    pub load_sensitivity: f32,
    /// 载荷敏感的参考法向载荷（N），须为正才启用载荷敏感。
    pub reference_load: f32,
    /// 滚动阻力系数（无量纲）；非正值关闭。
    pub rolling_resistance: f32,
    /// 静摩擦相对动摩擦的倍率；非正值按 1 处理，上限 2。
    pub static_friction_scale: f32,
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

/// 约束式耦合所需的力界与外部力。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TireConstraint {
    /// 纵向动摩擦上限（N），摩擦椭圆的长轴界。
    pub longitudinal_limit: f32,
    /// 侧向动摩擦上限（N），摩擦椭圆的短轴界。
    pub lateral_limit: f32,
    /// 纵向静摩擦上限（N），低速/静止时使用。
    pub static_longitudinal_limit: f32,
    /// 侧向静摩擦上限（N），低速/静止时使用。
    pub static_lateral_limit: f32,
    /// 侧向力（N），正侧偏角产生负向力，已按侧向动摩擦界裁剪。
    pub lateral_force: f32,
    /// 滚动阻力（N），逆向地面速度，不产生车轮反作用。
    pub rolling_force: f32,
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

/// 单次求值的四轴摩擦界。
#[derive(Debug, Clone, Copy, Default)]
struct FrictionLimits {
    longitudinal: f32,
    lateral: f32,
    static_longitudinal: f32,
    static_lateral: f32,
}

impl Default for TireModel {
    fn default() -> Self {
        Self {
            friction_coefficient: 0.0,
            lateral_friction_coefficient: 0.0,
            lateral_stiffness: 0.0,
            load_sensitivity: 0.0,
            reference_load: 4_000.0,
            rolling_resistance: 0.0,
            static_friction_scale: 1.0,
        }
    }
}

impl TireModel {
    /// 创建纵/侧向附着系数相同的模型，载荷敏感、静摩擦放大与滚动阻力关闭。
    pub fn new(friction_coefficient: f32, lateral_stiffness: f32) -> Self {
        Self {
            friction_coefficient,
            lateral_friction_coefficient: friction_coefficient,
            lateral_stiffness,
            ..Self::default()
        }
    }

    /// 冲量解：纵向由本步内使轮面线速度对齐地面速度的冲量折算，侧向由侧偏刚度与
    /// 低速速度阻尼共同给出，合力按摩擦椭圆裁剪，并叠加滚动阻力。
    /// 返回轮胎坐标系力（N）与车轮反作用力矩（N·m）。
    ///
    /// 纵向等效刚度为 `J_eff/R²/dt`，与摩擦界共同决定纵向力，无需额外刚度参数；
    /// 冲量按目标速度对齐计算，低速时不会跨零震荡。
    ///
    /// `lateral_speed` 为接触点沿侧向的相对速度（m/s）。地面速度低于
    /// [`LOW_SPEED_REFERENCE`] 时，侧向力叠加与速度反向的阻尼项，使车辆在
    /// 静止与低速工况下具备侧向保持力；速度升高后该项平滑退出，侧向力回归
    /// 侧偏刚度模型。
    pub fn impulse_forces(
        &self,
        motion: WheelMotion,
        slip_angle: f32,
        lateral_speed: f32,
        normal_force: f32,
        dt: f32,
    ) -> TireForces {
        let limits = self.friction_limits(normal_force);
        let valid = dt.is_finite()
            && dt > 0.0
            && motion.radius.is_finite()
            && motion.radius > 0.0
            && motion.inertia.is_finite()
            && motion.inertia > 0.0
            && motion.angular_velocity.is_finite()
            && motion.ground_speed.is_finite()
            && slip_angle.is_finite();
        if !valid || (limits.longitudinal <= 0.0 && limits.lateral <= 0.0) {
            return TireForces::default();
        }

        // 1) 纵向冲量：本步内使轮面线速度对齐地面速度所需冲量，再折算为力
        let slip_speed = motion.angular_velocity * motion.radius - motion.ground_speed;
        if !slip_speed.is_finite() {
            return TireForces::default();
        }
        let impulse = motion.inertia * slip_speed / (motion.radius * motion.radius);
        let longitudinal = if limits.longitudinal > 0.0 {
            (impulse / dt).clamp(-limits.longitudinal, limits.longitudinal)
        } else {
            0.0
        };

        // 2) 侧向力：侧偏刚度模型 + 低速速度阻尼
        let lateral_slip = finite_or_zero(-self.lateral_stiffness * slip_angle);
        let lateral_speed = finite_or_zero(lateral_speed);
        let speed_ratio = (motion.ground_speed.abs() / LOW_SPEED_REFERENCE).min(1.0);
        let low_speed_weight = (1.0 - speed_ratio) * (1.0 - speed_ratio);
        let lateral_damping = self.lateral_stiffness / LOW_SPEED_REFERENCE;
        let lateral = lateral_slip - lateral_damping * lateral_speed * low_speed_weight;

        // 3) 摩擦椭圆裁剪；车轮只承受摩擦力矩，滚动阻力由地面偏置法向力承担
        let (longitudinal, lateral) =
            clip_to_friction_ellipse(longitudinal, lateral, limits.longitudinal, limits.lateral);

        // 4) 滚动阻力：逆向地面速度，不产生车轮反作用力矩
        let rolling_force =
            self.rolling_force(motion.ground_speed, normal_force, limits.longitudinal);

        TireForces {
            longitudinal: longitudinal + rolling_force,
            lateral,
            wheel_torque: -longitudinal * motion.radius,
        }
    }

    /// 约束式耦合：输出四轴摩擦界、侧向力与滚动阻力。
    ///
    /// 纵向不再给出力解：适配层以纵向/侧向界构造摩擦椭圆，由宿主求解器在速度
    /// 迭代内完成目标速度对齐。静摩擦界供适配层在低滑移速度下替换动摩擦界。
    pub fn constraint_forces(
        &self,
        ground_speed: f32,
        slip_angle: f32,
        lateral_speed: f32,
        normal_force: f32,
    ) -> TireConstraint {
        let limits = self.friction_limits(normal_force);
        if !ground_speed.is_finite()
            || !slip_angle.is_finite()
            || (limits.longitudinal <= 0.0 && limits.lateral <= 0.0)
        {
            return TireConstraint::default();
        }

        let lateral_slip = finite_or_zero(-self.lateral_stiffness * slip_angle);
        let lateral_speed = finite_or_zero(lateral_speed);
        let speed_ratio = (ground_speed.abs() / LOW_SPEED_REFERENCE).min(1.0);
        let low_speed_weight = (1.0 - speed_ratio) * (1.0 - speed_ratio);
        let lateral_damping = self.lateral_stiffness / LOW_SPEED_REFERENCE;
        let lateral = lateral_slip - lateral_damping * lateral_speed * low_speed_weight;
        let lateral = if limits.lateral > 0.0 {
            lateral.clamp(-limits.lateral, limits.lateral)
        } else {
            0.0
        };

        TireConstraint {
            longitudinal_limit: limits.longitudinal,
            lateral_limit: limits.lateral,
            static_longitudinal_limit: limits.static_longitudinal,
            static_lateral_limit: limits.static_lateral,
            lateral_force: lateral,
            rolling_force: self.rolling_force(ground_speed, normal_force, limits.longitudinal),
        }
    }

    /// 计算载荷敏感后的纵向、侧向动/静摩擦界（N）。
    fn friction_limits(&self, normal_force: f32) -> FrictionLimits {
        let load = finite_or_zero(normal_force).max(0.0);
        if load <= 0.0 {
            return FrictionLimits::default();
        }

        let load_scale = self.load_scale(load);
        let longitudinal = positive_or_zero(self.friction_coefficient) * load * load_scale;
        let lateral = positive_or_zero(self.lateral_friction_coefficient) * load * load_scale;
        let static_scale =
            if self.static_friction_scale.is_finite() && self.static_friction_scale > 0.0 {
                self.static_friction_scale.min(2.0)
            } else {
                1.0
            };

        FrictionLimits {
            longitudinal,
            lateral,
            static_longitudinal: longitudinal * static_scale,
            static_lateral: lateral * static_scale,
        }
    }

    /// 载荷敏感倍率；参数无效时返回 1。
    fn load_scale(&self, load: f32) -> f32 {
        let sensitivity = self.load_sensitivity;
        let reference = self.reference_load;
        if !sensitivity.is_finite()
            || sensitivity <= 0.0
            || !reference.is_finite()
            || reference <= 0.0
        {
            return 1.0;
        }

        // 1) 载荷比钳制到 [0.25, 4]，避免极端载荷下的数值发散
        // 2) k 钳制到 [0, 1]，载荷敏感度超出该区间没有物理意义
        let ratio = (load / reference).clamp(0.25, 4.0);
        ratio.powf(-sensitivity.min(1.0))
    }

    /// 滚动阻力（N）：逆向地面速度，钳制到纵向摩擦界。
    fn rolling_force(&self, ground_speed: f32, normal_force: f32, max_force: f32) -> f32 {
        let coefficient = self.rolling_resistance;
        let load = finite_or_zero(normal_force).max(0.0);
        if !coefficient.is_finite() || coefficient <= 0.0 || load <= 0.0 {
            return 0.0;
        }
        if ground_speed.abs() <= f32::EPSILON || max_force <= 0.0 {
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

/// 仅保留正有限值，其余归零。
fn positive_or_zero(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

/// 按椭圆界 `(x/longitudinal)² + (y/lateral)² ≤ 1` 等比缩放合力方向。
/// 任一轴界非正时退化为单轴钳制；两轴界都非正时输出为零。
fn clip_to_friction_ellipse(
    longitudinal: f32,
    lateral: f32,
    longitudinal_limit: f32,
    lateral_limit: f32,
) -> (f32, f32) {
    let limit_x = positive_or_zero(longitudinal_limit);
    let limit_y = positive_or_zero(lateral_limit);
    if limit_x <= 0.0 && limit_y <= 0.0 {
        return (0.0, 0.0);
    }
    if !longitudinal.is_finite() || !lateral.is_finite() {
        return (0.0, 0.0);
    }
    if limit_x <= 0.0 {
        return (0.0, lateral.clamp(-limit_y, limit_y));
    }
    if limit_y <= 0.0 {
        return (longitudinal.clamp(-limit_x, limit_x), 0.0);
    }

    let normalized_x = longitudinal / limit_x;
    let normalized_y = lateral / limit_y;
    let magnitude = normalized_x.hypot(normalized_y);
    if magnitude <= 1.0 || magnitude == 0.0 {
        return (longitudinal, lateral);
    }

    let scale = 1.0 / magnitude;
    (longitudinal * scale, lateral * scale)
}

#[cfg(test)]
mod tests {
    use super::{TireConstraint, TireForces, TireModel, WheelMotion};

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
        let forces = model().impulse_forces(motion(0.0, 0.0), 0.0, 0.0, 500.0, 0.01);
        assert_eq!(forces.longitudinal, 0.0);
        assert_eq!(forces.lateral, 0.0);
        assert_eq!(forces.wheel_torque, 0.0);
    }

    #[test]
    fn impulse_forces_saturate_at_friction_ellipse() {
        // J=1, R=0.5, ω=10 → 冲量 20 N·s，dt=0.01 → 2000 N；μF_N=500 → 裁剪到 500。
        let forces = model().impulse_forces(motion(10.0, 0.0), 0.0, 0.0, 500.0, 0.01);
        assert!((forces.longitudinal - 500.0).abs() < 1e-3);
        assert!((forces.wheel_torque + 250.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_are_linear_below_limit() {
        // J=1, R=0.5, ω=0, v=1 → 冲量 -4 N·s，dt=0.01 → -400 N；上限 1000 不裁剪。
        let forces = model().impulse_forces(motion(0.0, 1.0), 0.0, 0.0, 1_000.0, 0.01);
        assert!((forces.longitudinal + 400.0).abs() < 1e-3);
        assert!((forces.wheel_torque - 200.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_combine_with_lateral_ellipse() {
        let forces = model().impulse_forces(motion(10.0, 0.0), 1.0, 0.0, 500.0, 0.01);
        let magnitude = forces.longitudinal.hypot(forces.lateral);
        assert!((magnitude - 500.0).abs() < 1e-3);
    }

    #[test]
    fn negative_normal_force_yields_no_force() {
        let forces = model().impulse_forces(motion(10.0, 0.0), 0.5, 0.0, -10.0, 0.01);
        assert_eq!(forces, TireForces::default());
    }

    #[test]
    fn non_finite_slip_angle_yields_no_force() {
        let forces = model().impulse_forces(motion(10.0, 0.0), f32::NAN, 0.0, 500.0, 0.01);
        assert_eq!(forces, TireForces::default());
    }

    #[test]
    fn static_lateral_damping_holds_at_rest() {
        // 静止时侧偏角为零，速度阻尼提供侧向保持：Cα/v_ref = 100 N/(m/s)。
        let forces = model().impulse_forces(motion(0.0, 0.0), 0.0, 1.0, 1_000.0, 0.01);
        assert!((forces.lateral + 100.0).abs() < 1e-3);
    }

    #[test]
    fn static_lateral_damping_fades_with_ground_speed() {
        let forces = model().impulse_forces(motion(0.0, 5.0), 0.0, 1.0, 1_000.0, 0.01);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn static_lateral_damping_is_clipped_by_friction_ellipse() {
        let forces = model().impulse_forces(motion(0.0, 0.0), 0.0, 10.0, 100.0, 0.01);
        assert!((forces.lateral + 100.0).abs() < 1e-3);
    }

    #[test]
    fn non_finite_lateral_speed_yields_zero_damping() {
        let forces = model().impulse_forces(motion(0.0, 0.0), 0.0, f32::NAN, 1_000.0, 0.01);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn load_sensitivity_scales_friction_at_reference_and_high_load() {
        let model = TireModel {
            load_sensitivity: 0.1,
            reference_load: 1_000.0,
            ..model()
        };
        // ω=0、v=10、J=1、R=0.5、dt=0.01 → 原始纵向力 -4000 N，摩擦界饱和。
        let reference =
            model.impulse_forces(motion(0.0, 10.0), 0.0, 0.0, 1_000.0, 0.01).longitudinal;
        let high = model.impulse_forces(motion(0.0, 10.0), 0.0, 0.0, 4_000.0, 0.01).longitudinal;

        assert!((reference + 1_000.0).abs() < 1e-2);
        let expected = -4_000.0 * 4.0f32.powf(-0.1);
        assert!((high - expected).abs() < 1e-1);
        assert!(high > -4_000.0);
    }

    #[test]
    fn constraint_forces_expose_elliptical_limits_and_lateral_sign() {
        let model = model();
        let constraint = model.constraint_forces(0.0, 0.1, 0.0, 500.0);
        assert!((constraint.longitudinal_limit - 500.0).abs() < 1e-3);
        assert!((constraint.lateral_limit - 500.0).abs() < 1e-3);
        assert!((constraint.static_longitudinal_limit - 500.0).abs() < 1e-3);
        assert!((constraint.static_lateral_limit - 500.0).abs() < 1e-3);
        assert!((constraint.lateral_force + 10.0).abs() < 1e-3);
        assert_eq!(constraint.rolling_force, 0.0);
    }

    #[test]
    fn constraint_forces_separate_longitudinal_and_lateral_axes() {
        let model = TireModel {
            lateral_friction_coefficient: 0.4,
            static_friction_scale: 1.25,
            ..model()
        };
        let constraint = model.constraint_forces(0.0, 0.0, 0.0, 1_000.0);
        assert!((constraint.longitudinal_limit - 1_000.0).abs() < 1e-3);
        assert!((constraint.lateral_limit - 400.0).abs() < 1e-3);
        assert!((constraint.static_longitudinal_limit - 1_250.0).abs() < 1e-3);
        assert!((constraint.static_lateral_limit - 500.0).abs() < 1e-3);
    }

    #[test]
    fn constraint_forces_reject_invalid_input() {
        let model = model();
        assert_eq!(model.constraint_forces(f32::NAN, 0.0, 0.0, 500.0), TireConstraint::default());
        assert_eq!(model.constraint_forces(0.0, 0.0, 0.0, -1.0), TireConstraint::default());
    }

    #[test]
    fn impulse_forces_respect_unequal_friction_ellipse() {
        let model = TireModel {
            lateral_friction_coefficient: 0.5,
            ..model()
        };
        // 纵向原始需求 2000 N、侧向 100 N；界为 1000/500 N，椭圆缩放后两者都应保留。
        let forces = model.impulse_forces(motion(10.0, 0.0), 1.0, 0.0, 1_000.0, 0.01);
        let normalized = (forces.longitudinal / 1_000.0).hypot(forces.lateral / 500.0);
        assert!(normalized <= 1.0 + 1e-5);
        assert!(forces.longitudinal > 0.0);
        assert!(forces.lateral < 0.0);
    }

    #[test]
    fn constraint_lateral_force_is_clipped_to_lateral_limit() {
        let model = TireModel {
            lateral_friction_coefficient: 0.05,
            ..model()
        };
        let constraint = model.constraint_forces(0.0, 1.0, 0.0, 1_000.0);
        assert!((constraint.lateral_limit - 50.0).abs() < 1e-3);
        assert!((constraint.lateral_force + 50.0).abs() < 1e-3);
    }

    #[test]
    fn static_friction_scale_is_clamped_and_invalid_falls_back_to_one() {
        let mut model = model();
        model.static_friction_scale = 3.0;
        let clamped = model.constraint_forces(0.0, 0.0, 0.0, 1_000.0);
        assert!((clamped.static_longitudinal_limit - 2_000.0).abs() < 1e-3);

        model.static_friction_scale = -1.0;
        let fallback = model.constraint_forces(0.0, 0.0, 0.0, 1_000.0);
        assert!((fallback.static_longitudinal_limit - 1_000.0).abs() < 1e-3);
    }

    #[test]
    fn zero_lateral_limit_allows_longitudinal_force() {
        let model = TireModel {
            lateral_friction_coefficient: 0.0,
            ..model()
        };
        let forces = model.impulse_forces(motion(10.0, 0.0), 0.5, 0.0, 500.0, 0.01);
        assert!((forces.longitudinal - 500.0).abs() < 1e-3);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn invalid_reference_load_disables_load_sensitivity() {
        let model = TireModel {
            load_sensitivity: 0.5,
            reference_load: 0.0,
            ..model()
        };

        let forces = model.impulse_forces(motion(0.0, 10.0), 0.0, 0.0, 2_000.0, 0.01);
        assert!((forces.longitudinal + 2_000.0).abs() < 1e-3);
    }

    #[test]
    fn rolling_resistance_opposes_ground_speed_without_wheel_reaction() {
        let model = TireModel {
            rolling_resistance: 0.02,
            ..model()
        };
        // 自由滚动：ωR = v = 5 m/s，纵向摩擦力为零，仅剩滚动阻力。
        let forward = model.impulse_forces(motion(10.0, 5.0), 0.0, 0.0, 1_000.0, 0.01);
        assert!((forward.longitudinal + 20.0).abs() < 1e-3);
        assert_eq!(forward.wheel_torque, 0.0);

        let backward = model.impulse_forces(motion(-10.0, -5.0), 0.0, 0.0, 1_000.0, 0.01);
        assert!((backward.longitudinal - 20.0).abs() < 1e-3);
        assert_eq!(backward.wheel_torque, 0.0);
    }

    #[test]
    fn rolling_resistance_is_zero_when_stationary_or_disabled() {
        let mut tire = model();
        tire.rolling_resistance = 0.02;
        let stationary = tire.impulse_forces(motion(0.0, 0.0), 0.0, 0.0, 1_000.0, 0.01);
        assert_eq!(stationary.longitudinal, 0.0);

        tire.rolling_resistance = 0.0;
        let rolling = tire.impulse_forces(motion(10.0, 5.0), 0.0, 0.0, 1_000.0, 0.01);
        assert_eq!(rolling.longitudinal, 0.0);
    }

    #[test]
    fn impulse_forces_reject_invalid_step_or_geometry() {
        let model = model();
        assert_eq!(model.impulse_forces(motion(10.0, 0.0), 0.0, 0.0, 500.0, 0.0).longitudinal, 0.0);
        assert_eq!(
            model.impulse_forces(motion(10.0, 0.0), 0.0, 0.0, 500.0, -0.01).longitudinal,
            0.0
        );

        let no_radius = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.0,
            inertia: 1.0,
        };
        assert_eq!(model.impulse_forces(no_radius, 0.0, 0.0, 500.0, 0.01).longitudinal, 0.0);

        let no_inertia = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.5,
            inertia: 0.0,
        };
        assert_eq!(model.impulse_forces(no_inertia, 0.0, 0.0, 500.0, 0.01).longitudinal, 0.0);
    }
}
