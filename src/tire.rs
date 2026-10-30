//! 轮胎力模型：由滑移率、侧偏角与法向载荷计算纵向与侧向力。

/// 轮胎力模型参数。
#[derive(Debug, Clone, Copy)]
pub struct TireModel {
    /// 峰值附着系数。
    pub friction_coefficient: f32,
    /// 纵向刚度，每单位滑移率。
    pub longitudinal_stiffness: f32,
    /// 侧向刚度，每单位侧偏角（弧度）。
    pub lateral_stiffness: f32,
}

/// 轮胎坐标系下的合力。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TireForces {
    /// 纵向力：正滑移率（车轮转速高于地面速度）产生正向力。
    pub longitudinal: f32,
    /// 侧向力：正侧偏角产生负向力。
    pub lateral: f32,
}

/// 车轮运动状态：冲量模式轮胎解的输入。
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

impl TireModel {
    /// 计算纵向与侧向力，合力按摩擦圆 `μ·F_N` 裁剪。
    ///
    /// 法向载荷为负时视为接地失效，输出为零。
    pub fn forces(&self, slip_ratio: f32, slip_angle: f32, normal_force: f32) -> TireForces {
        let max_force = self.friction_coefficient * normal_force.max(0.0);
        let longitudinal = self.longitudinal_stiffness * slip_ratio;
        let lateral = -self.lateral_stiffness * slip_angle;
        let magnitude = (longitudinal * longitudinal + lateral * lateral).sqrt();
        if magnitude <= max_force || magnitude == 0.0 {
            return TireForces {
                longitudinal,
                lateral,
            };
        }
        let scale = max_force / magnitude;
        TireForces {
            longitudinal: longitudinal * scale,
            lateral: lateral * scale,
        }
    }

    /// 冲量模式求解：纵向由本步内使轮面线速度对齐地面速度的冲量折算，侧向由侧偏刚度，
    /// 合力按摩擦圆裁剪。返回轮胎坐标系力（N）。
    ///
    /// 纵向等效刚度为 `J/R²/dt`，与摩擦圆上限共同决定纵向力，无需额外刚度参数；
    /// 冲量按零过冲目标速度计算，不会产生显式刚度模型的跨零震荡。
    pub fn impulse_forces(
        &self,
        motion: WheelMotion,
        slip_angle: f32,
        normal_force: f32,
        dt: f32,
    ) -> TireForces {
        let max_force = self.friction_coefficient * normal_force.max(0.0);
        if dt <= 0.0 || motion.radius <= 0.0 || motion.inertia <= 0.0 {
            return TireForces {
                longitudinal: 0.0,
                lateral: 0.0,
            };
        }

        // 1) 纵向冲量：本步内使轮面线速度对齐地面速度所需冲量，再折算为力
        let slip_speed = motion.angular_velocity * motion.radius - motion.ground_speed;
        let impulse = motion.inertia * slip_speed / (motion.radius * motion.radius);
        let longitudinal = (impulse / dt).clamp(-max_force, max_force);

        // 2) 侧向力：侧偏刚度模型
        let lateral = -self.lateral_stiffness * slip_angle;

        // 3) 摩擦圆裁剪
        let magnitude = (longitudinal * longitudinal + lateral * lateral).sqrt();
        if magnitude <= max_force || magnitude == 0.0 {
            return TireForces {
                longitudinal,
                lateral,
            };
        }
        let scale = max_force / magnitude;
        TireForces {
            longitudinal: longitudinal * scale,
            lateral: lateral * scale,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{TireModel, WheelMotion};

    fn model() -> TireModel {
        TireModel {
            friction_coefficient: 1.0,
            longitudinal_stiffness: 100.0,
            lateral_stiffness: 100.0,
        }
    }

    #[test]
    fn small_slip_stays_below_friction_limit() {
        let forces = model().forces(0.01, 0.0, 500.0);
        assert!((forces.longitudinal - 1.0).abs() < 1e-5);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn large_slip_is_clipped_to_friction_circle() {
        let forces = model().forces(5.0, 5.0, 500.0);
        let magnitude = (forces.longitudinal.powi(2) + forces.lateral.powi(2)).sqrt();
        assert!((magnitude - 500.0).abs() < 1e-3);
    }

    #[test]
    fn negative_normal_force_yields_no_force() {
        let forces = model().forces(0.5, 0.5, -10.0);
        assert_eq!(forces.longitudinal, 0.0);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn impulse_forces_are_zero_at_rest() {
        let motion = WheelMotion {
            angular_velocity: 0.0,
            ground_speed: 0.0,
            radius: 0.5,
            inertia: 1.0,
        };
        let forces = model().impulse_forces(motion, 0.0, 500.0, 0.01);
        assert_eq!(forces.longitudinal, 0.0);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn impulse_forces_saturate_at_friction_circle() {
        // J=1, R=0.5, ω=10 → 冲量 20 N·s，dt=0.01 → 2000 N；μF_N=500 → 裁剪到 500。
        let motion = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.5,
            inertia: 1.0,
        };
        let forces = model().impulse_forces(motion, 0.0, 500.0, 0.01);
        assert!((forces.longitudinal - 500.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_are_linear_below_limit() {
        // J=1, R=0.5, ω=0, v=1 → 冲量 -4 N·s，dt=0.01 → -400 N；上限 1000 不裁剪。
        let motion = WheelMotion {
            angular_velocity: 0.0,
            ground_speed: 1.0,
            radius: 0.5,
            inertia: 1.0,
        };
        let forces = model().impulse_forces(motion, 0.0, 1000.0, 0.01);
        assert!((forces.longitudinal + 400.0).abs() < 1e-3);
    }

    #[test]
    fn impulse_forces_combine_with_lateral_clip() {
        let motion = WheelMotion {
            angular_velocity: 10.0,
            ground_speed: 0.0,
            radius: 0.5,
            inertia: 1.0,
        };
        let forces = model().impulse_forces(motion, 1.0, 500.0, 0.01);
        let magnitude = (forces.longitudinal.powi(2) + forces.lateral.powi(2)).sqrt();
        assert!((magnitude - 500.0).abs() < 1e-3);
    }
}
