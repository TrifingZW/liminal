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
}

#[cfg(test)]
mod tests {
    use super::TireModel;

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
}
