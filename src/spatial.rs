//! 空间域：单元在 `Spatial6DoF` 连接上附加的相对运动自由度。

/// 单元在空间连接上附加的相对运动自由度。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpatialJointDesc {
    /// 刚性焊死。
    Rigid,
    /// 单轴旋转，用于车轮滚动或立轴转向。
    Revolute {
        /// 本地旋转轴。
        axis: [f32; 3],
    },
    /// 带弹簧阻尼的平移，用于悬挂。
    PrismaticSpring {
        /// 本地平移轴。
        axis: [f32; 3],
        /// 弹簧刚度。
        stiffness: f32,
        /// 阻尼系数。
        damping: f32,
        /// 自由长度。
        rest_length: f32,
    },
}
