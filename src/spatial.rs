//! 空间域：挂载位姿 `Frame` 与空间连接上附加的相对运动自由度。
//!
//! 挂载约定：空间端口可声明一个 `Frame`，表示挂载点在节点本地坐标系中的位姿。
//! 一条空间连接在静止位形下令两端挂载位姿重合，由此可链式推导每个子节点相对
//! 父节点（直至运动学树根）的静止位姿，见 `CompiledVehicle::spatial_links`。

/// 刚体变换：平移与单位四元数旋转，旋转分量顺序为 (x, y, z, w)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// 平移分量。
    pub translation: [f32; 3],
    /// 旋转四元数 (x, y, z, w)，须为单位四元数。
    pub rotation: [f32; 4],
}

impl Frame {
    /// 单位变换。
    pub const IDENTITY: Self = Self {
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    /// 仅含平移、无旋转的变换。
    pub const fn translation(translation: [f32; 3]) -> Self {
        Self {
            translation,
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    /// 逆变换。
    pub fn inverse(&self) -> Self {
        let rotation = quat_conjugate(self.rotation);
        let translation = quat_rotate(
            rotation,
            [-self.translation[0], -self.translation[1], -self.translation[2]],
        );
        Self {
            translation,
            rotation,
        }
    }

    /// 复合变换：`self.then(&other)` 表示先应用 `self` 再应用 `other`。
    pub fn then(&self, other: &Self) -> Self {
        let rotated = quat_rotate(self.rotation, other.translation);
        Self {
            translation: [
                self.translation[0] + rotated[0],
                self.translation[1] + rotated[1],
                self.translation[2] + rotated[2],
            ],
            rotation: quat_mul(self.rotation, other.rotation),
        }
    }
}

/// 四元数共轭。
fn quat_conjugate(quaternion: [f32; 4]) -> [f32; 4] {
    [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]
}

/// 四元数哈密顿积。
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// 四元数旋转向量。
fn quat_rotate(quaternion: [f32; 4], vector: [f32; 3]) -> [f32; 3] {
    let [x, y, z, w] = quaternion;
    let cross = [
        2.0 * (y * vector[2] - z * vector[1]),
        2.0 * (z * vector[0] - x * vector[2]),
        2.0 * (x * vector[1] - y * vector[0]),
    ];
    [
        vector[0] + w * cross[0] + (y * cross[2] - z * cross[1]),
        vector[1] + w * cross[1] + (z * cross[0] - x * cross[2]),
        vector[2] + w * cross[2] + (x * cross[1] - y * cross[0]),
    ]
}

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
        /// 自由长度：受力零点，也是静止位形下的弹簧长度。
        rest_length: f32,
        /// 行程下限：弹簧可压缩到的最短长度。
        min_length: f32,
        /// 行程上限：弹簧可伸展到的最长长度。
        max_length: f32,
    },
}

#[cfg(test)]
mod tests {
    use super::Frame;

    #[test]
    fn compose_with_inverse_yields_identity() {
        let raw = [0.3f32, 0.4, 0.5, 0.7];
        let norm = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2] + raw[3] * raw[3]).sqrt();
        let frame = Frame {
            translation: [1.0, -2.0, 0.5],
            rotation: [raw[0] / norm, raw[1] / norm, raw[2] / norm, raw[3] / norm],
        };

        let roundtrip = frame.then(&frame.inverse());

        assert!(roundtrip.translation.iter().all(|&value| value.abs() < 1e-6));
        assert!((roundtrip.rotation[3] - 1.0).abs() < 1e-6);
        assert!(roundtrip.rotation[..3].iter().all(|&value| value.abs() < 1e-6));
    }

    #[test]
    fn translation_frame_moves_points() {
        let frame = Frame::translation([1.0, 2.0, 3.0]);
        let composed = frame.then(&Frame::translation([10.0, 0.0, 0.0]));
        assert_eq!(composed.translation, [11.0, 2.0, 3.0]);
    }
}
