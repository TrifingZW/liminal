//! 端口运行态数据交换区：与拓扑结构分离，供单元在物理帧内读写。

use slotmap::SecondaryMap;
use smallvec::SmallVec;

use crate::topology::VehicleTopology;
use crate::types::{PortDomain, PortId};

/// 端口在一个物理帧内交换的数据。
///
/// 空间关节描述由编译产物中的 `SpatialJointDesc` 提供，载荷只承载运行态位姿。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PortPayload {
    /// 标量控制信号：油门、刹车、转向角。
    Signal(f32),
    /// 一维旋转：扭矩与角速度。
    Rotational {
        torque: f32,
        angular_velocity: f32,
    },
    /// 空间：相对位移与四元数旋转，旋转分量顺序为 (x, y, z, w)。
    Spatial {
        translation: [f32; 3],
        rotation: [f32; 4],
    },
    /// 燃料管路：质量流量。
    Fluid {
        mass_flow: f32,
    },
}

impl PortPayload {
    /// 按物理域取零值：信号与流量为零，旋转静止，空间为单位位姿。
    pub fn zero_for(domain: PortDomain) -> Self {
        match domain {
            PortDomain::SignalScalar => Self::Signal(0.0),
            PortDomain::Rotational1D => Self::Rotational {
                torque: 0.0,
                angular_velocity: 0.0,
            },
            PortDomain::Spatial6DoF => Self::Spatial {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            PortDomain::FluidFuel => Self::Fluid {
                mass_flow: 0.0,
            },
        }
    }
}

/// 端口载荷表：键为端口 ID，值为该端口当前的交换数据。
#[derive(Default)]
pub struct PortPayloads {
    slots: SecondaryMap<PortId, PortPayload>,
}

impl PortPayloads {
    pub fn new() -> Self {
        Self::default()
    }

    /// 按拓扑对齐槽位：移除已失效端口，补齐新端口，已有端口保留当前载荷。
    ///
    /// 在拓扑重编译后调用，运行态状态不因结构变更而整体重置。
    pub fn rebuild(&mut self, topology: &VehicleTopology) {
        let stale: SmallVec<[PortId; 16]> = self
            .slots
            .iter()
            .map(|(port, _)| port)
            .filter(|port| !topology.ports().contains_key(*port))
            .collect();
        for port in stale {
            self.slots.remove(port);
        }

        for (port_id, port) in topology.ports().iter() {
            if !self.slots.contains_key(port_id) {
                self.slots.insert(port_id, PortPayload::zero_for(port.desc.domain));
            }
        }
    }

    /// 读取端口当前载荷的快照；端口不在交换区时返回 `None`。
    pub fn read(&self, port: PortId) -> Option<PortPayload> {
        self.slots.get(port).copied()
    }

    /// 写入端口载荷；端口不在交换区时返回 `false`，不产生幽灵槽位。
    pub fn write(&mut self, port: PortId, payload: PortPayload) -> bool {
        let Some(slot) = self.slots.get_mut(port) else {
            return false;
        };
        *slot = payload;
        true
    }

    /// 以可写引用访问端口载荷，供需要原位修改多个字段的单元使用。
    pub fn get_mut(&mut self, port: PortId) -> Option<&mut PortPayload> {
        self.slots.get_mut(port)
    }
}
