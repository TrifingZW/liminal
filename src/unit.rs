//! 单元（Unit）：拓扑图中可组合的零部件行为抽象。

use smallvec::SmallVec;

use crate::payload::{PortPayload, PortPayloads};
use crate::spatial::SpatialJointDesc;
use crate::types::{NodeId, PortDescriptor, PortId};

/// 单元在图中的实例化节点：世代 ID、孵化出的端口，以及行为实现。
pub struct Node {
    pub id: NodeId,
    pub ports: SmallVec<[PortId; 8]>,
    pub unit: Box<dyn VehicleUnit>,
}

/// 单元运行时上下文：以只读方式暴露所属节点的端口表与载荷表。
pub struct UnitContext<'a> {
    ports: &'a [PortId],
    payloads: &'a PortPayloads,
}

impl<'a> UnitContext<'a> {
    /// 由节点端口表与载荷表构造。
    pub fn new(ports: &'a [PortId], payloads: &'a PortPayloads) -> Self {
        Self {
            ports,
            payloads,
        }
    }

    /// 按端口在节点端口表中的索引读取载荷。
    pub fn read(&self, index: usize) -> Option<PortPayload> {
        self.ports.get(index).and_then(|&port| self.payloads.read(port))
    }

    /// 端口数量。
    pub fn len(&self) -> usize {
        self.ports.len()
    }

    /// 端口表是否为空。
    pub fn is_empty(&self) -> bool {
        self.ports.is_empty()
    }
}

/// 单元内部旋转端口之间的耦合方式。
#[derive(Debug, Clone, PartialEq)]
pub enum RotationalCoupling {
    /// 全部旋转端口按 1:1 刚性耦合。
    Rigid,
    /// 内部断开，各端口独立运动，例如空挡。
    Disconnected,
    /// 显式耦合：`(节点端口索引, 相对首个旋转端口的角速度比)`，未列出的端口不参与。
    Ratios(SmallVec<[(usize, f32); 4]>),
}

/// 零部件单元的统一特征。
///
/// 单元只声明自身的端口能力，不感知图结构；连接关系与合法性由 `VehicleTopology` 维护。
pub trait VehicleUnit: Send + Sync {
    /// 声明端口列表与容量限制。
    ///
    /// 在实例化时调用，允许按配置动态生成端口，例如低级引擎输出为 1、高级引擎输出为 N。
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]>;

    /// 旋转部件的转动惯量，非旋转单元保持默认值 0。
    fn rotational_inertia(&self) -> f32 {
        0.0
    }

    /// 作用在指定端口上的外力矩，非驱动端口返回 0。
    ///
    /// 端口索引与 `port_descriptors` 的声明顺序一致；上下文提供端口与载荷，
    /// 供单元读取转速、油门、离合压紧等状态。多端口单元可对不同端口施加方向相反的力矩。
    fn port_torque(&self, _context: &UnitContext<'_>, _port_index: usize) -> f32 {
        0.0
    }

    /// 内部旋转端口之间的耦合方式，默认全部刚性耦合。
    ///
    /// 上下文提供端口与载荷，供单元按挡位等信号动态决定传动比或断开。
    fn rotational_coupling(&self, _context: &UnitContext<'_>) -> RotationalCoupling {
        RotationalCoupling::Rigid
    }

    /// 单元在空间连接上附加的相对运动自由度，默认刚性。
    fn spatial_joint(&self) -> SpatialJointDesc {
        SpatialJointDesc::Rigid
    }

    /// 是否为空间运动学树的根，例如底盘。
    fn is_spatial_root(&self) -> bool {
        false
    }

    /// 驱动空间关节的信号端口索引，供适配层读取目标角等控制量。
    fn spatial_control_port(&self) -> Option<usize> {
        None
    }
}
