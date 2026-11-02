//! 单元（Unit）：零部件行为抽象与具体实现。

pub mod rotational;
pub mod signal;
pub mod spatial;
pub mod transmission;

use smallvec::SmallVec;

use crate::payload::{PortPayload, PortPayloads};
use crate::spatial::{Frame, SpatialJointDesc};
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

/// 1D 旋转域的速度级约束描述。
///
/// 端口索引为节点端口表顺序，与 [`VehicleUnit::port_descriptors`] 一致。
/// 求解器把端口映射到转速连通组后统一求解，摩擦约束的冲量受 `max_torque·dt` 限制。
#[derive(Debug, Clone, PartialEq)]
pub enum RotationalConstraint {
    /// 开式差速：`ω_in = (1/N)·Σω_out`，输出端等扭矩分配。
    OpenDifferential {
        /// 输入端端口索引。
        input_port: usize,
        /// 输出端端口索引列表。
        output_ports: SmallVec<[usize; 4]>,
    },
    /// 库仑摩擦：`ω_a - ω_b = 0`（`port_b` 为 `None` 时对地），
    /// 传递扭矩上限为 `max_torque`（N·m）。
    Friction {
        /// 约束端口 a。
        port_a: usize,
        /// 约束端口 b；`None` 表示对地制动。
        port_b: Option<usize>,
        /// 最大传递扭矩（N·m），非正值表示无约束。
        max_torque: f32,
    },
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

    /// 空间端口在节点本地坐标系中的挂载位姿，未声明时视为单位变换。
    ///
    /// 连接两侧端口的挂载位姿在静止位形下重合，编译器据此推导空间运动学树中
    /// 每个子节点相对父节点的静止位姿。关节量（轴、行程）定义在子节点本地坐标系中。
    fn port_mount(&self, _port_index: usize) -> Option<Frame> {
        None
    }

    /// 内部旋转端口之间的耦合方式，默认全部刚性耦合。
    ///
    /// 上下文提供端口与载荷，供单元按挡位等信号动态决定传动比或断开。
    fn rotational_coupling(&self, _context: &UnitContext<'_>) -> RotationalCoupling {
        RotationalCoupling::Rigid
    }

    /// 1D 旋转域的速度级约束，默认无。
    ///
    /// 约束在动力岛求解阶段与端口耦合一并处理：
    /// 开式差速强制输入转速为输出转速均值，摩擦约束在扭矩上限内锁止相对转速。
    fn rotational_constraints(
        &self,
        _context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        SmallVec::new()
    }

    /// 信号端口转发映射：`(输入端口索引, 输出端口索引)`。
    ///
    /// 在信号同步阶段按单元拓扑序执行，用于分发器等无状态信号单元。
    fn signal_routes(&self) -> SmallVec<[(usize, usize); 4]> {
        SmallVec::new()
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
