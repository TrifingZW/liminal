//! 拓扑数据层的纯类型定义：世代索引、物理域、端口、边与连接错误。
//!
//! 本模块不包含任何图操作逻辑，编排与校验由 `VehicleTopology` 承担。

use std::borrow::Cow;
use std::fmt;

use slotmap::new_key_type;
use smallvec::SmallVec;

new_key_type! {
    /// 节点（模块实例）的世代索引。
    ///
    /// 删除后旧索引自动失效，无需生命周期标注，也不会悬垂。
    pub struct NodeId;

    /// 端口的世代索引。
    pub struct PortId;

    /// 边的世代索引。
    pub struct EdgeId;
}

/// 物理域：决定端口之间能否连接，也决定端口上流动的数据语义。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PortDomain {
    /// 一维机械旋转，承载扭矩与角速度。
    Rotational1D,
    /// 三维刚体挂接，承载底盘、悬挂、转向节与车轮之间的空间连接。
    Spatial6DoF,
    /// 标量控制信号，如油门、刹车、离合器开合度。
    SignalScalar,
    /// 燃料与能量管路。
    FluidFuel,
}

/// 端口方向与受力角色。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PortDirection {
    Input,
    Output,
    /// 机械与空间连接通常双向受力。
    Bidirectional,
}

/// 端口描述符：单元初始化时声明的端口配置，与实例状态解耦。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortDescriptor {
    /// 端口名称：静态配置借用字面量，动态生成的端口使用拥有所有权的字符串。
    pub name: Cow<'static, str>,
    pub domain: PortDomain,
    pub direction: PortDirection,
    /// 最大连接数。低级引擎输出常为 1，须经轮轴或分动箱分发；分动箱可配置为 N。
    pub max_connections: usize,
}

/// 实例化后的端口状态。
#[derive(Debug, Clone)]
pub struct Port {
    pub id: PortId,
    pub owner: NodeId,
    pub desc: PortDescriptor,
    /// 栈上内联，避免高频堆分配。
    pub connected_edges: SmallVec<[EdgeId; 4]>,
}

/// 两个端口之间的连接。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Edge {
    pub id: EdgeId,
    pub source_port: PortId,
    pub target_port: PortId,
    /// 冗余存储源端口的物理域，供图编译阶段按域分派时免于反查端口。
    pub domain: PortDomain,
}

/// 连接拓扑校验失败的原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectError {
    /// 引用了不存在或已失效的端口 ID。
    PortNotFound(PortId),
    /// 两端端口物理域不同。
    DomainMismatch(PortDomain, PortDomain),
    /// 方向组合不允许建立连接。
    DirectionIncompatible,
    /// 源端口连接数已达上限。
    SourcePortCapacityReached {
        max: usize,
    },
    /// 目标端口连接数已达上限。
    TargetPortCapacityReached {
        max: usize,
    },
    /// 同一节点内部的端口自连。
    SelfConnectionNotAllowed,
}

impl fmt::Display for ConnectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PortNotFound(port) => write!(f, "端口不存在或已失效：{port:?}"),
            Self::DomainMismatch(src, dst) => {
                write!(f, "端口物理域不匹配：{src:?} 与 {dst:?}")
            }
            Self::DirectionIncompatible => f.write_str("端口方向不兼容"),
            Self::SourcePortCapacityReached {
                max,
            } => {
                write!(f, "源端口连接数已达上限 {max}")
            }
            Self::TargetPortCapacityReached {
                max,
            } => {
                write!(f, "目标端口连接数已达上限 {max}")
            }
            Self::SelfConnectionNotAllowed => f.write_str("禁止同一节点内的端口自连"),
        }
    }
}

impl std::error::Error for ConnectError {}
