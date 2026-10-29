//! 拓扑图容器与连接校验。
//!
//! 拓扑是编辑态数据：任何结构变更都会置脏，运行态由编译器按需重建。

use slotmap::SlotMap;
use smallvec::SmallVec;

use crate::types::{ConnectError, Edge, EdgeId, NodeId, Port, PortDirection, PortId};
use crate::units::{Node, VehicleUnit};

/// 多域端口图容器，持有节点、端口与边。
#[derive(Default)]
pub struct VehicleTopology {
    nodes: SlotMap<NodeId, Node>,
    ports: SlotMap<PortId, Port>,
    edges: SlotMap<EdgeId, Edge>,
    is_dirty: bool,
}

impl VehicleTopology {
    pub fn new() -> Self {
        Self::default()
    }

    /// 图结构自上次编译后是否发生变更。
    pub fn is_dirty(&self) -> bool {
        self.is_dirty
    }

    /// 编译器完成运行态重建后清除脏标记。
    pub fn clear_dirty(&mut self) {
        self.is_dirty = false;
    }

    pub fn nodes(&self) -> &SlotMap<NodeId, Node> {
        &self.nodes
    }

    pub fn ports(&self) -> &SlotMap<PortId, Port> {
        &self.ports
    }

    pub fn edges(&self) -> &SlotMap<EdgeId, Edge> {
        &self.edges
    }

    /// 注册单元并按需孵化端口。
    ///
    /// 端口描述符由单元在此时声明，按声明顺序为每个描述符孵化一个端口。
    pub fn add_node(&mut self, unit: impl VehicleUnit + 'static) -> NodeId {
        let node_id = self.nodes.insert_with_key(|id| Node {
            id,
            ports: SmallVec::new(),
            unit: Box::new(unit),
        });

        let descriptors = self.nodes[node_id].unit.port_descriptors();
        let mut ports: SmallVec<[PortId; 8]> = SmallVec::with_capacity(descriptors.len());
        for desc in descriptors {
            let port_id = self.ports.insert_with_key(|id| Port {
                id,
                owner: node_id,
                desc,
                connected_edges: SmallVec::new(),
            });
            ports.push(port_id);
        }
        self.nodes[node_id].ports = ports;

        self.is_dirty = true;
        node_id
    }

    /// 移除节点并级联清理其端口与全部关联边。
    ///
    /// 返回 false 表示节点 ID 已失效。
    pub fn remove_node(&mut self, node_id: NodeId) -> bool {
        let Some(node) = self.nodes.remove(node_id) else {
            return false;
        };

        for port_id in node.ports {
            let Some(port) = self.ports.remove(port_id) else {
                continue;
            };
            for edge_id in port.connected_edges {
                let Some(edge) = self.edges.remove(edge_id) else {
                    continue;
                };
                let other_port = if edge.source_port == port_id {
                    edge.target_port
                } else {
                    edge.source_port
                };
                if let Some(other) = self.ports.get_mut(other_port) {
                    other.connected_edges.retain(|connected| *connected != edge_id);
                }
            }
        }

        self.is_dirty = true;
        true
    }

    /// 建立端口连接，依次校验自连、物理域、方向与容量四条规则。
    pub fn connect(&mut self, src: PortId, dst: PortId) -> Result<EdgeId, ConnectError> {
        let src_port = self.ports.get(src).ok_or(ConnectError::PortNotFound(src))?;
        let dst_port = self.ports.get(dst).ok_or(ConnectError::PortNotFound(dst))?;

        // 1) 禁止同一节点内部的端口自连
        if src_port.owner == dst_port.owner {
            return Err(ConnectError::SelfConnectionNotAllowed);
        }

        // 2) 两端物理域必须一致
        if src_port.desc.domain != dst_port.desc.domain {
            return Err(ConnectError::DomainMismatch(src_port.desc.domain, dst_port.desc.domain));
        }

        // 3) 源端须能发出数据，目标端须能接收数据
        if !can_emit(src_port.desc.direction) || !can_accept(dst_port.desc.direction) {
            return Err(ConnectError::DirectionIncompatible);
        }

        // 4) 两端连接数均须小于各自容量
        if src_port.connected_edges.len() >= src_port.desc.max_connections {
            return Err(ConnectError::SourcePortCapacityReached {
                max: src_port.desc.max_connections,
            });
        }
        if dst_port.connected_edges.len() >= dst_port.desc.max_connections {
            return Err(ConnectError::TargetPortCapacityReached {
                max: dst_port.desc.max_connections,
            });
        }

        let domain = src_port.desc.domain;
        let edge_id = self.edges.insert_with_key(|id| Edge {
            id,
            source_port: src,
            target_port: dst,
            domain,
        });

        self.ports[src].connected_edges.push(edge_id);
        self.ports[dst].connected_edges.push(edge_id);

        self.is_dirty = true;
        Ok(edge_id)
    }

    /// 断开一条边并同步两端端口的连接记录。
    ///
    /// 幂等：边已不存在时返回 false。
    #[must_use]
    pub fn disconnect(&mut self, edge_id: EdgeId) -> bool {
        let Some(edge) = self.edges.remove(edge_id) else {
            return false;
        };

        for port_id in [edge.source_port, edge.target_port] {
            if let Some(port) = self.ports.get_mut(port_id) {
                port.connected_edges.retain(|connected| *connected != edge_id);
            }
        }

        self.is_dirty = true;
        true
    }
}

/// 源端是否可以发出数据。
fn can_emit(direction: PortDirection) -> bool {
    matches!(direction, PortDirection::Output | PortDirection::Bidirectional)
}

/// 目标端是否可以接收数据。
fn can_accept(direction: PortDirection) -> bool {
    matches!(direction, PortDirection::Input | PortDirection::Bidirectional)
}
