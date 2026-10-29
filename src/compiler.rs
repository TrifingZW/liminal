//! 拓扑编译器：将编辑态的端口图拍平为运行态执行数据。

use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt;

use smallvec::SmallVec;

use crate::payload::{PortPayload, PortPayloads};
use crate::spatial::SpatialJointDesc;
use crate::topology::VehicleTopology;
use crate::types::{NodeId, PortDomain, PortId};

/// 拓扑编译失败的原因。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompileError {
    /// 信号域存在有向环，无法拓扑排序。
    SignalCycle {
        ports: SmallVec<[PortId; 8]>,
    },
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SignalCycle {
                ports,
            } => {
                write!(f, "信号域存在有向环，未能排序的端口：{ports:?}")
            }
        }
    }
}

impl std::error::Error for CompileError {}

/// 1D 旋转动力岛：通过旋转边或单元内部潜在耦合相互连通的端口组。
///
/// 岛描述静态结构；单元内部的传动比与断开状态（如空挡）在求解时按载荷动态解析，
/// 因此一个岛内可能同时存在多个转速不同的旋转连通组。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DrivetrainIsland {
    ports: SmallVec<[PortId; 8]>,
    nodes: SmallVec<[NodeId; 8]>,
    links: SmallVec<[(PortId, PortId); 8]>,
}

impl DrivetrainIsland {
    /// 岛内的旋转端口。
    pub fn ports(&self) -> &[PortId] {
        &self.ports
    }

    /// 岛内端口所属的去重节点。
    pub fn nodes(&self) -> &[NodeId] {
        &self.nodes
    }

    /// 岛内由旋转边直接连接（1:1 刚性）的端口对。
    pub fn links(&self) -> &[(PortId, PortId)] {
        &self.links
    }
}

/// 空间运动学树中的一条父子连接。
#[derive(Debug, Clone, PartialEq)]
pub struct SpatialLink {
    parent: NodeId,
    child: NodeId,
    joint: SpatialJointDesc,
    control_port: Option<PortId>,
}

impl SpatialLink {
    /// 父节点。
    pub fn parent(&self) -> NodeId {
        self.parent
    }

    /// 子节点。
    pub fn child(&self) -> NodeId {
        self.child
    }

    /// 子节点在连接上附加的相对运动自由度。
    pub fn joint(&self) -> SpatialJointDesc {
        self.joint
    }

    /// 驱动该关节的信号端口，供宿主适配层读取目标角等控制量。
    pub fn control_port(&self) -> Option<PortId> {
        self.control_port
    }
}

/// 拓扑编译产物：运行态使用的扁平化执行数据。
#[derive(Debug, Default)]
pub struct CompiledVehicle {
    signal_plan: Vec<(PortId, PortId)>,
    drivetrain_islands: Vec<DrivetrainIsland>,
    spatial_links: Vec<SpatialLink>,
}

impl CompiledVehicle {
    /// 信号域的扁平指令表，按拓扑序排列，源端口在前。
    pub fn signal_plan(&self) -> &[(PortId, PortId)] {
        &self.signal_plan
    }

    /// 1D 旋转域的动力岛列表。
    pub fn drivetrain_islands(&self) -> &[DrivetrainIsland] {
        &self.drivetrain_islands
    }

    /// 空间域的运动学树连接。
    pub fn spatial_links(&self) -> &[SpatialLink] {
        &self.spatial_links
    }

    /// 按指令表同步一轮信号载荷。
    ///
    /// 源端口未持有信号载荷时跳过该指令；槽位由 `PortPayloads::rebuild` 保证存在。
    pub fn sync_signals(&self, payloads: &mut PortPayloads) {
        for &(source, target) in &self.signal_plan {
            if let Some(PortPayload::Signal(value)) = payloads.read(source) {
                payloads.write(target, PortPayload::Signal(value));
            }
        }
    }
}

/// 将拓扑编译为运行态执行数据。
///
/// 信号域执行 Kahn 拓扑排序生成扁平拷贝指令；旋转域执行连通分量分解，
/// 将动力链打包为可独立求解的岛；空间域从底盘出发生成运动学树。
pub fn compile(topology: &VehicleTopology) -> Result<CompiledVehicle, CompileError> {
    let signal_plan = compile_signal_plan(topology)?;
    let drivetrain_islands = compile_drivetrain_islands(topology);
    let spatial_links = compile_spatial_tree(topology);

    Ok(CompiledVehicle {
        signal_plan,
        drivetrain_islands,
        spatial_links,
    })
}

fn compile_signal_plan(topology: &VehicleTopology) -> Result<Vec<(PortId, PortId)>, CompileError> {
    let mut indegree: HashMap<PortId, usize> = HashMap::new();
    let mut adjacency: HashMap<PortId, SmallVec<[PortId; 4]>> = HashMap::new();
    for edge in topology.edges().values() {
        if edge.domain != PortDomain::SignalScalar {
            continue;
        }
        adjacency.entry(edge.source_port).or_default().push(edge.target_port);
        *indegree.entry(edge.target_port).or_insert(0) += 1;
    }

    let mut signal_port_count = 0usize;
    let mut queue = VecDeque::new();
    for (port_id, port) in topology.ports().iter() {
        if port.desc.domain != PortDomain::SignalScalar {
            continue;
        }
        signal_port_count += 1;
        if !indegree.contains_key(&port_id) {
            queue.push_back(port_id);
        }
    }

    let mut plan = Vec::with_capacity(topology.edges().len());
    let mut processed = 0usize;
    while let Some(port) = queue.pop_front() {
        processed += 1;
        let Some(targets) = adjacency.get(&port) else {
            continue;
        };
        for &target in targets {
            plan.push((port, target));
            let remaining = indegree.get_mut(&target).expect("信号边的目标端口必然已登记入度");
            *remaining -= 1;
            if *remaining == 0 {
                queue.push_back(target);
            }
        }
    }

    if processed != signal_port_count {
        let ports = indegree
            .iter()
            .filter(|(_, remaining)| **remaining > 0)
            .map(|(&port, _)| port)
            .collect();
        return Err(CompileError::SignalCycle {
            ports,
        });
    }

    Ok(plan)
}

fn compile_drivetrain_islands(topology: &VehicleTopology) -> Vec<DrivetrainIsland> {
    let mut adjacency: HashMap<PortId, SmallVec<[PortId; 4]>> = HashMap::new();
    for edge in topology.edges().values() {
        if edge.domain != PortDomain::Rotational1D {
            continue;
        }
        adjacency.entry(edge.source_port).or_default().push(edge.target_port);
        adjacency.entry(edge.target_port).or_default().push(edge.source_port);
    }

    // 单元内部旋转端口默认刚性耦合（传动比 1），使动力可穿越多端口单元。
    for node in topology.nodes().values() {
        let rotational_ports: SmallVec<[PortId; 4]> = node
            .ports
            .iter()
            .copied()
            .filter(|&port| {
                topology
                    .ports()
                    .get(port)
                    .is_some_and(|entry| entry.desc.domain == PortDomain::Rotational1D)
            })
            .collect();
        let Some((&first, rest)) = rotational_ports.split_first() else {
            continue;
        };
        for &port in rest {
            adjacency.entry(first).or_default().push(port);
            adjacency.entry(port).or_default().push(first);
        }
    }

    let mut visited: HashSet<PortId> = HashSet::new();
    let mut islands = Vec::new();
    for (port_id, port) in topology.ports().iter() {
        if port.desc.domain != PortDomain::Rotational1D || !visited.insert(port_id) {
            continue;
        }

        let mut island = DrivetrainIsland::default();
        let mut stack = vec![port_id];
        while let Some(current) = stack.pop() {
            island.ports.push(current);
            if let Some(owner_port) = topology.ports().get(current) {
                if !island.nodes.contains(&owner_port.owner) {
                    island.nodes.push(owner_port.owner);
                }
            }
            if let Some(neighbors) = adjacency.get(&current) {
                for &neighbor in neighbors {
                    if visited.insert(neighbor) {
                        stack.push(neighbor);
                    }
                }
            }
        }
        islands.push(island);
    }

    let mut island_of: HashMap<PortId, usize> = HashMap::new();
    for (index, island) in islands.iter().enumerate() {
        for &port in island.ports() {
            island_of.insert(port, index);
        }
    }
    for edge in topology.edges().values() {
        if edge.domain != PortDomain::Rotational1D {
            continue;
        }
        if let Some(&index) = island_of.get(&edge.source_port) {
            islands[index].links.push((edge.source_port, edge.target_port));
        }
    }

    islands
}

fn compile_spatial_tree(topology: &VehicleTopology) -> Vec<SpatialLink> {
    let Some(root) = topology
        .nodes()
        .iter()
        .find(|(_, node)| node.unit.is_spatial_root())
        .map(|(node_id, _)| node_id)
    else {
        return Vec::new();
    };

    let mut owners: HashMap<PortId, NodeId> = HashMap::new();
    for (port_id, port) in topology.ports().iter() {
        if port.desc.domain == PortDomain::Spatial6DoF {
            owners.insert(port_id, port.owner);
        }
    }

    let mut adjacency: HashMap<PortId, SmallVec<[PortId; 4]>> = HashMap::new();
    for edge in topology.edges().values() {
        if edge.domain != PortDomain::Spatial6DoF {
            continue;
        }
        adjacency.entry(edge.source_port).or_default().push(edge.target_port);
        adjacency.entry(edge.target_port).or_default().push(edge.source_port);
    }

    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue = VecDeque::new();
    let mut links = Vec::new();
    visited.insert(root);
    queue.push_back(root);

    while let Some(parent) = queue.pop_front() {
        let Some(node) = topology.nodes().get(parent) else {
            continue;
        };
        for &port in &node.ports {
            let Some(neighbors) = adjacency.get(&port) else {
                continue;
            };
            for &neighbor in neighbors {
                let Some(&child) = owners.get(&neighbor) else {
                    continue;
                };
                if !visited.insert(child) {
                    continue;
                }
                let Some(child_node) = topology.nodes().get(child) else {
                    continue;
                };
                let control_port = child_node
                    .unit
                    .spatial_control_port()
                    .and_then(|index| child_node.ports.get(index).copied());
                links.push(SpatialLink {
                    parent,
                    child,
                    joint: child_node.unit.spatial_joint(),
                    control_port,
                });
                queue.push_back(child);
            }
        }
    }

    links
}

#[cfg(test)]
mod tests {
    use super::compile;
    use crate::topology::VehicleTopology;
    use crate::types::{PortDescriptor, PortDirection, PortDomain};
    use crate::unit::VehicleUnit;
    use smallvec::{SmallVec, smallvec};

    struct Shaft;

    impl VehicleUnit for Shaft {
        fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
            smallvec![
                PortDescriptor {
                    name: "a".into(),
                    domain: PortDomain::Rotational1D,
                    direction: PortDirection::Bidirectional,
                    max_connections: 1,
                },
                PortDescriptor {
                    name: "b".into(),
                    domain: PortDomain::Rotational1D,
                    direction: PortDirection::Bidirectional,
                    max_connections: 1,
                },
            ]
        }
    }

    #[test]
    fn internal_coupling_and_edges_form_islands() {
        let mut topology = VehicleTopology::new();
        let first = topology.add_node(Shaft);
        let second = topology.add_node(Shaft);
        topology.add_node(Shaft);
        let first_ports = topology.nodes()[first].ports.clone();
        let second_ports = topology.nodes()[second].ports.clone();

        topology.connect(first_ports[0], second_ports[0]).unwrap();

        let compiled = compile(&topology).unwrap();
        let islands = compiled.drivetrain_islands();

        assert_eq!(islands.len(), 2);
        let coupled = islands.iter().find(|island| island.ports().len() == 4).unwrap();
        assert!(coupled.nodes().contains(&first));
        assert!(coupled.nodes().contains(&second));
        assert!(coupled.ports().contains(&first_ports[0]));
        assert!(coupled.ports().contains(&second_ports[0]));

        let total_ports: usize = islands.iter().map(|island| island.ports().len()).sum();
        assert_eq!(total_ports, 6);
    }
}
