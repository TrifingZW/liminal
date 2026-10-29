//! 1D 刚性轴系求解器：按当前传动比把旋转连通组折算到参考轴，更新角速度。

use std::collections::{HashMap, HashSet};

use smallvec::SmallVec;

use crate::compiler::{CompiledVehicle, DrivetrainIsland};
use crate::payload::{PortPayload, PortPayloads};
use crate::topology::VehicleTopology;
use crate::types::{PortDomain, PortId};
use crate::units::{RotationalCoupling, UnitContext};

type RotationGraph = HashMap<PortId, SmallVec<[(PortId, f32); 4]>>;

/// 以固定步长推进所有 1D 动力岛。
///
/// 岛的连接关系在每步按单元当前状态解析：旋转边为 1:1 刚性连接，
/// 单元内部按 `RotationalCoupling` 提供传动比或断开（空挡、滑摩）。
/// 每个旋转连通组折算到参考轴：J_eq = ΣJ·r²，τ_eq = Στ·r，
/// ω_ref += (τ_eq / J_eq)·dt，再按 ω_port = r·ω_ref 回写。
pub fn step(
    topology: &VehicleTopology,
    compiled: &CompiledVehicle,
    payloads: &mut PortPayloads,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    for island in compiled.drivetrain_islands() {
        let graph = build_rotation_graph(topology, island, payloads);
        solve_island(topology, island, &graph, payloads, dt);
    }
}

fn build_rotation_graph(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    payloads: &PortPayloads,
) -> RotationGraph {
    let mut graph: RotationGraph = HashMap::new();
    for &(source, target) in island.links() {
        link(&mut graph, source, target, 1.0);
    }

    for &node_id in island.nodes() {
        let Some(node) = topology.nodes().get(node_id) else {
            continue;
        };
        let Some(anchor) = first_rotational_port(topology, &node.ports) else {
            continue;
        };
        let context = UnitContext::new(&node.ports, payloads);
        match node.unit.rotational_coupling(&context) {
            RotationalCoupling::Rigid => {
                for &port in &node.ports {
                    if port != anchor && is_rotational(topology, port) {
                        link(&mut graph, anchor, port, 1.0);
                    }
                }
            }
            RotationalCoupling::Disconnected => {}
            RotationalCoupling::Ratios(ratios) => {
                for &(index, ratio) in ratios.iter() {
                    if ratio == 0.0 {
                        continue;
                    }
                    let Some(&port) = node.ports.get(index) else {
                        continue;
                    };
                    if port == anchor || !is_rotational(topology, port) {
                        continue;
                    }
                    link(&mut graph, anchor, port, ratio);
                }
            }
        }
    }

    graph
}

fn solve_island(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    graph: &RotationGraph,
    payloads: &mut PortPayloads,
    dt: f32,
) {
    let mut visited: HashSet<PortId> = HashSet::new();
    let mut updates: Vec<(PortId, f32)> = Vec::new();

    for &start in island.ports() {
        if !visited.insert(start) {
            continue;
        }

        let mut ratios: HashMap<PortId, f32> = HashMap::new();
        ratios.insert(start, 1.0);
        let mut component = vec![start];
        let mut stack = vec![start];
        while let Some(current) = stack.pop() {
            let current_ratio = ratios[&current];
            let Some(neighbors) = graph.get(&current) else {
                continue;
            };
            for &(neighbor, factor) in neighbors {
                if visited.contains(&neighbor) {
                    continue;
                }
                visited.insert(neighbor);
                ratios.insert(neighbor, current_ratio * factor);
                component.push(neighbor);
                stack.push(neighbor);
            }
        }

        let mut total_torque = 0.0;
        let mut total_inertia = 0.0;
        for &node_id in island.nodes() {
            let Some(node) = topology.nodes().get(node_id) else {
                continue;
            };
            let context = UnitContext::new(&node.ports, payloads);
            if let Some(anchor) = first_rotational_port(topology, &node.ports) {
                if let Some(&ratio) = ratios.get(&anchor) {
                    total_inertia += node.unit.rotational_inertia() * ratio * ratio;
                }
            }
            for (index, &port) in node.ports.iter().enumerate() {
                let Some(&ratio) = ratios.get(&port) else {
                    continue;
                };
                total_torque += node.unit.port_torque(&context, index) * ratio;
            }
        }

        if total_inertia <= 0.0 {
            continue;
        }

        let reference_velocity = match payloads.read(start) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => 0.0,
        };
        let next_reference = reference_velocity + total_torque / total_inertia * dt;

        for &port in &component {
            updates.push((port, ratios[&port] * next_reference));
        }
    }

    for (port, angular_velocity) in updates {
        if let Some(PortPayload::Rotational {
            torque,
            ..
        }) = payloads.read(port)
        {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque,
                    angular_velocity,
                },
            );
        }
    }
}

fn link(graph: &mut RotationGraph, source: PortId, target: PortId, ratio: f32) {
    graph.entry(source).or_default().push((target, ratio));
    graph.entry(target).or_default().push((source, 1.0 / ratio));
}

fn first_rotational_port(topology: &VehicleTopology, ports: &[PortId]) -> Option<PortId> {
    ports.iter().copied().find(|&port| is_rotational(topology, port))
}

fn is_rotational(topology: &VehicleTopology, port: PortId) -> bool {
    topology.ports().get(port).is_some_and(|entry| entry.desc.domain == PortDomain::Rotational1D)
}

#[cfg(test)]
mod tests {
    use super::step;
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::topology::VehicleTopology;
    use crate::types::{PortDescriptor, PortDirection, PortDomain};
    use crate::units::{UnitContext, VehicleUnit};
    use smallvec::{SmallVec, smallvec};

    struct Rotor {
        inertia: f32,
        torque: f32,
    }

    impl VehicleUnit for Rotor {
        fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
            smallvec![PortDescriptor {
                name: "shaft".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            }]
        }

        fn rotational_inertia(&self) -> f32 {
            self.inertia
        }

        fn port_torque(&self, _context: &UnitContext<'_>, _port_index: usize) -> f32 {
            self.torque
        }
    }

    #[test]
    fn connected_rotors_share_angular_acceleration() {
        let mut topology = VehicleTopology::new();
        let driver = topology.add_node(Rotor {
            inertia: 1.0,
            torque: 2.0,
        });
        let load = topology.add_node(Rotor {
            inertia: 3.0,
            torque: 0.0,
        });
        let driver_port = topology.nodes()[driver].ports[0];
        let load_port = topology.nodes()[load].ports[0];
        topology.connect(driver_port, load_port).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        let compiled = compile(&topology).unwrap();

        step(&topology, &compiled, &mut payloads, 0.1);

        let Some(PortPayload::Rotational {
            angular_velocity,
            torque,
        }) = payloads.read(load_port)
        else {
            panic!("负载端口应持有旋转载荷");
        };
        assert!((angular_velocity - 0.05).abs() < 1e-6);
        assert_eq!(torque, 0.0);
    }

    #[test]
    fn disconnected_rotors_advance_independently() {
        let mut topology = VehicleTopology::new();
        let spinning = topology.add_node(Rotor {
            inertia: 2.0,
            torque: 1.0,
        });
        let idle = topology.add_node(Rotor {
            inertia: 1.0,
            torque: 0.0,
        });
        let spinning_port = topology.nodes()[spinning].ports[0];
        let idle_port = topology.nodes()[idle].ports[0];

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        let compiled = compile(&topology).unwrap();

        step(&topology, &compiled, &mut payloads, 0.1);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        assert!((angular_velocity(spinning_port) - 0.05).abs() < 1e-6);
        assert_eq!(angular_velocity(idle_port), 0.0);
    }
}
