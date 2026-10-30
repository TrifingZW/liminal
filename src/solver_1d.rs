//! 1D 刚性轴系求解器：按当前传动比把旋转连通组折算到参考轴，更新角速度。

use std::collections::{HashMap, HashSet};

use smallvec::SmallVec;

use crate::compiler::{CompiledVehicle, DrivetrainIsland};
use crate::payload::{PortPayload, PortPayloads};
use crate::topology::VehicleTopology;
use crate::types::{PortDomain, PortId};
use crate::units::{RotationalCoupling, TorqueKind, UnitContext};

type RotationGraph = HashMap<PortId, SmallVec<[(PortId, f32); 4]>>;

/// 以固定步长推进所有 1D 动力岛。
///
/// 岛的连接关系在每步按单元当前状态解析：旋转边为 1:1 刚性连接，
/// 单元内部按 `RotationalCoupling` 提供传动比或断开（空挡、滑摩）。
/// 每个旋转连通组折算到参考轴：J_eq = ΣJ·r²，τ_eq = Στ·r，
/// ω_ref += (τ_eq / J_eq)·dt，再按 ω_port = r·ω_ref 回写。
///
/// 扭矩分驱动与阻性两类：驱动扭矩直接积分；阻性扭矩（摩擦、制动）破钳制到
/// “本步内恰好停转”的上限，避免符号翻转类阻力矩在显式积分下的静止震荡。
/// 端口载荷中的 `torque` 字段作为外部注入扭矩（轮胎反作用力矩等）参与驱动侧，
/// 消费后清零；断开端口的残留注入不会生效。
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

        let mut drive_torque = 0.0;
        let mut resist_torque = 0.0;
        let mut total_inertia = 0.0;
        for &node_id in island.nodes() {
            let Some(node) = topology.nodes().get(node_id) else {
                continue;
            };
            let context = UnitContext::new(&node.ports, payloads);

            // 惯量按节点在本连通组内的首个旋转端口计提，
            // 避免空挡变速箱/分离离合器把另一侧组件的惯量算漏。
            let anchor = node
                .ports
                .iter()
                .copied()
                .find(|&port| ratios.contains_key(&port) && is_rotational(topology, port));
            if let Some(&ratio) = anchor.and_then(|port| ratios.get(&port)) {
                total_inertia += node.unit.rotational_inertia() * ratio * ratio;
            }

            for (index, &port) in node.ports.iter().enumerate() {
                let Some(&ratio) = ratios.get(&port) else {
                    continue;
                };
                let torque = node.unit.port_torque(&context, index) * ratio;
                match node.unit.port_torque_kind(&context, index) {
                    TorqueKind::Drive => drive_torque += torque,
                    TorqueKind::Resist => resist_torque += torque,
                }
                // 外部注入扭矩（轮胎反作用力矩等）：作为驱动类参与。
                if let Some(PortPayload::Rotational {
                    torque,
                    ..
                }) = payloads.read(port)
                {
                    drive_torque += torque * ratio;
                }
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

        // 阻性合力钳制：本步内不得使参考轴转速反向穿越零点。
        // 上限为归零所需扭矩与驱动合力幅值之和，允许制动恰好抵消驱动并停转。
        let resist_limit = reference_velocity.abs() * total_inertia / dt + drive_torque.abs();
        let resist_torque = resist_torque.clamp(-resist_limit, resist_limit);
        let next_reference =
            reference_velocity + (drive_torque + resist_torque) / total_inertia * dt;

        for &port in &component {
            updates.push((port, ratios[&port] * next_reference));
        }
    }

    // 外部注入扭矩随本步消费立即清零，避免跨步重复施加。
    for (port, angular_velocity) in updates {
        if let Some(PortPayload::Rotational {
            ..
        }) = payloads.read(port)
        {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque: 0.0,
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
    use crate::units::transmission::Brake;
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

    #[test]
    fn resist_torque_stops_shaft_without_overshoot() {
        let mut topology = VehicleTopology::new();
        let rotor = topology.add_node(Rotor {
            inertia: 1.0,
            torque: 0.0,
        });
        let brake = topology.add_node(Brake::new(1_000.0));
        let rotor_port = topology.nodes()[rotor].ports[0];
        let brake_ports = topology.nodes()[brake].ports.clone();

        topology.connect(rotor_port, brake_ports[Brake::SHAFT_PORT]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(
            rotor_port,
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 5.0,
            },
        );
        payloads.write(brake_ports[Brake::BRAKE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        for _ in 0..10 {
            step(&topology, &compiled, &mut payloads, 0.01);
        }

        let Some(PortPayload::Rotational {
            angular_velocity,
            ..
        }) = payloads.read(rotor_port)
        else {
            panic!("端口应持有旋转载荷");
        };
        assert!(angular_velocity.abs() < 1e-6);
    }

    #[test]
    fn injected_external_torque_is_consumed_once() {
        let mut topology = VehicleTopology::new();
        let rotor = topology.add_node(Rotor {
            inertia: 2.0,
            torque: 0.0,
        });
        let rotor_port = topology.nodes()[rotor].ports[0];

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(
            rotor_port,
            PortPayload::Rotational {
                torque: 4.0,
                angular_velocity: 0.0,
            },
        );

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.5);

        let omega_after_injection = match payloads.read(rotor_port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        assert!((omega_after_injection - 1.0).abs() < 1e-6);

        // 注入扭矩已被消费，再步进转速不再变化。
        step(&topology, &compiled, &mut payloads, 0.5);
        let omega_after_free_step = match payloads.read(rotor_port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        assert!((omega_after_free_step - 1.0).abs() < 1e-6);
    }
}
