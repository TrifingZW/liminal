//! 1D 旋转域约束求解器：动力岛 → 转速连通组（刚体）→ 速度级约束（PGS）。
//!
//! 岛的连接关系在每步按单元当前状态解析：旋转边为 1:1 刚性连接，
//! 单元内部按 [`RotationalCoupling`] 提供传动比或断开（空挡、滑摩）。
//! 固定传动比把端口合并为一个转速连通组（刚体），组内折算到参考轴：
//! `J_eq = ΣJ·r²`，`ω_port = r·ω_body`。
//!
//! 单元可通过 [`RotationalConstraint`] 声明速度级约束：
//! - 开式差速把输入端转速约束为输出端均值，输出端获得等扭矩分配；
//! - 摩擦约束在扭矩上限内锁止相对转速（制动器、离合器、车轮摩擦），
//!   因此静止时不会因残矩缓慢蠕行。
//!
//! 约束以顺序冲量（PGS）迭代求解，冲量上限为 `max_torque·dt`；
//! 外部注入扭矩（轮胎反作用等）按驱动冲量施加，单步消费后清零。

use std::collections::{HashMap, HashSet};

use smallvec::SmallVec;

use crate::compiler::{CompiledVehicle, DrivetrainIsland};
use crate::payload::{PortPayload, PortPayloads};
use crate::topology::VehicleTopology;
use crate::types::{PortDomain, PortId};
use crate::units::{RotationalConstraint, RotationalCoupling, UnitContext};

/// 约束迭代次数：动力岛规模小，固定迭代足以收敛。
const CONSTRAINT_ITERATIONS: usize = 16;

type RotationGraph = HashMap<PortId, SmallVec<[(PortId, f32); 4]>>;

/// 动态转速连通组：端口按固定传动比共享一个参考角速度。
struct Body {
    /// 组内端口及其相对参考轴的传动比。
    ports: SmallVec<[(PortId, f32); 8]>,
    /// 折算到参考轴的等效转动惯量。
    inertia: f32,
    /// 参考轴角速度。
    angular_velocity: f32,
}

/// 速度级约束行：`Σ jacobian·ω = 0`，累积冲量钳制到 `[lower, upper]`。
struct ConstraintRow {
    terms: SmallVec<[(usize, f32); 8]>,
    lower: f32,
    upper: f32,
    accumulated: f32,
}

/// 以固定步长推进所有 1D 动力岛。
pub fn step(
    topology: &VehicleTopology,
    compiled: &CompiledVehicle,
    payloads: &mut PortPayloads,
    dt: f32,
) {
    if dt <= 0.0 || !dt.is_finite() {
        return;
    }

    for island in compiled.drivetrain_islands() {
        solve_island(topology, island, payloads, dt);
    }
}

/// 查询端口所在的转速连通组折算到该端口的等效转动惯量（kg·m²）。
///
/// 该值包含传动链上所有刚性耦合部件的惯量，按固定传动比折算；端口不在任何
/// 动力岛内、或所在连通组惯量为零时返回 `None`。宿主在显式轮胎力耦合中应使用
/// 该值而不是单轮惯量，否则锁止轴与低挡位工况会低估车轮端的有效惯量。
pub fn port_effective_inertia(
    topology: &VehicleTopology,
    compiled: &CompiledVehicle,
    payloads: &PortPayloads,
    port: PortId,
) -> Option<f32> {
    let island =
        compiled.drivetrain_islands().iter().find(|island| island.ports().contains(&port))?;
    let graph = build_rotation_graph(topology, island, payloads);
    let (mut bodies, port_body) = build_bodies(island, &graph, payloads);
    collect_inertia(topology, island, &port_body, &mut bodies);

    let &(body_index, ratio) = port_body.get(&port)?;
    if !ratio.is_finite() || ratio == 0.0 {
        return None;
    }
    let inertia = bodies[body_index].inertia / (ratio * ratio);
    (inertia.is_finite() && inertia > 0.0).then_some(inertia)
}

fn solve_island(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    payloads: &mut PortPayloads,
    dt: f32,
) {
    let graph = build_rotation_graph(topology, island, payloads);
    let (mut bodies, port_body) = build_bodies(island, &graph, payloads);
    if bodies.is_empty() {
        clear_injected_torques(island, payloads);
        return;
    }

    collect_inertia(topology, island, &port_body, &mut bodies);
    apply_external_torques(topology, island, payloads, &port_body, &mut bodies, dt);

    let mut constraints = build_constraints(topology, island, payloads, &bodies, &port_body, dt);
    solve_constraints(&mut constraints, &mut bodies);

    write_back(island, payloads, &bodies);
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
                    if ratio == 0.0 || !ratio.is_finite() {
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

/// 依据固定传动比图划分转速连通组，并记录每个端口的 `(刚体索引, 传动比)`。
fn build_bodies(
    island: &DrivetrainIsland,
    graph: &RotationGraph,
    payloads: &PortPayloads,
) -> (Vec<Body>, HashMap<PortId, (usize, f32)>) {
    let mut bodies = Vec::new();
    let mut port_body: HashMap<PortId, (usize, f32)> = HashMap::new();
    let mut visited: HashSet<PortId> = HashSet::new();

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
                let next_ratio = current_ratio * factor;
                if !next_ratio.is_finite() {
                    continue;
                }
                visited.insert(neighbor);
                ratios.insert(neighbor, next_ratio);
                component.push(neighbor);
                stack.push(neighbor);
            }
        }

        let body_index = bodies.len();
        let angular_velocity = read_angular_velocity(payloads, start);
        let mut ports = SmallVec::with_capacity(component.len());
        for port in component {
            let ratio = ratios[&port];
            ports.push((port, ratio));
            port_body.insert(port, (body_index, ratio));
        }
        bodies.push(Body {
            ports,
            inertia: 0.0,
            angular_velocity,
        });
    }

    (bodies, port_body)
}

/// 惯量按节点首个所属旋转端口计提，避免空挡变速箱/分离离合器把另一侧惯量算漏。
fn collect_inertia(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    port_body: &HashMap<PortId, (usize, f32)>,
    bodies: &mut [Body],
) {
    for &node_id in island.nodes() {
        let Some(node) = topology.nodes().get(node_id) else {
            continue;
        };
        let Some(&(body_index, ratio)) = node.ports.iter().find_map(|port| port_body.get(port))
        else {
            continue;
        };

        let inertia = node.unit.rotational_inertia();
        let contribution = inertia * ratio * ratio;
        if contribution.is_finite() && contribution > 0.0 {
            bodies[body_index].inertia += contribution;
        }
    }
}

/// 施加单元驱动力矩与外部注入扭矩：统一折算为冲量，单步消费。
fn apply_external_torques(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    payloads: &PortPayloads,
    port_body: &HashMap<PortId, (usize, f32)>,
    bodies: &mut [Body],
    dt: f32,
) {
    let mut drive_impulse = vec![0.0f32; bodies.len()];

    for &node_id in island.nodes() {
        let Some(node) = topology.nodes().get(node_id) else {
            continue;
        };
        let context = UnitContext::new(&node.ports, payloads);

        for (index, &port) in node.ports.iter().enumerate() {
            let Some(&(body_index, ratio)) = port_body.get(&port) else {
                continue;
            };

            let torque = node.unit.port_torque(&context, index) * ratio;
            if torque.is_finite() {
                drive_impulse[body_index] += torque * dt;
            }

            // 外部注入扭矩（轮胎反作用等）作为驱动冲量，单步消费。
            if let Some(PortPayload::Rotational {
                torque: injected,
                ..
            }) = payloads.read(port)
                && injected.is_finite()
            {
                drive_impulse[body_index] += injected * ratio * dt;
            }
        }
    }

    for (index, body) in bodies.iter_mut().enumerate() {
        if body.inertia <= 0.0 {
            continue;
        }
        body.angular_velocity += drive_impulse[index] / body.inertia;
    }
}

/// 把单元声明的约束映射为刚体雅可比行。
fn build_constraints(
    topology: &VehicleTopology,
    island: &DrivetrainIsland,
    payloads: &PortPayloads,
    bodies: &[Body],
    port_body: &HashMap<PortId, (usize, f32)>,
    dt: f32,
) -> Vec<ConstraintRow> {
    let mut rows = Vec::new();

    for &node_id in island.nodes() {
        let Some(node) = topology.nodes().get(node_id) else {
            continue;
        };
        let context = UnitContext::new(&node.ports, payloads);

        for constraint in node.unit.rotational_constraints(&context) {
            let Some(row) = constraint_row(&constraint, &node.ports, bodies, port_body, dt) else {
                continue;
            };
            rows.push(row);
        }
    }

    rows
}

fn constraint_row(
    constraint: &RotationalConstraint,
    node_ports: &[PortId],
    bodies: &[Body],
    port_body: &HashMap<PortId, (usize, f32)>,
    dt: f32,
) -> Option<ConstraintRow> {
    let mut terms: SmallVec<[(usize, f32); 8]> = SmallVec::new();
    let (lower, upper) = match constraint {
        RotationalConstraint::OpenDifferential {
            input_port,
            output_ports,
        } => {
            let &input = node_ports.get(*input_port)?;
            let mut outputs = SmallVec::<[PortId; 4]>::with_capacity(output_ports.len());
            for &index in output_ports {
                let &port = node_ports.get(index)?;
                outputs.push(port);
            }
            if outputs.is_empty() {
                return None;
            }

            add_term(&mut terms, bodies, port_body, input, 1.0);
            let weight = -1.0 / outputs.len() as f32;
            for &port in &outputs {
                add_term(&mut terms, bodies, port_body, port, weight);
            }
            (f32::NEG_INFINITY, f32::INFINITY)
        }
        RotationalConstraint::Friction {
            port_a,
            port_b,
            max_torque,
        } => {
            if !max_torque.is_finite() || *max_torque <= 0.0 {
                return None;
            }
            let &port_a = node_ports.get(*port_a)?;
            add_term(&mut terms, bodies, port_body, port_a, 1.0);
            if let Some(index) = port_b {
                let &port_b = node_ports.get(*index)?;
                add_term(&mut terms, bodies, port_body, port_b, -1.0);
            }

            let bound = *max_torque * dt;
            (-bound, bound)
        }
    };

    terms.retain(|(_, jacobian)| jacobian.abs() > f32::EPSILON);
    if terms.is_empty() {
        return None;
    }

    Some(ConstraintRow {
        terms,
        lower,
        upper,
        accumulated: 0.0,
    })
}

fn add_term(
    terms: &mut SmallVec<[(usize, f32); 8]>,
    bodies: &[Body],
    port_body: &HashMap<PortId, (usize, f32)>,
    port: PortId,
    weight: f32,
) {
    let Some(&(body_index, ratio)) = port_body.get(&port) else {
        return;
    };
    // 零惯量刚体视为运动学锚点，不参与冲量分配。
    if bodies[body_index].inertia <= 0.0 {
        return;
    }

    let jacobian = weight * ratio;
    if !jacobian.is_finite() {
        return;
    }
    if let Some(entry) = terms.iter_mut().find(|(index, _)| *index == body_index) {
        entry.1 += jacobian;
    } else {
        terms.push((body_index, jacobian));
    }
}

/// 顺序冲量迭代：每行独立求解并累积冲量，直到所有行满足约束或达到迭代上限。
fn solve_constraints(rows: &mut [ConstraintRow], bodies: &mut [Body]) {
    for _ in 0..CONSTRAINT_ITERATIONS {
        for row in rows.iter_mut() {
            let mut relative_velocity = 0.0;
            let mut inverse_mass = 0.0;
            for &(body_index, jacobian) in &row.terms {
                let body = &bodies[body_index];
                relative_velocity += jacobian * body.angular_velocity;
                inverse_mass += jacobian * jacobian / body.inertia;
            }
            if inverse_mass <= f32::EPSILON {
                continue;
            }

            let lambda = -relative_velocity / inverse_mass;
            if !lambda.is_finite() {
                continue;
            }
            let next = (row.accumulated + lambda).clamp(row.lower, row.upper);
            let delta = next - row.accumulated;
            if delta.abs() <= f32::EPSILON {
                continue;
            }
            row.accumulated = next;

            for &(body_index, jacobian) in &row.terms {
                bodies[body_index].angular_velocity +=
                    jacobian * delta / bodies[body_index].inertia;
            }
        }
    }
}

/// 回写角速度，并清除本步消费的外部注入扭矩。
fn write_back(island: &DrivetrainIsland, payloads: &mut PortPayloads, bodies: &[Body]) {
    for body in bodies {
        for &(port, ratio) in &body.ports {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque: 0.0,
                    angular_velocity: ratio * body.angular_velocity,
                },
            );
        }
    }

    for &port in island.ports() {
        if let Some(PortPayload::Rotational {
            angular_velocity,
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

fn clear_injected_torques(island: &DrivetrainIsland, payloads: &mut PortPayloads) {
    for &port in island.ports() {
        if let Some(PortPayload::Rotational {
            angular_velocity,
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

fn read_angular_velocity(payloads: &PortPayloads, port: PortId) -> f32 {
    match payloads.read(port) {
        Some(PortPayload::Rotational {
            angular_velocity,
            ..
        }) if angular_velocity.is_finite() => angular_velocity,
        _ => 0.0,
    }
}

fn link(graph: &mut RotationGraph, source: PortId, target: PortId, ratio: f32) {
    if !ratio.is_finite() || ratio == 0.0 {
        return;
    }
    let inverse = 1.0 / ratio;
    if !inverse.is_finite() {
        return;
    }
    graph.entry(source).or_default().push((target, ratio));
    graph.entry(target).or_default().push((source, inverse));
}

fn first_rotational_port(topology: &VehicleTopology, ports: &[PortId]) -> Option<PortId> {
    ports.iter().copied().find(|&port| is_rotational(topology, port))
}

fn is_rotational(topology: &VehicleTopology, port: PortId) -> bool {
    topology.ports().get(port).is_some_and(|entry| entry.desc.domain == PortDomain::Rotational1D)
}

#[cfg(test)]
mod tests {
    use super::{port_effective_inertia, step};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::topology::VehicleTopology;
    use crate::types::{PortDescriptor, PortDirection, PortDomain, PortId};
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

    fn angular_velocity(payloads: &PortPayloads, port: PortId) -> f32 {
        match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
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
    fn effective_inertia_includes_coupled_rotors() {
        let mut topology = VehicleTopology::new();
        let driver = topology.add_node(Rotor {
            inertia: 1.0,
            torque: 0.0,
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

        let inertia = port_effective_inertia(&topology, &compiled, &payloads, load_port).unwrap();
        assert!((inertia - 4.0).abs() < 1e-6);
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

        assert!((angular_velocity(&payloads, spinning_port) - 0.05).abs() < 1e-6);
        assert_eq!(angular_velocity(&payloads, idle_port), 0.0);
    }

    #[test]
    fn brake_stops_shaft_without_overshoot_or_creep() {
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
        payloads.write(
            brake_ports[Brake::SHAFT_PORT],
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

        assert!(angular_velocity(&payloads, rotor_port).abs() < 1e-6);

        // 继续步进，残矩不得使静止轴重新加速。
        payloads.write(
            rotor_port,
            PortPayload::Rotational {
                torque: 1.0,
                angular_velocity: 0.0,
            },
        );
        for _ in 0..10 {
            step(&topology, &compiled, &mut payloads, 0.01);
        }
        assert!(angular_velocity(&payloads, rotor_port).abs() < 1e-6);
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
        assert!((angular_velocity(&payloads, rotor_port) - 1.0).abs() < 1e-6);

        // 注入扭矩已被消费，再步进转速不再变化。
        step(&topology, &compiled, &mut payloads, 0.5);
        assert!((angular_velocity(&payloads, rotor_port) - 1.0).abs() < 1e-6);
    }
}
