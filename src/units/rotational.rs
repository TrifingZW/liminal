//! 旋转单元：发动机、轮轴与车轮。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::spatial::SpatialJointDesc;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::units::{RotationalCoupling, TorqueKind, UnitContext, VehicleUnit};

/// 发动机：油门信号输入，单旋转轴输出。
#[derive(Debug, Clone, Copy)]
pub struct Engine {
    max_torque: f32,
    max_angular_velocity: f32,
    inertia: f32,
}

impl Engine {
    /// 油门信号端口在节点端口表中的索引。
    pub const THROTTLE_PORT: usize = 0;
    /// 旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 1;

    /// 扭矩随转速线性衰减，`max_angular_velocity` 须为正，作为衰减上限。
    pub fn new(max_torque: f32, max_angular_velocity: f32, inertia: f32) -> Self {
        Self {
            max_torque,
            max_angular_velocity,
            inertia,
        }
    }
}

impl VehicleUnit for Engine {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "throttle".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "shaft".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Output,
                max_connections: 1,
            },
        ]
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        if port_index != Self::SHAFT_PORT {
            return 0.0;
        }
        let throttle = match context.read(Self::THROTTLE_PORT) {
            Some(PortPayload::Signal(value)) => value.clamp(0.0, 1.0),
            _ => 0.0,
        };
        let angular_velocity = match context.read(Self::SHAFT_PORT) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => 0.0,
        };
        let speed_ratio = (angular_velocity / self.max_angular_velocity).clamp(0.0, 1.0);
        self.max_torque * throttle * (1.0 - speed_ratio)
    }
}

/// 轮轴：一路输入分发到多路输出的刚性旋转节点。
#[derive(Debug, Clone, Copy)]
pub struct Axle {
    output_count: usize,
    inertia: f32,
}

impl Axle {
    /// 输入端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 0;

    /// `output_count` 通常为 2~4。
    pub fn new(output_count: usize, inertia: f32) -> Self {
        Self {
            output_count,
            inertia,
        }
    }

    /// 第 `index` 个输出端口在节点端口表中的索引。
    pub const fn output_port(index: usize) -> usize {
        index + 1
    }
}

impl VehicleUnit for Axle {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        let mut descriptors = SmallVec::with_capacity(self.output_count + 1);
        descriptors.push(PortDescriptor {
            name: "in".into(),
            domain: PortDomain::Rotational1D,
            direction: PortDirection::Input,
            max_connections: 1,
        });
        for index in 0..self.output_count {
            descriptors.push(PortDescriptor {
                name: format!("out{index}").into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Output,
                max_connections: 1,
            });
        }
        descriptors
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }
}

/// 差速器：一个输入端与 N 个输出端之间以转速差阻尼传递扭矩的耦合。
///
/// 对开式差速器的粘性近似：`coupling` 越小越接近开式（允许更大的转速差），
/// 越大越接近锁止；传递扭矩按 `coupling·Δω` 计算并钳制到 `max_torque`。
/// 输入端反作用扭矩为各输出端耦合扭矩之和的负数，功率经转速差耦合传递。
#[derive(Debug, Clone, Copy)]
pub struct Differential {
    output_count: usize,
    coupling: f32,
    max_torque: f32,
    inertia: f32,
}

impl Differential {
    /// 输入端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 0;

    /// 创建指定输出路数、耦合系数、扭矩容量与转动惯量的差速器。
    pub fn new(output_count: usize, coupling: f32, max_torque: f32, inertia: f32) -> Self {
        Self {
            output_count,
            coupling,
            max_torque,
            inertia,
        }
    }

    /// 第 `index` 个输出端口在节点端口表中的索引。
    pub const fn output_port(index: usize) -> usize {
        index + 1
    }

    fn angular_velocity(&self, context: &UnitContext<'_>, index: usize) -> f32 {
        match context.read(index) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => 0.0,
        }
    }

    fn output_torque(&self, context: &UnitContext<'_>, output_index: usize) -> f32 {
        let input = self.angular_velocity(context, Self::INPUT_PORT);
        let output = self.angular_velocity(context, Self::output_port(output_index));
        (self.coupling * (input - output)).clamp(-self.max_torque, self.max_torque)
    }
}

impl VehicleUnit for Differential {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        let mut descriptors = SmallVec::with_capacity(self.output_count + 1);
        descriptors.push(PortDescriptor {
            name: "in".into(),
            domain: PortDomain::Rotational1D,
            direction: PortDirection::Input,
            max_connections: 1,
        });
        for index in 0..self.output_count {
            descriptors.push(PortDescriptor {
                name: format!("out{index}").into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Output,
                max_connections: 1,
            });
        }
        descriptors
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }

    fn rotational_coupling(&self, _context: &UnitContext<'_>) -> RotationalCoupling {
        RotationalCoupling::Disconnected
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        let Some(output_index) = port_index.checked_sub(1) else {
            let reaction: f32 =
                (0..self.output_count).map(|index| self.output_torque(context, index)).sum();
            return -reaction;
        };
        if output_index >= self.output_count {
            return 0.0;
        }
        self.output_torque(context, output_index)
    }
}

/// 车轮：接收驱动力矩，并提供与转速反向的恒定摩擦阻力矩。
#[derive(Debug, Clone, Copy)]
pub struct Wheel {
    inertia: f32,
    friction_torque: f32,
    radius: f32,
}

impl Wheel {
    /// 旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 0;
    /// 空间挂载端口在节点端口表中的索引。
    pub const MOUNT_PORT: usize = 1;

    /// 创建指定转动惯量、摩擦阻力矩与滚动半径的车轮。
    pub fn new(inertia: f32, friction_torque: f32, radius: f32) -> Self {
        Self {
            inertia,
            friction_torque,
            radius,
        }
    }

    /// 滚动半径，供适配层换算轮心线速度与地面反作用力矩。
    pub fn radius(&self) -> f32 {
        self.radius
    }
}

impl VehicleUnit for Wheel {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "shaft".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Input,
                max_connections: 2,
            },
            PortDescriptor {
                name: "mount".into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
        ]
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }

    fn spatial_joint(&self) -> SpatialJointDesc {
        SpatialJointDesc::Revolute {
            axis: [1.0, 0.0, 0.0],
        }
    }

    fn port_torque_kind(&self, _context: &UnitContext<'_>, port_index: usize) -> TorqueKind {
        if port_index == Self::SHAFT_PORT {
            TorqueKind::Resist
        } else {
            TorqueKind::Drive
        }
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        if port_index != Self::SHAFT_PORT {
            return 0.0;
        }
        let angular_velocity = match context.read(Self::SHAFT_PORT) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => 0.0,
        };
        if angular_velocity.abs() <= f32::EPSILON {
            return 0.0;
        }
        -angular_velocity.signum() * self.friction_torque
    }
}

#[cfg(test)]
mod tests {
    use super::{Axle, Differential, Engine, Wheel};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::solver_1d::step;
    use crate::topology::VehicleTopology;
    use crate::units::signal::CockpitInput;
    use crate::units::transmission::{Brake, Gearbox};

    #[test]
    fn engine_drives_wheel_directly() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 5.0, 0.3));

        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let engine_ports = topology.nodes()[engine].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                engine_ports[Engine::THROTTLE_PORT],
            )
            .unwrap();
        topology.connect(engine_ports[Engine::SHAFT_PORT], wheel_ports[Wheel::SHAFT_PORT]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);
        step(&topology, &compiled, &mut payloads, 0.01);

        let Some(PortPayload::Rotational {
            angular_velocity,
            ..
        }) = payloads.read(wheel_ports[Wheel::SHAFT_PORT])
        else {
            panic!("车轮端口应持有旋转载荷");
        };
        assert!(angular_velocity > 0.0);
    }

    #[test]
    fn axle_distributes_engine_torque_to_wheels() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let axle = topology.add_node(Axle::new(2, 1.0));
        let left = topology.add_node(Wheel::new(1.0, 5.0, 0.3));
        let right = topology.add_node(Wheel::new(1.0, 5.0, 0.3));

        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let engine_ports = topology.nodes()[engine].ports.clone();
        let axle_ports = topology.nodes()[axle].ports.clone();
        let left_ports = topology.nodes()[left].ports.clone();
        let right_ports = topology.nodes()[right].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                engine_ports[Engine::THROTTLE_PORT],
            )
            .unwrap();
        topology.connect(engine_ports[Engine::SHAFT_PORT], axle_ports[Axle::INPUT_PORT]).unwrap();
        topology.connect(axle_ports[Axle::output_port(0)], left_ports[Wheel::SHAFT_PORT]).unwrap();
        topology.connect(axle_ports[Axle::output_port(1)], right_ports[Wheel::SHAFT_PORT]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);
        step(&topology, &compiled, &mut payloads, 0.01);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("车轮端口应持有旋转载荷"),
        };
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);
        assert!(left_omega > 0.0);
        assert!((left_omega - right_omega).abs() < 1e-6);
    }

    #[test]
    fn differential_couples_output_speeds_toward_input() {
        let mut topology = VehicleTopology::new();
        let differential = topology.add_node(Differential::new(2, 100.0, 2000.0, 0.5));
        let left = topology.add_node(Wheel::new(1.0, 0.0, 0.3));
        let right = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

        let differential_ports = topology.nodes()[differential].ports.clone();
        let left_ports = topology.nodes()[left].ports.clone();
        let right_ports = topology.nodes()[right].ports.clone();

        topology
            .connect(
                differential_ports[Differential::output_port(0)],
                left_ports[Wheel::SHAFT_PORT],
            )
            .unwrap();
        topology
            .connect(
                differential_ports[Differential::output_port(1)],
                right_ports[Wheel::SHAFT_PORT],
            )
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(
            differential_ports[Differential::INPUT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 5.0,
            },
        );
        // 刚性连通组是单自由度：同一组内各端口的初速须一致。
        for port in
            [differential_ports[Differential::output_port(0)], left_ports[Wheel::SHAFT_PORT]]
        {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque: 0.0,
                    angular_velocity: 10.0,
                },
            );
        }

        let compiled = compile(&topology).unwrap();
        // 与适配层一致的子步节拍：粘性耦合适用 dt ≤ 2.5 ms。
        for _ in 0..100 {
            step(&topology, &compiled, &mut payloads, 0.0025);
        }

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        // 输出端初始转速差 10 rad/s 被有界耦合扭矩拉向输入转速；对称反作用使输入保持原速。
        assert!((angular_velocity(left_ports[Wheel::SHAFT_PORT]) - 5.0).abs() < 0.5);
        assert!((angular_velocity(right_ports[Wheel::SHAFT_PORT]) - 5.0).abs() < 0.5);
        assert!(
            (angular_velocity(differential_ports[Differential::INPUT_PORT]) - 5.0).abs() < 0.01
        );
    }
}
