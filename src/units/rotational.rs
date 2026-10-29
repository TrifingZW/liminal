//! 旋转单元：发动机、轮轴与车轮。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::spatial::SpatialJointDesc;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::unit::{UnitContext, VehicleUnit};

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

/// 车轮：接收驱动力矩，并提供与转速反向的恒定摩擦阻力矩。
#[derive(Debug, Clone, Copy)]
pub struct Wheel {
    inertia: f32,
    friction_torque: f32,
}

impl Wheel {
    /// 旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 0;
    /// 空间挂载端口在节点端口表中的索引。
    pub const MOUNT_PORT: usize = 1;

    /// 创建指定转动惯量与摩擦阻力矩的车轮。
    pub fn new(inertia: f32, friction_torque: f32) -> Self {
        Self {
            inertia,
            friction_torque,
        }
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
    use super::{Axle, Engine, Wheel};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::solver_1d::step;
    use crate::topology::VehicleTopology;
    use crate::units::signal::CockpitInput;

    #[test]
    fn engine_drives_wheel_directly() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 5.0));

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
        let left = topology.add_node(Wheel::new(1.0, 5.0));
        let right = topology.add_node(Wheel::new(1.0, 5.0));

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
}
