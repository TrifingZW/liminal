//! 空间单元：底盘、悬挂与转向节。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::spatial::SpatialJointDesc;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::units::{UnitContext, VehicleUnit};

/// 底盘：空间运动学树的根，提供多个挂载点。
#[derive(Debug, Clone, Copy)]
pub struct Chassis {
    mount_count: usize,
}

impl Chassis {
    /// 创建带指定挂载点数量的底盘。
    pub fn new(mount_count: usize) -> Self {
        Self {
            mount_count,
        }
    }
}

impl VehicleUnit for Chassis {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        let mut descriptors = SmallVec::with_capacity(self.mount_count);
        for index in 0..self.mount_count {
            descriptors.push(PortDescriptor {
                name: format!("mount{index}").into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            });
        }
        descriptors
    }

    fn is_spatial_root(&self) -> bool {
        true
    }
}

/// 悬挂：底盘与转向节之间的弹簧阻尼平移。
#[derive(Debug, Clone, Copy)]
pub struct Suspension {
    stiffness: f32,
    damping: f32,
    rest_length: f32,
}

impl Suspension {
    /// 上端挂载端口在节点端口表中的索引。
    pub const TOP_PORT: usize = 0;
    /// 下端挂载端口在节点端口表中的索引。
    pub const BOTTOM_PORT: usize = 1;

    /// 创建指定刚度、阻尼与自由长度的悬挂。
    pub fn new(stiffness: f32, damping: f32, rest_length: f32) -> Self {
        Self {
            stiffness,
            damping,
            rest_length,
        }
    }

    /// 由压缩量与压缩速度计算悬挂法向力，只推不拉。
    pub fn normal_force(&self, compression: f32, compression_velocity: f32) -> f32 {
        (self.stiffness * compression + self.damping * compression_velocity).max(0.0)
    }
}

impl VehicleUnit for Suspension {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "top".into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
            PortDescriptor {
                name: "bottom".into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
        ]
    }

    fn spatial_joint(&self) -> SpatialJointDesc {
        SpatialJointDesc::PrismaticSpring {
            axis: [0.0, 1.0, 0.0],
            stiffness: self.stiffness,
            damping: self.damping,
            rest_length: self.rest_length,
        }
    }
}

/// 转向节：悬挂与车轮之间的转向轴，并读取制动信号对车轮施加阻力矩。
#[derive(Debug, Clone, Copy)]
pub struct SteeringKnuckle {
    max_brake_torque: f32,
}

impl SteeringKnuckle {
    /// 上端挂载端口在节点端口表中的索引。
    pub const TOP_PORT: usize = 0;
    /// 下端挂载端口在节点端口表中的索引。
    pub const BOTTOM_PORT: usize = 1;
    /// 转向信号端口在节点端口表中的索引。
    pub const STEERING_PORT: usize = 2;
    /// 制动信号端口在节点端口表中的索引。
    pub const BRAKE_PORT: usize = 3;
    /// 车轮旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 4;

    pub fn new(max_brake_torque: f32) -> Self {
        Self {
            max_brake_torque,
        }
    }

    fn signal(&self, context: &UnitContext<'_>, index: usize) -> f32 {
        match context.read(index) {
            Some(PortPayload::Signal(value)) => value.clamp(0.0, 1.0),
            _ => 0.0,
        }
    }
}

impl VehicleUnit for SteeringKnuckle {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "top".into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
            PortDescriptor {
                name: "bottom".into(),
                domain: PortDomain::Spatial6DoF,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
            PortDescriptor {
                name: "steering".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "brake".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "shaft".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Bidirectional,
                max_connections: 1,
            },
        ]
    }

    fn spatial_joint(&self) -> SpatialJointDesc {
        SpatialJointDesc::Revolute {
            axis: [0.0, 1.0, 0.0],
        }
    }

    fn spatial_control_port(&self) -> Option<usize> {
        Some(Self::STEERING_PORT)
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        if port_index != Self::SHAFT_PORT {
            return 0.0;
        }
        let engagement = self.signal(context, Self::BRAKE_PORT);
        if engagement <= 0.0 {
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
        -angular_velocity.signum() * engagement * self.max_brake_torque
    }
}

#[cfg(test)]
mod tests {
    use super::{Chassis, SteeringKnuckle, Suspension};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::solver_1d::step;
    use crate::spatial::SpatialJointDesc;
    use crate::topology::VehicleTopology;
    use crate::units::rotational::{Engine, Wheel};

    #[test]
    fn spatial_chain_compiles_joint_sequence() {
        let mut topology = VehicleTopology::new();
        let chassis = topology.add_node(Chassis::new(1));
        let suspension = topology.add_node(Suspension::new(30.0, 3.0, 0.2));
        let knuckle = topology.add_node(SteeringKnuckle::new(50.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0));

        let chassis_ports = topology.nodes()[chassis].ports.clone();
        let suspension_ports = topology.nodes()[suspension].ports.clone();
        let knuckle_ports = topology.nodes()[knuckle].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology.connect(chassis_ports[0], suspension_ports[Suspension::TOP_PORT]).unwrap();
        topology
            .connect(
                suspension_ports[Suspension::BOTTOM_PORT],
                knuckle_ports[SteeringKnuckle::TOP_PORT],
            )
            .unwrap();
        topology
            .connect(knuckle_ports[SteeringKnuckle::BOTTOM_PORT], wheel_ports[Wheel::MOUNT_PORT])
            .unwrap();

        let compiled = compile(&topology).unwrap();
        let links = compiled.spatial_links();

        assert_eq!(links.len(), 3);
        assert_eq!(
            links[0].joint(),
            SpatialJointDesc::PrismaticSpring {
                axis: [0.0, 1.0, 0.0],
                stiffness: 30.0,
                damping: 3.0,
                rest_length: 0.2,
            }
        );
        assert_eq!(
            links[1].joint(),
            SpatialJointDesc::Revolute {
                axis: [0.0, 1.0, 0.0],
            }
        );
        assert_eq!(links[1].control_port(), Some(knuckle_ports[SteeringKnuckle::STEERING_PORT]));
        assert_eq!(
            links[2].joint(),
            SpatialJointDesc::Revolute {
                axis: [1.0, 0.0, 0.0],
            }
        );
        assert_eq!(links[2].child(), wheel);
    }

    #[test]
    fn removing_intermediate_spatial_units_yields_hard_mount() {
        let mut topology = VehicleTopology::new();
        let chassis = topology.add_node(Chassis::new(1));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0));

        let chassis_ports = topology.nodes()[chassis].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology.connect(chassis_ports[0], wheel_ports[Wheel::MOUNT_PORT]).unwrap();

        let compiled = compile(&topology).unwrap();
        let links = compiled.spatial_links();

        assert_eq!(links.len(), 1);
        assert_eq!(links[0].parent(), chassis);
        assert_eq!(links[0].child(), wheel);
        assert_eq!(
            links[0].joint(),
            SpatialJointDesc::Revolute {
                axis: [1.0, 0.0, 0.0],
            }
        );
        assert_eq!(links[0].control_port(), None);
    }

    #[test]
    fn suspension_force_only_pushes() {
        let suspension = Suspension::new(100.0, 10.0, 0.3);
        assert!((suspension.normal_force(0.1, 0.0) - 10.0).abs() < 1e-5);
        assert_eq!(suspension.normal_force(-0.1, 0.0), 0.0);
    }

    #[test]
    fn knuckle_brake_torque_acts_on_wheel_shaft() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(20.0, 100.0, 1.0));
        let knuckle = topology.add_node(SteeringKnuckle::new(50.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0));

        let engine_ports = topology.nodes()[engine].ports.clone();
        let knuckle_ports = topology.nodes()[knuckle].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology.connect(engine_ports[Engine::SHAFT_PORT], wheel_ports[Wheel::SHAFT_PORT]).unwrap();
        topology
            .connect(knuckle_ports[SteeringKnuckle::SHAFT_PORT], wheel_ports[Wheel::SHAFT_PORT])
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(engine_ports[Engine::THROTTLE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);
        let before = match payloads.read(wheel_ports[Wheel::SHAFT_PORT]) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("车轮端口应持有旋转载荷"),
        };

        payloads.write(knuckle_ports[SteeringKnuckle::BRAKE_PORT], PortPayload::Signal(1.0));
        step(&topology, &compiled, &mut payloads, 0.01);
        let after = match payloads.read(wheel_ports[Wheel::SHAFT_PORT]) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("车轮端口应持有旋转载荷"),
        };

        assert!(before > 0.0);
        assert!(after < before);
    }
}
