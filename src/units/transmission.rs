//! 传动单元：变速箱、离合器与制动器。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::unit::{RotationalCoupling, UnitContext, VehicleUnit};

/// 变速箱：挡位信号选择传动比，空挡断开内部耦合。
///
/// 传动比定义为输出角速度与输入角速度之比：`ω_out = ratio · ω_in`。
#[derive(Debug, Clone)]
pub struct Gearbox {
    ratios: SmallVec<[f32; 6]>,
    inertia: f32,
}

impl Gearbox {
    /// 挡位信号端口在节点端口表中的索引。
    pub const GEAR_PORT: usize = 0;
    /// 输入轴端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 1;
    /// 输出轴端口在节点端口表中的索引。
    pub const OUTPUT_PORT: usize = 2;

    /// 挡位信号取整后：0 为空挡，`k` 选择第 `k` 个传动比（从 1 开始）。
    pub fn new(ratios: impl IntoIterator<Item = f32>, inertia: f32) -> Self {
        Self {
            ratios: ratios.into_iter().collect(),
            inertia,
        }
    }
}

impl VehicleUnit for Gearbox {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "gear".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "in".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "out".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Output,
                max_connections: 1,
            },
        ]
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }

    fn rotational_coupling(&self, context: &UnitContext<'_>) -> RotationalCoupling {
        let gear = match context.read(Self::GEAR_PORT) {
            Some(PortPayload::Signal(value)) => value.max(0.0).round() as usize,
            _ => 0,
        };
        if gear == 0 {
            return RotationalCoupling::Disconnected;
        }
        let Some(&ratio) = self.ratios.get(gear - 1) else {
            return RotationalCoupling::Disconnected;
        };
        RotationalCoupling::Ratios(smallvec![(Self::OUTPUT_PORT, ratio)])
    }
}

/// 离合器：按压紧信号在两端轴之间传递摩擦扭矩，转速差足够小时锁死合并。
///
/// 锁死判据简化为转速差阈值，未计入扭矩容量校验。
#[derive(Debug, Clone, Copy)]
pub struct Clutch {
    max_torque: f32,
    lock_threshold: f32,
    inertia: f32,
}

impl Clutch {
    /// 压紧信号端口在节点端口表中的索引。
    pub const ENGAGE_PORT: usize = 0;
    /// 输入端轴端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 1;
    /// 输出端轴端口在节点端口表中的索引。
    pub const OUTPUT_PORT: usize = 2;

    /// `lock_threshold` 为锁死判据的转速差，单位 rad/s。
    pub fn new(max_torque: f32, lock_threshold: f32, inertia: f32) -> Self {
        Self {
            max_torque,
            lock_threshold,
            inertia,
        }
    }

    fn engagement(&self, context: &UnitContext<'_>) -> f32 {
        match context.read(Self::ENGAGE_PORT) {
            Some(PortPayload::Signal(value)) => value.clamp(0.0, 1.0),
            _ => 0.0,
        }
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

    fn is_locked(&self, context: &UnitContext<'_>) -> bool {
        if self.engagement(context) <= 0.0 {
            return false;
        }
        let delta = self.angular_velocity(context, Self::INPUT_PORT)
            - self.angular_velocity(context, Self::OUTPUT_PORT);
        delta.abs() <= self.lock_threshold
    }
}

impl VehicleUnit for Clutch {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "engage".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "in".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Input,
                max_connections: 1,
            },
            PortDescriptor {
                name: "out".into(),
                domain: PortDomain::Rotational1D,
                direction: PortDirection::Output,
                max_connections: 1,
            },
        ]
    }

    fn rotational_inertia(&self) -> f32 {
        self.inertia
    }

    fn rotational_coupling(&self, context: &UnitContext<'_>) -> RotationalCoupling {
        if self.is_locked(context) {
            RotationalCoupling::Ratios(smallvec![(Self::OUTPUT_PORT, 1.0)])
        } else {
            RotationalCoupling::Disconnected
        }
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        if self.is_locked(context) {
            return 0.0;
        }
        let engagement = self.engagement(context);
        if engagement <= 0.0 {
            return 0.0;
        }
        let delta = self.angular_velocity(context, Self::INPUT_PORT)
            - self.angular_velocity(context, Self::OUTPUT_PORT);
        let magnitude = engagement * self.max_torque;
        match port_index {
            Self::INPUT_PORT => -delta.signum() * magnitude,
            Self::OUTPUT_PORT => delta.signum() * magnitude,
            _ => 0.0,
        }
    }
}

/// 制动器：按制动信号在所在旋转轴上施加与转向相反的阻力矩。
#[derive(Debug, Clone, Copy)]
pub struct Brake {
    max_torque: f32,
}

impl Brake {
    /// 制动信号端口在节点端口表中的索引。
    pub const BRAKE_PORT: usize = 0;
    /// 旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 1;

    pub fn new(max_torque: f32) -> Self {
        Self {
            max_torque,
        }
    }

    fn engagement(&self, context: &UnitContext<'_>) -> f32 {
        match context.read(Self::BRAKE_PORT) {
            Some(PortPayload::Signal(value)) => value.clamp(0.0, 1.0),
            _ => 0.0,
        }
    }
}

impl VehicleUnit for Brake {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
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
                max_connections: 2,
            },
        ]
    }

    fn port_torque(&self, context: &UnitContext<'_>, port_index: usize) -> f32 {
        if port_index != Self::SHAFT_PORT {
            return 0.0;
        }
        let engagement = self.engagement(context);
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
        -angular_velocity.signum() * engagement * self.max_torque
    }
}

#[cfg(test)]
mod tests {
    use super::{Brake, Clutch, Gearbox};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::solver_1d::step;
    use crate::topology::VehicleTopology;
    use crate::types::PortId;
    use crate::units::rotational::{Engine, Wheel};
    use crate::units::signal::CockpitInput;

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
    fn gearbox_scales_output_speed_by_ratio() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let gearbox = topology.add_node(Gearbox::new([0.5], 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 5.0));

        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let engine_ports = topology.nodes()[engine].ports.clone();
        let gearbox_ports = topology.nodes()[gearbox].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                engine_ports[Engine::THROTTLE_PORT],
            )
            .unwrap();
        topology
            .connect(engine_ports[Engine::SHAFT_PORT], gearbox_ports[Gearbox::INPUT_PORT])
            .unwrap();
        topology
            .connect(gearbox_ports[Gearbox::OUTPUT_PORT], wheel_ports[Wheel::SHAFT_PORT])
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(1.0));
        payloads.write(gearbox_ports[Gearbox::GEAR_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);
        step(&topology, &compiled, &mut payloads, 0.01);

        let engine_omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(&payloads, wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega > 0.0);
        assert!((wheel_omega - 0.5 * engine_omega).abs() < 1e-4);
    }

    #[test]
    fn neutral_gearbox_disconnects_sides() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let gearbox = topology.add_node(Gearbox::new([0.5], 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 5.0));

        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let engine_ports = topology.nodes()[engine].ports.clone();
        let gearbox_ports = topology.nodes()[gearbox].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                engine_ports[Engine::THROTTLE_PORT],
            )
            .unwrap();
        topology
            .connect(engine_ports[Engine::SHAFT_PORT], gearbox_ports[Gearbox::INPUT_PORT])
            .unwrap();
        topology
            .connect(gearbox_ports[Gearbox::OUTPUT_PORT], wheel_ports[Wheel::SHAFT_PORT])
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(1.0));
        payloads.write(gearbox_ports[Gearbox::GEAR_PORT], PortPayload::Signal(0.0));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);
        step(&topology, &compiled, &mut payloads, 0.01);

        let engine_omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(&payloads, wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega > 0.0);
        assert_eq!(wheel_omega, 0.0);
    }

    #[test]
    fn slipping_clutch_transfers_opposing_torques() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let clutch = topology.add_node(Clutch::new(20.0, 0.5, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0));

        let engine_ports = topology.nodes()[engine].ports.clone();
        let clutch_ports = topology.nodes()[clutch].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology
            .connect(engine_ports[Engine::SHAFT_PORT], clutch_ports[Clutch::INPUT_PORT])
            .unwrap();
        topology
            .connect(clutch_ports[Clutch::OUTPUT_PORT], wheel_ports[Wheel::SHAFT_PORT])
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(clutch_ports[Clutch::ENGAGE_PORT], PortPayload::Signal(1.0));
        payloads.write(
            engine_ports[Engine::SHAFT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 10.0,
            },
        );
        payloads.write(
            clutch_ports[Clutch::INPUT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 10.0,
            },
        );

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let engine_omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(&payloads, wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega < 10.0);
        assert!(wheel_omega > 0.0);
    }

    #[test]
    fn clutch_locks_when_sides_are_synchronized() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let clutch = topology.add_node(Clutch::new(100.0, 0.5, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0));

        let engine_ports = topology.nodes()[engine].ports.clone();
        let clutch_ports = topology.nodes()[clutch].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        topology
            .connect(engine_ports[Engine::SHAFT_PORT], clutch_ports[Clutch::INPUT_PORT])
            .unwrap();
        topology
            .connect(clutch_ports[Clutch::OUTPUT_PORT], wheel_ports[Wheel::SHAFT_PORT])
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(engine_ports[Engine::THROTTLE_PORT], PortPayload::Signal(1.0));
        payloads.write(clutch_ports[Clutch::ENGAGE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let engine_omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(&payloads, wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega > 0.0);
        assert!((engine_omega - wheel_omega).abs() < 1e-6);
    }

    #[test]
    fn brake_decelerates_shaft() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(20.0, 100.0, 1.0));
        let brake = topology.add_node(Brake::new(50.0));

        let engine_ports = topology.nodes()[engine].ports.clone();
        let brake_ports = topology.nodes()[brake].ports.clone();

        topology.connect(engine_ports[Engine::SHAFT_PORT], brake_ports[Brake::SHAFT_PORT]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(engine_ports[Engine::THROTTLE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);
        let before = angular_velocity(&payloads, brake_ports[Brake::SHAFT_PORT]);

        payloads.write(brake_ports[Brake::BRAKE_PORT], PortPayload::Signal(1.0));
        step(&topology, &compiled, &mut payloads, 0.01);
        let after = angular_velocity(&payloads, brake_ports[Brake::SHAFT_PORT]);

        assert!(before > 0.0);
        assert!(after < before);
    }
}
