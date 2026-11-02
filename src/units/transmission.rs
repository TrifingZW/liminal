//! 传动单元：变速箱、离合器与制动器。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::units::{RotationalConstraint, RotationalCoupling, UnitContext, VehicleUnit};

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
            inertia: inertia.max(0.0),
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

/// 离合器：按压紧信号在两端轴之间传递摩擦扭矩。
///
/// 以库仑摩擦约束 `ω_in - ω_out = 0` 建模，扭矩上限为 `engagement·max_torque`：
/// 需求扭矩在上限内时锁死同步，超出时以最大扭矩滑摩。
#[derive(Debug, Clone, Copy)]
pub struct Clutch {
    max_torque: f32,
    inertia: f32,
}

impl Clutch {
    /// 压紧信号端口在节点端口表中的索引。
    pub const ENGAGE_PORT: usize = 0;
    /// 输入端轴端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 1;
    /// 输出端轴端口在节点端口表中的索引。
    pub const OUTPUT_PORT: usize = 2;

    /// 创建指定扭矩上限与转动惯量的离合器。
    pub fn new(max_torque: f32, inertia: f32) -> Self {
        Self {
            max_torque: max_torque.max(0.0),
            inertia: inertia.max(0.0),
        }
    }

    fn engagement(&self, context: &UnitContext<'_>) -> f32 {
        match context.read(Self::ENGAGE_PORT) {
            Some(PortPayload::Signal(value)) => value.clamp(0.0, 1.0),
            _ => 0.0,
        }
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

    fn rotational_coupling(&self, _context: &UnitContext<'_>) -> RotationalCoupling {
        RotationalCoupling::Disconnected
    }

    fn rotational_constraints(
        &self,
        context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        let engagement = self.engagement(context);
        if engagement <= 0.0 {
            return SmallVec::new();
        }
        smallvec![RotationalConstraint::Friction {
            port_a: Self::INPUT_PORT,
            port_b: Some(Self::OUTPUT_PORT),
            max_torque: engagement * self.max_torque,
        }]
    }
}

/// 制动器：按制动信号对所在旋转轴施加库仑摩擦约束。
///
/// 扭矩上限为 `engagement·max_torque`；需求扭矩在上限内时轴保持静止，
/// 超出时以最大扭矩滑摩，因此不存在静止蠕行。
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
            max_torque: max_torque.max(0.0),
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

    fn rotational_constraints(
        &self,
        context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        let engagement = self.engagement(context);
        if engagement <= 0.0 {
            return SmallVec::new();
        }
        smallvec![RotationalConstraint::Friction {
            port_a: Self::SHAFT_PORT,
            port_b: None,
            max_torque: engagement * self.max_torque,
        }]
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
        let wheel = topology.add_node(Wheel::new(1.0, 5.0, 0.3));

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
        let wheel = topology.add_node(Wheel::new(1.0, 5.0, 0.3));

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
        let clutch = topology.add_node(Clutch::new(20.0, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

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
        let clutch = topology.add_node(Clutch::new(100.0, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

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
    fn clutch_holds_load_within_torque_capacity() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(10.0, 1_000.0, 1.0));
        let clutch = topology.add_node(Clutch::new(50.0, 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

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
        for _ in 0..20 {
            step(&topology, &compiled, &mut payloads, 0.0025);
        }

        let engine_omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(&payloads, wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega > 0.0);
        assert!((engine_omega - wheel_omega).abs() < 1e-4);
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
