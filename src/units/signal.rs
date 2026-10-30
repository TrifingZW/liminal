//! 信号单元：驾驶员输入与信号分发。

use smallvec::{SmallVec, smallvec};

use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::units::VehicleUnit;

/// 驾驶员输入单元：输出油门、转向、挡位与制动四路标量信号。
#[derive(Debug, Default, Clone, Copy)]
pub struct CockpitInput;

impl CockpitInput {
    /// 油门输出端口在节点端口表中的索引。
    pub const THROTTLE_PORT: usize = 0;
    /// 转向输出端口在节点端口表中的索引。
    pub const STEERING_PORT: usize = 1;
    /// 挡位输出端口在节点端口表中的索引。
    pub const GEAR_PORT: usize = 2;
    /// 制动输出端口在节点端口表中的索引。
    pub const BRAKE_PORT: usize = 3;
}

impl VehicleUnit for CockpitInput {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        smallvec![
            PortDescriptor {
                name: "throttle".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Output,
                max_connections: 1,
            },
            PortDescriptor {
                name: "steering".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Output,
                max_connections: 1,
            },
            PortDescriptor {
                name: "gear".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Output,
                max_connections: 1,
            },
            PortDescriptor {
                name: "brake".into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Output,
                max_connections: 1,
            },
        ]
    }
}

/// 信号分发单元：一路输入复制到多路输出。
#[derive(Debug, Clone, Copy)]
pub struct SignalSplitter {
    output_count: usize,
}

impl SignalSplitter {
    /// 输入端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 0;

    /// 创建指定输出路数的分发单元。
    pub fn new(output_count: usize) -> Self {
        Self {
            output_count,
        }
    }

    /// 第 `index` 个输出端口在节点端口表中的索引。
    pub const fn output_port(index: usize) -> usize {
        index + 1
    }
}

impl VehicleUnit for SignalSplitter {
    fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
        let mut descriptors = SmallVec::with_capacity(self.output_count + 1);
        descriptors.push(PortDescriptor {
            name: "in".into(),
            domain: PortDomain::SignalScalar,
            direction: PortDirection::Input,
            max_connections: 1,
        });
        for index in 0..self.output_count {
            descriptors.push(PortDescriptor {
                name: format!("out{index}").into(),
                domain: PortDomain::SignalScalar,
                direction: PortDirection::Output,
                max_connections: 1,
            });
        }
        descriptors
    }

    fn signal_routes(&self) -> SmallVec<[(usize, usize); 4]> {
        let mut routes = SmallVec::with_capacity(self.output_count);
        for index in 0..self.output_count {
            routes.push((Self::INPUT_PORT, Self::output_port(index)));
        }
        routes
    }
}

#[cfg(test)]
mod tests {
    use super::{CockpitInput, SignalSplitter};
    use crate::compiler::{CompileError, compile};
    use crate::payload::{PortPayload, PortPayloads};
    use crate::topology::VehicleTopology;
    use crate::types::{PortDescriptor, PortDirection, PortDomain};
    use crate::units::VehicleUnit;
    use smallvec::{SmallVec, smallvec};

    #[test]
    fn throttle_signal_reaches_downstream_port() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let splitter = topology.add_node(SignalSplitter::new(2));
        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let splitter_ports = topology.nodes()[splitter].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                splitter_ports[SignalSplitter::INPUT_PORT],
            )
            .unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(0.8));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);

        assert_eq!(
            payloads.read(splitter_ports[SignalSplitter::INPUT_PORT]),
            Some(PortPayload::Signal(0.8))
        );
    }

    #[test]
    fn signal_cycle_is_rejected() {
        struct BusUnit;

        impl VehicleUnit for BusUnit {
            fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
                smallvec![PortDescriptor {
                    name: "bus".into(),
                    domain: PortDomain::SignalScalar,
                    direction: PortDirection::Bidirectional,
                    max_connections: 2,
                }]
            }
        }

        let mut topology = VehicleTopology::new();
        let first = topology.add_node(BusUnit);
        let second = topology.add_node(BusUnit);
        let first_port = topology.nodes()[first].ports[0];
        let second_port = topology.nodes()[second].ports[0];

        topology.connect(first_port, second_port).unwrap();
        topology.connect(second_port, first_port).unwrap();

        assert!(matches!(compile(&topology), Err(CompileError::SignalCycle { .. })));
    }

    #[test]
    fn splitter_forwards_input_to_all_outputs() {
        struct SinkUnit;

        impl VehicleUnit for SinkUnit {
            fn port_descriptors(&self) -> SmallVec<[PortDescriptor; 8]> {
                smallvec![PortDescriptor {
                    name: "in".into(),
                    domain: PortDomain::SignalScalar,
                    direction: PortDirection::Input,
                    max_connections: 1,
                }]
            }
        }

        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let splitter = topology.add_node(SignalSplitter::new(2));
        let first = topology.add_node(SinkUnit);
        let second = topology.add_node(SinkUnit);

        let cockpit_ports = topology.nodes()[cockpit].ports.clone();
        let splitter_ports = topology.nodes()[splitter].ports.clone();
        let first_ports = topology.nodes()[first].ports.clone();
        let second_ports = topology.nodes()[second].ports.clone();

        topology
            .connect(
                cockpit_ports[CockpitInput::THROTTLE_PORT],
                splitter_ports[SignalSplitter::INPUT_PORT],
            )
            .unwrap();
        topology.connect(splitter_ports[SignalSplitter::output_port(0)], first_ports[0]).unwrap();
        topology.connect(splitter_ports[SignalSplitter::output_port(1)], second_ports[0]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(0.7));

        let compiled = compile(&topology).unwrap();
        compiled.sync_signals(&mut payloads);

        assert_eq!(payloads.read(first_ports[0]), Some(PortPayload::Signal(0.7)));
        assert_eq!(payloads.read(second_ports[0]), Some(PortPayload::Signal(0.7)));
    }
}
