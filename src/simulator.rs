//! 主仿真循环：脏拓扑重编译、信号同步与 1D 动力流求解。

use crate::compiler::{CompileError, CompiledVehicle, compile};
use crate::payload::PortPayloads;
use crate::solver_1d;
use crate::topology::VehicleTopology;

/// 载具仿真器：持有编辑态拓扑、编译产物与运行态载荷。
///
/// 空间域与轮胎接触由宿主物理后端负责：适配层读取
/// [`CompiledVehicle::spatial_links`] 与信号载荷，构建或更新原生约束；
/// 悬挂法向力由 [`crate::units::spatial::Suspension::normal_force`] 按宿主回传的
/// 压缩量计算。
pub struct Simulator {
    topology: VehicleTopology,
    compiled: CompiledVehicle,
    payloads: PortPayloads,
}

impl Simulator {
    /// 编译初始拓扑并对齐载荷槽位。
    pub fn new(mut topology: VehicleTopology) -> Result<Self, CompileError> {
        let compiled = compile(&topology)?;
        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        topology.clear_dirty();

        Ok(Self {
            topology,
            compiled,
            payloads,
        })
    }

    /// 编辑态拓扑的只读访问。
    pub fn topology(&self) -> &VehicleTopology {
        &self.topology
    }

    /// 编辑态拓扑的可变访问；结构变更会在下一次 [`Simulator::step`] 触发重编译。
    pub fn topology_mut(&mut self) -> &mut VehicleTopology {
        &mut self.topology
    }

    /// 当前编译产物。
    pub fn compiled(&self) -> &CompiledVehicle {
        &self.compiled
    }

    /// 运行态载荷的只读访问。
    pub fn payloads(&self) -> &PortPayloads {
        &self.payloads
    }

    /// 运行态载荷的可变访问，用于写入驾驶输入等外部信号。
    pub fn payloads_mut(&mut self) -> &mut PortPayloads {
        &mut self.payloads
    }

    /// 按当前拓扑立即重编译，并对齐载荷槽位。
    pub fn recompile(&mut self) -> Result<(), CompileError> {
        self.compiled = compile(&self.topology)?;
        self.payloads.rebuild(&self.topology);
        self.topology.clear_dirty();
        Ok(())
    }

    /// 推进一个物理帧。
    ///
    /// 拓扑变更时先重编译并对齐载荷槽位，再同步控制信号、推进 1D 动力流。
    /// 空间刚体积分与轮胎接触由宿主适配层在调用本方法前后完成。
    pub fn step(&mut self, dt: f32) -> Result<(), CompileError> {
        if self.topology.is_dirty() {
            self.recompile()?;
        }

        self.compiled.sync_signals(&mut self.payloads);
        solver_1d::step(&self.topology, &self.compiled, &mut self.payloads, dt);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::Simulator;
    use crate::payload::PortPayload;
    use crate::topology::VehicleTopology;
    use crate::units::rotational::{Engine, Wheel};
    use crate::units::signal::CockpitInput;
    use crate::units::transmission::Gearbox;

    fn angular_velocity(
        payloads: &crate::payload::PortPayloads,
        port: crate::types::PortId,
    ) -> f32 {
        match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        }
    }

    #[test]
    fn hot_swap_rewires_drivetrain_without_code_changes() {
        let mut topology = VehicleTopology::new();
        let cockpit = topology.add_node(CockpitInput);
        let engine = topology.add_node(Engine::new(100.0, 100.0, 1.0));
        let gearbox = topology.add_node(Gearbox::new([0.5], 1.0));
        let wheel = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

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

        let mut simulator = Simulator::new(topology).unwrap();
        simulator
            .payloads_mut()
            .write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(1.0));
        simulator.payloads_mut().write(gearbox_ports[Gearbox::GEAR_PORT], PortPayload::Signal(1.0));
        simulator.step(0.01).unwrap();

        let engine_omega = angular_velocity(simulator.payloads(), engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(simulator.payloads(), wheel_ports[Wheel::SHAFT_PORT]);
        assert!((wheel_omega - 0.5 * engine_omega).abs() < 1e-4);

        let topology = simulator.topology_mut();
        topology.remove_node(gearbox);
        topology.connect(engine_ports[Engine::SHAFT_PORT], wheel_ports[Wheel::SHAFT_PORT]).unwrap();

        simulator.step(0.01).unwrap();

        let engine_omega = angular_velocity(simulator.payloads(), engine_ports[Engine::SHAFT_PORT]);
        let wheel_omega = angular_velocity(simulator.payloads(), wheel_ports[Wheel::SHAFT_PORT]);
        assert!(engine_omega > 0.0);
        assert!((engine_omega - wheel_omega).abs() < 1e-6);
    }
}
