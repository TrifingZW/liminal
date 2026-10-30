use liminal::payload::{PortPayload, PortPayloads};
use liminal::simulator::Simulator;
use liminal::spatial::Frame;
use liminal::topology::VehicleTopology;
use liminal::types::PortId;
use liminal::units::rotational::{Axle, Engine, Wheel};
use liminal::units::signal::{CockpitInput, SignalSplitter};
use liminal::units::spatial::{Chassis, SteeringKnuckle, Suspension};
use liminal::units::transmission::Gearbox;

const WHEELS: usize = 4;
const DT: f32 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut topology = VehicleTopology::new();

    // 挂点位于四角：X 右，Y 上，Z 前（liminal 载具坐标系）。
    let mounts: Vec<Frame> =
        [[-0.8, 0.0, 1.4], [0.8, 0.0, 1.4], [-0.8, 0.0, -1.4], [0.8, 0.0, -1.4]]
            .iter()
            .map(|&translation| Frame::translation(translation))
            .collect();

    let cockpit = topology.add_node(CockpitInput);
    let chassis = topology.add_node(Chassis::new(mounts));
    let engine = topology.add_node(Engine::new(300.0, 600.0, 0.5));
    let gearbox = topology.add_node(Gearbox::new([0.15, 0.25, 0.4, 0.6], 0.2));
    let axle = topology.add_node(Axle::new(WHEELS, 0.5));
    let steering_splitter = topology.add_node(SignalSplitter::new(WHEELS));
    let brake_splitter = topology.add_node(SignalSplitter::new(WHEELS));

    let cockpit_ports = topology.nodes()[cockpit].ports.clone();
    let chassis_ports = topology.nodes()[chassis].ports.clone();
    let engine_ports = topology.nodes()[engine].ports.clone();
    let gearbox_ports = topology.nodes()[gearbox].ports.clone();
    let axle_ports = topology.nodes()[axle].ports.clone();
    let steering_ports = topology.nodes()[steering_splitter].ports.clone();
    let brake_splitter_ports = topology.nodes()[brake_splitter].ports.clone();

    // 控制信号：驾驶舱 → 发动机 / 变速箱 / 转向与制动分发器
    topology
        .connect(cockpit_ports[CockpitInput::THROTTLE_PORT], engine_ports[Engine::THROTTLE_PORT])?;
    topology.connect(cockpit_ports[CockpitInput::GEAR_PORT], gearbox_ports[Gearbox::GEAR_PORT])?;
    topology.connect(
        cockpit_ports[CockpitInput::STEERING_PORT],
        steering_ports[SignalSplitter::INPUT_PORT],
    )?;
    topology.connect(
        cockpit_ports[CockpitInput::BRAKE_PORT],
        brake_splitter_ports[SignalSplitter::INPUT_PORT],
    )?;

    // 动力链：发动机 → 变速箱 → 轮轴 → 四轮
    topology.connect(engine_ports[Engine::SHAFT_PORT], gearbox_ports[Gearbox::INPUT_PORT])?;
    topology.connect(gearbox_ports[Gearbox::OUTPUT_PORT], axle_ports[Axle::INPUT_PORT])?;

    let mut wheel_shafts = Vec::with_capacity(WHEELS);
    for index in 0..WHEELS {
        let suspension = topology.add_node(Suspension::new(35_000.0, 3_000.0, 0.35, 0.2, 0.5));
        let knuckle = topology.add_node(SteeringKnuckle::new(2_000.0));
        let wheel = topology.add_node(Wheel::new(12.0, 30.0, 0.3));

        let suspension_ports = topology.nodes()[suspension].ports.clone();
        let knuckle_ports = topology.nodes()[knuckle].ports.clone();
        let wheel_ports = topology.nodes()[wheel].ports.clone();

        // 空间链：底盘 → 悬挂 → 转向节 → 车轮
        topology.connect(chassis_ports[index], suspension_ports[Suspension::TOP_PORT])?;
        topology.connect(
            suspension_ports[Suspension::BOTTOM_PORT],
            knuckle_ports[SteeringKnuckle::TOP_PORT],
        )?;
        topology
            .connect(knuckle_ports[SteeringKnuckle::BOTTOM_PORT], wheel_ports[Wheel::MOUNT_PORT])?;

        // 旋转与制动：轮轴 → 车轮，转向节 → 车轮
        topology.connect(axle_ports[Axle::output_port(index)], wheel_ports[Wheel::SHAFT_PORT])?;
        topology
            .connect(knuckle_ports[SteeringKnuckle::SHAFT_PORT], wheel_ports[Wheel::SHAFT_PORT])?;

        // 转向与制动信号：分发器 → 转向节
        topology.connect(
            steering_ports[SignalSplitter::output_port(index)],
            knuckle_ports[SteeringKnuckle::STEERING_PORT],
        )?;
        topology.connect(
            brake_splitter_ports[SignalSplitter::output_port(index)],
            knuckle_ports[SteeringKnuckle::BRAKE_PORT],
        )?;

        wheel_shafts.push(wheel_ports[Wheel::SHAFT_PORT]);
    }

    let engine_shaft = engine_ports[Engine::SHAFT_PORT];
    let mut simulator = Simulator::new(topology)?;
    println!(
        "compiled: {} signal ops, {} drivetrain islands, {} spatial links",
        simulator.compiled().signal_plan().len(),
        simulator.compiled().drivetrain_islands().len(),
        simulator.compiled().spatial_links().len(),
    );

    let steps = 3_000;
    for step in 0..steps {
        let time = step as f32 * DT;
        {
            let payloads = simulator.payloads_mut();
            payloads.write(cockpit_ports[CockpitInput::THROTTLE_PORT], PortPayload::Signal(0.6));
            payloads.write(cockpit_ports[CockpitInput::STEERING_PORT], PortPayload::Signal(0.05));
            payloads.write(
                cockpit_ports[CockpitInput::GEAR_PORT],
                PortPayload::Signal(if time < 1.0 {
                    1.0
                } else {
                    2.0
                }),
            );
            payloads.write(cockpit_ports[CockpitInput::BRAKE_PORT], PortPayload::Signal(0.0));
        }

        simulator.step(DT)?;

        if step % 500 == 0 {
            println!(
                "t={:.2}s engine={:.1} rad/s wheel={:.1} rad/s",
                time,
                read_angular_velocity(simulator.payloads(), engine_shaft),
                read_angular_velocity(simulator.payloads(), wheel_shafts[0]),
            );
        }
    }

    Ok(())
}

fn read_angular_velocity(payloads: &PortPayloads, port: PortId) -> f32 {
    match payloads.read(port) {
        Some(PortPayload::Rotational {
            angular_velocity,
            ..
        }) => angular_velocity,
        _ => 0.0,
    }
}
