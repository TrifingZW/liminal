//! 旋转单元：发动机、轮轴与车轮。

use smallvec::{SmallVec, smallvec};

use crate::payload::PortPayload;
use crate::spatial::SpatialJointDesc;
use crate::types::{PortDescriptor, PortDirection, PortDomain};
use crate::units::{RotationalConstraint, RotationalCoupling, UnitContext, VehicleUnit};

/// 全油门扭矩曲线采样点。
///
/// `speed_ratio` 为归一化角速度 `ω/ω_max`，`torque_scale` 为全油门扭矩相对
/// `max_torque` 的比例；查询时在相邻点之间线性插值。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueCurvePoint {
    pub speed_ratio: f32,
    pub torque_scale: f32,
}

/// 发动机参数。
///
/// 默认（[`EngineParameters::new`]）为线性衰减、关闭怠速与发动机制动。
#[derive(Debug, Clone)]
pub struct EngineParameters {
    pub max_torque: f32,
    pub max_angular_velocity: f32,
    pub inertia: f32,
    /// 怠速目标角速度；0 表示关闭怠速控制。
    pub idle_angular_velocity: f32,
    /// 怠速控制器可输出的最大扭矩（N·m）。
    pub idle_torque_limit: f32,
    /// 满转速下的发动机制动扭矩（N·m）；0 表示关闭。
    pub engine_braking_torque: f32,
    /// 全油门扭矩曲线；为空时扭矩随转速线性衰减。
    pub curve: SmallVec<[TorqueCurvePoint; 8]>,
}

impl EngineParameters {
    /// 默认参数：线性衰减，无怠速与发动机制动。
    pub fn new(max_torque: f32, max_angular_velocity: f32, inertia: f32) -> Self {
        Self {
            max_torque,
            max_angular_velocity,
            inertia,
            idle_angular_velocity: 0.0,
            idle_torque_limit: 0.0,
            engine_braking_torque: 0.0,
            curve: SmallVec::new(),
        }
    }
}

/// 发动机：油门信号输入，单旋转轴输出。
///
/// 驱动力矩 = `max_torque · curve(ω/ω_max) · throttle`，与怠速控制器输出取较大值；
/// 节气门关闭且转速高于怠速时，以速度比例的库仑摩擦约束施加发动机制动。
#[derive(Debug, Clone)]
pub struct Engine {
    max_torque: f32,
    max_angular_velocity: f32,
    inertia: f32,
    idle_angular_velocity: f32,
    idle_torque_limit: f32,
    engine_braking_torque: f32,
    curve: SmallVec<[TorqueCurvePoint; 8]>,
}

// 节气门低于该阈值时视为关闭，怠速与发动机制动生效。
const CLOSED_THROTTLE: f32 = 0.01;
// 怠速控制器的比例带宽，占目标怠速转速的比例。
const IDLE_BAND_RATIO: f32 = 0.2;

impl Engine {
    /// 油门信号端口在节点端口表中的索引。
    pub const THROTTLE_PORT: usize = 0;
    /// 旋转轴端口在节点端口表中的索引。
    pub const SHAFT_PORT: usize = 1;

    /// 默认参数：扭矩随转速线性衰减，关闭怠速与发动机制动。
    pub fn new(max_torque: f32, max_angular_velocity: f32, inertia: f32) -> Self {
        Self::with_parameters(EngineParameters::new(max_torque, max_angular_velocity, inertia))
    }

    /// 按参数构造，并清理非法值：非有限/越界曲线点被丢弃或钳制，怠速与制动参数钳正。
    pub fn with_parameters(parameters: EngineParameters) -> Self {
        let max_angular_velocity = if parameters.max_angular_velocity.is_finite() {
            parameters.max_angular_velocity.max(f32::EPSILON)
        } else {
            f32::EPSILON
        };
        let max_torque = if parameters.max_torque.is_finite() {
            parameters.max_torque.max(0.0)
        } else {
            0.0
        };

        let mut curve = parameters.curve;
        curve.retain(|point| point.speed_ratio.is_finite() && point.torque_scale.is_finite());
        for point in curve.iter_mut() {
            point.speed_ratio = point.speed_ratio.clamp(0.0, 1.0);
            point.torque_scale = point.torque_scale.clamp(0.0, 1.0);
        }
        curve.sort_by(|left, right| left.speed_ratio.total_cmp(&right.speed_ratio));

        let idle_angular_velocity = if parameters.idle_angular_velocity.is_finite() {
            parameters.idle_angular_velocity.clamp(0.0, max_angular_velocity)
        } else {
            0.0
        };
        let idle_torque_limit = if parameters.idle_torque_limit.is_finite() {
            parameters.idle_torque_limit.clamp(0.0, max_torque)
        } else {
            0.0
        };
        let engine_braking_torque = if parameters.engine_braking_torque.is_finite() {
            parameters.engine_braking_torque.max(0.0)
        } else {
            0.0
        };

        Self {
            max_torque,
            max_angular_velocity,
            inertia: if parameters.inertia.is_finite() {
                parameters.inertia.max(0.0)
            } else {
                0.0
            },
            idle_angular_velocity,
            idle_torque_limit,
            engine_braking_torque,
            curve,
        }
    }

    /// 全油门扭矩比例 `[0, 1]`，按归一化转速在曲线上线性插值。
    ///
    /// 曲线为空时退化为 `1 - speed_ratio`（线性衰减）；
    /// 查询转速超出曲线首尾点时取端点值。
    pub fn torque_scale(&self, speed_ratio: f32) -> f32 {
        let ratio = if speed_ratio.is_finite() {
            speed_ratio.clamp(0.0, 1.0)
        } else {
            0.0
        };
        if self.curve.is_empty() {
            return (1.0 - ratio).clamp(0.0, 1.0);
        }
        if self.curve.len() == 1 {
            return self.curve[0].torque_scale;
        }

        let mut previous = self.curve[0];
        for &point in &self.curve[1..] {
            if ratio <= point.speed_ratio {
                let width = point.speed_ratio - previous.speed_ratio;
                if width <= f32::EPSILON {
                    return point.torque_scale;
                }
                let t = ((ratio - previous.speed_ratio) / width).clamp(0.0, 1.0);
                return previous.torque_scale + (point.torque_scale - previous.torque_scale) * t;
            }
            previous = point;
        }
        self.curve.last().map(|point| point.torque_scale).unwrap_or(0.0)
    }

    /// 当前工况的驱动力矩（N·m）：全油门曲线与怠速控制器取较大值。
    pub fn torque(&self, throttle: f32, angular_velocity: f32) -> f32 {
        let throttle = if throttle.is_finite() {
            throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let angular_velocity = if angular_velocity.is_finite() {
            angular_velocity
        } else {
            0.0
        };
        let speed_ratio = (angular_velocity / self.max_angular_velocity).clamp(0.0, 1.0);
        let full = self.max_torque * self.torque_scale(speed_ratio) * throttle;

        let idle = if self.idle_angular_velocity > 0.0
            && self.idle_torque_limit > 0.0
            && angular_velocity < self.idle_angular_velocity
        {
            let band = (self.idle_angular_velocity * IDLE_BAND_RATIO).max(f32::EPSILON);
            let error = (self.idle_angular_velocity - angular_velocity) / band;
            self.idle_torque_limit * error.clamp(0.0, 1.0)
        } else {
            0.0
        };

        full.max(idle)
    }

    /// 当前工况的发动机制动扭矩（N·m，非负），随转速线性增长。
    pub fn braking_torque(&self, angular_velocity: f32) -> f32 {
        if self.engine_braking_torque <= 0.0 || !angular_velocity.is_finite() {
            return 0.0;
        }
        let speed_ratio = (angular_velocity / self.max_angular_velocity).clamp(0.0, 1.0);
        self.engine_braking_torque * speed_ratio
    }

    fn signal(&self, context: &UnitContext<'_>, index: usize) -> f32 {
        match context.read(index) {
            Some(PortPayload::Signal(value)) if value.is_finite() => value.clamp(0.0, 1.0),
            _ => 0.0,
        }
    }

    fn shaft_angular_velocity(&self, context: &UnitContext<'_>) -> f32 {
        match context.read(Self::SHAFT_PORT) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) if angular_velocity.is_finite() => angular_velocity,
            _ => 0.0,
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
        let throttle = self.signal(context, Self::THROTTLE_PORT);
        let angular_velocity = self.shaft_angular_velocity(context);
        self.torque(throttle, angular_velocity)
    }

    fn rotational_constraints(
        &self,
        context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        let throttle = self.signal(context, Self::THROTTLE_PORT);
        if throttle > CLOSED_THROTTLE {
            return SmallVec::new();
        }
        let angular_velocity = self.shaft_angular_velocity(context);
        if angular_velocity <= self.idle_angular_velocity {
            return SmallVec::new();
        }
        let braking = self.braking_torque(angular_velocity);
        if braking <= 0.0 {
            return SmallVec::new();
        }
        smallvec![RotationalConstraint::Friction {
            port_a: Self::SHAFT_PORT,
            port_b: None,
            max_torque: braking,
        }]
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
            inertia: inertia.max(0.0),
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

/// 差速器：开式差速 + 摩擦限滑。
///
/// 开式差速约束 `ω_in = (1/N)·Σω_out`，输出端等扭矩分配；
/// 限滑为输出端之间的库仑摩擦约束 `ω_out0 = ω_outi`，传递扭矩上限为 `max_torque`：
/// 需求扭矩在上限内时输出端同步（接近锁止），超出时以最大扭矩滑摩。
/// `max_torque = 0` 即纯开式差速。
#[derive(Debug, Clone, Copy)]
pub struct Differential {
    output_count: usize,
    max_torque: f32,
    inertia: f32,
}

impl Differential {
    /// 输入端口在节点端口表中的索引。
    pub const INPUT_PORT: usize = 0;

    /// 创建指定输出路数、限滑扭矩容量与转动惯量的差速器。
    pub fn new(output_count: usize, max_torque: f32, inertia: f32) -> Self {
        Self {
            output_count,
            max_torque: max_torque.max(0.0),
            inertia: inertia.max(0.0),
        }
    }

    /// 第 `index` 个输出端口在节点端口表中的索引。
    pub const fn output_port(index: usize) -> usize {
        index + 1
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

    fn rotational_constraints(
        &self,
        _context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        if self.output_count == 0 {
            return SmallVec::new();
        }

        let outputs: SmallVec<[usize; 4]> = (0..self.output_count).map(Self::output_port).collect();
        let mut constraints = SmallVec::new();
        constraints.push(RotationalConstraint::OpenDifferential {
            input_port: Self::INPUT_PORT,
            output_ports: outputs,
        });

        // 限滑：每个输出端与首个输出端建立摩擦约束，输出端在扭矩容量内保持同步。
        if self.max_torque > 0.0 {
            for index in 1..self.output_count {
                constraints.push(RotationalConstraint::Friction {
                    port_a: Self::output_port(0),
                    port_b: Some(Self::output_port(index)),
                    max_torque: self.max_torque,
                });
            }
        }
        constraints
    }
}

/// 车轮：接收驱动力矩，并以库仑摩擦约束提供轴承/滚动阻力矩。
///
/// 摩擦约束把车轮转速锁止在零（扭矩上限为 `friction_torque`），
/// 因此静止时不会因微小残矩缓慢蠕行。
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
            inertia: inertia.max(0.0),
            friction_torque: friction_torque.max(0.0),
            radius: radius.max(0.0),
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

    fn rotational_constraints(
        &self,
        _context: &UnitContext<'_>,
    ) -> SmallVec<[RotationalConstraint; 8]> {
        if self.friction_torque <= 0.0 {
            return SmallVec::new();
        }
        smallvec![RotationalConstraint::Friction {
            port_a: Self::SHAFT_PORT,
            port_b: None,
            max_torque: self.friction_torque,
        }]
    }
}

#[cfg(test)]
mod tests {
    use super::{Axle, Differential, Engine, EngineParameters, TorqueCurvePoint, Wheel};
    use crate::compiler::compile;
    use crate::payload::{PortPayload, PortPayloads};
    use crate::solver_1d::step;
    use crate::topology::VehicleTopology;
    use crate::types::PortId;
    use crate::units::signal::CockpitInput;
    use crate::units::transmission::Brake;
    use crate::units::{RotationalConstraint, UnitContext};
    use smallvec::smallvec;

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
    fn engine_legacy_linear_torque_decline() {
        let engine = Engine::new(100.0, 100.0, 1.0);
        assert!((engine.torque(1.0, 0.0) - 100.0).abs() < 1e-4);
        assert!((engine.torque(1.0, 50.0) - 50.0).abs() < 1e-4);
        assert!(engine.torque(1.0, 100.0).abs() < 1e-4);
        assert!((engine.torque(0.5, 0.0) - 50.0).abs() < 1e-4);
        assert!(engine.torque(1.0, 150.0).abs() < 1e-4);
        assert_eq!(engine.braking_torque(50.0), 0.0);
    }

    #[test]
    fn engine_torque_curve_interpolates() {
        let mut parameters = EngineParameters::new(100.0, 100.0, 1.0);
        parameters.curve = smallvec![
            TorqueCurvePoint {
                speed_ratio: 0.0,
                torque_scale: 0.2,
            },
            TorqueCurvePoint {
                speed_ratio: 0.5,
                torque_scale: 1.0,
            },
            TorqueCurvePoint {
                speed_ratio: 1.0,
                torque_scale: 0.4,
            },
        ];
        let engine = Engine::with_parameters(parameters);

        assert!((engine.torque(1.0, 0.0) - 20.0).abs() < 1e-4);
        assert!((engine.torque(1.0, 25.0) - 60.0).abs() < 1e-4);
        assert!((engine.torque(1.0, 50.0) - 100.0).abs() < 1e-4);
        assert!((engine.torque(1.0, 75.0) - 70.0).abs() < 1e-4);
    }

    #[test]
    fn engine_curve_sanitizes_invalid_points() {
        let mut parameters = EngineParameters::new(100.0, 100.0, 1.0);
        parameters.curve = smallvec![
            TorqueCurvePoint {
                speed_ratio: 0.8,
                torque_scale: 0.9,
            },
            TorqueCurvePoint {
                speed_ratio: f32::NAN,
                torque_scale: 0.5,
            },
            TorqueCurvePoint {
                speed_ratio: 0.2,
                torque_scale: -1.0,
            },
            TorqueCurvePoint {
                speed_ratio: 0.2,
                torque_scale: 0.5,
            },
            TorqueCurvePoint {
                speed_ratio: 0.5,
                torque_scale: 2.0,
            },
        ];
        let engine = Engine::with_parameters(parameters);

        for ratio in [0.0, 0.2, 0.5, 0.8, 1.0, f32::NAN] {
            let scale = engine.torque_scale(ratio);
            assert!(scale.is_finite());
            assert!((0.0..=1.0).contains(&scale));
        }
        assert!((engine.torque_scale(0.5) - 1.0).abs() < 1e-4);
        assert!((engine.torque_scale(1.0) - 0.9).abs() < 1e-4);
    }

    #[test]
    fn engine_idle_lifts_stationary_engine() {
        let mut parameters = EngineParameters::new(100.0, 600.0, 0.5);
        parameters.idle_angular_velocity = 100.0;
        parameters.idle_torque_limit = 50.0;
        parameters.engine_braking_torque = 60.0;
        let engine = Engine::with_parameters(parameters);
        assert!(engine.torque(0.0, 0.0) > 0.0);
        assert!(engine.torque(0.0, 100.0) <= f32::EPSILON);

        let mut topology = VehicleTopology::new();
        let engine_node = topology.add_node(engine);
        let engine_ports = topology.nodes()[engine_node].ports.clone();
        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        let compiled = compile(&topology).unwrap();

        for _ in 0..2_000 {
            step(&topology, &compiled, &mut payloads, 0.0025);
        }

        let omega = angular_velocity(&payloads, engine_ports[Engine::SHAFT_PORT]);
        assert!((omega - 100.0).abs() < 10.0, "omega = {omega}");
    }

    #[test]
    fn engine_braking_constraint_tracks_speed() {
        let mut parameters = EngineParameters::new(100.0, 600.0, 0.5);
        parameters.idle_angular_velocity = 100.0;
        parameters.engine_braking_torque = 60.0;
        let engine = Engine::with_parameters(parameters);

        assert!((engine.braking_torque(600.0) - 60.0).abs() < 1e-4);
        assert!((engine.braking_torque(300.0) - 30.0).abs() < 1e-4);

        let mut topology = VehicleTopology::new();
        let engine_node = topology.add_node(engine);
        let engine_ports = topology.nodes()[engine_node].ports.clone();
        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(
            engine_ports[Engine::SHAFT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 300.0,
            },
        );

        let node = &topology.nodes()[engine_node];
        let context = UnitContext::new(&node.ports, &payloads);
        let constraints = node.unit.rotational_constraints(&context);
        assert_eq!(constraints.len(), 1);
        match &constraints[0] {
            RotationalConstraint::Friction {
                port_a,
                port_b,
                max_torque,
            } => {
                assert_eq!(*port_a, Engine::SHAFT_PORT);
                assert!(port_b.is_none());
                assert!((*max_torque - 30.0).abs() < 1e-4);
            }
            other => panic!("应为摩擦约束，实际为 {other:?}"),
        }

        payloads.write(engine_ports[Engine::THROTTLE_PORT], PortPayload::Signal(1.0));
        let context = UnitContext::new(&node.ports, &payloads);
        assert!(node.unit.rotational_constraints(&context).is_empty());
    }

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
    fn open_differential_enforces_mean_output_speed() {
        let mut topology = VehicleTopology::new();
        let differential = topology.add_node(Differential::new(2, 0.0, 0.5));
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
        for (output, wheel) in [
            (Differential::output_port(0), left_ports[Wheel::SHAFT_PORT]),
            (Differential::output_port(1), right_ports[Wheel::SHAFT_PORT]),
        ] {
            for port in [differential_ports[output], wheel] {
                payloads.write(
                    port,
                    PortPayload::Rotational {
                        torque: 0.0,
                        angular_velocity: 10.0,
                    },
                );
            }
        }

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        let input = angular_velocity(differential_ports[Differential::INPUT_PORT]);
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);

        assert!((input - (left_omega + right_omega) * 0.5).abs() < 1e-5);
        assert!((left_omega - right_omega).abs() < 1e-6);
        assert!((input - 9.0).abs() < 1e-4);
    }

    #[test]
    fn open_differential_equalizes_output_torque_impulses() {
        let mut topology = VehicleTopology::new();
        let differential = topology.add_node(Differential::new(2, 0.0, 0.5));
        let left = topology.add_node(Wheel::new(1.0, 0.0, 0.3));
        let right = topology.add_node(Wheel::new(2.0, 0.0, 0.3));

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
        payloads.write(
            differential_ports[Differential::output_port(0)],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 10.0,
            },
        );
        payloads.write(
            left_ports[Wheel::SHAFT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 10.0,
            },
        );
        payloads.write(
            differential_ports[Differential::output_port(1)],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 20.0,
            },
        );
        payloads.write(
            right_ports[Wheel::SHAFT_PORT],
            PortPayload::Rotational {
                torque: 0.0,
                angular_velocity: 20.0,
            },
        );

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        let input = angular_velocity(differential_ports[Differential::INPUT_PORT]);
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);

        // 输入 5→13.421、左 10→7.895、右 20→18.947（λ = 10/2.375）；
        // 输出端角冲量相等：J_left·Δω_left = J_right·Δω_right = −2.105。
        assert!((input - (left_omega + right_omega) * 0.5).abs() < 1e-5);
        let left_impulse = 1.0 * (left_omega - 10.0);
        let right_impulse = 2.0 * (right_omega - 20.0);
        assert!((left_impulse - right_impulse).abs() < 1e-4);
        assert!(left_impulse < 0.0);
    }

    #[test]
    fn limited_slip_locks_outputs_within_torque_capacity() {
        let mut topology = VehicleTopology::new();
        let differential = topology.add_node(Differential::new(2, 2_000.0, 0.5));
        let left = topology.add_node(Wheel::new(1.0, 0.0, 0.3));
        let right = topology.add_node(Wheel::new(2.0, 0.0, 0.3));

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
        for port in
            [differential_ports[Differential::output_port(1)], right_ports[Wheel::SHAFT_PORT]]
        {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque: 0.0,
                    angular_velocity: 20.0,
                },
            );
        }

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        let input = angular_velocity(differential_ports[Differential::INPUT_PORT]);
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);

        // 容量充足：输出端同步；总角动量 (0.5·5 + 1·10 + 2·20) / 3.5 = 15。
        assert!((left_omega - right_omega).abs() < 1e-4);
        assert!((input - left_omega).abs() < 1e-4);
        assert!((left_omega - 15.0).abs() < 1e-3);
    }

    #[test]
    fn limited_slip_saturates_beyond_torque_capacity() {
        let mut topology = VehicleTopology::new();
        let differential = topology.add_node(Differential::new(2, 10.0, 0.5));
        let left = topology.add_node(Wheel::new(1.0, 0.0, 0.3));
        let right = topology.add_node(Wheel::new(2.0, 0.0, 0.3));

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
        for port in
            [differential_ports[Differential::output_port(1)], right_ports[Wheel::SHAFT_PORT]]
        {
            payloads.write(
                port,
                PortPayload::Rotational {
                    torque: 0.0,
                    angular_velocity: 20.0,
                },
            );
        }

        let compiled = compile(&topology).unwrap();
        step(&topology, &compiled, &mut payloads, 0.01);

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);

        // 需求冲量约 7.4 N·m·s 远超上限 0.1 N·m·s，输出端保持明显转速差。
        assert!((left_omega - right_omega).abs() > 9.0);
    }

    #[test]
    fn open_differential_doubles_free_output_when_one_is_braked() {
        let mut topology = VehicleTopology::new();
        let engine = topology.add_node(Engine::new(100.0, 1_000.0, 0.5));
        let differential = topology.add_node(Differential::new(2, 0.0, 0.5));
        let brake = topology.add_node(Brake::new(1_000.0));
        let left = topology.add_node(Wheel::new(1.0, 0.0, 0.3));
        let right = topology.add_node(Wheel::new(1.0, 0.0, 0.3));

        let engine_ports = topology.nodes()[engine].ports.clone();
        let differential_ports = topology.nodes()[differential].ports.clone();
        let brake_ports = topology.nodes()[brake].ports.clone();
        let left_ports = topology.nodes()[left].ports.clone();
        let right_ports = topology.nodes()[right].ports.clone();

        topology
            .connect(engine_ports[Engine::SHAFT_PORT], differential_ports[Differential::INPUT_PORT])
            .unwrap();
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
        topology.connect(brake_ports[Brake::SHAFT_PORT], left_ports[Wheel::SHAFT_PORT]).unwrap();

        let mut payloads = PortPayloads::new();
        payloads.rebuild(&topology);
        payloads.write(engine_ports[Engine::THROTTLE_PORT], PortPayload::Signal(1.0));
        payloads.write(brake_ports[Brake::BRAKE_PORT], PortPayload::Signal(1.0));

        let compiled = compile(&topology).unwrap();
        for _ in 0..200 {
            step(&topology, &compiled, &mut payloads, 0.0025);
        }

        let angular_velocity = |port| match payloads.read(port) {
            Some(PortPayload::Rotational {
                angular_velocity,
                ..
            }) => angular_velocity,
            _ => panic!("端口应持有旋转载荷"),
        };
        let input = angular_velocity(differential_ports[Differential::INPUT_PORT]);
        let left_omega = angular_velocity(left_ports[Wheel::SHAFT_PORT]);
        let right_omega = angular_velocity(right_ports[Wheel::SHAFT_PORT]);

        assert!(input > 0.0);
        assert!(left_omega.abs() < 1e-3);
        assert!((right_omega - 2.0 * input).abs() < 1e-2);
    }
}
