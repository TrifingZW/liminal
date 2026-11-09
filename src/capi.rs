//! C ABI：宿主（Chaos/UE）适配壳与库之间的稳定边界。
//!
//! 设计约束：
//! - 只使用 `#[repr(C)]` 的 POD 与不透明句柄，不跨语言传递 Rust 所有权类型；
//! - 所有函数检查空指针，错误以 [`LiminalStatus`] 返回；
//! - `NodeId`/`PortId`/`EdgeId` 通过 slotmap 的 FFI 键值导出为 `u64`；
//! - 句柄非线程安全，调用方需自行同步。
//
// 安全契约统一见模块文档与适配契约：所有 extern "C" 函数要求有效句柄与独占访问。
#![allow(clippy::missing_safety_doc)]

use std::ptr;
use std::slice;

use slotmap::{Key, KeyData};
use smallvec::SmallVec;

use crate::payload::PortPayload;
use crate::simulator::Simulator;
use crate::spatial::{Frame, SpatialJointDesc};
use crate::tire::{TireConstraint, TireForces, TireModel, WheelMotion};
use crate::topology::VehicleTopology;
use crate::types::{ConnectError, EdgeId, NodeId, PortId};
use crate::units::rotational::{
    Axle, Differential, Engine, EngineParameters, TorqueCurvePoint, Wheel,
};
use crate::units::signal::{CockpitInput, SignalSplitter};
use crate::units::spatial::{Chassis, SteeringKnuckle, Suspension, anti_roll_force};
use crate::units::transmission::{Brake, Clutch, Gearbox};

/// 三维向量。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 轮胎坐标系下的合力与车轮反作用。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalTireForces {
    pub longitudinal: f32,
    pub lateral: f32,
    pub wheel_torque: f32,
}

impl From<TireForces> for LiminalTireForces {
    fn from(forces: TireForces) -> Self {
        Self {
            longitudinal: forces.longitudinal,
            lateral: forces.lateral,
            wheel_torque: forces.wheel_torque,
        }
    }
}

/// 约束式轮胎耦合的摩擦椭圆界与外部力。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalTireConstraint {
    pub longitudinal_limit: f32,
    pub lateral_limit: f32,
    pub static_longitudinal_limit: f32,
    pub static_lateral_limit: f32,
    pub lateral_force: f32,
    pub rolling_force: f32,
}

impl From<TireConstraint> for LiminalTireConstraint {
    fn from(constraint: TireConstraint) -> Self {
        Self {
            longitudinal_limit: constraint.longitudinal_limit,
            lateral_limit: constraint.lateral_limit,
            static_longitudinal_limit: constraint.static_longitudinal_limit,
            static_lateral_limit: constraint.static_lateral_limit,
            lateral_force: constraint.lateral_force,
            rolling_force: constraint.rolling_force,
        }
    }
}

/// 轮胎模型参数。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalTireModel {
    pub friction_coefficient: f32,
    pub lateral_friction_coefficient: f32,
    pub lateral_stiffness: f32,
    pub load_sensitivity: f32,
    pub reference_load: f32,
    pub rolling_resistance: f32,
    pub static_friction_scale: f32,
}

impl From<LiminalTireModel> for TireModel {
    fn from(params: LiminalTireModel) -> Self {
        Self {
            friction_coefficient: params.friction_coefficient,
            lateral_friction_coefficient: params.lateral_friction_coefficient,
            lateral_stiffness: params.lateral_stiffness,
            load_sensitivity: params.load_sensitivity,
            reference_load: params.reference_load,
            rolling_resistance: params.rolling_resistance,
            static_friction_scale: params.static_friction_scale,
        }
    }
}

/// C ABI 状态码。
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiminalStatus {
    Ok = 0,
    InvalidHandle = 1,
    NodeNotFound = 2,
    PortNotFound = 3,
    SelfConnection = 4,
    DomainMismatch = 5,
    DirectionIncompatible = 6,
    CapacityReached = 7,
    CompileFailed = 8,
    WrongPayload = 9,
    IndexOutOfRange = 10,
    InvalidArgument = 11,
}

/// 空间关节类型。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LiminalJointKind {
    #[default]
    Rigid = 0,
    Revolute = 1,
    PrismaticSpring = 2,
}

/// 空间关节描述。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalJointDesc {
    pub kind: LiminalJointKind,
    pub axis: LiminalVec3,
    pub stiffness: f32,
    pub damping: f32,
    pub rest_length: f32,
    pub min_length: f32,
    pub max_length: f32,
}

/// 发动机全油门扭矩曲线采样点。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalEngineCurvePoint {
    /// 归一化角速度 `ω/ω_max`。
    pub speed_ratio: f32,
    /// 全油门扭矩相对 `max_torque` 的比例。
    pub torque_scale: f32,
}

/// 发动机参数。
///
/// `curve` 为 `nullptr` 或 `curve_count` 为 0 时使用线性衰减；
/// `idle_angular_velocity` 或 `idle_torque_limit` 为 0 时关闭怠速控制；
/// `engine_braking_torque` 为 0 时关闭发动机制动。
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiminalEngineParams {
    pub max_torque: f32,
    pub max_angular_velocity: f32,
    pub inertia: f32,
    pub idle_angular_velocity: f32,
    pub idle_torque_limit: f32,
    pub engine_braking_torque: f32,
    pub curve: *const LiminalEngineCurvePoint,
    pub curve_count: usize,
}

impl Default for LiminalEngineParams {
    fn default() -> Self {
        Self {
            max_torque: 0.0,
            max_angular_velocity: 0.0,
            inertia: 0.0,
            idle_angular_velocity: 0.0,
            idle_torque_limit: 0.0,
            engine_braking_torque: 0.0,
            curve: ptr::null(),
            curve_count: 0,
        }
    }
}

/// 刚体变换：平移与四元数旋转，旋转分量顺序为 (x, y, z, w)。
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiminalFrame {
    pub translation: LiminalVec3,
    pub rotation: [f32; 4],
}

impl Default for LiminalFrame {
    fn default() -> Self {
        Self {
            translation: LiminalVec3::default(),
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

/// 空间运动学树中的一条父子连接。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalSpatialLink {
    pub parent: u64,
    pub child: u64,
    pub joint: LiminalJointDesc,
    /// `has_control` 为 0 时无意义。
    pub control_port: u64,
    pub has_control: u32,
    /// 静止位形下子节点相对父节点的位姿。
    pub rest_pose: LiminalFrame,
    /// 静止位形下子节点相对运动学树根的位姿。
    pub pose_in_root: LiminalFrame,
}

/// 信号执行计划中的一条拷贝指令。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LiminalSignalOp {
    pub source_port: u64,
    pub target_port: u64,
}

/// C ABI 版本：布局或语义发生不兼容变更时递增。
pub const LIMINAL_ABI_VERSION: u32 = 3;

/// 返回 C ABI 版本，供宿主在初始化时校验头文件与静态库是否匹配。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_abi_version() -> u32 {
    LIMINAL_ABI_VERSION
}

/// 不透明载具句柄。
pub struct LiminalVehicle {
    simulator: Simulator,
}

impl From<ConnectError> for LiminalStatus {
    fn from(error: ConnectError) -> Self {
        match error {
            ConnectError::PortNotFound(_) => Self::PortNotFound,
            ConnectError::DomainMismatch(_, _) => Self::DomainMismatch,
            ConnectError::DirectionIncompatible => Self::DirectionIncompatible,
            ConnectError::SourcePortCapacityReached {
                ..
            }
            | ConnectError::TargetPortCapacityReached {
                ..
            } => Self::CapacityReached,
            ConnectError::SelfConnectionNotAllowed => Self::SelfConnection,
        }
    }
}

fn node_to_ffi(node: NodeId) -> u64 {
    node.data().as_ffi()
}

fn node_from_ffi(raw: u64) -> NodeId {
    KeyData::from_ffi(raw).into()
}

fn port_to_ffi(port: PortId) -> u64 {
    port.data().as_ffi()
}

fn port_from_ffi(raw: u64) -> PortId {
    KeyData::from_ffi(raw).into()
}

fn edge_to_ffi(edge: EdgeId) -> u64 {
    edge.data().as_ffi()
}

fn edge_from_ffi(raw: u64) -> EdgeId {
    KeyData::from_ffi(raw).into()
}

fn joint_to_ffi(joint: SpatialJointDesc) -> LiminalJointDesc {
    match joint {
        SpatialJointDesc::Rigid => LiminalJointDesc {
            kind: LiminalJointKind::Rigid,
            axis: LiminalVec3::default(),
            stiffness: 0.0,
            damping: 0.0,
            rest_length: 0.0,
            min_length: 0.0,
            max_length: 0.0,
        },
        SpatialJointDesc::Revolute {
            axis,
        } => LiminalJointDesc {
            kind: LiminalJointKind::Revolute,
            axis: LiminalVec3 {
                x: axis[0],
                y: axis[1],
                z: axis[2],
            },
            stiffness: 0.0,
            damping: 0.0,
            rest_length: 0.0,
            min_length: 0.0,
            max_length: 0.0,
        },
        SpatialJointDesc::PrismaticSpring {
            axis,
            stiffness,
            damping,
            rest_length,
            min_length,
            max_length,
        } => LiminalJointDesc {
            kind: LiminalJointKind::PrismaticSpring,
            axis: LiminalVec3 {
                x: axis[0],
                y: axis[1],
                z: axis[2],
            },
            stiffness,
            damping,
            rest_length,
            min_length,
            max_length,
        },
    }
}

fn frame_to_ffi(frame: Frame) -> LiminalFrame {
    LiminalFrame {
        translation: LiminalVec3 {
            x: frame.translation[0],
            y: frame.translation[1],
            z: frame.translation[2],
        },
        rotation: frame.rotation,
    }
}

fn frame_from_ffi(frame: LiminalFrame) -> Frame {
    Frame {
        translation: [frame.translation.x, frame.translation.y, frame.translation.z],
        rotation: frame.rotation,
    }
}

unsafe fn push_node(
    vehicle: *mut LiminalVehicle,
    out_node: *mut u64,
    add: impl FnOnce(&mut VehicleTopology) -> NodeId,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_node) = (unsafe { out_node.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    let node = add(vehicle.simulator.topology_mut());
    *out_node = node_to_ffi(node);
    LiminalStatus::Ok
}

/// 创建空载具句柄，失败时返回空指针。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_vehicle_new() -> *mut LiminalVehicle {
    let Ok(simulator) = Simulator::new(VehicleTopology::new()) else {
        return ptr::null_mut();
    };
    Box::into_raw(Box::new(LiminalVehicle {
        simulator,
    }))
}

/// 释放载具句柄；允许传入空指针。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_free(vehicle: *mut LiminalVehicle) {
    if vehicle.is_null() {
        return;
    }
    drop(unsafe { Box::from_raw(vehicle) });
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_cockpit(
    vehicle: *mut LiminalVehicle,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe { push_node(vehicle, out_node, |topology| topology.add_node(CockpitInput)) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_engine(
    vehicle: *mut LiminalVehicle,
    max_torque: f32,
    max_angular_velocity: f32,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Engine::new(max_torque, max_angular_velocity, inertia))
        })
    }
}

#[unsafe(no_mangle)]
/// 按高级参数创建发动机节点。
///
/// # Safety
///
/// `params` 必须指向有效的 [`LiminalEngineParams`]；`curve` 非空时须指向至少
/// `curve_count` 个有效的 [`LiminalEngineCurvePoint`]。
pub unsafe extern "C" fn liminal_vehicle_add_engine_with_params(
    vehicle: *mut LiminalVehicle,
    params: *const LiminalEngineParams,
    out_node: *mut u64,
) -> LiminalStatus {
    if params.is_null() {
        return LiminalStatus::InvalidArgument;
    }
    let params = unsafe { *params };

    if !params.max_torque.is_finite()
        || !params.max_angular_velocity.is_finite()
        || !params.inertia.is_finite()
        || !params.idle_angular_velocity.is_finite()
        || !params.idle_torque_limit.is_finite()
        || !params.engine_braking_torque.is_finite()
        || (params.curve.is_null() && params.curve_count > 0)
    {
        return LiminalStatus::InvalidArgument;
    }

    let curve: SmallVec<[TorqueCurvePoint; 8]> = if params.curve_count == 0 {
        SmallVec::new()
    } else {
        let points = unsafe { slice::from_raw_parts(params.curve, params.curve_count) };
        points
            .iter()
            .map(|point| TorqueCurvePoint {
                speed_ratio: point.speed_ratio,
                torque_scale: point.torque_scale,
            })
            .collect()
    };

    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Engine::with_parameters(EngineParameters {
                max_torque: params.max_torque,
                max_angular_velocity: params.max_angular_velocity,
                inertia: params.inertia,
                idle_angular_velocity: params.idle_angular_velocity,
                idle_torque_limit: params.idle_torque_limit,
                engine_braking_torque: params.engine_braking_torque,
                curve,
            }))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_gearbox(
    vehicle: *mut LiminalVehicle,
    ratios: *const f32,
    ratio_count: usize,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    if ratio_count == 0 || (ratios.is_null() && ratio_count > 0) {
        return LiminalStatus::InvalidArgument;
    }
    let ratios = unsafe { slice::from_raw_parts(ratios, ratio_count) };
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Gearbox::new(ratios.iter().copied(), inertia))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_axle(
    vehicle: *mut LiminalVehicle,
    output_count: usize,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| topology.add_node(Axle::new(output_count, inertia)))
    }
}

/// 添加差速器：开式差速约束 + 输出端之间的摩擦限滑，`max_torque` 为限滑扭矩容量。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_differential(
    vehicle: *mut LiminalVehicle,
    output_count: usize,
    max_torque: f32,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    if output_count == 0 {
        return LiminalStatus::InvalidArgument;
    }
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Differential::new(output_count, max_torque, inertia))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_wheel(
    vehicle: *mut LiminalVehicle,
    inertia: f32,
    friction_torque: f32,
    radius: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Wheel::new(inertia, friction_torque, radius))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_brake(
    vehicle: *mut LiminalVehicle,
    max_torque: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe { push_node(vehicle, out_node, |topology| topology.add_node(Brake::new(max_torque))) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_clutch(
    vehicle: *mut LiminalVehicle,
    max_torque: f32,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| topology.add_node(Clutch::new(max_torque, inertia)))
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_chassis(
    vehicle: *mut LiminalVehicle,
    mounts: *const LiminalFrame,
    mount_count: usize,
    out_node: *mut u64,
) -> LiminalStatus {
    if mounts.is_null() || mount_count == 0 {
        return LiminalStatus::InvalidArgument;
    }
    let mounts = unsafe { slice::from_raw_parts(mounts, mount_count) }
        .iter()
        .map(|&frame| frame_from_ffi(frame))
        .collect();
    unsafe { push_node(vehicle, out_node, |topology| topology.add_node(Chassis::new(mounts))) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_suspension(
    vehicle: *mut LiminalVehicle,
    stiffness: f32,
    damping: f32,
    rest_length: f32,
    min_length: f32,
    max_length: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Suspension::new(
                stiffness,
                damping,
                rest_length,
                min_length,
                max_length,
            ))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_knuckle(
    vehicle: *mut LiminalVehicle,
    max_brake_torque: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(SteeringKnuckle::new(max_brake_torque))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_splitter(
    vehicle: *mut LiminalVehicle,
    output_count: usize,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(SignalSplitter::new(output_count))
        })
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_port_count(
    vehicle: *const LiminalVehicle,
    node: u64,
    out_count: *mut usize,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_count) = (unsafe { out_count.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    let Some(node) = vehicle.simulator.topology().nodes().get(node_from_ffi(node)) else {
        return LiminalStatus::NodeNotFound;
    };
    *out_count = node.ports.len();
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_port(
    vehicle: *const LiminalVehicle,
    node: u64,
    index: usize,
    out_port: *mut u64,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_port) = (unsafe { out_port.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    let Some(node) = vehicle.simulator.topology().nodes().get(node_from_ffi(node)) else {
        return LiminalStatus::NodeNotFound;
    };
    let Some(&port) = node.ports.get(index) else {
        return LiminalStatus::IndexOutOfRange;
    };
    *out_port = port_to_ffi(port);
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_connect(
    vehicle: *mut LiminalVehicle,
    source_port: u64,
    target_port: u64,
    out_edge: *mut u64,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    match vehicle
        .simulator
        .topology_mut()
        .connect(port_from_ffi(source_port), port_from_ffi(target_port))
    {
        Ok(edge) => {
            if let Some(out_edge) = unsafe { out_edge.as_mut() } {
                *out_edge = edge_to_ffi(edge);
            }
            LiminalStatus::Ok
        }
        Err(error) => error.into(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_disconnect(
    vehicle: *mut LiminalVehicle,
    edge: u64,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    if vehicle.simulator.topology_mut().disconnect(edge_from_ffi(edge)) {
        LiminalStatus::Ok
    } else {
        LiminalStatus::IndexOutOfRange
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_remove_node(
    vehicle: *mut LiminalVehicle,
    node: u64,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    if vehicle.simulator.topology_mut().remove_node(node_from_ffi(node)) {
        LiminalStatus::Ok
    } else {
        LiminalStatus::NodeNotFound
    }
}

/// 拓扑自上次编译后是否变更：1 需要重编译，0 已同步，负数表示无效句柄。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_is_dirty(vehicle: *const LiminalVehicle) -> i32 {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return -1;
    };
    i32::from(vehicle.simulator.topology().is_dirty())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_recompile(vehicle: *mut LiminalVehicle) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    match vehicle.simulator.recompile() {
        Ok(()) => LiminalStatus::Ok,
        Err(_) => LiminalStatus::CompileFailed,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_step(
    vehicle: *mut LiminalVehicle,
    dt: f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    match vehicle.simulator.step(dt) {
        Ok(()) => LiminalStatus::Ok,
        Err(_) => LiminalStatus::CompileFailed,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_write_signal(
    vehicle: *mut LiminalVehicle,
    port: u64,
    value: f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    if vehicle.simulator.payloads_mut().write(port_from_ffi(port), PortPayload::Signal(value)) {
        LiminalStatus::Ok
    } else {
        LiminalStatus::PortNotFound
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_read_signal(
    vehicle: *const LiminalVehicle,
    port: u64,
    out_value: *mut f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_value) = (unsafe { out_value.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    match vehicle.simulator.payloads().read(port_from_ffi(port)) {
        Some(PortPayload::Signal(value)) => {
            *out_value = value;
            LiminalStatus::Ok
        }
        Some(_) => LiminalStatus::WrongPayload,
        None => LiminalStatus::PortNotFound,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_read_omega(
    vehicle: *const LiminalVehicle,
    port: u64,
    out_omega: *mut f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_omega) = (unsafe { out_omega.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    match vehicle.simulator.payloads().read(port_from_ffi(port)) {
        Some(PortPayload::Rotational {
            angular_velocity,
            ..
        }) => {
            *out_omega = angular_velocity;
            LiminalStatus::Ok
        }
        Some(_) => LiminalStatus::WrongPayload,
        None => LiminalStatus::PortNotFound,
    }
}

/// 查询旋转端口折算到该端口的等效转动惯量（kg·m²）。
///
/// 端口不在任何动力岛内或惯量为零时返回 `PortNotFound`。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_effective_inertia(
    vehicle: *const LiminalVehicle,
    port: u64,
    out_inertia: *mut f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_inertia) = (unsafe { out_inertia.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    match vehicle.simulator.port_effective_inertia(port_from_ffi(port)) {
        Some(inertia) => {
            *out_inertia = inertia;
            LiminalStatus::Ok
        }
        None => LiminalStatus::PortNotFound,
    }
}

/// 向旋转端口注入外部扭矩（如轮胎反作用力矩），覆盖式写入，在下一次步进中被消费并清零。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_write_torque(
    vehicle: *mut LiminalVehicle,
    port: u64,
    torque: f32,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_mut() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(slot) = vehicle.simulator.payloads_mut().get_mut(port_from_ffi(port)) else {
        return LiminalStatus::PortNotFound;
    };
    let PortPayload::Rotational {
        torque: slot_torque,
        ..
    } = slot
    else {
        return LiminalStatus::WrongPayload;
    };
    *slot_torque = torque;
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_signal_op_count(
    vehicle: *const LiminalVehicle,
    out_count: *mut usize,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_count) = (unsafe { out_count.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    *out_count = vehicle.simulator.compiled().signal_plan().len();
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_signal_op(
    vehicle: *const LiminalVehicle,
    index: usize,
    out_op: *mut LiminalSignalOp,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_op) = (unsafe { out_op.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    let Some(&(source, target)) = vehicle.simulator.compiled().signal_plan().get(index) else {
        return LiminalStatus::IndexOutOfRange;
    };
    *out_op = LiminalSignalOp {
        source_port: port_to_ffi(source),
        target_port: port_to_ffi(target),
    };
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_spatial_link_count(
    vehicle: *const LiminalVehicle,
    out_count: *mut usize,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_count) = (unsafe { out_count.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    *out_count = vehicle.simulator.compiled().spatial_links().len();
    LiminalStatus::Ok
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_spatial_link(
    vehicle: *const LiminalVehicle,
    index: usize,
    out_link: *mut LiminalSpatialLink,
) -> LiminalStatus {
    let Some(vehicle) = (unsafe { vehicle.as_ref() }) else {
        return LiminalStatus::InvalidHandle;
    };
    let Some(out_link) = (unsafe { out_link.as_mut() }) else {
        return LiminalStatus::InvalidArgument;
    };
    let Some(link) = vehicle.simulator.compiled().spatial_links().get(index) else {
        return LiminalStatus::IndexOutOfRange;
    };
    *out_link = LiminalSpatialLink {
        parent: node_to_ffi(link.parent()),
        child: node_to_ffi(link.child()),
        joint: joint_to_ffi(link.joint()),
        control_port: link.control_port().map(port_to_ffi).unwrap_or(0),
        has_control: u32::from(link.control_port().is_some()),
        rest_pose: frame_to_ffi(link.rest_pose()),
        pose_in_root: frame_to_ffi(link.pose_in_root()),
    };
    LiminalStatus::Ok
}

/// 轮胎冲量解：纵向按本步内对齐轮面与地面速度所需冲量折算，
/// 侧向按侧偏刚度，合力按摩擦椭圆裁剪，并叠加滚动阻力。
///
/// `wheel_torque` 仅含摩擦力矩，滚动阻力不产生车轮反作用。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_tire_impulse_forces(
    model: *const LiminalTireModel,
    angular_velocity: f32,
    ground_speed: f32,
    radius: f32,
    wheel_inertia: f32,
    slip_angle: f32,
    lateral_speed: f32,
    normal_force: f32,
    dt: f32,
) -> LiminalTireForces {
    let Some(model) = (unsafe { model.as_ref() }) else {
        return LiminalTireForces::default();
    };
    TireModel::from(*model)
        .impulse_forces(
            WheelMotion {
                angular_velocity,
                ground_speed,
                radius,
                inertia: wheel_inertia,
            },
            slip_angle,
            lateral_speed,
            normal_force,
            dt,
        )
        .into()
}

/// 约束式轮胎耦合：输出摩擦椭圆四轴界、侧向力与滚动阻力（N）。
///
/// 适配层以动/静摩擦界构造联合椭圆，由宿主求解器在速度迭代内完成目标速度
/// 对齐；侧向力用于遥测与外力回退，滚动阻力作为外部力施加。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_tire_constraint(
    model: *const LiminalTireModel,
    ground_speed: f32,
    slip_angle: f32,
    lateral_speed: f32,
    normal_force: f32,
) -> LiminalTireConstraint {
    let Some(model) = (unsafe { model.as_ref() }) else {
        return LiminalTireConstraint::default();
    };
    TireModel::from(*model)
        .constraint_forces(ground_speed, slip_angle, lateral_speed, normal_force)
        .into()
}

/// 由悬挂压缩量与压缩速度计算法向力。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_suspension_force(
    stiffness: f32,
    damping: f32,
    compression: f32,
    compression_velocity: f32,
) -> f32 {
    Suspension::new(stiffness, damping, 0.0, 0.0, 0.0)
        .normal_force(compression, compression_velocity)
}

/// 防倾杆：由两侧悬挂压缩量计算附加在 A 侧的法向力（N），B 侧取反。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_anti_roll_force(
    stiffness: f32,
    compression_a: f32,
    compression_b: f32,
) -> f32 {
    anti_roll_force(stiffness, compression_a, compression_b)
}

#[cfg(test)]
mod tests {
    use super::{
        LiminalEngineCurvePoint, LiminalEngineParams, LiminalFrame, LiminalJointKind,
        LiminalSpatialLink, LiminalStatus, LiminalTireModel, LiminalVec3, LiminalVehicle,
        liminal_anti_roll_force, liminal_suspension_force, liminal_tire_impulse_forces,
        liminal_vehicle_add_chassis, liminal_vehicle_add_engine,
        liminal_vehicle_add_engine_with_params, liminal_vehicle_add_knuckle,
        liminal_vehicle_add_suspension, liminal_vehicle_add_wheel, liminal_vehicle_connect,
        liminal_vehicle_effective_inertia, liminal_vehicle_free, liminal_vehicle_is_dirty,
        liminal_vehicle_new, liminal_vehicle_port, liminal_vehicle_read_omega,
        liminal_vehicle_recompile, liminal_vehicle_spatial_link,
        liminal_vehicle_spatial_link_count, liminal_vehicle_step, liminal_vehicle_write_signal,
    };
    use std::ptr;

    unsafe fn port(vehicle: *const LiminalVehicle, node: u64, index: usize) -> u64 {
        let mut value = 0u64;
        let status = unsafe { liminal_vehicle_port(vehicle, node, index, &mut value) };
        assert_eq!(status, LiminalStatus::Ok);
        value
    }

    #[test]
    fn drivetrain_runs_through_c_abi() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            assert!(!vehicle.is_null());

            let mut engine = 0u64;
            assert_eq!(
                liminal_vehicle_add_engine(vehicle, 100.0, 100.0, 1.0, &mut engine),
                LiminalStatus::Ok
            );
            let mut wheel = 0u64;
            assert_eq!(
                liminal_vehicle_add_wheel(vehicle, 1.0, 0.0, 0.3, &mut wheel),
                LiminalStatus::Ok
            );

            let throttle = port(vehicle, engine, 0);
            let engine_shaft = port(vehicle, engine, 1);
            let wheel_shaft = port(vehicle, wheel, 0);

            assert_eq!(liminal_vehicle_is_dirty(vehicle), 1);

            assert_eq!(
                liminal_vehicle_connect(vehicle, engine_shaft, wheel_shaft, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(liminal_vehicle_recompile(vehicle), LiminalStatus::Ok);
            assert_eq!(liminal_vehicle_is_dirty(vehicle), 0);
            assert_eq!(liminal_vehicle_write_signal(vehicle, throttle, 1.0), LiminalStatus::Ok);
            assert_eq!(liminal_vehicle_step(vehicle, 0.01), LiminalStatus::Ok);

            let mut omega = 0.0f32;
            assert_eq!(
                liminal_vehicle_read_omega(vehicle, wheel_shaft, &mut omega),
                LiminalStatus::Ok
            );
            assert!(omega > 0.0);

            liminal_vehicle_free(vehicle);
        }
    }

    #[test]
    fn effective_inertia_is_exposed_through_c_abi() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            let mut engine = 0u64;
            let mut wheel = 0u64;
            assert_eq!(
                liminal_vehicle_add_engine(vehicle, 100.0, 100.0, 1.0, &mut engine),
                LiminalStatus::Ok
            );
            assert_eq!(
                liminal_vehicle_add_wheel(vehicle, 1.0, 0.0, 0.3, &mut wheel),
                LiminalStatus::Ok
            );

            let engine_shaft = port(vehicle, engine, 1);
            let wheel_shaft = port(vehicle, wheel, 0);
            assert_eq!(
                liminal_vehicle_connect(vehicle, engine_shaft, wheel_shaft, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(liminal_vehicle_recompile(vehicle), LiminalStatus::Ok);

            let mut inertia = 0.0f32;
            assert_eq!(
                liminal_vehicle_effective_inertia(vehicle, wheel_shaft, &mut inertia),
                LiminalStatus::Ok
            );
            assert!((inertia - 2.0).abs() < 1e-5);

            liminal_vehicle_free(vehicle);
        }
    }

    #[test]
    fn domain_mismatch_maps_to_status() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            let mut engine = 0u64;
            let mut wheel = 0u64;
            assert_eq!(
                liminal_vehicle_add_engine(vehicle, 100.0, 100.0, 1.0, &mut engine),
                LiminalStatus::Ok
            );
            assert_eq!(
                liminal_vehicle_add_wheel(vehicle, 1.0, 0.0, 0.3, &mut wheel),
                LiminalStatus::Ok
            );

            let throttle = port(vehicle, engine, 0);
            let wheel_shaft = port(vehicle, wheel, 0);
            assert_eq!(
                liminal_vehicle_connect(vehicle, throttle, wheel_shaft, ptr::null_mut()),
                LiminalStatus::DomainMismatch
            );
            liminal_vehicle_free(vehicle);
        }
    }

    #[test]
    fn spatial_links_are_exported() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            let mut chassis = 0u64;
            let mut suspension = 0u64;
            let mut knuckle = 0u64;
            let mut wheel = 0u64;
            let mount = LiminalFrame {
                translation: LiminalVec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 1.0,
                },
                rotation: [0.0, 0.0, 0.0, 1.0],
            };
            assert_eq!(
                liminal_vehicle_add_chassis(vehicle, &mount, 1, &mut chassis),
                LiminalStatus::Ok
            );
            assert_eq!(
                liminal_vehicle_add_suspension(vehicle, 30.0, 3.0, 0.2, 0.1, 0.3, &mut suspension),
                LiminalStatus::Ok
            );
            assert_eq!(liminal_vehicle_add_knuckle(vehicle, 50.0, &mut knuckle), LiminalStatus::Ok);
            assert_eq!(
                liminal_vehicle_add_wheel(vehicle, 1.0, 0.0, 0.3, &mut wheel),
                LiminalStatus::Ok
            );

            let chassis_mount = port(vehicle, chassis, 0);
            let suspension_top = port(vehicle, suspension, 0);
            let suspension_bottom = port(vehicle, suspension, 1);
            let knuckle_top = port(vehicle, knuckle, 0);
            let knuckle_bottom = port(vehicle, knuckle, 1);
            let knuckle_steering = port(vehicle, knuckle, 2);
            let wheel_mount = port(vehicle, wheel, 1);

            assert_eq!(
                liminal_vehicle_connect(vehicle, chassis_mount, suspension_top, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(
                liminal_vehicle_connect(vehicle, suspension_bottom, knuckle_top, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(
                liminal_vehicle_connect(vehicle, knuckle_bottom, wheel_mount, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(liminal_vehicle_recompile(vehicle), LiminalStatus::Ok);

            let mut count = 0usize;
            assert_eq!(liminal_vehicle_spatial_link_count(vehicle, &mut count), LiminalStatus::Ok);
            assert_eq!(count, 3);

            let mut link = LiminalSpatialLink::default();
            assert_eq!(liminal_vehicle_spatial_link(vehicle, 1, &mut link), LiminalStatus::Ok);
            assert_eq!(link.joint.kind, LiminalJointKind::Revolute);
            assert_eq!(link.has_control, 1);
            assert_eq!(link.control_port, knuckle_steering);

            // 车轮链接：静止位姿链 = 挂点 [0.5, 0, 1] 下悬自由长度 0.2。
            assert_eq!(liminal_vehicle_spatial_link(vehicle, 2, &mut link), LiminalStatus::Ok);
            assert!((link.pose_in_root.translation.x - 0.5).abs() < 1e-6);
            assert!((link.pose_in_root.translation.y + 0.2).abs() < 1e-6);
            assert!((link.pose_in_root.translation.z - 1.0).abs() < 1e-6);
            assert_eq!(link.pose_in_root.rotation, [0.0, 0.0, 0.0, 1.0]);

            liminal_vehicle_free(vehicle);
        }
    }

    #[test]
    fn null_handle_is_rejected() {
        unsafe {
            assert_eq!(liminal_vehicle_step(ptr::null_mut(), 0.01), LiminalStatus::InvalidHandle);
            assert_eq!(
                liminal_vehicle_read_omega(ptr::null(), 0, ptr::null_mut()),
                LiminalStatus::InvalidHandle
            );
            liminal_vehicle_free(ptr::null_mut());
        }
    }

    #[test]
    fn suspension_and_anti_roll_helpers_match_core_models() {
        let force = liminal_suspension_force(100.0, 10.0, 0.1, 0.0);
        assert!((force - 10.0).abs() < 1e-5);

        let anti_roll = liminal_anti_roll_force(1_000.0, 0.06, 0.04);
        assert!((anti_roll - 20.0).abs() < 1e-5);
    }

    #[test]
    fn impulse_tire_helper_matches_core_model() {
        let model = LiminalTireModel {
            friction_coefficient: 1.0,
            lateral_stiffness: 100.0,
            ..LiminalTireModel::default()
        };
        // J=1, R=0.5, ω=0, v=1, dt=0.01 → 冲量 -4 N·s → 力 -400 N；上限 1000 不裁剪。
        let forces = unsafe {
            liminal_tire_impulse_forces(&model, 0.0, 1.0, 0.5, 1.0, 0.0, 0.0, 1_000.0, 0.01)
        };
        assert!((forces.longitudinal + 400.0).abs() < 1e-3);
        assert!((forces.wheel_torque - 200.0).abs() < 1e-3);
        assert_eq!(forces.lateral, 0.0);
    }

    #[test]
    fn tire_helper_applies_load_sensitivity_and_rolling_resistance() {
        let model = LiminalTireModel {
            friction_coefficient: 1.0,
            lateral_friction_coefficient: 1.0,
            lateral_stiffness: 0.0,
            load_sensitivity: 0.1,
            reference_load: 1_000.0,
            rolling_resistance: 0.02,
            static_friction_scale: 1.0,
        };
        // 自由滚动（ωR = v）：纵向摩擦力为零，仅剩滚动阻力。
        let forces = unsafe {
            liminal_tire_impulse_forces(&model, 10.0, 5.0, 0.5, 1.0, 0.0, 0.0, 1_000.0, 0.01)
        };
        assert!((forces.longitudinal + 20.0).abs() < 1e-3);
        assert_eq!(forces.wheel_torque, 0.0);
    }

    #[test]
    fn null_tire_model_yields_zero_impulse() {
        let impulses = unsafe {
            liminal_tire_impulse_forces(ptr::null(), 10.0, 0.0, 0.5, 1.0, 0.0, 0.0, 500.0, 0.01)
        };
        assert_eq!(impulses.longitudinal, 0.0);
        assert_eq!(impulses.lateral, 0.0);
        assert_eq!(impulses.wheel_torque, 0.0);
    }

    #[test]
    fn engine_params_control_idle_and_curve() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            assert!(!vehicle.is_null());

            let curve = [
                LiminalEngineCurvePoint {
                    speed_ratio: 0.0,
                    torque_scale: 0.5,
                },
                LiminalEngineCurvePoint {
                    speed_ratio: 1.0,
                    torque_scale: 0.0,
                },
            ];
            let params = LiminalEngineParams {
                max_torque: 100.0,
                max_angular_velocity: 600.0,
                inertia: 0.5,
                idle_angular_velocity: 100.0,
                idle_torque_limit: 50.0,
                engine_braking_torque: 30.0,
                curve: curve.as_ptr(),
                curve_count: curve.len(),
            };

            let mut engine = 0u64;
            assert_eq!(
                liminal_vehicle_add_engine_with_params(vehicle, &params, &mut engine),
                LiminalStatus::Ok
            );

            let shaft = port(vehicle, engine, 1);
            assert_eq!(liminal_vehicle_step(vehicle, 0.01), LiminalStatus::Ok);

            let mut omega = 0.0f32;
            assert_eq!(liminal_vehicle_read_omega(vehicle, shaft, &mut omega), LiminalStatus::Ok);
            assert!(omega > 0.0, "怠速控制器应使发动机转动，omega = {omega}");

            liminal_vehicle_free(vehicle);
        }
    }

    #[test]
    fn engine_params_reject_invalid_values() {
        unsafe {
            let vehicle = liminal_vehicle_new();
            assert!(!vehicle.is_null());
            let mut engine = 0u64;

            assert_eq!(
                liminal_vehicle_add_engine_with_params(vehicle, ptr::null(), &mut engine),
                LiminalStatus::InvalidArgument
            );

            let mut params = LiminalEngineParams {
                max_torque: 100.0,
                max_angular_velocity: 600.0,
                ..Default::default()
            };
            params.max_torque = f32::NAN;
            assert_eq!(
                liminal_vehicle_add_engine_with_params(vehicle, &params, &mut engine),
                LiminalStatus::InvalidArgument
            );

            params.max_torque = 100.0;
            params.curve_count = 1;
            assert_eq!(
                liminal_vehicle_add_engine_with_params(vehicle, &params, &mut engine),
                LiminalStatus::InvalidArgument
            );

            // 非有限值与结构错误被拒绝；负值由单元构造钳制。
            params.curve_count = 0;
            params.max_torque = -1.0;
            params.max_angular_velocity = 0.0;
            assert_eq!(
                liminal_vehicle_add_engine_with_params(vehicle, &params, &mut engine),
                LiminalStatus::Ok
            );

            liminal_vehicle_free(vehicle);
        }
    }
}
