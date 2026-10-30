//! C ABI：宿主（Chaos/UE）适配壳与库之间的稳定边界。
//!
//! 设计约束：
//! - 只使用 `#[repr(C)]` 的 POD 与不透明句柄，不跨语言传递 Rust 所有权类型；
//! - 所有函数检查空指针，错误以 [`LiminalStatus`] 返回；
//! - `NodeId`/`PortId`/`EdgeId` 通过 slotmap 的 FFI 键值导出为 `u64`；
//! - 句柄非线程安全，调用方需自行同步。

use std::ptr;
use std::slice;

use slotmap::{Key, KeyData};

use crate::payload::PortPayload;
use crate::simulator::Simulator;
use crate::spatial::{Frame, SpatialJointDesc};
use crate::tire::{TireModel, WheelMotion};
use crate::topology::VehicleTopology;
use crate::types::{ConnectError, EdgeId, NodeId, PortId};
use crate::units::rotational::{Axle, Differential, Engine, Wheel};
use crate::units::signal::{CockpitInput, SignalSplitter};
use crate::units::spatial::{Chassis, SteeringKnuckle, Suspension};
use crate::units::transmission::{Brake, Clutch, Gearbox};

/// 三维向量。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 轮胎坐标系下的合力。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LiminalTireForces {
    pub longitudinal: f32,
    pub lateral: f32,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiminalJointKind {
    Rigid = 0,
    Revolute = 1,
    PrismaticSpring = 2,
}

impl Default for LiminalJointKind {
    fn default() -> Self {
        Self::Rigid
    }
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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn liminal_vehicle_add_differential(
    vehicle: *mut LiminalVehicle,
    output_count: usize,
    coupling: f32,
    max_torque: f32,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    if output_count == 0 {
        return LiminalStatus::InvalidArgument;
    }
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Differential::new(output_count, coupling, max_torque, inertia))
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
    lock_threshold: f32,
    inertia: f32,
    out_node: *mut u64,
) -> LiminalStatus {
    unsafe {
        push_node(vehicle, out_node, |topology| {
            topology.add_node(Clutch::new(max_torque, lock_threshold, inertia))
        })
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

/// 由滑移率、侧偏角与法向载荷计算轮胎力。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_tire_forces(
    friction_coefficient: f32,
    longitudinal_stiffness: f32,
    lateral_stiffness: f32,
    slip_ratio: f32,
    slip_angle: f32,
    normal_force: f32,
) -> LiminalTireForces {
    let model = TireModel {
        friction_coefficient,
        longitudinal_stiffness,
        lateral_stiffness,
    };
    let forces = model.forces(slip_ratio, slip_angle, normal_force);
    LiminalTireForces {
        longitudinal: forces.longitudinal,
        lateral: forces.lateral,
    }
}

/// 冲量模式轮胎解：纵向按本步内对齐轮面与地面速度所需冲量折算，
/// 侧向按侧偏刚度，合力按摩擦圆裁剪。
#[unsafe(no_mangle)]
pub extern "C" fn liminal_tire_impulse_forces(
    friction_coefficient: f32,
    lateral_stiffness: f32,
    angular_velocity: f32,
    ground_speed: f32,
    radius: f32,
    wheel_inertia: f32,
    slip_angle: f32,
    normal_force: f32,
    dt: f32,
) -> LiminalTireForces {
    let model = TireModel {
        friction_coefficient,
        longitudinal_stiffness: 0.0,
        lateral_stiffness,
    };
    let forces = model.impulse_forces(
        WheelMotion {
            angular_velocity,
            ground_speed,
            radius,
            inertia: wheel_inertia,
        },
        slip_angle,
        normal_force,
        dt,
    );
    LiminalTireForces {
        longitudinal: forces.longitudinal,
        lateral: forces.lateral,
    }
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

#[cfg(test)]
mod tests {
    use super::{
        LiminalFrame, LiminalJointKind, LiminalSpatialLink, LiminalStatus, LiminalVec3,
        LiminalVehicle, liminal_suspension_force, liminal_tire_forces, liminal_tire_impulse_forces,
        liminal_vehicle_add_chassis, liminal_vehicle_add_engine, liminal_vehicle_add_knuckle,
        liminal_vehicle_add_suspension, liminal_vehicle_add_wheel, liminal_vehicle_connect,
        liminal_vehicle_free, liminal_vehicle_is_dirty, liminal_vehicle_new, liminal_vehicle_port,
        liminal_vehicle_read_omega, liminal_vehicle_recompile, liminal_vehicle_spatial_link,
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

            assert_eq!(unsafe { liminal_vehicle_is_dirty(vehicle) }, 1);

            assert_eq!(
                liminal_vehicle_connect(vehicle, engine_shaft, wheel_shaft, ptr::null_mut()),
                LiminalStatus::Ok
            );
            assert_eq!(liminal_vehicle_recompile(vehicle), LiminalStatus::Ok);
            assert_eq!(unsafe { liminal_vehicle_is_dirty(vehicle) }, 0);
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
    fn pure_helpers_match_core_models() {
        let forces = liminal_tire_forces(1.0, 100.0, 100.0, 0.01, 0.0, 500.0);
        assert!((forces.longitudinal - 1.0).abs() < 1e-5);
        let force = liminal_suspension_force(100.0, 10.0, 0.1, 0.0);
        assert!((force - 10.0).abs() < 1e-5);
    }

    #[test]
    fn impulse_tire_helper_matches_core_model() {
        // J=1, R=0.5, ω=0, v=1, dt=0.01 → 冲量 -4 N·s → 力 -400 N；上限 1000 不裁剪。
        let forces = liminal_tire_impulse_forces(1.0, 100.0, 0.0, 1.0, 0.5, 1.0, 0.0, 1000.0, 0.01);
        assert!((forces.longitudinal + 400.0).abs() < 1e-3);
        assert_eq!(forces.lateral, 0.0);
    }
}
