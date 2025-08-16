use crate::msg_define::*;
use crate::basic::rotation::Rotation;
use core::slice;
use rpos::channel::Sender;
use rpos::ctor::ctor;
use rpos::hrt::Timespec;
use rpos::module::Module;
use rpos::msg::{get_new_tx_of_message, get_new_rx_of_message};
use std::sync::Arc;
// use std::{cell::RefCell, sync::Arc, time::Duration};

use std::sync::mpsc;
use std::thread;
use std::time::Duration;


use mujoco_rs_sys::no_render::*;
// use crate::mujoco_camera;


unsafe impl Send for MujocoSim {}
unsafe impl Sync for MujocoSim {}

enum SimEvent {
    MixerUpdate(Vec<f64>),
    TimerTick,
    Quit,
}

struct MujocoSim {
    model: mujoco_rust::Model,
    simulation: mujoco_rust::Simulation,
    gyro_tx: Sender<Vector3>,
    acc_tx: Sender<Vector3>,
    attitude_tx: Sender<Vector4>,
}

impl MujocoSim{
    pub fn update_mj_sensor(&self) {
        // Update the Mujoco sensor data
        let mj_model = unsafe { *self.model.ptr() };
        // 0: gyro
        let gyro_start = unsafe { *mj_model.sensor_adr.add(0) } as usize;
        let gyro_dim = unsafe { *mj_model.sensor_dim.add(0) } as usize;
        let gyro_data = &self.simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

        // 1: accelerometer
        let acc_start = unsafe { *mj_model.sensor_adr.add(1) } as usize;
        let acc_dim = unsafe { *mj_model.sensor_dim.add(1) } as usize;
        let acc_data = &self.simulation.sensordata()[acc_start..acc_start + acc_dim];

        // 2: framequat (attitude quaternion)
        let att_start = unsafe { *mj_model.sensor_adr.add(2) } as usize;
        let att_dim = unsafe { *mj_model.sensor_dim.add(2) } as usize;
        let att_data = &self.simulation.sensordata()[att_start..att_start + att_dim];


        let rotation = Rotation::Yaw270;
        self.gyro_tx.send(rotation.rotate_v(Vector3 {
            x: gyro_data[0] as f32,
            y: gyro_data[1] as f32,
            z: gyro_data[2] as f32,
        }));
        self.acc_tx.send(rotation.rotate_v(Vector3 {
            x: acc_data[0] as f32,
            y: acc_data[1] as f32,
            z: acc_data[2] as f32,
        }));
        let imu_q: quaternion_core::Quaternion<f32> = (
            att_data[0] as f32,
            [
                att_data[1] as f32,
                att_data[2] as f32,
                att_data[3] as f32,
            ],
        );
        let imu_q = rotation.rotate_q(imu_q);
        /*
        imu_q is the rotate quaternion from gazebo axis to body axis 
        rotate_q is the rotate quaternion from gazebo axis to world axis(our defination): x -> -y_old ,  y -> x_old. 
        */
        self.attitude_tx.send(Vector4 {
            w: imu_q.0,
            x: imu_q.1[0],
            y: imu_q.1[1],
            z: imu_q.1[2],
        });

    }
    pub fn detect_movement_and_heading(sim: &Arc<MujocoSim>, last_qpos: Vec<f64>) -> (bool, bool) {
        let qpos = sim.simulation.qpos();
        // 假设 qpos[0], qpos[1], qpos[2] 是 x, y, z 位置
        // qpos[3], qpos[4], qpos[5], qpos[6] 是四元数（朝向）
        let dx = qpos[0] - last_qpos[0];
        let dy = qpos[1] - last_qpos[1];
        let dz = qpos[2] - last_qpos[2]; // 加上 Z 轴
        let pos_change = (dx * dx + dy * dy + dz * dz).sqrt();
        let pos_changed = pos_change > 1e-4;
        let current_quat = &qpos[3..7];
        let last_quat = &last_qpos[3..7];

        // 计算两个四元数的点积
        let dot_product: f64 = current_quat.iter().zip(last_quat.iter()).map(|(a, b)| a * b).sum();

        // 如果点积的绝对值非常接近 1.0，则认为朝向没有变化
        let heading_changed = (1.0 - dot_product.abs()) > 1e-4;
        (pos_changed, heading_changed)
    }

    pub fn mujoco_sim_event_loop(sim: Arc<MujocoSim>, actuator_num: usize) {
      // 获取mixer_output的Receiver
        let mut mixer_rx = get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();
        
        let mut ctrl = vec![0.0; actuator_num as usize];
        let mut i = 0;

        // let (mut vopt, mut cam, mut scene, mut context) = mujoco_camera::init_camera(&sim.simulation, 200, 400);
        
        loop {
            i += 1;
            // 1. 应用最新的mixer_output到ctrl
            if let Some(mixer) = mixer_rx.try_read() {
                for (i, val) in mixer.output.iter().enumerate() {
                    if i < ctrl.len() {
                        ctrl[i] = *val as f64;
                    }
                }
            }
            // println!("mixer_output: {:?}", ctrl);
            let ctrl_f64: Vec<f64> = ctrl.iter().map(|&x| x as f64).collect();

            println!("ctrl:{:?}", ctrl_f64);
            // 2. 推进仿真一步
            sim.simulation.control(&ctrl_f64);
            sim.simulation.step();

            let pos = sim.simulation.qpos(); 
            // detect_movement_and_heading(sim, pos);
            println!("Position Changed?: {:?}", MujocoSim::detect_movement_and_heading(&sim, pos));
            sim.update_mj_sensor();
            
            // mujoco_camera::get_camera_jpg(&sim.simulation, 200, 400, &mut vopt, &mut cam, &mut scene, &mut context);

            // 控制时间步长
            std::thread::sleep(std::time::Duration::from_millis(10));
            
        }
    }

    fn new(xml_filename: &str) -> Arc<Self> {
        let model = mujoco_rust::Model::from_xml(xml_filename).unwrap();
        // let mj_model = unsafe { *model.ptr() };
        let simulation = mujoco_rust::Simulation::new(model.clone());

        let sim = Arc::new_cyclic(|_| {
            let a = MujocoSim {
                model: model.clone(),
                simulation: simulation,
                gyro_tx: get_new_tx_of_message("gyro").unwrap(),
                acc_tx: get_new_tx_of_message("acc").unwrap(),
                attitude_tx: get_new_tx_of_message("attitude").unwrap(),
            };
            a
        });
        sim
    }
    
}

pub fn init_mujoco_sim(_argc: u32, _argv: *const &str){
    assert!(_argc == 2);
    let argv = unsafe { slice::from_raw_parts(_argv, _argc as usize) };
    let sim = MujocoSim::new(argv[1]);
    let actuator_num = unsafe { (*sim.simulation.model.ptr()).nu as usize};

    std::thread::spawn(move || {
        MujocoSim::mujoco_sim_event_loop(sim, actuator_num);
    });
    
    println!("MujocoSim inited!");
}


#[ctor]
fn register() {
    Module::register("mujoco_sim", init_mujoco_sim);
}