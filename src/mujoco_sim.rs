use crate::msg_define::*;
use crate::basic::rotation::Rotation;
use core::slice;
use rpos::channel::Sender;
use rpos::ctor::ctor;
use rpos::hrt::Timespec;
use rpos::module::Module;
use rpos::msg::get_new_tx_of_message;
use std::sync::Arc;
// use std::{cell::RefCell, sync::Arc, time::Duration};

use mujoco_rs_sys::no_render::*;

unsafe impl Send for MujocoSim {}
unsafe impl Sync for MujocoSim {}


struct MujocoSim {
    mj_model: mujoco_rs_sys::mjModel_,
    simulation: mujoco_rust::Simulation,
    gyro_tx: Sender<Vector3>,
    acc_tx: Sender<Vector3>,
    attitude_tx: Sender<Vector4>,
}

impl MujocoSim{
    pub fn run(self: Arc<Self>, mut ctrl: Vec<f64>) {
        // 获取mixer_output的Receiver
        let mut mixer_rx = rpos::msg::get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();
        loop {
            // 1. 应用最新的mixer_output到ctrl
            if let Some(mixer) = mixer_rx.try_read() {
                for (i, val) in mixer.output.iter().enumerate() {
                    if i < ctrl.len() {
                        ctrl[i] = *val as f64;
                    }
                }
            }
            let ctrl_f64: Vec<f64> = ctrl.iter().map(|&x| x as f64).collect();

            // 2. 推进仿真一步
            self.simulation.control(&ctrl_f64);
            self.simulation.step();

            // 0: gyro
            let gyro_start = unsafe { *self.mj_model.sensor_adr.add(0) } as usize;
            let gyro_dim = unsafe { *self.mj_model.sensor_dim.add(0) } as usize;
            let gyro_data = &self.simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

            // 1: accelerometer
            let acc_start = unsafe { *self.mj_model.sensor_adr.add(1) } as usize;
            let acc_dim = unsafe { *self.mj_model.sensor_dim.add(1) } as usize;
            let acc_data = &self.simulation.sensordata()[acc_start..acc_start + acc_dim];

            // 2: framequat (attitude quaternion)
            let att_start = unsafe { *self.mj_model.sensor_adr.add(2) } as usize;
            let att_dim = unsafe { *self.mj_model.sensor_dim.add(2) } as usize;
            let att_data = &self.simulation.sensordata()[att_start..att_start + att_dim];

            // 例如：上传gyro/acc/attitude
            self.gyro_tx.send(Vector3 { x: gyro_data[0] as f32, y: gyro_data[1] as f32, z: gyro_data[2] as f32});
            self.acc_tx.send(Vector3 { x: acc_data[0] as f32, y: acc_data[1] as f32, z: acc_data[2] as f32});
            self.attitude_tx.send(Vector4 { w: att_data[0] as f32, x: att_data[1] as f32, y: att_data[2] as f32, z: att_data[3] as f32});

            // 4. 控制仿真步长（如需要可sleep或同步外部时钟）
        }
    
    }

    fn new(xml_filename: &str) -> Arc<Self> {
        let model = mujoco_rust::Model::from_xml(xml_filename).unwrap();
        let mj_model = unsafe { *model.ptr() };
        let simulation = mujoco_rust::Simulation::new(model.clone());

        let sim = Arc::new_cyclic(|_| {
            let a = MujocoSim {
                mj_model: mj_model.clone(),
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
    let actuator_num = unsafe { (*sim.simulation.model.ptr()).nu };

    let ctrl = vec![0.0; actuator_num as usize]; 
    std::thread::spawn(move || {
        sim.run(ctrl);
    });
    println!("MujocoSim inited!");
}


#[ctor]
fn register() {
    Module::register("mujoco_sim", init_mujoco_sim);
}