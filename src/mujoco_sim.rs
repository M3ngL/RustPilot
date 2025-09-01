use crate::msg_define::*;
use crate::basic::rotation::Rotation;
use core::slice;
use rpos::channel::Sender;
use rpos::ctor::ctor;
use rpos::module::Module;
use rpos::msg::{get_new_tx_of_message, get_new_rx_of_message};
use std::sync::Arc;

use std::sync::mpsc;
use std::thread;
use std::ptr;

use mujoco_rust::Simulation;
use mujoco_rs_sys::render::*;
use std::io::Write;
use crate::mujoco_ui;
use crate::mujoco_vedio_streaming;

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

        self.attitude_tx.send(Vector4 {
            w: imu_q.0,
            x: imu_q.1[0],
            y: imu_q.1[1],
            z: imu_q.1[2],
        });

    }

    pub fn mujoco_sim_event_loop(sim: Arc<MujocoSim>, actuator_num: usize) {
        let mut mixer_rx = get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();
        
        let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize];
        let mut i = 0;

        // ui
        // let (mut cam, mut opt, mut scn, mut con, mut window) = mujoco_ui::init_glfw(&sim.simulation);


        // vedio streaming
        let mut ffmpeg = mujoco_vedio_streaming::init_ffmpeg();
        let mut stdin = ffmpeg.stdin.take().unwrap(); // Take the stdin handle
        let (mut window, mut VS_cam, mut VS_vopt, mut VS_scene, mut VS_context) = mujoco_vedio_streaming::init_glfw(&sim.simulation);

        // distance between view and model
        VS_cam.distance = 10.0;

        sim.simulation.control(&ctrl);
        loop {
            i += 1;
            if let Some(mixer) = mixer_rx.try_read() {
                for (i, val) in mixer.output.iter().enumerate() {
                    if i < ctrl.len() {
                        ctrl[i] = *val as f64;
                    }
                }
            }
            let ctrl_f64: Vec<f64> = ctrl.iter().map(|&x| x as f64).collect();

            sim.simulation.control(&ctrl);

            sim.simulation.step();

            let pos = sim.simulation.qpos(); 
            sim.update_mj_sensor();

            let frame = mujoco_vedio_streaming::update_mjscene(&sim.simulation, &mut VS_cam, &mut VS_vopt, &mut VS_scene, &mut VS_context);
            stdin.write_all(&frame);

            // ui界面
            // mujoco_ui::update_Mjscene(&sim.simulation, &mut window, &mut cam, &mut opt, &mut scn, &mut con);

            // time step
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        unsafe {
            mjv_freeScene(&mut VS_scene);
            mjr_freeContext(&mut VS_context);
        }

        // 关闭 stdin
        drop(stdin);

        // 等待 FFmpeg 进程结束
        match ffmpeg.wait_with_output() {
            Ok(output) => {
                if !output.status.success() {
                    eprintln!("FFmpeg 错误: {}", String::from_utf8_lossy(&output.stderr));
                    return;
                }
            }
            Err(e) => {
                eprintln!("错误: 等待 FFmpeg 失败: {}", e);
                return;
            }
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
    
    println!("MujocoSim inited!");

    std::thread::spawn(move || {
        MujocoSim::mujoco_sim_event_loop(sim, actuator_num);
    });
    
}


#[ctor]
fn register() {
    Module::register("mujoco_sim", init_mujoco_sim);
}