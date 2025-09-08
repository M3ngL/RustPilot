use mujoco_rust::{Simulation, model::ObjType};
// use nalgebra::{Vector3, Quaternion};
use image::{RgbImage, ImageBuffer};
use glfw::{Context, Window};

use mujoco_rs_sys::render::*;
use mujoco_rs_sys as sys;
use std::ptr;

use std::process::{Command, Stdio};


pub fn init_ffmpeg() -> std::process::Child {
    let rtsp_url = "rtsp://localhost:8554/mystream".to_string();
    let mut ffmpeg = Command::new("ffmpeg")
        .args([
            "-f", "rawvideo",
            "-pixel_format", "rgb24",
            "-video_size", "640x480",
            "-framerate", "30",
            "-i", "pipe:",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            &rtsp_url
        ])
        .stdin(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn() // Spawn the process
        .expect("Failed to start FFmpeg");
    
    ffmpeg
}



pub fn init_glfw(sim: &Simulation) -> (Window, mjvCamera_, mjvOption_, mjvScene_, mjrContext_) {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    let width = 640;
    let height = 480;

    // 创建隐藏窗口
    glfw.window_hint(glfw::WindowHint::Visible(false));
    let (mut window, _events) = glfw
        .create_window(
            width.try_into().unwrap(),
            height.try_into().unwrap(),
            "hidden",
            glfw::WindowMode::Windowed,
        )
        .expect("无法创建 GLFW 窗口");

    // 激活 OpenGL 上下文
    window.make_current();
    glfw.make_context_current(Some(&window));

    // 加载 OpenGL 函数（使用 GLAD 或其他加载器）
    // gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    let mut cam = mjvCamera_::default();
    let mut opt = mjvOption_::default();
    let mut scn = mjvScene_::default();
    let mut con = mjrContext_::default();

    unsafe {
        mjv_defaultCamera(&mut cam);
        mjv_defaultOption(&mut opt);
        // mjv_defaultScene(&mut scn); // 不需要，手动初始化
        mjr_defaultContext(&mut con);
        mjv_makeScene(sim.model.ptr(), &mut scn, 2000); // 正确设置 maxgeom
        mjr_makeContext(sim.model.ptr(), &mut con, 200); // mjFONTSCALE_200
    }

    (window, cam, opt, scn, con)
}

pub fn update_mjscene(
    sim: &Simulation,
    cam: &mut mjvCamera_,
    opt: &mut mjvOption_,
    scn: &mut mjvScene_,
    con: &mut mjrContext_,
) -> Vec<u8> {
    let width = 640;
    let height = 480;
    cam.distance = 10.0;
    unsafe {
        sys::mjv_updateScene(
            sim.model.ptr(),
            sim.state.ptr(),
            opt, // 应为 &opt
            ptr::null(),
            cam,
            0xFFFFFF,
            scn,
        );

        let viewport = sys::render::mjrRect_ {
            left: 0,
            bottom: 0,
            width,
            height,
        };

        sys::mjr_render(viewport, scn, con);

        // 分配 RGB 缓冲区
        let mut rgb = vec![0u8; 640 * 480 * 3];
        let mut depth = vec![0f32; 640 * 480];

        // 读取像素
        sys::mjr_readPixels(rgb.as_mut_ptr(), depth.as_mut_ptr(), viewport, con);

        // 垂直翻转图像
        let mut flipped_rgb = vec![0u8; (width * height * 3) as usize] ;
        for y in 0..height {
            for x in 0..width {
                let src_idx :usize = ((y * width + x) * 3) as usize;
                let dst_idx :usize = (((height - 1 - y) * width + x) * 3) as usize;
                flipped_rgb[dst_idx] = rgb[src_idx];
                flipped_rgb[dst_idx + 1] = rgb[src_idx + 1];
                flipped_rgb[dst_idx + 2] = rgb[src_idx + 2];
            }
        }
        flipped_rgb
    }
}

// 释放 scene/context
pub fn free_camera(scene: &mut mjvScene_,
    context: &mut sys::render::mjrContext_){
        unsafe{
            mjv_freeScene(scene);
            sys::mjr_freeContext(context);
        }
}


// fn get_camera_world_pose(sim: &Simulation, cam_name: &str) -> Option<(Vector3<f64>, Quaternion<f64>)> {
//     let model = &sim.model;
//     let cam_id = model.name_to_id(ObjType::CAMERA, cam_name)? as usize;
//     let mj_model = unsafe { *model.ptr() };

//     // 相机在绑定body坐标系下的位置
//     let pos = unsafe {
//         let ptr = mj_model.cam_pos.add(cam_id * 3);
//         Vector3::new(*ptr, *ptr.add(1), *ptr.add(2))
//     };
//     // 相机在绑定body坐标系下的四元数
//     let quat = unsafe {
//         let ptr = mj_model.cam_quat.add(cam_id * 4);
//         Quaternion::new(*ptr, *ptr.add(1), *ptr.add(2), *ptr.add(3))
//     };
//     // 相机绑定的body id
//     let body_id = unsafe { *mj_model.cam_bodyid.add(cam_id) } as usize;

//     // 获取body的世界坐标和朝向
//     let body_pos = sim.model.bodies()[body_id].pos;
//     let body_quat = sim.model.bodies()[body_id].quat;

//     // 相机的世界坐标 = body世界坐标 + body朝向旋转后的相机局部坐标
//     let world_pos = body_pos + body_quat * pos;
//     let world_quat = body_quat * quat;

//     Some((world_pos, world_quat))
// }
