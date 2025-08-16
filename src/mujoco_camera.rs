use mujoco_rust::{Simulation, model::ObjType};
// use nalgebra::{Vector3, Quaternion};
use image::{RgbImage, ImageBuffer};
use glfw::Context;

use mujoco_rs_sys::no_render::*;
use mujoco_rs_sys as sys;
use std::ptr;


pub fn init_camera(simulation: &Simulation, height: usize, width: usize) -> (mjvOption_, mjvCamera_, mjvScene_, sys::render::mjrContext_){
    let cam_id = simulation.model.name_to_id(ObjType::CAMERA, "onboard").unwrap();

    unsafe {
        // 初始化渲染上下文
        let mut vopt = mjvOption_::default();
        let mut cam = mjvCamera_::default();
        let mut scene = mjvScene_::default();
        let mut context = sys::render::mjrContext_::default();
        let mut glfw: glfw::Glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
        
        glfw.window_hint(glfw::WindowHint::Visible(false));
        let (mut window, _events) = glfw.create_window(width.try_into().unwrap(), height.try_into().unwrap(), "hidden", glfw::WindowMode::Windowed)
            .expect("Failed to create GLFW window.");
        
        window.make_current();


        mjv_defaultOption(&mut vopt);
        mjv_defaultCamera(&mut cam);

        let maxgeom = 500; // 根据模型复杂度调整
        mjv_makeScene(simulation.model.ptr(), &mut scene, maxgeom);

        sys::mjr_defaultContext(&mut context);

        // 初始化 context
        sys::mjr_makeContext(simulation.model.ptr(), &mut context, 150);

        // 设置相机为你要的那个
        cam.fixedcamid = cam_id as i32;

        // 固定相机，而不是第三方视角
        cam.type_ = 2;

        (vopt, cam, scene, context)
    }
}
        

pub fn get_camera_jpg(simulation: &Simulation, height: usize, width: usize, 
    vopt: &mut mjvOption_,
    cam: &mut mjvCamera_,
    scene: &mut mjvScene_,
    context: &mut sys::render::mjrContext_){
    // let cam_id = simulation.model.name_to_id(ObjType::CAMERA, "track").unwrap();

    unsafe {

        // 更新场景
        mjv_updateScene(
            simulation.model.ptr(),
            simulation.state.ptr(),
            vopt,
            ptr::null(),
            cam,
            0xFFFFFF,
            scene,
        );

        // 分配 buffer
        let mut rgb = vec![0u8; width * height * 3];
        let mut depth = vec![0f32; width * height];

        // 渲染到 buffer
        let viewport = sys::render::mjrRect_ { left: 0, bottom: 0, width: width as i32, height: height as i32 };
        sys::mjr_render(
            viewport,
            scene,
            context,
        );
        sys::mjr_readPixels(
            rgb.as_mut_ptr(),
            depth.as_mut_ptr(),
            viewport,
            context,
        );
        let img: RgbImage = ImageBuffer::from_raw(width as u32, height as u32, rgb).unwrap();
        img.save("camera.jpg").unwrap();
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
