// ui.rs
use mujoco_rs_sys::mjr_render;
use mujoco_rs_sys::render::*;
use mujoco_rs_sys::no_render::{mjv_makeScene};
use glfw;
use glfw::Context;
use mujoco_rust::Simulation;
use std::ptr;


pub fn init_glfw(sim: &Simulation) -> (mjvCamera_, mjvOption_, mjvScene_, mjrContext_, glfw::Window) {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    // 2. 创建窗口，并设置 OpenGL 上下文
    let (mut window, events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("Unable to create GLFW window.");
    
    window.make_current();

    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    // gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    let mut cam = mjvCamera_::default();
    let mut opt = mjvOption_::default();
    let mut scn = mjvScene_::default();
    let mut con = mjrContext_::default();

    unsafe{
        mjv_defaultCamera(&mut cam);
        mjv_defaultOption(&mut opt);
        mjv_defaultScene(&mut scn);
        mjr_defaultContext(&mut con);
        mjv_makeScene(sim.model.ptr(), &mut scn, 2000);
        mjr_makeContext(sim.model.ptr(), &mut con, 200);
    }
    (cam, opt, scn, con, window)
}


pub fn update_Mjscene(
    sim: &Simulation,
    window: &mut glfw::Window,
    cam: &mut mujoco_rs_sys::no_render::mjvCamera_,
    opt: &mut mujoco_rs_sys::no_render::mjvOption_,
    scn: &mut mujoco_rs_sys::no_render::mjvScene_,
    con: &mut mujoco_rs_sys::render::mjrContext_,
) {
    unsafe {
        mujoco_rs_sys::no_render::mjv_updateScene(
            sim.model.ptr(),
            sim.state.ptr(),
            opt,
            ptr::null(),
            cam,
            0xFFFFFF,
            scn,
        );

        let (width, height) = window.get_framebuffer_size();
        let viewport = mujoco_rs_sys::render::mjrRect_ {
            left: 0,
            bottom: 0,
            width,
            height,
        };

        mujoco_rs_sys::mjr_render(viewport, scn, con);
        window.swap_buffers();
    }
}