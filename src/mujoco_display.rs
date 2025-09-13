// ui.rs
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;

use mujoco_rust::model::ObjType;
use image::{RgbImage, ImageBuffer};
use std::process::{Command, Stdio};

pub struct UIState {
    pub cameras: Vec<render::mjvCamera_>,
    pub opt: render::mjvOption_,
    pub scenes: Vec<render::mjvScene_>,
    pub contexts: Vec<render::mjrContext_>, // 改为 Vec 以支持多个上下文
    pub window: glfw::Window,
    // pub events: glfw::Receiver<(f64, glfw::WindowEvent)>,
}

pub fn glfw_init(simulation: &Simulation, cam_ids: &[i32]) -> UIState {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    glfw.default_window_hints();

    let (mut window, events) = if cam_ids[0] == 0x7FFFFFFF {
        glfw.create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
            .expect("无法创建 GLFW 窗口")
    } else {
        glfw.window_hint(glfw::WindowHint::Visible(false));
        glfw.create_window(640, 480, "hidden", glfw::WindowMode::Windowed)
            .expect("无法创建 GLFW 窗口")
    };

    // 设置窗口为当前上下文
    window.make_current();
    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    // 初始化 MuJoCo 结构
    let mut cameras = Vec::new();
    let mut scenes = Vec::new();
    let mut contexts = Vec::new();
    let mut opt = render::mjvOption_::default();

    for &cam_id in cam_ids {
        let mut cam = render::mjvCamera_::default();
        let mut scn = render::mjvScene_::default();
        let mut con = render::mjrContext_::default();

        unsafe {
            no_render::mjv_defaultCamera(&mut cam);
            render::mjv_defaultScene(&mut scn);
            render::mjr_defaultContext(&mut con);

            // 为每个相机初始化场景和上下文
            no_render::mjv_makeScene(simulation.model.ptr(), &mut scn, 2000);
            render::mjr_makeContext(simulation.model.ptr(), &mut con, 200);
        }
        if cam_id == 0x7FFFFFFF {
            cam.type_ = 1; // 自由相机
            cam.trackbodyid = 1; // 设置跟踪的物体 ID
            // cam.fixedcamid = -1; // 跟踪模式不需要 fixedcamid
            cam.distance = 5.0; // 设置初始距离
        } else {
            cam.fixedcamid = cam_id;
            cam.type_ = 2; // 固定相机
        }

        cameras.push(cam);
        scenes.push(scn);
        contexts.push(con);
    }

    unsafe {
        no_render::mjv_defaultOption(&mut opt);
    }

    UIState {
        cameras,
        opt,
        scenes,
        contexts,
        window,
        // events, // 返回 events 以供主循环使用
    }
}

pub fn glfw_update_scene(simulation: &Simulation, ui_state: &mut UIState) {
    ui_state.window.make_current();
    unsafe {
        // 获取窗口大小
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // 清除帧缓冲区
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // 更新并渲染每个场景
        for i in 0..num_cameras {
            // 更新当前相机的场景
            no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );

            // 计算网格中的行和列
            let row = i / cols; // 行索引
            let col = i % cols; // 列索引

            // 定义当前子窗口的视口
            let viewport = render::mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height, // 从底部开始，翻转行顺序
                width: sub_window_width,
                height: sub_window_height,
            };

            // 在视口中渲染场景
            render::mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // 交换缓冲区以显示
        ui_state.window.swap_buffers();
    }
}

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

pub fn update_mjscene(simulation: &Simulation, ui_state: &mut UIState) -> Vec<u8> {
    ui_state.window.make_current();
    unsafe {
        // 获取窗口大小
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // 清除帧缓冲区
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // 更新并渲染每个场景
        for i in 0..num_cameras {
            // 更新当前相机的场景
            no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );

            // 计算网格中的行和列
            let row = i / cols; // 行索引
            let col = i % cols; // 列索引

            // 定义当前子窗口的视口
            let viewport = render::mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height, // 从底部开始，翻转行顺序
                width: sub_window_width,
                height: sub_window_height,
            };

            // 在视口中渲染场景
            render::mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // 分配 RGB 缓冲区
        let mut rgb = vec![0u8; (width * height * 3) as usize];

        // 定义整个窗口的视口
        let full_viewport = render::mjrRect_ {
            left: 0,
            bottom: 0,
            width: width,
            height: height,
        };

        gl::PixelStorei(gl::PACK_ALIGNMENT, 1);

        // 读取整个窗口的像素
        render::mjr_readPixels(rgb.as_mut_ptr(), ptr::null_mut(), full_viewport, &mut ui_state.contexts[0]);

        // 垂直翻转图像
        let mut flipped_rgb = vec![0u8; (width * height * 3) as usize];
        for y in 0..height {
            for x in 0..width {
                let src_idx = ((y * width + x) * 3) as usize;
                let dst_idx = (((height - 1 - y) * width + x) * 3) as usize;
                flipped_rgb[dst_idx] = rgb[src_idx];
                flipped_rgb[dst_idx + 1] = rgb[src_idx + 1];
                flipped_rgb[dst_idx + 2] = rgb[src_idx + 2];
            }
        }
        flipped_rgb
    }
}