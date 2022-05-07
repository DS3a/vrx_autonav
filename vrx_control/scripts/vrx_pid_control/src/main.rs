#[macro_use]
extern crate rosrust;
use std::sync::Mutex;
use std::thread;
use std::sync::Arc;

const RATE: f64 = 100.0; // 100 pings per second
const SETPOINT_DEGRADATION: f64 = 0.0057;
const PROPELLER_SEPARATION: f64 = 1.2; // in meters

rosrust::rosmsg_include!(std_msgs/Float64, std_msgs/Float32, geometry_msgs/Twist, nav_msgs/Odometry);

fn main() {
    let zero: f64 = 0 as f64;

    let req_left_vel = Arc::new(Mutex::new(0 as f64));
    let req_right_vel = Arc::new(Mutex::new(0 as f64));

    let curr_left_vel = Arc::new(Mutex::new(0 as f64));
    let curr_right_vel = Arc::new(Mutex::new(0 as f64));

    rosrust::init("wamv_controller");

    let rate = rosrust::rate(RATE);

    let left_thruster_publisher = rosrust::publish::<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 100).unwrap();
    let right_thruster_publisher = rosrust::publish::<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 100).unwrap();
    let left_state_publisher = rosrust::publish::<std_msgs::Float64>("/left_thruster_controller/state", 100).unwrap();
    let right_state_publisher = rosrust::publish::<std_msgs::Float64>("/right_thruster_controller/state", 100).unwrap();
    let left_setpoint_publisher = rosrust::publish::<std_msgs::Float64>("/left_thruster_controller/setpoint", 100).unwrap();
    let right_setpoint_publisher = rosrust::publish::<std_msgs::Float64>("/right_thruster_controller/setpoint", 100).unwrap();

    let rlv_counter = Arc::clone(&req_left_vel);
    let rrv_counter = Arc::clone(&req_right_vel);
    let cmd_vel_subscriber = rosrust::subscribe("/cmd_vel", 100, move |msg: geometry_msgs::Twist| {
        let req_linear_vel: f64 = msg.linear.x as f64;
        let req_angular_vel: f64 = msg.angular.z as f64;

        let mut l = rlv_counter.lock().unwrap();
        let mut r = rrv_counter.lock().unwrap();
        *r = req_linear_vel;
        *l = req_linear_vel;

        *r += PROPELLER_SEPARATION * 0.5 * req_angular_vel;
        *l -= PROPELLER_SEPARATION * 0.5 * req_angular_vel;
    }).unwrap();

    let odom_subscriber = rosrust::subscribe("/odometry/filtered/local", 100, move |msg: nav_msgs::Odometry| {
        let linear_vel: f64 = msg.twist.twist.linear.x as f64;
        let angular_vel: f64 = msg.twist.twist.angular.z as f64;
        let clv_counter = Arc::clone(&curr_left_vel);
        let crv_counter = Arc::clone(&curr_right_vel);

        let mut l = clv_counter.lock().unwrap();
        let mut r = crv_counter.lock().unwrap();

        *r = linear_vel;
        *l = linear_vel;

        *r += PROPELLER_SEPARATION * 0.5 * angular_vel;
        *l -= PROPELLER_SEPARATION * 0.5 * angular_vel;

        let mut right_msg = std_msgs::Float64::default();
        right_msg.data = *r;
        right_state_publisher.send(right_msg).unwrap();

        let mut left_msg = std_msgs::Float64::default();
        left_msg.data = *l;
        left_state_publisher.send(left_msg).unwrap();
    }).unwrap();


    let left_control_effort_subscriber = rosrust::subscribe("/left_thruster_controller/control_effort", 100, move |msg: std_msgs::Float64| {
        let mut msg2 = std_msgs::Float32::default();
        msg2.data = msg.data as f32;
        left_thruster_publisher.send(msg2).unwrap();
    }).unwrap();

    let right_control_effort_subscriber = rosrust::subscribe("/right_thruster_controller/control_effort", 100, move |msg: std_msgs::Float64| {
        let mut msg2 = std_msgs::Float32::default();
        msg2.data = msg.data as f32;
        right_thruster_publisher.send(msg2).unwrap();
    }).unwrap();

    let rlv_counter = Arc::clone(&req_left_vel);
    let rrv_counter = Arc::clone(&req_right_vel);
    let setpoints_handler = thread::spawn(move || {
        loop {
    
            let mut l = rlv_counter.lock().unwrap();
            let mut r = rrv_counter.lock().unwrap();
    
            let mut right_msg = std_msgs::Float64::default();
            right_msg.data = *r;
            right_setpoint_publisher.send(right_msg).unwrap();
    
            let mut left_msg = std_msgs::Float64::default();
            left_msg.data = *l;
            left_setpoint_publisher.send(left_msg).unwrap();
            let rr = (*r).clone();
            let ll = (*l).clone();

            *r -= rr*SETPOINT_DEGRADATION;
            *l -= ll*SETPOINT_DEGRADATION;

            std::mem::drop(r);
            std::mem::drop(l);
            rate.sleep();    
        }
    });
    rosrust::spin();
}