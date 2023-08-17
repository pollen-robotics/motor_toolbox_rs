mod coherency;

mod motor_controller;
pub use motor_controller::{MotorController, Result};

mod multiple_motors_controller;
pub use multiple_motors_controller::{MultipleMotorsController, MultipleMotorsControllerWrapper};
