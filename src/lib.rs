mod coherency;

mod fake_motor;
pub use fake_motor::FakeMotor;

mod limit;
pub use limit::Limit;

mod motor_controller;
pub use motor_controller::{MotorController, Result};

mod multiple_motors_controller;
pub use multiple_motors_controller::{MultipleMotorsController, MultipleMotorsControllerWrapper};

mod pid;
pub use pid::PID;
