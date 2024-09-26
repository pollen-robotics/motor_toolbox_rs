// #![feature(generic_const_exprs)]
#![allow(incomplete_features)]

mod fake_motor;
pub use fake_motor::FakeMotorsController;

mod limit;
pub use limit::Limit;

mod motors_io;
pub use motors_io::RawMotorsIO;
mod motors_controller;
pub use motors_controller::{MissingResisterErrror, MotorsController};

mod pid;
pub use pid::PID;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
