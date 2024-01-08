
use crate::{Result, PID};

pub trait RawMotorsIO<const N: usize> {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; N]>;
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; N]) -> Result<()>;

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f64; N]>;
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f64; N]>;
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f64; N]>;

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f64; N]>;
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f64; N]) -> Result<()>;

    /// Set the current target position and returns the motor feeback (position, velocity, torque)
    fn set_target_position_fb(&mut self, position: [f64; N]) -> Result<[f64; N*3]>;

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f64; N]>;
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()>;

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; N]>;
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()>;

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[PID; N]>;
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()>;
}
