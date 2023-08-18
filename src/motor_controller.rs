use crate::PID;

/// Result generic wrapper using `std::error::Error` trait
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Low level motor controller interface
pub trait MotorController {
    /// Name of the controller (used for Debug trait)
    fn name(&self) -> String;

    /// Check if the motor is ON or OFF
    fn is_torque_on(&mut self) -> Result<bool>;
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: bool) -> Result<()>;
    /// Enable the torque
    ///
    /// # Arguments
    /// * `reset_target` - If true, reset the target position to the current position
    fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        if reset_target {
            let position = self.get_current_position()?;
            self.set_target_position(position)?;
        }

        self.set_torque(true)?;

        Ok(())
    }
    /// Disable the torque
    fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }

    /// Get the current position of the motor (in radians)
    fn get_current_position(&mut self) -> Result<f64>;
    /// Get the current velocity of the motor (in radians per second)
    fn get_current_velocity(&mut self) -> Result<f64>;
    /// Get the current torque of the motor (in Nm)
    fn get_current_torque(&mut self) -> Result<f64>;

    /// Get the current target position of the motor (in radians)
    fn get_target_position(&mut self) -> Result<f64>;
    /// Set the current target position of the motor (in radians)
    fn set_target_position(&mut self, position: f64) -> Result<()>;

    /// Get the velocity limit of the motor (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<f64>;
    /// Set the velocity limit of the motor (in radians per second)
    fn set_velocity_limit(&mut self, velocity: f64) -> Result<()>;

    /// Get the torque limit of the motor (in Nm)
    fn get_torque_limit(&mut self) -> Result<f64>;
    /// Set the torque limit of the motor (in Nm)
    fn set_torque_limit(&mut self, torque: f64) -> Result<()>;

    /// Get the current PID gains of the motor
    fn get_pid_gains(&mut self) -> Result<PID>;
    /// Set the current PID gains of the motor
    fn set_pid_gains(&mut self, pid: PID) -> Result<()>;
}

impl std::fmt::Debug for dyn MotorController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotorController")
            .field("name", &self.name())
            .finish()
    }
}
