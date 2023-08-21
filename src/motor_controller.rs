use crate::PID;

/// Result generic wrapper using `std::error::Error` trait
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Low level motor controller interface
pub trait MotorController {
    /// Name of the controller (used for Debug trait)
    fn name(&self) -> String;

    /// Get motor offset (in radians)
    fn get_offset(&mut self) -> f64;
    /// Get motor reduction ratio
    fn get_reduction_ratio(&mut self) -> f64;

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
    fn get_current_position(&mut self) -> Result<f64> {
        Ok(self.get_raw_position()? / self.get_reduction_ratio() - self.get_offset())
    }
    /// Get the motor raw position (in radians)
    /// ie. without offset and reduction ratio
    fn get_raw_position(&mut self) -> Result<f64>;

    /// Get the current velocity of the motor (in radians per second)
    fn get_current_velocity(&mut self) -> Result<f64> {
        Ok(self.get_raw_velocity()? / self.get_reduction_ratio())
    }
    /// Get the motor raw velocity (in radians per second)
    /// ie. without reduction ratio
    fn get_raw_velocity(&mut self) -> Result<f64>;

    /// Get the current torque of the motor (in Nm)
    fn get_current_torque(&mut self) -> Result<f64> {
        Ok(self.get_raw_torque()? / self.get_reduction_ratio())
    }
    /// Get the motor raw torque (in Nm)
    /// ie. without reduction ratio
    fn get_raw_torque(&mut self) -> Result<f64>;

    /// Get the current target position of the motor (in radians)
    fn get_target_position(&mut self) -> Result<f64> {
        Ok(self.get_raw_target_position()? / self.get_reduction_ratio() - self.get_offset())
    }
    /// Get raw target position of the motor (in radians)
    /// ie. without offset and reduction ratio
    fn get_raw_target_position(&mut self) -> Result<f64>;

    /// Set the current target position of the motor (in radians)
    fn set_target_position(&mut self, position: f64) -> Result<()> {
        let raw_target_position = position * self.get_reduction_ratio() + self.get_offset();
        self.set_raw_target_position(raw_target_position)
    }
    /// Set raw target position of the motor (in radians)
    /// ie. without offset and reduction ratio
    fn set_raw_target_position(&mut self, position: f64) -> Result<()>;

    /// Get the velocity limit of the motor (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<f64> {
        Ok(self.get_raw_velocity_limit()? / self.get_reduction_ratio())
    }
    /// Get raw velocity limit of the motor (in radians per second)
    /// ie. without reduction ratio
    fn get_raw_velocity_limit(&mut self) -> Result<f64>;
    /// Set the velocity limit of the motor (in radians per second)
    fn set_velocity_limit(&mut self, velocity: f64) -> Result<()> {
        let raw_velocity_limit = velocity * self.get_reduction_ratio();
        self.set_raw_velocity_limit(raw_velocity_limit)
    }
    /// Set raw velocity limit of the motor (in radians per second)
    /// ie. without reduction ratio
    fn set_raw_velocity_limit(&mut self, velocity: f64) -> Result<()>;

    /// Get the torque limit of the motor (in Nm)
    fn get_torque_limit(&mut self) -> Result<f64> {
        Ok(self.get_raw_torque_limit()? / self.get_reduction_ratio())
    }
    /// Get raw torque limit of the motor (in Nm)
    /// ie. without reduction ratio
    fn get_raw_torque_limit(&mut self) -> Result<f64>;

    /// Set the torque limit of the motor (in Nm)
    fn set_torque_limit(&mut self, torque: f64) -> Result<()> {
        let raw_torque_limit = torque * self.get_reduction_ratio();
        self.set_raw_torque_limit(raw_torque_limit)
    }
    /// Set raw torque limit of the motor (in Nm)
    /// ie. without reduction ratio
    fn set_raw_torque_limit(&mut self, torque: f64) -> Result<()>;

    /// Get the current PID gains of the motor
    fn get_pid_gains(&mut self) -> Result<PID>;
    /// Set the current PID gains of the motor
    fn set_pid_gains(&mut self, pid: PID) -> Result<()>;
}

#[derive(Debug)]
pub struct MissingResisterErrror(pub String);
impl std::fmt::Display for MissingResisterErrror {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = &self.0;
        write!(f, "(missing register \"{name}\")",)
    }
}
impl std::error::Error for MissingResisterErrror {}

impl std::fmt::Debug for dyn MotorController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotorController")
            .field("name", &self.name())
            .finish()
    }
}
