use crate::{coherency::CoherentResult, motor_controller::Result, MotorController};

pub trait MultipleMotorsController<const N: usize> {
    /// Check if the motor is ON or OFF
    fn is_torque_on(&self) -> Result<bool>;
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: bool) -> Result<()>;
    /// Enable the torque
    ///
    /// # Arguments
    /// * `reset_target` - If true, reset the target position to the current position
    fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        self.set_torque(true)?;
        if reset_target {
            let position = self.get_current_position()?;
            self.set_target_position(position)?;
        }
        Ok(())
    }
    /// Disable the torque
    fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }

    /// Get the current position of the motor (in radians)
    fn get_current_position(&mut self) -> Result<[f64; N]>;
    /// Get the current velocity of the motor (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f64; N]>;
    /// Get the current torque of the motor (in Nm)
    fn get_current_torque(&mut self) -> Result<[f64; N]>;

    /// Get the current target position of the motor (in radians)
    fn get_target_position(&mut self) -> Result<[f64; N]>;
    /// Set the current target position of the motor (in radians)
    fn set_target_position(&mut self, position: [f64; N]) -> Result<()>;

    /// Get the velocity limit of the motor (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f64; N]>;
    /// Set the velocity limit of the motor (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()>;

    /// Get the torque limit of the motor (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; N]>;
    /// Set the torque limit of the motor (in Nm)
    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()>;

    /// Get the current PID gains of the motor
    fn get_pid_gains(&mut self) -> Result<[(f64, f64, f64); N]>;
    /// Set the current PID gains of the motor
    fn set_pid_gains(&mut self, pid_gains: [(f64, f64, f64); N]) -> Result<()>;
}

pub struct MultipleMotorsControllerWrapper<const N: usize> {
    controllers: [Box<dyn MotorController>; N],
}

impl<const N: usize> MultipleMotorsControllerWrapper<N> {
    pub fn new(controllers: [Box<dyn MotorController>; N]) -> Self {
        Self { controllers }
    }
}

impl<const N: usize> MultipleMotorsController<N> for MultipleMotorsControllerWrapper<N> {
    fn is_torque_on(&self) -> Result<bool> {
        self.controllers.iter().map(|c| c.is_torque_on()).coherent()
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        for c in self.controllers.iter_mut() {
            c.set_torque(on)?;
        }
        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; N]> {
        let mut pos = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_current_position() {
                Ok(p) => pos.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pos.try_into().unwrap())
    }

    fn get_current_velocity(&mut self) -> Result<[f64; N]> {
        let mut vel = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_current_velocity() {
                Ok(v) => vel.push(v),
                Err(e) => return Err(e),
            }
        }

        Ok(vel.try_into().unwrap())
    }

    fn get_current_torque(&mut self) -> Result<[f64; N]> {
        let mut torque = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_current_torque() {
                Ok(t) => torque.push(t),
                Err(e) => return Err(e),
            }
        }

        Ok(torque.try_into().unwrap())
    }

    fn get_target_position(&mut self) -> Result<[f64; N]> {
        let mut pos = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_target_position() {
                Ok(p) => pos.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pos.try_into().unwrap())
    }

    fn set_target_position(&mut self, position: [f64; N]) -> Result<()> {
        for (c, p) in self.controllers.iter_mut().zip(position.iter()) {
            c.set_target_position(*p)?;
        }
        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        let mut vel = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_velocity_limit() {
                Ok(v) => vel.push(v),
                Err(e) => return Err(e),
            }
        }

        Ok(vel.try_into().unwrap())
    }

    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        for (c, v) in self.controllers.iter_mut().zip(velocity.iter()) {
            c.set_velocity_limit(*v)?;
        }
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        let mut torque = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_torque_limit() {
                Ok(t) => torque.push(t),
                Err(e) => return Err(e),
            }
        }

        Ok(torque.try_into().unwrap())
    }

    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        for (c, t) in self.controllers.iter_mut().zip(torque.iter()) {
            c.set_torque_limit(*t)?;
        }
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[(f64, f64, f64); N]> {
        let mut pid_gains = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_pid_gains() {
                Ok(p) => pid_gains.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pid_gains.try_into().unwrap())
    }

    fn set_pid_gains(&mut self, pid_gains: [(f64, f64, f64); N]) -> Result<()> {
        for (c, p) in self.controllers.iter_mut().zip(pid_gains.iter()) {
            c.set_pid_gains(*p)?;
        }
        Ok(())
    }
}
