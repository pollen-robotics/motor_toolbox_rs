use crate::coherency::IncoherentError;
use crate::{motor_controller::Result, MotorController, PID};

pub trait MultipleMotorsController<const N: usize> {
    /// Name of the controller (used for Debug trait)
    fn name(&self) -> String;

    /// Get motor offset (in radians)
    fn get_offset(&mut self) -> [f64; N];
    /// Get motor reduction ratio
    fn get_reduction_ratio(&mut self) -> [f64; N];

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
    fn get_current_position(&mut self) -> Result<[f64; N]> {
        let pos = self.get_raw_position()?;
        let offset = self.get_offset();
        let reduction_ratio = self.get_reduction_ratio();

        let mut position = [0.0; N];
        for i in 0..N {
            position[i] = pos[i] / reduction_ratio[i] - offset[i];
        }
        Ok(position)
    }
    /// Get the motor raw position (in radians)
    /// ie. without offset and reduction ratio
    fn get_raw_position(&mut self) -> Result<[f64; N]>;

    /// Get the current velocity of the motor (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f64; N]> {
        let vel = self.get_raw_velocity()?;
        let reduction_ratio = self.get_reduction_ratio();

        let mut velocity = [0.0; N];
        for i in 0..N {
            velocity[i] = vel[i] / reduction_ratio[i];
        }
        Ok(velocity)
    }
    /// Get the motor raw velocity (in radians per second)
    /// ie. without reduction ratio
    fn get_raw_velocity(&mut self) -> Result<[f64; N]>;

    /// Get the current torque of the motor (in Nm)
    fn get_current_torque(&mut self) -> Result<[f64; N]> {
        let raw_torque = self.get_raw_torque()?;
        let reduction_ratio = self.get_reduction_ratio();

        let mut torque = [0.0; N];
        for i in 0..N {
            torque[i] = raw_torque[i] / reduction_ratio[i];
        }
        Ok(torque)
    }
    /// Get the motor raw torque (in Nm)
    /// ie. without reduction ratio
    fn get_raw_torque(&mut self) -> Result<[f64; N]>;

    /// Get the current target position of the motor (in radians)
    fn get_target_position(&mut self) -> Result<[f64; N]> {
        let pos = self.get_raw_target_position()?;
        let offset = self.get_offset();
        let reduction_ratio = self.get_reduction_ratio();

        let mut position = [0.0; N];
        for i in 0..N {
            position[i] = pos[i] / reduction_ratio[i] - offset[i];
        }
        Ok(position)
    }
    /// Get raw target position of the motor (in radians)
    /// ie. without offset and reduction ratio
    fn get_raw_target_position(&mut self) -> Result<[f64; N]>;

    /// Set the current target position of the motor (in radians)
    fn set_target_position(&mut self, position: [f64; N]) -> Result<()> {
        let mut raw_target_position = [0.0; N];
        let offset = self.get_offset();
        let reduction_ratio = self.get_reduction_ratio();
        for i in 0..N {
            raw_target_position[i] = position[i] * reduction_ratio[i] + offset[i];
        }
        self.set_raw_target_position(raw_target_position)
    }
    /// Set raw target position of the motor (in radians)
    /// ie. without offset and reduction ratio
    fn set_raw_target_position(&mut self, position: [f64; N]) -> Result<()>;

    /// Get the velocity limit of the motor (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        let vel = self.get_raw_velocity_limit()?;
        let reduction_ratio = self.get_reduction_ratio();

        let mut velocity = [0.0; N];
        for i in 0..N {
            velocity[i] = vel[i] / reduction_ratio[i];
        }
        Ok(velocity)
    }
    /// Get raw velocity limit of the motor (in radians per second)
    /// ie. without reduction ratio
    fn get_raw_velocity_limit(&mut self) -> Result<[f64; N]>;

    /// Set the velocity limit of the motor (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        let mut raw_velocity_limit = [0.0; N];
        let reduction_ratio = self.get_reduction_ratio();
        for i in 0..N {
            raw_velocity_limit[i] = velocity[i] * reduction_ratio[i];
        }
        self.set_raw_velocity_limit(raw_velocity_limit)
    }
    /// Set raw velocity limit of the motor (in radians per second)
    /// ie. without reduction ratio
    fn set_raw_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()>;

    /// Get the torque limit of the motor (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        let raw_torque = self.get_raw_torque_limit()?;
        let reduction_ratio = self.get_reduction_ratio();

        let mut torque = [0.0; N];
        for i in 0..N {
            torque[i] = raw_torque[i] / reduction_ratio[i];
        }
        Ok(torque)
    }
    /// Get raw torque limit of the motor (in Nm)
    /// ie. without reduction ratio
    fn get_raw_torque_limit(&mut self) -> Result<[f64; N]>;

    /// Set the torque limit of the motor (in Nm)
    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        let mut raw_torque_limit = [0.0; N];
        let reduction_ratio = self.get_reduction_ratio();
        for i in 0..N {
            raw_torque_limit[i] = torque[i] * reduction_ratio[i];
        }
        self.set_raw_torque_limit(raw_torque_limit)
    }
    /// Set raw torque limit of the motor (in Nm)
    /// ie. without reduction ratio
    fn set_raw_torque_limit(&mut self, torque: [f64; N]) -> Result<()>;

    /// Get the current PID gains of the motor
    fn get_pid_gains(&mut self) -> Result<[PID; N]>;
    /// Set the current PID gains of the motor
    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()>;
}

impl<const N: usize> std::fmt::Debug for dyn MultipleMotorsController<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MultipleMotorsController")
            .field("name", &self.name())
            .finish()
    }
}

#[derive(Debug)]
pub struct MultipleMotorsControllerWrapper<const N: usize> {
    controllers: [Box<dyn MotorController>; N],
}

impl<const N: usize> MultipleMotorsControllerWrapper<N> {
    pub fn new(controllers: [Box<dyn MotorController>; N]) -> Self {
        Self { controllers }
    }
}

impl<const N: usize> MultipleMotorsController<N> for MultipleMotorsControllerWrapper<N> {
    fn name(&self) -> String {
        "MultipleMotorsController".to_string()
    }

    fn get_offset(&mut self) -> [f64; N] {
        todo!()
    }
    fn get_reduction_ratio(&mut self) -> [f64; N] {
        todo!()
    }

    fn is_torque_on(&mut self) -> Result<bool> {
        let mut torques = vec![];
        for c in self.controllers.iter_mut() {
            match c.is_torque_on() {
                Ok(p) => torques.push(p),
                Err(e) => return Err(e),
            }
        }

        let torques: [bool; 3] = torques.try_into().unwrap();

        if torques[0] == torques[1] && torques[1] == torques[2] {
            Ok(torques[0])
        } else {
            Err(Box::new(IncoherentError {}))
        }
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        for c in self.controllers.iter_mut() {
            c.set_torque(on)?;
        }
        Ok(())
    }

    fn get_raw_position(&mut self) -> Result<[f64; N]> {
        let mut pos = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_position() {
                Ok(p) => pos.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pos.try_into().unwrap())
    }

    fn get_raw_velocity(&mut self) -> Result<[f64; N]> {
        let mut vel = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_velocity() {
                Ok(v) => vel.push(v),
                Err(e) => return Err(e),
            }
        }

        Ok(vel.try_into().unwrap())
    }

    fn get_raw_torque(&mut self) -> Result<[f64; N]> {
        let mut torque = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_torque() {
                Ok(t) => torque.push(t),
                Err(e) => return Err(e),
            }
        }

        Ok(torque.try_into().unwrap())
    }

    fn get_raw_target_position(&mut self) -> Result<[f64; N]> {
        let mut pos = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_target_position() {
                Ok(p) => pos.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pos.try_into().unwrap())
    }

    fn set_raw_target_position(&mut self, position: [f64; N]) -> Result<()> {
        for (c, p) in self.controllers.iter_mut().zip(position.iter()) {
            c.set_raw_target_position(*p)?;
        }
        Ok(())
    }

    fn get_raw_velocity_limit(&mut self) -> Result<[f64; N]> {
        let mut vel = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_velocity_limit() {
                Ok(v) => vel.push(v),
                Err(e) => return Err(e),
            }
        }

        Ok(vel.try_into().unwrap())
    }

    fn set_raw_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        for (c, v) in self.controllers.iter_mut().zip(velocity.iter()) {
            c.set_raw_velocity_limit(*v)?;
        }
        Ok(())
    }

    fn get_raw_torque_limit(&mut self) -> Result<[f64; N]> {
        let mut torque = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_raw_torque_limit() {
                Ok(t) => torque.push(t),
                Err(e) => return Err(e),
            }
        }

        Ok(torque.try_into().unwrap())
    }

    fn set_raw_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        for (c, t) in self.controllers.iter_mut().zip(torque.iter()) {
            c.set_raw_torque_limit(*t)?;
        }
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; N]> {
        let mut pid_gains = vec![];
        for c in self.controllers.iter_mut() {
            match c.get_pid_gains() {
                Ok(p) => pid_gains.push(p),
                Err(e) => return Err(e),
            }
        }

        Ok(pid_gains.try_into().unwrap())
    }

    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()> {
        for (c, p) in self.controllers.iter_mut().zip(pid.iter()) {
            c.set_pid_gains(*p)?;
        }
        Ok(())
    }
}
