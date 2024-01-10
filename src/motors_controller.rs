use crate::{Limit, RawMotorsIO, Result, PID};

pub trait MotorsController<const N: usize> {
    fn io(&mut self) -> &mut dyn RawMotorsIO<N>;

    /// Get the offsets of the motors (in radians)
    fn offsets(&self) -> [Option<f64>; N];
    /// Get the reduction of the motors
    fn reduction(&self) -> [Option<f64>; N];
    /// Get the limits of the motors
    fn limits(&self) -> [Option<Limit>; N];

    /// Check if the torque is ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; N]> {
        self.io().is_torque_on()
    }
    /// Enable the torque
    fn set_torque(&mut self, on: [bool; N]) -> Result<()> {
        self.io().set_torque(on)
    }

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f64; N]> {
        let mut position = self.io().get_current_position()?;
        log::debug!(target: "controller::get_current_position", "raw current_position: {:?}", position);

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                position[i] /= reductions;
            }
            if let Some(offsets) = offsets[i] {
                position[i] -= offsets;
            }
        }
        log::debug!(target: "controller::get_current_position", "after offset/reduction current_position: {:?}", position);

        Ok(position)
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f64; N]> {
        let mut velocity = self.io().get_current_velocity()?;
        log::debug!(target: "controller::get_current_velocity", "raw current_velocity: {:?}", velocity);

        let reductions = self.reduction();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                velocity[i] /= reductions;
            }
        }
        log::debug!(target: "controller::get_current_velocity", "after reduction current_velocity: {:?}", velocity);

        Ok(velocity)
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f64; N]> {
        let mut torque = self.io().get_current_torque()?;
        log::debug!(target: "controller::get_current_torque", "raw current_torque: {:?}", torque);

        let reductions = self.reduction();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                torque[i] /= reductions;
            }
        }
        log::debug!(target: "controller::get_current_torque", "after reduction current_torque: {:?}", torque);

        Ok(torque)
    }
    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f64; N]> {
        let mut position = self.io().get_target_position()?;
        log::debug!(target: "controller::get_target_position", "raw target_position: {:?}", position);

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                position[i] /= reductions;
            }
            if let Some(offsets) = offsets[i] {
                position[i] -= offsets;
            }
        }
        log::debug!(target: "controller::get_target_position", "after offset/reduction target_position: {:?}", position);

        Ok(position)
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_target_position", "real target_position: {:?}", position);

        let mut limited_position = position;
        for i in 0..N {
            if let Some(limits) = self.limits()[i] {
                limited_position[i] = limits.clamp(position[i]);
            }
        }

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(offsets) = offsets[i] {
                limited_position[i] += offsets;
            }
            if let Some(reductions) = reductions[i] {
                limited_position[i] *= reductions;
            }
        }

        log::debug!(target: "controller::set_target_position", "raw target_position: {:?}", limited_position);

        self.io().set_target_position(limited_position)
    }

    /// Set the current target position and returns the motor feeback (position, velocity, torque)
    fn set_target_position_fb(&mut self, position: [f64; N]) -> Result<[f64; N*3]> {
        log::debug!(target: "controller::set_target_position", "real target_position: {:?}", position);

        let mut limited_position = position;
        for i in 0..N {
            if let Some(limits) = self.limits()[i] {
                limited_position[i] = limits.clamp(position[i]);
            }
        }

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(offsets) = offsets[i] {
                limited_position[i] += offsets;
            }
            if let Some(reductions) = reductions[i] {
                limited_position[i] *= reductions;
            }
        }

        log::debug!(target: "controller::set_target_position", "raw target_position: {:?}", limited_position);

        let mut fb=self.io().set_target_position_fb(limited_position)?;
	// let ret:[f64;N*3]=fb?;
	// let ret=[0.0;N*3];


        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                fb[i] /= reductions;
            }
            if let Some(offsets) = offsets[i] {
                fb[i] -= offsets;
            }

        }


	Ok(fb)

    }




    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        let mut velocity = self.io().get_velocity_limit()?;
        log::debug!(target: "controller::get_velocity_limit", "raw velocity_limit: {:?}", velocity);

        let reductions = self.reduction();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                velocity[i] /= reductions;
            }
        }
        log::debug!(target: "controller::get_velocity_limit", "after reduction velocity_limit: {:?}", velocity);

        Ok(velocity)
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_velocity_limit", "real velocity_limit: {:?}", velocity);

        let reductions = self.reduction();
        let mut velocity = velocity;

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                velocity[i] *= reductions;
            }
        }
        log::debug!(target: "controller::set_velocity_limit", "raw velocity_limit: {:?}", velocity);

        self.io().set_velocity_limit(velocity)
    }
    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        let mut torque = self.io().get_torque_limit()?;
        log::debug!(target: "controller::get_torque_limit", "raw torque_limit: {:?}", torque);

        let reductions = self.reduction();

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                torque[i] /= reductions;
            }
        }
        log::debug!(target: "controller::get_torque_limit", "after reduction torque_limit: {:?}", torque);

        Ok(torque)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_torque_limit", "real torque_limit: {:?}", torque);

        let reductions = self.reduction();
        let mut torque = torque;

        for i in 0..N {
            if let Some(reductions) = reductions[i] {
                torque[i] *= reductions;
            }
        }
        log::debug!(target: "controller::set_torque_limit", "raw torque_limit: {:?}", torque);

        self.io().set_torque_limit(torque)
    }
    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[PID; N]> {
        self.io().get_pid_gains()
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()> {
        self.io().set_pid_gains(pid)
    }
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
