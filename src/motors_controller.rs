use crate::{Limit, RawMotorsIO, Result, PID};

pub trait MotorsController<const N: usize> {
    fn io(&mut self) -> &mut dyn RawMotorsIO<N>;

    /// Get the name of the controller
    fn name(&self) -> String {
        format!("MotorsController<{:?}>", N)
    }

    /// Get the axes inversion of the output
    fn output_inverted_axes(&self) -> [Option<bool>; N] {
        [None; N]
    }
    /// Get the offsets of the motors (in radians)
    fn offsets(&self) -> [Option<f64>; N];
    /// Get the reduction of the motors
    fn reduction(&self) -> [Option<f64>; N];
    /// Get the limits of the motors
    fn limits(&self) -> [Option<Limit>; N];

    /// Get the axes invertion
    fn inverted_axes(&self) -> [Option<bool>; N] {
        [None; N]
    }

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
        let inverted_axes = self.inverted_axes();

        for i in 0..N {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    position[i] = -position[i];
                }
            }
            if let Some(reductions) = reductions[i] {
                position[i] /= reductions;
            }
            if let Some(offsets) = offsets[i] {
                position[i] -= offsets;
            }
        }

        log::debug!(target: "controller::get_current_position", "after offset/reduction current_position: {:?} (reductions {:?} offsets {:?})", position,reductions,offsets);

        Ok(position)
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f64; N]> {
        let mut velocity = self.io().get_current_velocity()?;
        log::debug!(target: "controller::get_current_velocity", "raw current_velocity: {:?}", velocity);

        let reductions = self.reduction();
        let inverted_axes = self.inverted_axes();

        for i in 0..N {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    velocity[i] = -velocity[i];
                }
            }
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
        let inverted_axes = self.inverted_axes();

        for i in 0..N {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    torque[i] = -torque[i];
                }
            }
            if let Some(reductions) = reductions[i] {
                // torque output is reductions times higher than the input
                torque[i] *= reductions;
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
        let inverted_axes = self.inverted_axes();

        for i in 0..N {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    position[i] = -position[i];
                }
            }
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
        let inverted_axes = self.inverted_axes();
        (0..N).for_each(|i| {
            if let Some(limits) = self.limits()[i] {
                limited_position[i] = limits.clamp(limited_position[i]);
            }
        });

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(offsets) = offsets[i] {
                limited_position[i] += offsets;
            }
            if let Some(reductions) = reductions[i] {
                limited_position[i] *= reductions;
            }
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    limited_position[i] = -limited_position[i];
                }
            }
        }

        log::debug!(target: "controller::set_target_position", "raw target_position: {:?}", limited_position);

        self.io().set_target_position(limited_position)
    }

    /// Set the current target torque of the motors (in Nm) TODO: inverted_axes?
    fn set_target_torque(&mut self, torque: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_target_torque", "real target_torque: {:?}", torque);

        self.io().set_target_torque(torque)
    }

    /// Set the current target velocity of the motors (in rad/s) TODO: inverted_axes?
    fn set_target_velocity(&mut self, velocity: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_target_velocity", "real target_velocity: {:?}", velocity);

        self.io().set_target_velocity(velocity)
    }

    /// Set control mode
    fn set_control_mode(&mut self, mode: [u8; N]) -> Result<()> {
        log::debug!(target: "controller::set_control_mode", "real control_mode: {:?}", mode);

        self.io().set_control_mode(mode)
    }

    /// Get the current target torque of the motors (in Nm) TODO: inverted_axes?
    fn get_target_torque(&mut self) -> Result<[f64; N]> {
        let torque = self.io().get_target_torque()?;
        log::debug!(target: "controller::get_target_torque", "raw target_torque: {:?}", torque);
        Ok(torque)
    }

    /// Get the current target velocity of the motors (in rad/s) TODO: inverted_axes?
    fn get_target_velocity(&mut self) -> Result<[f64; N]> {
        let velocity = self.io().get_target_velocity()?;
        log::debug!(target: "controller::get_target_velocity", "raw target_velocity: {:?}", velocity);
        Ok(velocity)
    }

    /// Get the current control mode
    fn get_control_mode(&mut self) -> Result<[u8; N]> {
        let mode = self.io().get_control_mode()?;
        log::debug!(target: "controller::get_control_mode", "raw control_mode: {:?}", mode);
        Ok(mode)
    }

    /// Set the current target position and returns the motor feeback (position, velocity, torque)
    fn set_target_position_fb(&mut self, position: [f64; N]) -> Result<[f64; N]> {
        log::debug!(target: "controller::set_target_position", "real target_position: {:?}", position);

        let mut limited_position = position;
        let inverted_axes = self.inverted_axes();

        (0..N).for_each(|i| {
            if let Some(limits) = self.limits()[i] {
                limited_position[i] = limits.clamp(limited_position[i]);
            }
        });

        let reductions = self.reduction();
        let offsets = self.offsets();

        for i in 0..N {
            if let Some(offsets) = offsets[i] {
                limited_position[i] += offsets;
            }
            if let Some(reductions) = reductions[i] {
                limited_position[i] *= reductions;
            }
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    limited_position[i] = -limited_position[i];
                }
            }
        }

        log::debug!(target: "controller::set_target_position", "raw target_position: {:?}", limited_position);

        let mut fb = self.io().set_target_position_fb(limited_position)?;
        // let ret:[f64;N*3]=fb?;
        // let ret=[0.0;N*3];

        for i in 0..N {
            if let Some(inverted) = inverted_axes[i] {
                if inverted {
                    fb[i] = -fb[i];
                }
            }
            if let Some(reductions) = reductions[i] {
                fb[i] /= reductions; //position
                                     // fb[i + N] /= reductions; //velocity
                                     // fb[i+N*2] /= reductions; //torque
            }
            if let Some(offsets) = offsets[i] {
                fb[i] -= offsets;
            }
        }

        Ok(fb)
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        let velocity = self.io().get_velocity_limit()?;
        log::debug!(target: "controller::get_velocity_limit", "raw velocity_limit: {:?}", velocity);
        /*
            let reductions = self.reduction();

            for i in 0..N {
                if let Some(reductions) = reductions[i] {
                    velocity[i] /= reductions;
                }
            }
            log::debug!(target: "controller::get_velocity_limit", "after reduction velocity_limit: {:?}", velocity);
        */
        Ok(velocity)
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_velocity_limit", "real velocity_limit: {:?}", velocity);
        /*
            let reductions = self.reduction();
            let mut velocity = velocity;

            for i in 0..N {
                if let Some(reductions) = reductions[i] {
                    velocity[i] *= reductions;
                }
            }
            log::debug!(target: "controller::set_velocity_limit", "raw velocity_limit: {:?}", velocity);
        */
        self.io().set_velocity_limit(velocity)
    }
    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        let torque = self.io().get_torque_limit()?;
        log::debug!(target: "controller::get_torque_limit", "raw torque_limit: {:?}", torque);
        /*
            let reductions = self.reduction();

            for i in 0..N {
                if let Some(reductions) = reductions[i] {
                    torque[i] /= reductions;
                }
            }
            log::debug!(target: "controller::get_torque_limit", "after reduction torque_limit: {:?}", torque);
        */
        Ok(torque)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        log::debug!(target: "controller::set_torque_limit", "real torque_limit: {:?}", torque);
        /*
            let reductions = self.reduction();
            let mut torque = torque;

            for i in 0..N {
                if let Some(reductions) = reductions[i] {
                    torque[i] *= reductions;
                }
            }
            log::debug!(target: "controller::set_torque_limit", "raw torque_limit: {:?}", torque);
        */
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

    /// Get the current axis sensors of the articulation TODO: inverted_axes?
    fn get_axis_sensors(&mut self) -> Result<[f64; N]> {
        self.io().get_axis_sensors()
    }

    /// Get the motors error codes
    fn get_error_codes(&mut self) -> Result<[i32; N]> {
        self.io().get_error_codes()
    }

    /// Get the current zeros of the axis sensors of the articulation TODO: inverted_axes?
    fn get_axis_sensor_zeros(&mut self) -> Result<[f64; N]> {
        self.io().get_axis_sensor_zeros()
    }

    /// Get the current state of the articulation control board
    fn get_board_state(&mut self) -> Result<u8> {
        self.io().get_board_state()
    }
    /// Set the current state of the articulation control board (clear error)
    fn set_board_state(&mut self, state: u8) -> Result<()> {
        self.io().set_board_state(state)
    }

    /// Get the motor temperatures (in Celsius)
    fn get_motor_temperatures(&mut self) -> Result<[f64; N]> {
        self.io().get_motor_temperatures()
    }
    /// Get the board H-bridges temperatures (in Celsius)
    fn get_board_temperatures(&mut self) -> Result<[f64; N]> {
        self.io().get_board_temperatures()
    }

    /// Trigger an emergency stop
    fn emergency_stop(&mut self) {
        self.io().emergency_stop()
    }
}

#[derive(Debug)]
pub struct MissingRegisterError(pub String);
impl std::fmt::Display for MissingRegisterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = &self.0;
        write!(f, "(missing register \"{name}\")",)
    }
}
impl std::error::Error for MissingRegisterError {}
