use crate::motor_controller::Result;

pub trait CoherentResult<T> {
    fn coherent(self) -> Result<T>;
}

#[derive(Debug)]
struct IncoherentError;
impl std::fmt::Display for IncoherentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(incoherent values)",)
    }
}
impl std::error::Error for IncoherentError {}
