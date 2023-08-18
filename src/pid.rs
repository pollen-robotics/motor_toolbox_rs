#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
/// PID gains wrapper
pub struct PID {
    /// Propotional gain
    pub p: f64,
    /// Integral gain
    pub i: f64,
    /// Derivative gain
    pub d: f64,
}
