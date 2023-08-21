use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
/// Limit wrapper
pub struct Limit {
    /// lower limit
    pub min: f64,
    /// upper limit
    pub max: f64,
}

impl Limit {
    /// Clamp value to limits
    pub fn clamp(&self, value: f64) -> f64 {
        value.clamp(self.min, self.max)
    }
}
