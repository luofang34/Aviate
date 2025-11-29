//! World State
//!
//! Backend-agnostic representation of simulation world state.
//! All backends update this common structure.

use std::collections::HashMap;

/// Unique identifier for entities in the world
pub type EntityId = u32;

/// 3D position (backend-agnostic, uses NED convention internally)
#[derive(Debug, Clone, Copy, Default)]
pub struct Position {
    pub north: f64,
    pub east: f64,
    pub down: f64,
}

impl Position {
    pub fn new(north: f64, east: f64, down: f64) -> Self {
        Self { north, east, down }
    }

    /// Altitude (positive up)
    pub fn altitude(&self) -> f64 {
        -self.down
    }

    /// Distance from another position (horizontal only)
    pub fn horizontal_distance(&self, other: &Position) -> f64 {
        let dn = self.north - other.north;
        let de = self.east - other.east;
        (dn * dn + de * de).sqrt()
    }

    /// 3D distance from another position
    pub fn distance(&self, other: &Position) -> f64 {
        let dn = self.north - other.north;
        let de = self.east - other.east;
        let dd = self.down - other.down;
        (dn * dn + de * de + dd * dd).sqrt()
    }
}

/// 3D velocity (NED convention)
#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity {
    pub north: f64,
    pub east: f64,
    pub down: f64,
}

impl Velocity {
    pub fn new(north: f64, east: f64, down: f64) -> Self {
        Self { north, east, down }
    }

    /// Horizontal speed
    pub fn horizontal_speed(&self) -> f64 {
        (self.north * self.north + self.east * self.east).sqrt()
    }

    /// Total speed (3D magnitude)
    pub fn speed(&self) -> f64 {
        (self.north * self.north + self.east * self.east + self.down * self.down).sqrt()
    }

    /// Climb rate (positive up)
    pub fn climb_rate(&self) -> f64 {
        -self.down
    }
}

/// Quaternion orientation (w, x, y, z)
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Create from Euler angles (roll, pitch, yaw in radians)
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll / 2.0).sin_cos();
        let (sp, cp) = (pitch / 2.0).sin_cos();
        let (sy, cy) = (yaw / 2.0).sin_cos();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// Extract Euler angles (roll, pitch, yaw in radians)
    pub fn to_euler(&self) -> (f64, f64, f64) {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f64::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }
}

/// Angular velocity (rad/s, body frame)
#[derive(Debug, Clone, Copy, Default)]
pub struct AngularVelocity {
    pub roll_rate: f64,
    pub pitch_rate: f64,
    pub yaw_rate: f64,
}

/// Entity state (position, velocity, orientation)
#[derive(Debug, Clone, Default)]
pub struct EntityState {
    pub position: Position,
    pub velocity: Velocity,
    pub orientation: Quaternion,
    pub angular_velocity: AngularVelocity,
}

/// Entity in the simulation world
#[derive(Debug, Clone)]
pub struct Entity {
    pub id: EntityId,
    pub name: String,
    pub model: String,
    pub instance: u8,
    pub state: EntityState,
    pub armed: bool,
    pub motor_speeds: [f64; 8], // Up to 8 motors
}

impl Entity {
    pub fn new(id: EntityId, name: &str, model: &str, instance: u8) -> Self {
        Self {
            id,
            name: name.to_string(),
            model: model.to_string(),
            instance,
            state: EntityState::default(),
            armed: false,
            motor_speeds: [0.0; 8],
        }
    }
}

/// Simulation world state
///
/// This is the central data structure that backends update and
/// the test runner/mission framework reads.
#[derive(Debug, Default)]
pub struct World {
    /// All entities (vehicles, obstacles, etc.)
    pub entities: HashMap<EntityId, Entity>,

    /// Entity lookup by name
    name_to_id: HashMap<String, EntityId>,

    /// Next available entity ID
    next_id: EntityId,
}

impl World {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an entity to the world
    pub fn add_entity(&mut self, name: &str, model: &str, instance: u8) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;

        let entity = Entity::new(id, name, model, instance);
        self.name_to_id.insert(name.to_string(), id);
        self.entities.insert(id, entity);

        id
    }

    /// Get entity by ID
    pub fn get(&self, id: EntityId) -> Option<&Entity> {
        self.entities.get(&id)
    }

    /// Get mutable entity by ID
    pub fn get_mut(&mut self, id: EntityId) -> Option<&mut Entity> {
        self.entities.get_mut(&id)
    }

    /// Get entity by name
    pub fn get_by_name(&self, name: &str) -> Option<&Entity> {
        self.name_to_id
            .get(name)
            .and_then(|id| self.entities.get(id))
    }

    /// Get mutable entity by name
    pub fn get_by_name_mut(&mut self, name: &str) -> Option<&mut Entity> {
        self.name_to_id
            .get(name)
            .copied()
            .and_then(move |id| self.entities.get_mut(&id))
    }

    /// Get entity by instance number
    pub fn get_by_instance(&self, instance: u8) -> Option<&Entity> {
        self.entities.values().find(|e| e.instance == instance)
    }

    /// Get mutable entity by instance number
    pub fn get_by_instance_mut(&mut self, instance: u8) -> Option<&mut Entity> {
        self.entities.values_mut().find(|e| e.instance == instance)
    }

    /// Iterate over all entities
    pub fn iter(&self) -> impl Iterator<Item = &Entity> {
        self.entities.values()
    }

    /// Number of entities
    pub fn len(&self) -> usize {
        self.entities.len()
    }

    /// Check if world is empty
    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }

    /// Clear all entities
    pub fn clear(&mut self) {
        self.entities.clear();
        self.name_to_id.clear();
        self.next_id = 0;
    }
}
