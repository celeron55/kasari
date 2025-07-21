use kasarisw::shared::rem_euclid_f32;
use std::f32::consts::PI;

#[derive(Clone, Copy)]
pub struct Rect {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

pub struct World {
    pub arena: Rect,
    pub objects: Vec<Rect>,
}

impl World {
    pub fn raycast_to_rect(
        &self,
        pos_x: f32,
        pos_y: f32,
        dir_x: f32,
        dir_y: f32,
        rect: &Rect,
    ) -> Option<f32> {
        let mut min_t = f32::INFINITY;

        // Left wall
        if dir_x != 0.0 {
            let t = (rect.min_x - pos_x) / dir_x;
            if t > 0.0 {
                let hit_y = pos_y + t * dir_y;
                if hit_y >= rect.min_y && hit_y <= rect.max_y {
                    min_t = min_t.min(t);
                }
            }
        }

        // Right wall
        if dir_x != 0.0 {
            let t = (rect.max_x - pos_x) / dir_x;
            if t > 0.0 {
                let hit_y = pos_y + t * dir_y;
                if hit_y >= rect.min_y && hit_y <= rect.max_y {
                    min_t = min_t.min(t);
                }
            }
        }

        // Bottom wall
        if dir_y != 0.0 {
            let t = (rect.min_y - pos_y) / dir_y;
            if t > 0.0 {
                let hit_x = pos_x + t * dir_x;
                if hit_x >= rect.min_x && hit_x <= rect.max_x {
                    min_t = min_t.min(t);
                }
            }
        }

        // Top wall
        if dir_y != 0.0 {
            let t = (rect.max_y - pos_y) / dir_y;
            if t > 0.0 {
                let hit_x = pos_x + t * dir_x;
                if hit_x >= rect.min_x && hit_x <= rect.max_x {
                    min_t = min_t.min(t);
                }
            }
        }

        if min_t.is_infinite() {
            None
        } else {
            Some(min_t)
        }
    }

    pub fn raycast(&self, pos_x: f32, pos_y: f32, dir_x: f32, dir_y: f32) -> f32 {
        let mut min_t = f32::INFINITY;

        if let Some(t) = self.raycast_to_rect(pos_x, pos_y, dir_x, dir_y, &self.arena) {
            min_t = min_t.min(t);
        }

        for obj in &self.objects {
            if let Some(t) = self.raycast_to_rect(pos_x, pos_y, dir_x, dir_y, obj) {
                min_t = min_t.min(t);
            }
        }

        if min_t.is_infinite() {
            3000.0
        } else {
            min_t
        }
    }
}

pub struct Robot {
    pub pos_x: f32,
    pub pos_y: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub theta: f32,
    pub rpm: f32,
}
