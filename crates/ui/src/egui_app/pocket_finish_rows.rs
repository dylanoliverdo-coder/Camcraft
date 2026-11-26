//! Wall finishing rows of the 2D Pocket page (Journeyman spec S4):
//! finishing passes, finishing spacing, spring passes, finish direction,
//! and finish levels.
//!
//! Provenance: Journeyman spec S4 (docs/journeyman/specs/S4-pocket-finish-passes.md);
//! M. Held 1991; P. Smid 2007; no vendor parameters, text or code.

use super::fields::*;
use camcraft_app::machining2d::{FinishDirection, FinishLevel, PocketFinish};

/// Draw the Passes tab's wall-finishing rows for 2D Pocket: finishing pass
/// count, spacing (when finishing passes > 0), spring pass count, finish
/// direction and finish levels (when finishing or spring passes > 0).
pub(super) fn pocket_finish_rows(ui: &mut Ui, u: Unit, finish: &mut PocketFinish) {
    let prev_passes = finish.finish_passes;
    let mut count = f64::from(finish.finish_passes);
    prop_row(ui, "Finishing passes", |ui| num_plain(ui, &mut count));
    finish.finish_passes = count.clamp(0.0, 64.0).round().max(0.0) as u32;
    if prev_passes == 0 && finish.finish_passes > 0 && finish.finish_spacing <= 0.0 {
        finish.finish_spacing = 0.25;
    }
    if finish.finish_passes > 0 {
        prop_row(ui, "Finishing spacing", |ui| num(ui, u, &mut finish.finish_spacing));
    }
    let mut spring = f64::from(finish.spring_passes);
    prop_row(ui, "Spring passes", |ui| num_plain(ui, &mut spring));
    finish.spring_passes = spring.clamp(0.0, 64.0).round().max(0.0) as u32;
    if finish.finish_passes > 0 || finish.spring_passes > 0 {
        let mut dir_idx = match finish.finish_direction {
            FinishDirection::Same => 0,
            FinishDirection::Climb => 1,
            FinishDirection::Conventional => 2,
        };
        prop_row(ui, "Finish direction", |ui| {
            combo(ui, "p_finish_dir", &mut dir_idx, &["Same as clearing", "Climb", "Conventional"])
        });
        finish.finish_direction = match dir_idx {
            1 => FinishDirection::Climb,
            2 => FinishDirection::Conventional,
            _ => FinishDirection::Same,
        };
        let mut lvl_idx = match finish.finish_level {
            FinishLevel::Every => 0,
            FinishLevel::Last => 1,
        };
        prop_row(ui, "Finish levels", |ui| {
            combo(ui, "p_finish_level", &mut lvl_idx, &["Every level", "Last level only"])
        });
        finish.finish_level = if lvl_idx == 1 { FinishLevel::Last } else { FinishLevel::Every };
    }
    hint(
        ui,
        "contour-parallel wall finishing passes after clearing with their own direction on every wall and island",
    );
}
