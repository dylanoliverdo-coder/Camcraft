//! The wall-pass rows of the 2D Contour page (Journeyman spec S1):
//! roughing passes and their spacing, finishing passes and their
//! spacing, spring passes, and the levels the finishing passes run on.
//!
//! Provenance: Journeyman spec S1
//! (docs/journeyman/specs/S1-wall-passes.md); no vendor parameters, text
//! or code.

use super::fields::*;
use camcraft_app::machining2d::{FinishLevel, WallPasses};

/// Draw the Passes tab's wall-pass rows: a count, and its spacing only
/// when the count asks for it.
pub(super) fn wall_pass_rows(ui: &mut Ui, u: Unit, passes: &mut WallPasses) {
    let mut count = f64::from(passes.rough_passes);
    prop_row(ui, "Roughing passes", |ui| num_plain(ui, &mut count));
    passes.rough_passes = count.clamp(1.0, 64.0).round().max(1.0) as u32;
    if passes.rough_passes > 1 {
        prop_row(ui, "Roughing spacing", |ui| num(ui, u, &mut passes.rough_spacing));
    }
    let mut count = f64::from(passes.finish_passes);
    prop_row(ui, "Finishing passes", |ui| num_plain(ui, &mut count));
    passes.finish_passes = count.clamp(0.0, 64.0).round().max(0.0) as u32;
    if passes.finish_passes > 0 {
        prop_row(ui, "Finishing spacing", |ui| num(ui, u, &mut passes.finish_spacing));
        let mut idx = match passes.finish_level {
            FinishLevel::Every => 0,
            FinishLevel::Last => 1,
        };
        prop_row(ui, "Finish levels", |ui| combo(ui, "c_finish_level", &mut idx, &["Every level", "Last level only"]));
        passes.finish_level = if idx == 1 { FinishLevel::Last } else { FinishLevel::Every };
    }
    let mut count = f64::from(passes.spring_passes);
    prop_row(ui, "Spring passes", |ui| num_plain(ui, &mut count));
    passes.spring_passes = count.clamp(0.0, 64.0).round().max(0.0) as u32;
    hint(ui, "wall passes ride several offsets of the wall per level, the farthest first and the final offset last");
}