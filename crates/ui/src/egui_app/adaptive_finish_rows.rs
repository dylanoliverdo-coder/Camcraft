//! The wall-finishing rows of the 2D Adaptive page (Journeyman spec S5):
//! finishing passes, finishing spacing, spring passes, and finish levels.
//!
//! Provenance: Journeyman spec S5
//! (docs/journeyman/specs/S5-adaptive-finish-passes.md); M. Held 1991;
//! M. Stori and P. K. Wright 2000; no vendor parameters, text or code.

use super::fields::*;
use camcraft_app::machining2d::{AdaptiveFinish, FinishLevel};

/// Draw the Passes tab's wall-finishing rows for 2D Adaptive: finishing pass
/// count, spacing (when finishing passes > 0), finish level (when finishing or
/// spring passes > 0), and spring pass count.
pub(super) fn adaptive_finish_rows(ui: &mut Ui, u: Unit, finish: &mut AdaptiveFinish) {
    let mut count = f64::from(finish.finish_passes);
    prop_row(ui, "Finishing passes", |ui| num_plain(ui, &mut count));
    finish.finish_passes = count.clamp(0.0, 64.0).round().max(0.0) as u32;
    if finish.finish_passes > 0 {
        prop_row(ui, "Finishing spacing", |ui| num(ui, u, &mut finish.finish_spacing));
    }
    let mut spring = f64::from(finish.spring_passes);
    prop_row(ui, "Spring passes", |ui| num_plain(ui, &mut spring));
    finish.spring_passes = spring.clamp(0.0, 64.0).round().max(0.0) as u32;
    if finish.finish_passes > 0 || finish.spring_passes > 0 {
        let mut idx = match finish.finish_level {
            FinishLevel::Every => 0,
            FinishLevel::Last => 1,
        };
        prop_row(ui, "Finish levels", |ui| {
            combo(ui, "a_finish_level", &mut idx, &["Every level", "Last level only"])
        });
        finish.finish_level = if idx == 1 { FinishLevel::Last } else { FinishLevel::Every };
    }
    hint(
        ui,
        "contour-parallel wall finishing passes after clearing, leaving the finishing allowance on every wall",
    );
}
