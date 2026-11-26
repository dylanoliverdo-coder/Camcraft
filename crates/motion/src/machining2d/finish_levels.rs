//! Finishing depth levels at a smaller stepdown (Journeyman spec S8):
//! roughing levels down to `z_r = z_b + n * s` by the shared level law
//! (remainder or equal steps, honoring material top), followed by `n`
//! finishing levels `z_r - s, z_r - 2s, ..., z_b` (or `n` repeats of `z_b`
//! when `s = 0`).
//!
//! Provenance: Journeyman spec S8 (`docs/journeyman/specs/S8-finish-levels.md`);
//! P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press 2007 (roughing
//! and finishing depth of cut); M. P. Groover, *Fundamentals of Modern
//! Manufacturing*, Wiley (light finishing depth after heavier roughing depths
//! for floor finish and accuracy). No vendor parameters, text or code.

use super::GenError;
use super::equal_steps::equal_stepdown;
use super::levels::cut_levels_under;

/// Refusal message when `finish_stepdown` is negative or non-finite.
pub const FINISH_STEPDOWN_REFUSAL: &str =
    "finish_stepdown must be zero or a finite non-negative length in mm";

/// Refuse a negative or non-finite `finish_stepdown` by name; `None` is valid.
#[must_use]
pub fn finish_stepdown_refusal(v: f64) -> Option<&'static str> {
    if v.is_finite() && v >= 0.0 {
        None
    } else {
        Some(FINISH_STEPDOWN_REFUSAL)
    }
}

/// Cut levels from `top` down to `bottom` with optional finishing levels:
///
/// - When `finish_levels == 0`: exact legacy `cut_levels_under` with `stepdown`
///   (or `equal_stepdown` when `equal_steps` is true).
/// - When `finish_levels > 0` and `finish_stepdown == 0.0`: today's levels down
///   to `bottom`, then `bottom` repeated `finish_levels` more times.
/// - When `finish_levels > 0` and `finish_stepdown > 0.0`:
///   - Refuses by name if `finish_levels * finish_stepdown >= top - bottom`
///     (leaving no roughing level), naming both values.
///   - Otherwise roughing levels down to `z_r = bottom + n * s`, followed by
///     `n` finishing levels ending at `bottom` exactly.
pub fn cut_levels_with_finish(
    top: f64,
    stock_top: Option<f64>,
    bottom: f64,
    stepdown: f64,
    equal_steps: bool,
    finish_levels: u32,
    finish_stepdown: f64,
) -> Result<Vec<f64>, GenError> {
    if let Some(why) = finish_stepdown_refusal(finish_stepdown) {
        return Err(GenError::Invalid(why.into()));
    }
    if finish_levels == 0 {
        let step = if equal_steps { equal_stepdown(top, bottom, stepdown) } else { stepdown };
        return cut_levels_under(top, stock_top, bottom, step);
    }
    if !top.is_finite() || !bottom.is_finite() {
        return Err(GenError::Invalid("parameters must be finite".into()));
    }
    if finish_stepdown == 0.0 {
        let step = if equal_steps { equal_stepdown(top, bottom, stepdown) } else { stepdown };
        let mut zs = cut_levels_under(top, stock_top, bottom, step)?;
        if zs.len() as f64 + f64::from(finish_levels) > super::MAX_PASSES {
            return Err(GenError::Invalid(format!(
                "finish_levels ({finish_levels}) exceeds the maximum pass budget ({})",
                super::MAX_PASSES
            )));
        }
        for _ in 0..finish_levels {
            zs.push(bottom);
        }
        return Ok(zs);
    }

    let n = finish_levels;
    let s = finish_stepdown;
    let depth = top - bottom;
    let finish_depth = f64::from(n) * s;
    if !depth.is_finite() || !finish_depth.is_finite() {
        return Err(GenError::Invalid("parameters must be finite".into()));
    }
    let eps = depth.abs().max(1.0) * 1e-12;
    if depth <= 0.0 || finish_depth >= depth - eps {
        return Err(GenError::Invalid(format!(
            "finish_levels ({n}) x finish_stepdown ({s:.3} mm) = {finish_depth:.3} mm is at or above the cut depth ({depth:.3} mm) - no roughing level remains; reduce finish_levels or finish_stepdown"
        )));
    }

    let z_r = bottom + finish_depth;
    let step = if equal_steps { equal_stepdown(top, z_r, stepdown) } else { stepdown };
    let mut zs = cut_levels_under(top, stock_top, z_r, step)?;
    if zs.len() as f64 + f64::from(n) > super::MAX_PASSES {
        return Err(GenError::Invalid(format!(
            "finish_levels ({n}) exceeds the maximum pass budget ({})",
            super::MAX_PASSES
        )));
    }
    for k in 1..=n {
        let z = if k == n { bottom } else { z_r - f64::from(k) * s };
        zs.push(z);
    }
    Ok(zs)
}

/// Cut levels for a Face operation (staged or single-level) with finishing levels.
pub(super) fn face_levels(
    top_z: Option<f64>,
    stock_top: Option<f64>,
    cut_z: f64,
    stepdown: Option<f64>,
    finish_levels: u32,
    finish_stepdown: f64,
) -> Result<Vec<f64>, GenError> {
    if let Some(why) = finish_stepdown_refusal(finish_stepdown) {
        return Err(GenError::Invalid(why.into()));
    }
    match (top_z, stepdown) {
        (Some(t), Some(sd)) => {
            cut_levels_with_finish(t, stock_top, cut_z, sd, false, finish_levels, finish_stepdown)
        }
        _ => {
            if finish_levels == 0 {
                return Ok(vec![cut_z]);
            }
            if finish_stepdown > 0.0 {
                let finish_depth = f64::from(finish_levels) * finish_stepdown;
                return Err(GenError::Invalid(format!(
                    "finish_levels ({finish_levels}) x finish_stepdown ({finish_stepdown:.3} mm) = {finish_depth:.3} mm is at or above the cut depth (0.000 mm) - no roughing level remains; reduce finish_levels or finish_stepdown"
                )));
            }
            if 1.0 + f64::from(finish_levels) > super::MAX_PASSES {
                return Err(GenError::Invalid(format!(
                    "finish_levels ({finish_levels}) exceeds the maximum pass budget ({})",
                    super::MAX_PASSES
                )));
            }
            Ok(vec![cut_z; 1 + finish_levels as usize])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_zs(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len(), "actual={actual:?} expected={expected:?}");
        for (&a, &e) in actual.iter().zip(expected) {
            assert!((a - e).abs() < 1e-12, "actual={actual:?} expected={expected:?}");
        }
    }

    #[test]
    fn s8_check_2_equal_steps_on() {
        let zs = cut_levels_with_finish(0.0, None, -3.81, 1.905, true, 1, 0.508).expect("check 2");
        assert_zs(&zs, &[-1.651, -3.302, -3.81]);
        assert_eq!(*zs.last().expect("last"), -3.81);
    }

    #[test]
    fn s8_check_3_equal_steps_off() {
        let zs = cut_levels_with_finish(0.0, None, -3.81, 1.905, false, 1, 0.508).expect("check 3");
        assert_zs(&zs, &[-1.905, -3.302, -3.81]);
        assert_eq!(*zs.last().expect("last"), -3.81);
    }

    #[test]
    fn s8_check_4_three_finish_levels() {
        let zs = cut_levels_with_finish(0.0, None, -6.0, 2.0, false, 3, 0.25).expect("check 4");
        assert_zs(&zs, &[-2.0, -4.0, -5.25, -5.5, -5.75, -6.0]);
        assert_eq!(*zs.last().expect("last"), -6.0);
    }

    #[test]
    fn s8_check_5_zero_finish_stepdown_repeats_bottom() {
        let zs = cut_levels_with_finish(0.0, None, -4.0, 2.0, false, 1, 0.0).expect("check 5");
        assert_zs(&zs, &[-2.0, -4.0, -4.0]);
    }

    #[test]
    fn s8_check_6_refusal_names_both_values() {
        let err = cut_levels_with_finish(0.0, None, -3.0, 2.0, false, 2, 2.0)
            .expect_err("n*s >= depth must refuse")
            .to_string();
        assert!(err.contains("finish_levels") && err.contains("finish_stepdown"), "{err}");
        assert!(err.contains("4.000") && err.contains("3.000"), "must name both values: {err}");

        // Boundary equality n*s == depth is also refused.
        let eq_err = cut_levels_with_finish(0.0, None, -3.0, 2.0, false, 3, 1.0)
            .expect_err("n*s == depth must refuse")
            .to_string();
        assert!(eq_err.contains("3.000"), "{eq_err}");
    }

    #[test]
    fn s8_check_8_face_levels() {
        let zs = face_levels(Some(0.0), None, -1.0, Some(0.6), 1, 0.3).expect("check 8");
        assert_zs(&zs, &[-0.6, -0.7, -1.0]);
    }

    #[test]
    fn s8_check_9_with_floor_allowance_shifted_bottom() {
        let zs = cut_levels_with_finish(0.0, None, -5.7, 2.0, false, 3, 0.25).expect("check 9");
        assert_zs(&zs, &[-2.0, -4.0, -4.95, -5.2, -5.45, -5.7]);
        assert_eq!(*zs.last().expect("last"), -5.7);
    }

    #[test]
    fn finish_stepdown_refusal_catches_negative_and_non_finite() {
        assert!(finish_stepdown_refusal(0.0).is_none());
        assert!(finish_stepdown_refusal(0.25).is_none());
        for bad in [-0.01, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let msg = finish_stepdown_refusal(bad).expect("bad finish_stepdown refused");
            assert!(msg.contains("finish_stepdown"), "{msg}");
        }
    }
}
