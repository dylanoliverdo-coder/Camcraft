//! The shared writers of `OpParams::stamp_with` and its link trailer
//! (#1849 split). Provenance: moved verbatim from `params.rs`; our own
//! digest law.

use super::*;

#[path = "digest_approach.rs"]
mod approach;

/// Pass starts at (#3316, #3354, #3355): written only off Auto, so every
/// earlier operation keeps the digest it had. One byte pair for every kind.
pub(super) fn write_pass_start(s: crate::machining2d::PassStart, h: &mut std::hash::DefaultHasher) {
    match s {
        crate::machining2d::PassStart::Auto => {}
        crate::machining2d::PassStart::Inside => h.write_u8(0xC1),
        crate::machining2d::PassStart::Outside => h.write_u8(0xC2),
    }
}

/// Corner relief (#2400): written only when non-default, so every
/// operation without relief keeps its digest.
pub(super) fn write_corner_relief(
    cr: &crate::machining2d::CornerRelief,
    h: &mut std::hash::DefaultHasher,
) {
    if cr.is_off() {
        return;
    }
    h.write_u8(0xC7);
    h.write_u8(match cr.kind {
        crate::machining2d::CornerReliefKind::None => 0,
        crate::machining2d::CornerReliefKind::Dogbone => 1,
        crate::machining2d::CornerReliefKind::TBone => 2,
    });
    h.write_u8(match cr.tbone_direction {
        crate::machining2d::TBoneDirection::LongEdge => 0,
        crate::machining2d::TBoneDirection::ShortEdge => 1,
        crate::machining2d::TBoneDirection::Horizontal => 2,
        crate::machining2d::TBoneDirection::Vertical => 3,
    });
    h.write_usize(cr.overrides.len());
    for ov in &cr.overrides {
        w(h, ov.at.x);
        w(h, ov.at.y);
        h.write_u8(match ov.choice {
            crate::machining2d::CornerReliefChoice::None => 0,
            crate::machining2d::CornerReliefChoice::Dogbone => 1,
            crate::machining2d::CornerReliefChoice::TBoneFirst => 2,
            crate::machining2d::CornerReliefChoice::TBoneSecond => 3,
            crate::machining2d::CornerReliefChoice::TBoneX => 4,
            crate::machining2d::CornerReliefChoice::TBoneY => 5,
        });
    }
}

/// The arc-turn radius (#1759), after the blend tag: written only when
/// set, so every earlier blended operation keeps its digest.
pub(super) fn write_blend_radius(h: &mut std::hash::DefaultHasher, r: Option<f64>) {
    if let Some(r) = r {
        h.write_u8(0xB4);
        w(h, r);
    }
}

/// The blend tag and its radius, written only for arc turns: a legacy
/// (sharp) operation keeps the digest it was cached under.
pub(super) fn write_blend(h: &mut std::hash::DefaultHasher, on: bool, r: Option<f64>) {
    if on {
        h.write_u8(0xB1);
        write_blend_radius(h, r);
    }
}

/// Rest machining (#1489, #2033, #1446): written only when ON, so
/// operations without rest machining keep their digest. `entered` is
/// `true` on 2D Pocket (`0xD7` + previous tool fields) and `false` on
/// 2D Adaptive (`0xD9` + allowance).
pub(super) fn write_rest(
    rest: &Option<crate::machining2d::RestParams>,
    h: &mut std::hash::DefaultHasher,
    entered: bool,
) {
    let Some(r) = rest else {
        return;
    };
    if entered {
        h.write_u8(0xD7);
        for v in [r.previous_tool_diameter, r.previous_corner_radius, r.previous_stock_to_leave, r.allowance] {
            w(h, v);
        }
    } else {
        h.write_u8(0xD9);
        w(h, r.allowance);
    }
    if r.source == crate::machining2d::RestSource::PreviousOperations {
        h.write_u8(0xD8);
    }
}

/// One scalar, as its bit pattern.
pub(super) fn w(h: &mut std::hash::DefaultHasher, v: f64) {
    h.write(&v.to_bits().to_le_bytes());
}

/// Per-chain DXF sketch overrides (#1864 / Journeyman spec S9 & S10): written
/// only when non-empty, and `open_side` only when non-`None`, so every earlier
/// operation keeps its cached digest byte for byte.
pub(super) fn write_chain_overrides(
    chain_overrides: &[ChainOverride],
    h: &mut std::hash::DefaultHasher,
) {
    if chain_overrides.is_empty() {
        return;
    }
    h.write(b"chain-overrides");
    let mut sorted: Vec<_> = chain_overrides.iter().collect();
    sorted.sort_by(|a, b| a.entity.cmp(&b.entity).then(a.chain.cmp(&b.chain)));
    h.write_usize(sorted.len());
    for o in sorted {
        h.write_usize(o.entity.len());
        h.write(o.entity.as_bytes());
        h.write(&o.chain.0.to_le_bytes());
        h.write_u8(match o.side {
            None => 0,
            Some(ChainSide::Inside) => 1,
            Some(ChainSide::Outside) => 2,
        });
        h.write_u8(u8::from(o.reverse));
        if o.open_side != OpenChainSide::None {
            h.write(b"open-side");
            h.write_u8(match o.open_side {
                OpenChainSide::None => 0,
                OpenChainSide::Left => 1,
                OpenChainSide::Right => 2,
            });
        }
        if let Some(ramp) = o.straight_ramp {
            h.write(b"straight-ramp");
            for word in ramp.words() {
                h.write_u64(word);
            }
        }
    }
}

/// The wall-pass fields of a Contour (Journeyman spec S1): written only
/// when they differ from the single pass, so every earlier contour keeps
/// its cached digest.
pub(super) fn write_wall_passes(p: &crate::machining2d::WallPasses, h: &mut std::hash::DefaultHasher) {
    if *p == crate::machining2d::WallPasses::default() {
        return;
    }
    h.write(b"wall-passes");
    h.write_u32(p.rough_passes);
    w(h, p.rough_spacing);
    h.write_u32(p.finish_passes);
    w(h, p.finish_spacing);
    h.write_u32(p.spring_passes);
    h.write_u8(match p.finish_level {
        crate::machining2d::FinishLevel::Every => 0,
        crate::machining2d::FinishLevel::Last => 1,
    });
}

/// Wall finishing passes of a 2D Adaptive operation (Journeyman spec S5):
/// written only off default so every earlier op keeps its cached digest.
pub(super) fn write_adaptive_finish(p: &crate::machining2d::AdaptiveFinish, h: &mut std::hash::DefaultHasher) {
    if p.is_default() { return; }
    h.write(b"adaptive-finish");
    h.write_u32(p.finish_passes);
    w(h, p.finish_spacing);
    h.write_u32(p.spring_passes);
    h.write_u8(u8::from(p.finish_level == crate::machining2d::FinishLevel::Last));
}

/// Wall finishing passes of a 2D Pocket (Journeyman spec S4): omitted at default.
pub(super) fn write_pocket_finish(p: &crate::machining2d::PocketFinish, h: &mut std::hash::DefaultHasher) {
    if p.is_default() { return; }
    h.write(b"pocket-finish");
    h.write_u32(p.finish_passes);
    w(h, p.finish_spacing);
    h.write_u32(p.spring_passes);
    h.write_u8(match p.finish_direction {
        crate::machining2d::FinishDirection::Same => 0,
        crate::machining2d::FinishDirection::Climb => 1,
        crate::machining2d::FinishDirection::Conventional => 2,
    });
    h.write_u8(u8::from(p.finish_level == crate::machining2d::FinishLevel::Last));
}

/// A height: its reference (tag, plus slot for a work offset) and its
/// offset bits; a non-Fixed reference also writes its resolved Z (or an
/// unresolved marker) - `stamp_with` says why.
pub(super) fn wh(
    world: &World,
    setup: Option<&str>,
    src: Option<&SrcRef>,
    h: &mut std::hash::DefaultHasher,
    v: Height,
) {
    let (tag, slot) = match v.reference {
        HeightReference::Fixed => (0u8, 0u8),
        HeightReference::TopOfStock => (1, 0),
        HeightReference::BottomOfStock => (2, 0),
        HeightReference::TopOfModel => (3, 0),
        HeightReference::BottomOfModel => (4, 0),
        // #329: the slot is the setup's; digesting the resolved
        // one keeps every migrated stamp exactly as it was.
        HeightReference::WorkOffset => (5, u8::try_from(crate::setup::wcs_slot(world, setup)).unwrap_or(u8::MAX)),
        HeightReference::TopOfSelection => (6, 0),
        HeightReference::BottomOfSelection => (7, 0),
    };
    h.write_u8(tag);
    h.write_u8(slot);
    w(h, v.offset);
    if tag != 0 {
        match resolve_height(world, setup, src, v) {
            Some(z) => {
                h.write_u8(1);
                w(h, z);
            }
            None => h.write_u8(0),
        }
    }
}

/// An optional height: presence first, then the height.
pub(super) fn wh_opt(
    world: &World,
    setup: Option<&str>,
    src: Option<&SrcRef>,
    h: &mut std::hash::DefaultHasher,
    v: Option<Height>,
) {
    match v {
        Some(v) => {
            h.write_u8(1);
            wh(world, setup, src, h, v);
        }
        None => h.write_u8(0),
    }
}

/// A 2D point, x then y.
pub(super) fn wp2(h: &mut std::hash::DefaultHasher, p: Point2) {
    w(h, p.x);
    w(h, p.y);
}

/// The 3+2 index (M5.1 stage B): a tilt changes every move the
/// generator makes, so it changes the digest. Presence first, then the
/// basis version (omitted for legacy version 0), then the two letters
/// and the anchor. A basis change invalidates an incompatible cache.
pub(super) fn wt(h: &mut std::hash::DefaultHasher, t: Option<Tilt>) {
    match t {
        Some(t) => {
            h.write_u8(1);
            // Legacy tilts keep their original digest and frame. A corrected
            // basis must never accept a cache made under the legacy rule.
            if t.basis_version != 0 {
                h.write_u8(0xB1);
                h.write_u8(t.basis_version);
            }
            w(h, t.a_deg);
            w(h, t.b_deg);
            for v in t.origin {
                w(h, v);
            }
        }
        None => h.write_u8(0),
    }
}

impl OpParams {
    /// The link trailer `stamp_with` writes after every kind's arm.
    pub(super) fn stamp_links(&self, h: &mut std::hash::DefaultHasher) {
        // M11.2 C/D: the level transitions and links change the moves,
        // so they change the digest - appended only when they differ
        // from the kind's pre-M11.2 shape, so every op that never
        // touched them keeps the digest it was cached under. (A contour
        // saved with the old stay-down ramp goes Stale once and
        // regenerates the same path - the mechanism saying so.)
        // (The runtime centre-cutting flag is the tool's, never the op's.)
        if let Some(l) = self.links().map(|l| crate::links::Links {
            approach: crate::links::Approach { center_cutting: true, ..l.approach },
            ..*l
        })
            && l != self.default_links()
        {
            h.write_u8(0xC2);
            let link = |k: crate::links::Link| match k {
                crate::links::Link::Clearance => 0u8,
                crate::links::Link::Retract => 1,
                crate::links::Link::KeepDown => 2,
            };
            h.write_u8(match l.transition {
                None => 0,
                Some(Entry3D::Plunge) => 1,
                Some(Entry3D::Ramp) => 2,
                Some(Entry3D::Helix) => 3,
                Some(Entry3D::Arc) => 4,
            });
            h.write_u8(link(l.level));
            h.write_u8(link(l.pass));
            for v in [l.ramp_angle, l.helix_pitch, l.max_stay_down] {
                w(h, v);
            }
            approach::write(h, l.approach);
        }
    }
}
