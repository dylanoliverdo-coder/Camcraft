//! Fork issue #6: synthetic pinning tests in the existing DXF suite.
//! Provenance: our own synthetic geometry and public ASCII DXF conventions.

use super::*;
use camcraft_app::machining2d::StraightContourRamp;

#[test]
fn a_selected_straight_ramp_keeps_its_seam_and_descends_once() {
    let (_, world, mut history) = sheet_world("straight-ramp");
    let id = plate_ids(&world).outer;
    let src = SrcRef::Selection(vec![GeoSel {
        entity: "plate".into(),
        feature: FeatureSel::SketchChain(id),
    }]);
    for (name, run_mm, direction_xy, needed) in [
        ("Along", 50.0, [0.0, 1.0], None),
        (
            "Too long",
            90.0,
            [0.0, 1.0],
            Some("matching compensated straight span"),
        ),
        (
            "Wrong direction",
            50.0,
            [0.0, -1.0],
            Some("matching compensated straight span"),
        ),
        ("Too steep", 0.2, [0.0, 1.0], Some("15 degrees or less")),
    ] {
        let mut p = stock_contour_params();
        if let OpParams::Contour {
            chain_overrides,
            climb,
            ..
        } = &mut p
        {
            *climb = true;
            *chain_overrides = vec![ChainOverride {
                entity: "plate".into(),
                chain: id,
                side: Some(ChainSide::Outside),
                reverse: false,
                open_side: OpenChainSide::None,
                straight_ramp: Some(StraightContourRamp {
                    start_xy: [-2.0, 0.0],
                    direction_xy,
                    drop_mm: 0.25,
                    run_mm,
                    normal_lead_mm: 0.25,
                }),
            }];
        }
        run(
            &world,
            &mut history,
            CreateOperation::new(&world, name, TOOL, src.clone(), p),
            "create outline",
        );
        let generated = GenerateOperation::new(&world, name);
        if let Some(required) = needed {
            let Err(e) = generated else {
                panic!("{name} generated");
            };
            assert!(e.to_string().contains(required), "{e}");
            continue;
        }
        run(&world, &mut history, generated, "generate outline");
        let e = model::find_by_name(&world, name).expect("outline");
        let cache = model::toolpath_of(&world, &e).expect("outline path");
        assert!(cache.warnings.is_empty(), "{:?}", cache.warnings);
        let moves = &cache.path.moves;
        let descending: Vec<_> = moves
            .iter()
            .filter(|m| m.kind == MoveKind::Feed && m.from.z > m.to.z)
            .collect();
        let [ramp] = descending.as_slice() else {
            panic!("unexpected ramp count");
        };
        assert!((ramp.from.x + 2.0).abs() < 1e-6);
        assert!(ramp.from.y.abs() < 1e-6);
        assert!((ramp.to.y - 50.0).abs() < 1e-6);
        assert!((ramp.from.z - ramp.to.z - 0.25).abs() < 1e-6);
        assert!(moves.iter().any(|m| m.arc.is_some()));
        assert!(!moves.iter().any(|m| m.kind == MoveKind::Feed
            && m.from.z == 8.0
            && (m.from.x + 2.0).abs() < 1e-8
            && m.from.y.abs() < 1e-8
            && m.to.y > 1.0));
    }
}

#[test]
fn a_straight_ramp_refuses_overlapping_and_ordinary_outside_neighbors() {
    for (tag, left, right, diagnostic) in [
        ("overlap", -10, 10, "outlines separated"),
        ("separate", 130, 140, "need their own ramp"),
    ] {
        let (dir, world, mut history) = sheet_world(tag);
        let dxf = format!(
            "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n\
             0\nLWPOLYLINE\n8\nneighbor\n90\n4\n70\n1\n10\n{left}\n20\n20\n10\n{right}\n20\n20\n\
             10\n{right}\n20\n40\n10\n{left}\n20\n40\n0\nENDSEC\n0\nEOF\n"
        );
        let drawing = write_dxf(&dir, "neighbor.dxf", &dxf);
        run(
            &world,
            &mut history,
            ImportDxf::new(&world, None, &drawing, &DxfOptions::default()),
            "neighbor",
        );
        let primary = plate_ids(&world).outer;
        let (_, imported) = sketch::find(&world, "neighbor").expect("neighbor sketch");
        let neighbor = imported.data.chains().by_layer("neighbor")[0].id();
        let src = SrcRef::Selection(
            [("plate", primary), ("neighbor", neighbor)]
                .into_iter()
                .map(|(name, chain)| GeoSel {
                    entity: name.into(),
                    feature: FeatureSel::SketchChain(chain),
                })
                .collect(),
        );
        let mut p = stock_contour_params();
        if let OpParams::Contour {
            side,
            chain_overrides,
            ..
        } = &mut p
        {
            *side = Side::Outside;
            *chain_overrides = vec![ChainOverride {
                entity: "plate".into(),
                chain: primary,
                side: None,
                reverse: false,
                open_side: OpenChainSide::None,
                straight_ramp: Some(StraightContourRamp {
                    start_xy: [-2.0, 0.0],
                    direction_xy: [0.0, 1.0],
                    drop_mm: 0.25,
                    run_mm: 50.0,
                    normal_lead_mm: 0.25,
                }),
            }];
        }
        run(
            &world,
            &mut history,
            CreateOperation::new(&world, "Mixed", TOOL, src, p),
            "neighbor and authored ramp",
        );
        let Err(e) = GenerateOperation::new(&world, "Mixed") else {
            panic!("{tag} neighbor generated");
        };
        assert!(e.to_string().contains(diagnostic), "{e}");
    }
}

#[test]
fn a_straight_ramp_refuses_an_open_companion() {
    let (_, world, mut history) = sheet_world("straight-ramp-open");
    let ids = plate_ids(&world);
    let src = SrcRef::Selection(
        [ids.outer, ids.open_line]
            .into_iter()
            .map(|chain| GeoSel {
                entity: "plate".into(),
                feature: FeatureSel::SketchChain(chain),
            })
            .collect(),
    );
    let mut p = stock_contour_params();
    if let OpParams::Contour {
        chain_overrides, ..
    } = &mut p
    {
        *chain_overrides = vec![ChainOverride {
            entity: "plate".into(),
            chain: ids.outer,
            side: Some(ChainSide::Outside),
            reverse: false,
            open_side: OpenChainSide::None,
            straight_ramp: Some(StraightContourRamp {
                start_xy: [-2.0, 0.0],
                direction_xy: [0.0, 1.0],
                drop_mm: 0.25,
                run_mm: 50.0,
                normal_lead_mm: 0.25,
            }),
        }];
    }
    run(
        &world,
        &mut history,
        CreateOperation::new(&world, "OpenCompanion", TOOL, src, p),
        "mixed ramp",
    );
    let Err(e) = GenerateOperation::new(&world, "OpenCompanion") else {
        panic!("mixed ramp generated");
    };
    assert!(e.to_string().contains("open chains cannot share"), "{e}");
}
