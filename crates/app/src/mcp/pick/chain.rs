//! Per-chain picks through the operation geometry command.
//! Provenance: the existing DXF pick contract and original ramp extension.

use super::{feature_id, parse_feature_id, req_pick_target, req_str};
use crate::machining2d::StraightContourRamp;
use crate::operation::{ChainOverride, ChainSide, FeatureSel, GeoSel, OpenChainSide};
use crate::script::Env;
use serde_json::{Value, json};

fn parse_chain_side(v: &Value) -> Result<Option<ChainSide>, String> {
    match v {
        Value::Null => Ok(None),
        Value::String(s) if s == "inside" => Ok(Some(ChainSide::Inside)),
        Value::String(s) if s == "outside" => Ok(Some(ChainSide::Outside)),
        other => Err(format!(
            "\"side\" must be \"inside\", \"outside\", or null, got {other}"
        )),
    }
}

fn parse_open_side(v: &Value) -> Result<OpenChainSide, String> {
    match v {
        Value::Null => Ok(OpenChainSide::None),
        Value::String(s) if s == "none" => Ok(OpenChainSide::None),
        Value::String(s) if s == "left" => Ok(OpenChainSide::Left),
        Value::String(s) if s == "right" => Ok(OpenChainSide::Right),
        other => Err(format!(
            "\"open_side\" must be \"none\", \"left\", \"right\", or null, got {other}"
        )),
    }
}

pub(super) struct ContourPick {
    pub(super) sel: GeoSel,
    pub(super) override_spec: Option<ChainOverride>,
}

pub(super) fn contour_pick_of(env: &Env, v: &Value) -> Result<ContourPick, String> {
    let Value::Object(m) = v else {
        return Err(format!(
            "a pick is an object {{\"model\", \"id\"}}, got {v}"
        ));
    };
    if let Some(k) = m.keys().find(|k| {
        !matches!(
            k.as_str(),
            "model" | "id" | "side" | "reverse" | "open_side" | "straight_ramp"
        )
    }) {
        return Err(format!(
            "unexpected pick key \"{k}\" - allowed: model, id, side, reverse, open_side, straight_ramp"
        ));
    }
    let entity = req_str(m, "model")?;
    req_pick_target(env, entity)?;
    let feature = parse_feature_id(req_str(m, "id")?)?;
    let pick = GeoSel {
        entity: entity.to_owned(),
        feature,
    };
    crate::pick::check_pickable(&env.world, &pick)?;
    let side = match m.get("side") {
        None => None,
        Some(sv) => parse_chain_side(sv)?,
    };
    let reverse = match m.get("reverse") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(other) => return Err(format!("\"reverse\" must be a boolean, got {other}")),
    };
    let open_side = match m.get("open_side") {
        None => OpenChainSide::None,
        Some(ov) => parse_open_side(ov)?,
    };
    let straight_ramp: Option<StraightContourRamp> = m
        .get("straight_ramp")
        .filter(|value| !value.is_null())
        .map(|value| {
            serde_json::from_value(value.clone()).map_err(|e| format!("invalid straight_ramp: {e}"))
        })
        .transpose()?;
    if straight_ramp.is_some_and(|r| !r.is_sane() || open_side != OpenChainSide::None) {
        return Err("invalid straight_ramp geometry or open-side combination".into());
    }
    let has_override =
        side.is_some() || reverse || open_side != OpenChainSide::None || straight_ramp.is_some();
    let override_spec = if has_override {
        let FeatureSel::SketchChain(chain) = pick.feature else {
            return Err(format!(
                "per-chain overrides require a sketch_chain id, not '{}'",
                feature_id(&pick.feature)
            ));
        };
        Some(ChainOverride {
            entity: pick.entity.clone(),
            chain,
            side,
            reverse,
            open_side,
            straight_ramp,
        })
    } else {
        None
    };
    Ok(ContourPick {
        sel: pick,
        override_spec,
    })
}

pub(super) fn straight_ramp_schema() -> Value {
    let xy = json!({"type": "array", "minItems": 2, "maxItems": 2, "items": {"type": "number"}});
    json!({
        "type": "object", "additionalProperties": false,
        "description": "Optional 2D Contour ramp on a closed outside outline. The cutter-center start and unit travel direction are in the sketch plane. Every selected exterior needs its own ramp and cutter clearance from the others.",
        "properties": {
            "start_xy": xy, "direction_xy": xy,
            "drop_mm": {"type": "number", "exclusiveMinimum": 0},
            "run_mm": {"type": "number", "exclusiveMinimum": 0},
            "normal_lead_mm": {"type": "number", "exclusiveMinimum": 0}
        },
        "required": ["start_xy", "direction_xy", "drop_mm", "run_mm", "normal_lead_mm"]
    })
}
