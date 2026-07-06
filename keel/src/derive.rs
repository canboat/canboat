//! Derivation of computed attributes - the Rust port of the C analyzer's
//! fill logic (fieldtype.c fillFieldType / getMinRange / getMaxRange /
//! fixupUnit / reservedCountForSize, analyzer-explain.c getMinimalPgnLength).
//!
//! Deliberately line-faithful, including C quirks (see QUIRKS.md):
//!  - min()/max() macros are `x <= y ? x : y`; NaN comparisons are false, so
//!    clamping a NaN yields the clamp value. Rust f64 comparisons behave
//!    identically, so the direct translation matches.
//!  - Range arithmetic runs in i128 before the final f64 conversion, exactly
//!    like the (arbitrary-precision) Python bootstrap the golden test proved.
//!
//! The migration golden test (emitted XML == analyzer-explain output) is what
//! proves this port correct across the whole database.

use crate::model::Database;

pub fn c_min(x: f64, y: f64) -> f64 {
    if x <= y { x } else { y } // analyzer.h: ((x) <= (y) ? (x) : (y))
}

pub fn c_max(x: f64, y: f64) -> f64 {
    if x >= y { x } else { y }
}

/// fieldtype.c reservedCountForSize(): top-of-range sentinels by bit width.
pub fn reserved_count_for_size(size: u32) -> u32 {
    match size {
        s if s >= 8 => 3,
        s if s >= 4 => 2,
        s if s >= 2 => 1,
        _ => 0,
    }
}

pub fn get_min_range(size: u32, resolution: f64, sign: bool, offset: i32) -> f64 {
    let highbit = if sign && offset == 0 { size - 1 } else { size };
    if !sign || offset != 0 {
        (offset as f64) * resolution
    } else {
        (((1i128 << highbit) - 1) as f64) * resolution * -1.0
    }
}

pub fn get_max_range(
    size: u32,
    resolution: f64,
    sign: bool,
    offset: i32,
    lookup: Option<&crate::model::Lookup>,
    specialvalues: u32,
) -> f64 {
    let highbit = if sign && offset == 0 { size - 1 } else { size };
    let mut max_value: i128 = if highbit >= 64 {
        u64::MAX as i128
    } else {
        (1i128 << highbit) - 1
    } - specialvalues as i128;
    if offset != 0 {
        max_value += offset as i128;
    }
    if let Some(lk) = lookup {
        if lk.kind == "pair" && !lk.pairs.is_empty() {
            // A lookup naming values in the sentinel region raises rangeMax
            let named_max = lk.pairs.iter().map(|(v, _)| *v as i128).max().unwrap();
            max_value = max_value.max(named_max);
        }
    }
    (max_value as f64) * resolution
}

/// Field types (by ROOT name) whose pgn.h field macros set no resolution;
/// every other macro sets an explicit resolution (usually 1).
const NO_DEFAULT_RESOLUTION_ROOTS: [&str; 5] = [
    "STRING_FIX",
    "STRING_LZ",
    "STRING_LAU",
    "VARIABLE",
    "DYNAMIC_FIELD_VALUE",
];

pub fn default_field_resolution(root_name: &str) -> f64 {
    if NO_DEFAULT_RESOLUTION_ROOTS.contains(&root_name) {
        0.0
    } else {
        1.0
    }
}

/// Port of fieldtype.c fillFieldType() part 1: percolate physical quantities
/// and base types, compute type-level ranges, resolve root names.
pub fn fill_fieldtypes(db: &mut Database) -> Result<(), String> {
    let pq_units: std::collections::HashMap<String, (Option<String>, Option<String>)> = db
        .physical_quantities
        .iter()
        .map(|pq| (pq.name.clone(), (pq.unit.clone(), pq.url.clone())))
        .collect();

    for ft in db.fieldtypes.iter_mut() {
        if let Some(phys) = &ft.physical {
            let (unit, url) = pq_units
                .get(phys)
                .ok_or_else(|| format!("FieldType '{}' has unlisted physical quantity '{phys}'", ft.name))?;
            if ft.unit.is_none() {
                ft.unit = unit.clone();
            }
            if ft.url.is_none() {
                ft.url = url.clone();
            }
        } else if ft.unit.is_some() {
            return Err(format!(
                "FieldType '{}' has unit '{}' but no physical quantity",
                ft.name,
                ft.unit.as_ref().unwrap()
            ));
        }
    }

    let mut seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for i in 0..db.fieldtypes.len() {
        if let Some(base_name) = db.fieldtypes[i].base.clone() {
            let b = *seen
                .get(&base_name)
                .ok_or_else(|| format!("baseFieldType '{base_name}' must precede '{}'", db.fieldtypes[i].name))?;
            let base = db.fieldtypes[b].clone(); // base is already resolved
            let ft = &mut db.fieldtypes[i];
            if ft.physical.is_none() {
                ft.physical = base.physical.clone();
            }
            if ft.has_sign.is_none() && base.has_sign.is_some() {
                ft.has_sign = base.has_sign;
            }
            if ft.unit.is_none() && base.unit.is_some() {
                ft.unit = base.unit.clone();
            }
            if ft.size == 0 && base.size != 0 {
                ft.size = base.size;
            }
            if ft.resolution == 0.0 && base.resolution != 0.0 {
                ft.resolution = base.resolution;
            } else if ft.resolution != 0.0 && base.resolution != 0.0 && ft.resolution != base.resolution {
                return Err(format!("Cannot overrule resolution of '{}' in '{}'", base.name, ft.name));
            }
            if ft.print_function.is_none() {
                ft.print_function = base.print_function.clone();
            }
            db.fieldtypes[i].root_name = base.root_name.clone();
            db.fieldtypes[i].root_sentinels = base.root_sentinels.clone();
        } else {
            db.fieldtypes[i].root_name = db.fieldtypes[i].name.clone();
            db.fieldtypes[i].root_sentinels = db.fieldtypes[i].sentinels.clone();
        }

        let ft = &mut db.fieldtypes[i];
        // NB: an explicit .rangeMax initializer trips the C `rangeMax == 0.0`
        // guard and lands in the NaN branch (QUIRKS.md Q11).
        let authored_inert = !matches!(ft.range_max_authored, None | Some(0.0));
        if ft.size != 0 && ft.resolution != 0.0 && ft.has_sign.is_some() && !authored_inert {
            ft.range_min = get_min_range(ft.size, ft.resolution, ft.has_sign.unwrap(), ft.offset);
            ft.range_max = get_max_range(
                ft.size,
                ft.resolution,
                ft.has_sign.unwrap(),
                ft.offset,
                None,
                reserved_count_for_size(ft.size),
            );
        } else {
            ft.range_min = f64::NAN;
            ft.range_max = f64::NAN;
        }
        seen.insert(db.fieldtypes[i].name.clone(), i);
    }
    Ok(())
}

/// Port of fieldtype.c fixupUnit(), SI branch only (showSI = true in explain).
fn fixup_unit(unit: &str, has_sign: bool, range_min: &mut f64, range_max: &mut f64) {
    if unit == "rad" {
        if has_sign {
            *range_min = c_max(*range_min, -3.1415926);
            *range_max = c_min(*range_max, 3.1415926);
        } else {
            *range_max = c_min(*range_max, 2.0 * 3.1415926);
        }
    }
}

/// Port of fieldtype.c fillFieldType() per-field body plus
/// analyzer-explain.c getMinimalPgnLength(), for every PGN.
pub fn fill(db: &mut Database) -> Result<(), String> {
    db.index();
    fill_fieldtypes(db)?;

    // Work around simultaneous &mut pgns / &fieldtypes borrows: take the list.
    let mut pgns = std::mem::take(&mut db.pgns);
    for pgn in pgns.iter_mut() {
        let mut order = 0u32;
        for f in pgn.fields.iter_mut() {
            order += 1;
            let fti = db.fieldtype(&f.type_)?;
            let ft = &db.fieldtypes[fti];
            f.ft = fti;
            let f_has_sign = ft.has_sign == Some(true);

            // resolution: authored -> fieldtype -> macro default
            let res = if let Some(r) = f.resolution {
                r
            } else if ft.resolution != 0.0 {
                ft.resolution
            } else {
                default_field_resolution(&ft.root_name)
            };
            if ft.resolution != 0.0 && ft.resolution != res {
                return Err(format!(
                    "PGN {} field '{}': cannot overrule resolution of '{}'",
                    pgn.pgn, f.name, ft.name
                ));
            }
            f.res_resolution = res;

            // size
            let mut bits = f.bits.unwrap_or(0);
            if ft.size != 0 && bits == 0 {
                bits = ft.size;
            }
            if ft.size != 0 && ft.size != bits {
                return Err(format!(
                    "PGN {} field '{}': cannot overrule size of '{}'",
                    pgn.pgn, f.name, ft.name
                ));
            }
            f.res_bits = bits;

            // offset
            let mut offset = f.offset.unwrap_or(0);
            if ft.offset != 0 && offset == 0 {
                offset = ft.offset;
            }
            if ft.offset != 0 && ft.offset != offset {
                return Err(format!(
                    "PGN {} field '{}': cannot overrule offset of '{}'",
                    pgn.pgn, f.name, ft.name
                ));
            }
            f.res_offset = offset;

            // unit
            f.res_unit = f.unit.clone().or_else(|| ft.unit.clone());

            // ranges: authored values behave like explicit C initializers
            f.res_range_min = f.range_min.unwrap_or(0.0);
            f.res_range_max = f.range_max.unwrap_or(0.0);
            if f.res_range_max.is_nan() || f.res_range_max == 0.0 {
                f.res_range_min = ft.range_min;
                f.res_range_max = ft.range_max;
            }
            if let Some(unit) = &f.res_unit {
                if f.res_resolution != 0.0 {
                    fixup_unit(unit, f_has_sign, &mut f.res_range_min, &mut f.res_range_max);
                }
            }

            let by_size = reserved_count_for_size(f.res_bits);
            let count = f.special_values.unwrap_or(by_size);

            let pair_lookup = f.lookup.as_ref().and_then(|n| db.lookups.get(n));
            let ft_has_sign = ft.has_sign;
            if f.res_bits != 0 && f.res_resolution != 0.0 && ft_has_sign.is_some() && f.res_range_max.is_nan() {
                f.res_range_min = get_min_range(f.res_bits, f.res_resolution, f_has_sign, f.res_offset);
                f.res_range_max =
                    get_max_range(f.res_bits, f.res_resolution, f_has_sign, f.res_offset, pair_lookup, count);
            }

            // reservedCount (fieldtype.c lines 448-462)
            if f.special_values.is_some() {
                f.reserved_count = count;
            } else if f.res_bits != 0 && f.res_bits < 64 && f.res_resolution > 0.0 && !f.res_range_max.is_nan() {
                let raw_max: u64 = (1u64 << f.res_bits) - 1;
                let raw_range_max = (f.res_range_max / f.res_resolution + 0.5) as u64;
                f.reserved_count = if raw_range_max >= raw_max {
                    0
                } else {
                    ((raw_max - raw_range_max).min(by_size as u64)) as u32
                };
            } else {
                f.reserved_count = by_size;
            }

            f.order = order;
        }

        fill_pgn_length(pgn)?;
    }
    db.pgns = pgns;
    Ok(())
}

/// Port of analyzer-explain.c getMinimalPgnLength().
fn fill_pgn_length(pgn: &mut crate::model::Pgn) -> Result<(), String> {
    let field_count = pgn.fields.len() as u32;
    let mut counted = field_count as usize;
    let mut is_variable = false;
    if let Some(rep1) = &pgn.repeating1 {
        if rep1.count > 0 {
            let rep2 = pgn.repeating2.as_ref().map(|r| r.count).unwrap_or(0);
            counted -= (rep1.count + rep2) as usize;
            is_variable = true;
        }
    }

    let mut length_bits = 0u32;
    for f in &pgn.fields[..counted] {
        if f.res_bits == 0 {
            is_variable = true;
        } else {
            length_bits += f.res_bits;
        }
    }
    if length_bits % 8 != 0 {
        return Err(format!(
            "PGN {} '{}' has a length of {} bits that does not fill bytes exactly",
            pgn.pgn, pgn.description, length_bits
        ));
    }
    pgn.field_count = field_count;
    pgn.length = length_bits / 8;
    pgn.is_variable = is_variable;
    Ok(())
}
