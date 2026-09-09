use std::collections::BTreeSet;

#[derive(Debug)]
pub struct MotorArc {
    pub min: u16,
    pub max: u16,
}

const FULL_RANGE: i32 = 4096;

pub fn calculate_arc(measures_set: &BTreeSet<u16>) -> MotorArc {
    if measures_set.len() < 2 {
        if measures_set.is_empty() {
            return MotorArc { min: 0, max: 0 };
        }
        let val = *measures_set.iter().next().unwrap();
        return MotorArc { min: val, max: val };
    }

    let min_reading = *measures_set.iter().next().unwrap();
    let max_reading = *measures_set.iter().next_back().unwrap();

    // Calculate gaps between consecutive points (including wrap-around)
    let mut gaps: Vec<i32> = Vec::with_capacity(measures_set.len());
    let mut gap_start_points: Vec<u16> = Vec::with_capacity(measures_set.len());

    let mut it = measures_set.iter().peekable();
    while let Some(current) = it.next() {
        gap_start_points.push(*current);
        if let Some(next) = it.peek() {
            gaps.push(**next as i32 - *current as i32);
        } else {
            // This is the last element, calculate wrap-around gap
            let gap = FULL_RANGE - *current as i32 + min_reading as i32;
            gaps.push(gap);
        }
    }

    // Find largest and second largest gaps
    let mut gap_indices: Vec<usize> = (0..gaps.len()).collect();
    gap_indices.sort_unstable_by(|&i, &j| gaps[j].cmp(&gaps[i]));

    let largest_gap_idx = gap_indices[0];
    let largest_gap_size = gaps[largest_gap_idx];
    let second_largest_gap_size = if gaps.len() > 1 {
        gaps[gap_indices[1]]
    } else {
        0
    };

    // Determine arc type based on gap analysis
    if largest_gap_idx == gaps.len() - 1 {
        // Wrap-around gap is largest - indicates direct arc
        MotorArc {
            min: min_reading,
            max: max_reading,
        }
    } else {
        // Largest gap is between consecutive readings
        let gap_before = gap_start_points[largest_gap_idx];
        let gap_after = gap_start_points[(largest_gap_idx + 1) % gap_start_points.len()];

        // Check if this is clearly a wrap-around arc
        let is_clearly_wrap_around = largest_gap_size > second_largest_gap_size * 3;

        if is_clearly_wrap_around {
            // Wrap-around arc: gap indicates where arc does NOT go
            let actual_min = gap_after;
            let actual_max = gap_before;

            MotorArc {
                min: actual_min,
                max: actual_max,
            }
        } else {
            // Gaps are evenly distributed - direct arc
            MotorArc {
                min: min_reading,
                max: max_reading,
            }
        }
    }
}
