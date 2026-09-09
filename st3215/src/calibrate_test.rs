#[cfg(test)]
use super::calibrate::*;
use std::collections::BTreeSet;

#[test]
fn test_simple_direct_arc() {
    let readings: BTreeSet<u16> = vec![100, 200, 300, 400].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 100);
    assert_eq!(result.max, 400);
}

#[test]
fn test_direct_arc_large_range() {
    let readings: BTreeSet<u16> = vec![500, 1000, 1500, 2000, 2500].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 500);
    assert_eq!(result.max, 2500);
}

#[test]
fn test_wrap_around_basic() {
    let readings: BTreeSet<u16> = vec![100, 200, 3900, 4000].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3900);
    assert_eq!(result.max, 200);
}

#[test]
fn test_wrap_around_large_gap() {
    let readings: BTreeSet<u16> = vec![50, 100, 150, 3950, 4000, 4050].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3950);
    assert_eq!(result.max, 150);
}

#[test]
fn test_direct_arc_near_boundary() {
    let readings: BTreeSet<u16> = vec![3800, 3900, 4000, 4050].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3800);
    assert_eq!(result.max, 4050);
}

#[test]
fn test_evenly_distributed() {
    let readings: BTreeSet<u16> = vec![0, 1024, 2048, 3072].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 0);
    assert_eq!(result.max, 3072);
}

#[test]
fn test_dense_with_large_gap() {
    let readings: BTreeSet<u16> = vec![10, 20, 30, 40, 50, 3990, 4000, 4010, 4020]
        .into_iter()
        .collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3990);
    assert_eq!(result.max, 50);
}

#[test]
fn test_two_points_direct() {
    let readings: BTreeSet<u16> = vec![1000, 2000].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 1000);
    assert_eq!(result.max, 2000);
}

#[test]
fn test_two_points_wrap() {
    let readings: BTreeSet<u16> = vec![100, 4000].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 4000);
    assert_eq!(result.max, 100);
}

#[test]
fn test_multiple_gaps_wrap() {
    let readings: BTreeSet<u16> = vec![50, 100, 150, 200, 3850, 3900, 3950, 4000]
        .into_iter()
        .collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3850);
    assert_eq!(result.max, 200);
}

#[test]
fn test_edge_case_boundaries() {
    let readings: BTreeSet<u16> = vec![0, 100, 3995, 4095].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 3995);
    assert_eq!(result.max, 100);
}

#[test]
fn test_clustered_readings() {
    let readings: BTreeSet<u16> = vec![1000, 1001, 1002, 1003, 1004].into_iter().collect();
    let result = calculate_arc(&readings);
    assert_eq!(result.min, 1000);
    assert_eq!(result.max, 1004);
}
