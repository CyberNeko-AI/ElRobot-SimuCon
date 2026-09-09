use super::units::normal_position;

#[test]
fn test_normal_position_positive_values() {
    // Test positive values within range
    assert_eq!(normal_position(0), 0);
    assert_eq!(normal_position(1), 1);
    assert_eq!(normal_position(100), 100);
    assert_eq!(normal_position(4095), 4095);

    // Test positive values that need masking
    assert_eq!(normal_position(4096), 0);
    assert_eq!(normal_position(4097), 1);
    assert_eq!(normal_position(8191), 4095);
}

#[test]
fn test_normal_position_negative_values() {
    // Test negative values (sign bit set)
    // -1 (0x8001) should become 4095
    assert_eq!(normal_position(0x8001), 4095);

    // -2 (0x8002) should become 4094
    assert_eq!(normal_position(0x8002), 4094);

    // -10 (0x800A) should become 4086
    assert_eq!(normal_position(0x800A), 4086);

    // -100 (0x8064) should become 3996
    assert_eq!(normal_position(0x8064), 3996);

    // -4095 (0x8FFF) should become 1
    assert_eq!(normal_position(0x8FFF), 1);

    // -4096 (0x9000) should become 0
    assert_eq!(normal_position(0x9000), 0);
}

#[test]
fn test_normal_position_edge_cases() {
    // Test maximum positive value before sign bit
    assert_eq!(normal_position(0x7FFF), 4095);

    // Test sign bit with zero magnitude
    assert_eq!(normal_position(0x8000), 0);

    // Test all bits set
    assert_eq!(normal_position(0xFFFF), 1);
}

#[test]
fn test_normal_position_wraparound() {
    // Test that values wrap around correctly
    for i in 0..4096u16 {
        let result = normal_position(i);
        assert!(
            result <= 4095,
            "Value {} produced result {} which is out of range",
            i,
            result
        );
        assert_eq!(result, i & 4095);
    }

    // Test negative wraparound
    for i in 1..4096u16 {
        let negative_value = 0x8000 | i;
        let result = normal_position(negative_value);
        assert!(
            result <= 4095,
            "Negative value {} produced result {} which is out of range",
            negative_value,
            result
        );
        assert_eq!(result, (4096 - i) & 4095);
    }
}
