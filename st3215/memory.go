package st3215

const ADDR_PRESENT_POSITION = 0x38
const ADDR_PRESENT_CURRENT = 0x45
const ADDR_PRESENT_VELOCITY = 0x3A
const ADDR_GOAL_POSITION = 0x2A
const ADDR_STATUS = 0x40
const ADDR_TORQUE_ENABLE = 0x28

const SIGN_BIT_MASK = 0x8000
const MAX_ANGLE_STEP = 4095
const BUFFER_SIZE = 0x47

func GetMotorPosition(data []byte) uint16 {
	if len(data) < ADDR_PRESENT_POSITION+2 {
		return 0
	}

	var position = uint16(data[ADDR_PRESENT_POSITION]) | uint16(data[ADDR_PRESENT_POSITION+1])<<8
	if position&SIGN_BIT_MASK != 0 {
		var magnitude = position & MAX_ANGLE_STEP
		return (MAX_ANGLE_STEP + 1 - magnitude) & MAX_ANGLE_STEP
	} else {
		return position & MAX_ANGLE_STEP
	}
}

func GetMotorGoalPosition(data []byte) uint16 {
	if len(data) < ADDR_GOAL_POSITION+2 {
		return 0
	}

	torqueEnabled := data[ADDR_TORQUE_ENABLE] != 0
	if !torqueEnabled {
		return GetMotorPosition(data)
	}

	return uint16(data[ADDR_GOAL_POSITION]) | uint16(data[ADDR_GOAL_POSITION+1])<<8
}

func GetMotorCurrent(data []byte) uint16 {
	if len(data) < ADDR_PRESENT_CURRENT+2 {
		return 0
	}

	return uint16(data[ADDR_PRESENT_CURRENT]) | uint16(data[ADDR_PRESENT_CURRENT+1])<<8
}

func GetMotorVelocity(data []byte) uint16 {
	if len(data) < ADDR_PRESENT_VELOCITY+2 {
		return 0
	}

	return uint16(data[ADDR_PRESENT_VELOCITY]) | uint16(data[ADDR_PRESENT_VELOCITY+1])<<8
}

func IsError(data []byte) bool {
	if len(data) < BUFFER_SIZE {
		return true
	}

	return data[ADDR_STATUS] != 0
}

func IsTorqueEnabled(data []byte) bool {
	if len(data) < ADDR_TORQUE_ENABLE+1 {
		return false
	}

	return data[ADDR_TORQUE_ENABLE] != 0
}
