package ar3

import (
	"fmt"
	"testing"
)

// TestMockArmInterface checks that the AR3simulate struct implements the Arm
// interface
func TestMockArmInterface(t *testing.T) {
	val := AR3simulate{}
	_, ok := interface{}(&val).(Arm)
	if !ok {
		t.Errorf("Failed. Ar3exec does not implement the Arm interface")
	}
}

func TestConnectMock(t *testing.T) {
	arm := ConnectMock()
	if arm.Echo() != nil {
		t.Errorf("Mock echo should always connect")
	}
}

func TestAR3simulate_SetDirections(t *testing.T) {
	arm := ConnectMock()
	arm.SetDirections([7]bool{true, false, true, false, true, false, true})
	if arm.GetDirections() != [7]bool{true, false, true, false, true, false, true} {
		t.Errorf("GetDirections should be equivalent to SetDirections")
	}
}

func TestAR3simulate_CurrentStepperPosition(t *testing.T) {
	arm := ConnectMock()
	currentStepperPositions := arm.CurrentStepperPosition()
	if currentStepperPositions != [7]int{0, 0, 0, 0, 0, 0, 0} {
		t.Errorf("Steppers should be equivalent to [7]int{0, 0, 0, 0, 0, 0, 0}. Got %v", currentStepperPositions)
	}
}

func TestAR3simulate_CurrentJointRadians(t *testing.T) {
	arm := ConnectMock()
	currentJoints := arm.CurrentJointRadians()
	if currentJoints != [7]float64{0, 0, 0, 0, 0, 0, 0} {
		t.Errorf("Joints should be equivalent to [7]float64{0, 0, 0, 0, 0, 0, 0}. Got %v", currentJoints)
	}
}

func TestAR3simulate_CurrentPose(t *testing.T) {
	arm := ConnectMock()
	currentPose := arm.CurrentPose()
	// x86 and ARM systems calculate kinematics slightly differently.
	if fmt.Sprintf("%5f", currentPose.Position.X) != "323.080000" {
		t.Errorf("X pose should be equivalent to 323.080000. Got %5f", currentPose.Position.X)
	}
}

func TestAR3simulate_MoveSteppers(t *testing.T) {
	arm := ConnectMock()
	// Move the arm. First 5 numbers are rational defaults, and each motor gets moved 500 steps
	err := arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500, 0)
	if err != nil {
		t.Errorf("Arm should succeed with initial move. Got error: %s", err)
	}
}

func TestAR3simulate_MoveSteppersTooLarge(t *testing.T) {
	// The following line establishes that mock DOES implement the AR3 interface.
	var arm Arm //nolint
	arm = ConnectMock()
	err := arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500000000, 0)
	if err == nil {
		t.Errorf("Arm should have failed with large j6 value")
	}
}

func TestAR3simulate_MoveJointRadians(t *testing.T) {
	arm := ConnectMock()
	// Move the arm 1 radian in each direction.
	err := arm.MoveJointRadians(5, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0)
	if err != nil {
		t.Errorf("Arm should succeed with radian move. Got error: %s", err)
	}
}

func TestAR3simulate_Move(t *testing.T) {
	arm := ConnectMock()
	// Establish position to move to
	err := arm.MoveJointRadians(5, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 0)
	if err != nil {
		t.Errorf("Arm should not have failed to MoveJointRadians. Got error: %s", err)
	}
	currPose := arm.CurrentPose()
	err = arm.Move(5, 10, 10, 10, 10, currPose)
	if err != nil {
		t.Errorf("Arm should succeed with move. Got error: %s", err)
	}

	// Move somewhere close by
	currPose.Position.Z = currPose.Position.Z + 10
	err = arm.Move(5, 10, 10, 10, 10, currPose)
	if err != nil {
		t.Errorf("Arm should succeed with small move. Got error: %s", err)
	}

	// Try to move somewhere ridiculous
	currPose.Position.X = 10000000000
	err = arm.Move(5, 10, 10, 10, 10, currPose)
	if err == nil {
		t.Errorf("Arm should fail at moving somewhere ridiculous. Got error: %s", err)
	}
}

func TestAR3simulate_Calibrate(t *testing.T) {
	arm := ConnectMock()
	err := arm.Calibrate(25, true, true, true, true, true, true, true)
	if err != nil {
		t.Errorf("Simulate arm should always succeed. Got error: %s", err)
	}
}
