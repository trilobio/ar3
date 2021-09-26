package ar3

import (
	"fmt"
	"github.com/trilobio/kinematics"
)

// AR3simulate struct represents an AR3 robotic arm interface for testing purposes.
type AR3simulate struct {
	jointVals        [7]int
	jointDirs        [7]bool
	limitSwitchSteps [7]int
}

// ConnectMock connects to a mock AR3simulate interface.
func ConnectMock() *AR3simulate {
	return &AR3simulate{}
}

// Calibrate simulates AR3exec.Calibrate()
func (ar3 *AR3simulate) Calibrate(speed int, j1, j2, j3, j4, j5, j6, tr bool) error {
	homeMotor := []bool{j1, j2, j3, j4, j5, j6, tr}
	for i := range ar3.jointVals {
		if homeMotor[i] {
			ar3.jointVals[i] = 0
		}
	}
	return nil
}

// Echo simulates AR3exec.Echo().
func (ar3 *AR3simulate) Echo() error {
	return nil
}

// SetDirections simulates AR3exec.SetDirections().
func (ar3 *AR3simulate) SetDirections(jointDirs [7]bool) {
	ar3.jointDirs = jointDirs
}

// GetDirections simulates AR3exec.GetDirections().
func (ar3 *AR3simulate) GetDirections() [7]bool {
	return ar3.jointDirs
}

// CurrentStepperPosition simulates AR3exec.CurrentStepperPosition().
func (ar3 *AR3simulate) CurrentStepperPosition() [7]int {
	return ar3.jointVals
}

// CurrentJointRadians simulates AR3exec.CurrentJointRadians().
func (ar3 *AR3simulate) CurrentJointRadians() [7]float64 {
	js := ar3.jointVals
	sl := ar3.limitSwitchSteps
	stepVals := [7]int{js[0] - sl[0], js[1] - sl[1], js[2] - sl[2], js[3] - sl[3], js[4] - sl[4], js[5] - sl[5], js[6] - sl[6]}
	jointVals := stepsToAngles(stepVals, false)
	return jointVals
}

// CurrentPose simulates AR3exec.CurrentPose()
func (ar3 *AR3simulate) CurrentPose() kinematics.Pose {
	ja := ar3.CurrentJointRadians()
	thetasInit := []float64{ja[0], ja[1], ja[2], ja[3], ja[4], ja[5]}
	return kinematics.ForwardKinematics(thetasInit, AR3DhParameters)
}

// MoveSteppers simulates AR3exec.MoveSteppers().
func (ar3 *AR3simulate) moveSteppersRelative(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr int) error {
	// First, check if the move can be made
	to := []int{j1, j2, j3, j4, j5, j6}
	from := []int{ar3.jointVals[0], ar3.jointVals[1], ar3.jointVals[2], ar3.jointVals[3], ar3.jointVals[4], ar3.jointVals[5]}
	limits := []int{j1stepLim, j2stepLim, j3stepLim, j4stepLim, j5stepLim, j6stepLim}
	motor := []string{"J1", "J2", "J3", "J4", "J5", "J6"}
	var newPositions [7]int
	for i := 0; i < 6; i++ {
		newJ := to[i] + from[i]
		if newJ < 0 || newJ > limits[i] {
			return fmt.Errorf("%s out of range. Must be between 0 and %d. Got %d", motor[i], limits[i], newJ)
		}
		newPositions[i] = newJ
	}
	// If all the limits check out, apply them.
	ar3.jointVals = newPositions

	// Since we are simulating, simply update and assume that there is no error.
	return nil
}

// MoveSteppers simulates AR3exec.MoveSteppers
func (ar3 *AR3simulate) MoveSteppers(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr int) error {
	js := ar3.jointVals
	sl := ar3.limitSwitchSteps
	return ar3.moveSteppersRelative(speed, accdur, accspd, dccdur, dccspd,
		j1-js[0]+sl[0], j2-js[1]+sl[1], j3-js[2]+sl[2], j4-js[3]+sl[3],
		j5-js[4]+sl[4], j6-js[5]+sl[5], tr-js[6]+sl[6])
}

// MoveJointRadians simulates AR3exec.MoveJointRadians
func (ar3 *AR3simulate) MoveJointRadians(speed, accdur, accspd, dccdur, dccspd int, j1, j2, j3, j4, j5, j6, tr float64) error {

	jointSteps := anglesToSteps([7]float64{j1, j2, j3, j4, j5, j6, tr}, false)

	return ar3.MoveSteppers(speed, accdur, accspd, dccdur, dccspd,
		jointSteps[0], jointSteps[1], jointSteps[2], jointSteps[3],
		jointSteps[4], jointSteps[5], jointSteps[6])
}

// Move to a new end effector Pose using inverse kinematics to solve for the
// joint angles.
func (ar3 *AR3simulate) Move(speed, accdur, accspd, dccdur, dccspd int, pose kinematics.Pose) error {
	ja := ar3.CurrentJointRadians()
	thetasInit := []float64{ja[0], ja[1], ja[2], ja[3], ja[4], ja[5]}
	tj, err := kinematics.InverseKinematics(pose, AR3DhParameters, thetasInit)
	if err != nil {
		return fmt.Errorf("Inverse Kinematics failed with error: %s", err)
	}
	return ar3.MoveJointRadians(speed, accdur, accspd, dccdur,
		dccspd, tj[0], tj[1], tj[2], tj[3], tj[4], tj[5], 0)
}
