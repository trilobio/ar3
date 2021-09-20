package main

import (
	"fmt"

	"github.com/koeng101/armos/devices/ar3"
)

func main() {
	jointDirs := [7]bool{true, false, false, true, false, true, false}
	// limitSwitchSteps := ar3.AnglesToSteps([7]float64{-170, 85, -60, -85, 90, 80, 0}, true)
	var robot ar3.Arm
	var err error
	robot, err = ar3.Connect("/dev/ttyUSB0", jointDirs, [7]int{0, 0, 0, 0, 0, 0, 0})
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	err = robot.Calibrate(25, true, true, true, true, true, true, false)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	err = robot.MoveJointRadians(5, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	currPose := robot.CurrentPose()
	err = robot.Move(5, 10, 10, 10, 10, currPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	// Move down 50 mm to avoid singularities during square
	targPose := currPose
	targPose.Pos.Z -= 50
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	// Now make a 100 mm square
	sideLength := 100.0
	targPose.Pos.Z -= sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Y -= sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Z += sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Y += sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
}
