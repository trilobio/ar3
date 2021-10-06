package main

import (
	"fmt"
	"math"

	"github.com/trilobio/ar3"
)

const DEG float64 = math.Pi / 180

func main() {
	jointDirs := [7]bool{true, false, false, true, false, true, false}
	robot, err := ar3.Connect("/dev/ttyUSB0", jointDirs)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	robot.Calibrate(25, true, true, true, true, true, true, false)
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
	targPose.Position.Z -= 50
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	// Now make a 100 mm square
	sideLength := 100.0
	targPose.Position.Z -= sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Position.Y -= sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Position.Z += sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Position.Y += sideLength
	err = robot.Move(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
}
