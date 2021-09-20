package ar3_test

import (
	"fmt"
	"github.com/koeng101/armos/devices/ar3"
)

// This example shows basic connection to the robot.
func Example_basic() {
	arm := ar3.ConnectMock() // arm := ar3.Connect("/dev/ttyUSB0")
	// Move the arm. First 5 are rational defaults, following 6 numbers are joint stepper counts, and the final is the track length.
	_ = arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500, 0)
	fmt.Println("Moved arm!")
	// Output: Moved arm!
}
