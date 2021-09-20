package ar3

import (
	"testing"
)

// TestArmInterface checks that the AR3exec struct implements the Arm interface
func TestArmInterface(t *testing.T) {
	val := AR3exec{}
	_, ok := interface{}(&val).(Arm)
	if !ok {
		t.Errorf("Failed. Ar3exec does not implement the Arm interface")
	}
}
