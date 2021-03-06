package main

import (
	"encoding/json"
	"fmt"
	"log"
	"os"
	"time"

	_ "embed"

	"github.com/jmoiron/sqlx"
	_ "github.com/mattn/go-sqlite3"
	"github.com/trilobio/ar3"
	"github.com/trilobio/kinematics"
	. "github.com/trilobio/quaternion"
	"github.com/urfave/cli/v2"
)

type State struct {
	robot *ar3.Arm
	db    *sqlx.DB
	speed int
}

//go:embed schema.sql
var Schema string

func main() {
	var s State
	app := &cli.App{
		Name:  "AR3 Controller",
		Usage: "Connect to and move an AR3 robot arm",
		Flags: []cli.Flag{
			&cli.StringFlag{
				Name:     "port",
				Aliases:  []string{"p"},
				Usage:    "Connect to the arm on port `PORT`",
				Required: true,
			},
			&cli.StringFlag{
				Name:    "dburl",
				Aliases: []string{"d"},
				Value:   ":memory:",
				Usage:   "Use the SQLite database at `DBURL`",
			},
			&cli.Int64Flag{
				Name:    "speed",
				Aliases: []string{"s"},
				Value:   10,
				Usage:   "Set the speed of the robot arm",
			},
			&cli.BoolFlag{
				Name:    "mock",
				Aliases: []string{"k"},
				Value:   false,
				Usage:   "Use the mock robot arm interface",
			},
		},
		Commands: []*cli.Command{
			{
				Name:    "calibrate",
				Aliases: []string{"c"},
				Usage:   "Calibrate the robot arm by moving to the limit switches",
				Action: func(c *cli.Context) error {
					err := (*s.robot).Calibrate(25, true, true, true, true, true, true, false)
					if err != nil {
						return fmt.Errorf("error calibrating robot: %v", err)
					}
					err = recordJoints(s.db, s.robot)
					if err != nil {
						return err
					}
					err = (*s.robot).MoveJointRadians(s.speed, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0)
					if err != nil {
						return fmt.Errorf("error moving to home %v", err)
					}
					err = recordJoints(s.db, s.robot)
					if err != nil {
						return err
					}
					return nil
				},
			},
			{
				Name:    "state",
				Aliases: []string{"s"},
				Usage:   "Get the current state of the robot arm.",
				Action: func(c *cli.Context) error {
					pose := (*s.robot).CurrentPose()

					poseJSON, err := json.MarshalIndent(pose, "", "  ")
					if err != nil {
						return err
					}
					fmt.Printf("%s\n", string(poseJSON))
					return nil
				},
			},
			{
				Name:  "home",
				Usage: "Move the robot arm to the home position.",
				Action: func(c *cli.Context) error {
					err := (*s.robot).MoveJointRadians(s.speed, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0)
					if err != nil {
						return fmt.Errorf("error moving to home %v", err)
					}

					err = recordJoints(s.db, s.robot)
					if err != nil {
						return err
					}
					return nil
				},
			},
			{
				Name:    "move",
				Aliases: []string{"m"},
				Usage: "Move the robot arm end effector to a position and" +
					" orientation. By default, move relative to the current" +
					" pose.",
				Flags: []cli.Flag{
					&cli.Float64Flag{
						Name:  "x",
						Usage: "Position in the X axis to move the end effector",
						Value: 0,
					},
					&cli.Float64Flag{
						Name:  "y",
						Usage: "Position in the Y axis to move the end effector",
						Value: 0,
					},
					&cli.Float64Flag{
						Name:  "z",
						Usage: "Position in the Z axis to move the end effector",
						Value: 0,
					},
					&cli.Float64Flag{
						Name: "qw",
						Usage: "W component of the rotation quaternion to move" +
							" the end effector",
						Value: 1,
					},
					&cli.Float64Flag{
						Name: "qx",
						Usage: "X component of the rotation quaternion to move" +
							" the end effector",
						Value: 0,
					},
					&cli.Float64Flag{
						Name: "qy",
						Usage: "Y component of the rotation quaternion to move" +
							" the end effector",
						Value: 0,
					},
					&cli.Float64Flag{
						Name: "qz",
						Usage: "Z component of the rotation quaternion to move" +
							" the end effector",
						Value: 0,
					},
					&cli.BoolFlag{
						Name:    "abs",
						Aliases: []string{"a"},
						Usage: "If True, then move in absolute coordinates" +
							" instead of relative",
						Value: false,
					},
				},
				Action: func(c *cli.Context) error {
					var targPose kinematics.Pose
					abs := c.Bool("abs")
					targPose.Position.X = c.Float64("x")
					targPose.Position.Y = c.Float64("y")
					targPose.Position.Z = c.Float64("z")
					targPose.Rotation.W = c.Float64("qw")
					targPose.Rotation.X = c.Float64("qx")
					targPose.Rotation.Y = c.Float64("qy")
					targPose.Rotation.Z = c.Float64("qz")

					// Handle relative transformations
					if !abs {
						currPose := (*s.robot).CurrentPose()

						var currRot = kinQuatToQuat(currPose.Rotation)
						var targRot = kinQuatToQuat(targPose.Rotation)

						targPose.Position.X += currPose.Position.X
						targPose.Position.Y += currPose.Position.Y
						targPose.Position.Z += currPose.Position.Z
						targPose.Rotation = quatToKinQuat(targRot.MulQuat(currRot))

					}
					targRot := kinQuatToQuat(targPose.Rotation)
					targRot.Normalize()
					targPose.Rotation = quatToKinQuat(targRot)

					err := (*s.robot).Move(s.speed, 10, 10, 10, 10, targPose)

					if err != nil {
						return fmt.Errorf("error moving to position %v", err)
					}

					err = recordJoints(s.db, s.robot)
					if err != nil {
						return err
					}

					return nil
				},
			},
		},
		Before: func(c *cli.Context) error {
			port := c.String("port")
			dbUrl := c.String("dburl")
			speed := c.Int("speed")
			mock := c.Bool("mock")

			s.speed = speed

			jointDirs := [7]bool{true, false, false, true, false, true, false}

			var r ar3.Arm
			var err error
			if !mock {
				r, err = ar3.Connect(port, jointDirs)
				if err != nil {
					return err
				}
			} else {
				r = ar3.ConnectMock()
			}

			s.robot = &r

			if dbUrl == "" {
				dbUrl = ":memory:"
			}
			s.db, err = sqlx.Open("sqlite3", dbUrl)
			if err != nil {
				return err
			}

			_, err = s.db.Exec(Schema)
			if err != nil {
				return fmt.Errorf("error executing schema: %v", err)
			}

			jointsRestore, err := getJoints(s.db, s.robot)
			if err == nil {
				(*s.robot).SetJointRadians(jointsRestore)
			}

			return nil
		},
	}

	err := app.Run(os.Args)
	if err != nil {
		log.Fatal(err)
	}

	if s.db != nil {
		s.db.Close()
	}

}

func recordJoints(db *sqlx.DB, robot *ar3.Arm) error {
	joints := (*robot).CurrentJointRadians()
	// fmt.Println("Record Joints: ", joints)
	tx, err := db.Begin()
	if err != nil {
		return fmt.Errorf("error beginning transaction: %v", err)
	}
	_, err = tx.Exec("INSERT INTO joints (J1, J2, J3, J4, J5, J6) VALUES"+
		" (?, ?, ?, ?, ?, ?);", joints[0], joints[1], joints[2], joints[3],
		joints[4], joints[5])

	if err != nil {
		errR := tx.Rollback()
		if errR != nil {
			return fmt.Errorf("error rolling back transaction: %v", errR)
		}
		return fmt.Errorf("error inserting pose: %v", err)
	}

	err = tx.Commit()
	if err != nil {
		return fmt.Errorf("error committing transaction: %v", err)
	}
	return nil
}

func getJoints(db *sqlx.DB, robot *ar3.Arm) ([7]float64, error) {
	var resJoints [7]float64

	var joints struct {
		Insertedat time.Time `db:"insertedat"`
		J1         float64   `db:"J1"`
		J2         float64   `db:"J2"`
		J3         float64   `db:"J3"`
		J4         float64   `db:"J4"`
		J5         float64   `db:"J5"`
		J6         float64   `db:"J6"`
	}

	err := db.Get(&joints, "SELECT * FROM joints ORDER BY insertedat DESC LIMIT 1")
	if err != nil {
		return resJoints, fmt.Errorf("error getting most recent pose: %v", err)
	}

	// Unpack temporary struct
	resJoints[0] = joints.J1
	resJoints[1] = joints.J2
	resJoints[2] = joints.J3
	resJoints[3] = joints.J4
	resJoints[4] = joints.J5
	resJoints[5] = joints.J6
	// fmt.Println("Get Joints: ", resJoints)
	return resJoints, nil
}

func kinQuatToQuat(kq kinematics.Quaternion) Quat {
	var q = Quat{}
	q.W = kq.W
	q.X = kq.X
	q.Y = kq.Y
	q.Z = kq.Z
	return q
}

func quatToKinQuat(q Quat) kinematics.Quaternion {
	var kq = kinematics.Quaternion{}
	kq.W = q.W
	kq.X = q.X
	kq.Y = q.Y
	kq.Z = q.Z
	return kq
}
