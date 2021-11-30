package main

import (
	"fmt"
	"log"
	"os"

	_ "embed"

	"github.com/jmoiron/sqlx"
	_ "github.com/mattn/go-sqlite3"
	"github.com/trilobio/ar3"
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
					err = recordPose(s.db, s.robot)
					if err != nil {
						return err
					}
					err = (*s.robot).MoveJointRadians(s.speed, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0)
					if err != nil {
						return fmt.Errorf("error moving to home %v", err)
					}
					err = recordPose(s.db, s.robot)
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
					err := getPose(s.db, s.robot)
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
					" orientation",
				Action: func(c *cli.Context) error {
					return nil
				},
			},
		},
		Before: func(c *cli.Context) error {
			port := c.String("port")
			dbUrl := c.String("dburl")
			speed := c.Int("speed")

			s.speed = speed

			jointDirs := [7]bool{true, false, false, true, false, true, false}

			r, err := ar3.Connect(port, jointDirs)
			if err != nil {
				return err
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

			return err
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

func recordPose(db *sqlx.DB, robot *ar3.Arm) error {
	pose := (*robot).CurrentPose()

	tx, err := db.Begin()
	if err != nil {
		return fmt.Errorf("error beginning transaction: %v", err)
	}
	_, err = tx.Exec("INSERT INTO pose (X, Y, Z, QW, QX, QY, QZ) VALUES"+
		" (?, ?, ?, ?, ?, ?, ?);", pose.Position.X, pose.Position.Y,
		pose.Position.Z, pose.Rotation.W, pose.Rotation.X, pose.Rotation.Y,
		pose.Rotation.Z)

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

func getPose(db *sqlx.DB, robot *ar3.Arm) error {
	var pose struct {
		X  float64 `db:"X"`
		Y  float64 `db:"Y"`
		Z  float64 `db:"Z"`
		QW float64 `db:"QW"`
		QX float64 `db:"QX"`
		QY float64 `db:"QY"`
		QZ float64 `db:"QZ"`
	}

	err := db.Get(&pose, "SELECT * FROM pose ORDER BY Timestamp")
	if err != nil {
		return fmt.Errorf("error getting most recent pose: %v", err)
	}

	fmt.Println(pose)

	return nil
}
