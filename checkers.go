package viamchess

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/utils/trace"
)

const grabZ = 175.0

var CheckersModel = family.WithModel("checkers")

func init() {
	resource.RegisterService(generic.API, CheckersModel,
		resource.Registration[resource.Resource, *CheckersConfig]{
			Constructor: checkersConstructor,
		},
	)
}

type CheckersConfig struct {
	PieceFinder string `json:"piece-finder"`

	Arm     string
	Gripper string
	Camera  string

	PoseStart string `json:"pose-start"`

	Engine       string
	EngineMillis int `json:"engine-millis"`

	CaptureDir string // mostly for vla data
}

func (cfg *CheckersConfig) Validate(path string) ([]string, []string, error) {
	if cfg.PieceFinder == "" {
		return nil, nil, fmt.Errorf("need a piece-finder")
	}
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("need an arm")
	}
	if cfg.Gripper == "" {
		return nil, nil, fmt.Errorf("need a gripper")
	}
	if cfg.PoseStart == "" {
		return nil, nil, fmt.Errorf("need a pose-start")
	}

	deps := []string{cfg.PieceFinder, cfg.Arm, cfg.Gripper, cfg.PoseStart, motion.Named("builtin").String()}

	if cfg.Camera != "" {
		deps = append(deps, cfg.Camera)
	}

	if cfg.CaptureDir != "" {
		if cfg.Camera == "" {
			return nil, nil, fmt.Errorf("need a cam if CaptureDir is set")
		}
	}

	// hard deps, soft deps, no errors
	return deps, nil, nil
}

type GameState struct {
	Pieces      map[string]PieceInfo
	WhiteToMove bool
}

type PieceInfo struct {
	Type  string
	Color string
}

type viamCheckers struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	conf   *CheckersConfig

	pieceFinder vision.Service
	arm         arm.Arm
	gripper     gripper.Gripper
	cam         camera.Camera
	poseStart   toggleswitch.Switch

	cancelFunc func()

	motion motion.Service
	rfs    framesystem.Service

	startPose *referenceframe.PoseInFrame

	gameState GameState

	doCommandLock   sync.Mutex
	doCommandCount  atomic.Int32
	movePieceStatus atomic.Int32
}

func checkersConstructor(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*CheckersConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewCheckers(ctx, deps, rawConf.ResourceName(), conf, logger)
}


func NewCheckers(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *CheckersConfig, logger logging.Logger) (resource.Resource, error) {

	var err error
	_, cancelFunc := context.WithCancel(context.Background())

	s := &viamCheckers{
		name:       name,
		logger:     logger,
		conf:       conf,
		cancelFunc: cancelFunc,
	}

	s.pieceFinder, err = vision.FromProvider(deps, conf.PieceFinder)
	if err != nil {
		return nil, err
	}
	s.arm, err = arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, err
	}
	s.gripper, err = gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, err
	}
	if conf.Camera != "" {
		s.cam, err = camera.FromProvider(deps, conf.Camera)
		if err != nil {
			return nil, err
		}
	}
	s.poseStart, err = toggleswitch.FromProvider(deps, conf.PoseStart)
	if err != nil {
		return nil, err
	}
	s.motion, err = motion.FromDependencies(deps, "builtin")
	if err != nil {
		return nil, err
	}
	s.rfs, err = framesystem.FromDependencies(deps)
	if err != nil {
		logger.Errorf("can't find framesystem: %v", err)
	}

	err = s.goToStart(ctx)
	if err != nil {
		return nil, fmt.Errorf("cannot goToStart in constructor: %w", err)
	}

	s.gameState = initializeGameState(true) // true for black squares



	return s, nil
}

func (s *viamCheckers) goToStart(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "goToStart")
	defer span.End()

	err := s.poseStart.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}
	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 250)

	s.startPose, err = s.rfs.GetPose(ctx, s.conf.Gripper, "world", nil, nil)
	if err != nil {
		return err
	}

	return nil
}


func initializeGameState(onBlack bool) GameState {
	num :=0
	if onBlack {
		num = 0
	} else {
		num = 1
	}
	pieces := make(map[string]PieceInfo)
	// Initialize pieces for a standard checkers game
	for i := 0; i < 8; i++ {
		for j := 0; j < 8; j++ {
			if (i+j)%2 == num && j < 3 {
				pieces[coordToSquare(i, j)] = PieceInfo{"basic", "black"}
			}
			if (i+j)%2 == num && j >= 5 {
				pieces[coordToSquare(i, j)] = PieceInfo{"basic", "white"}
			}
		}
	}
	return GameState{Pieces: pieces, WhiteToMove: true}
}

func (g *GameState) checkSquare(square string) (PieceInfo, bool) {
	piece, exists := g.Pieces[square]
	return piece, exists
}

func coordToSquare(x, y int) string {
	return string(rune('a'+x)) + string(rune('1'+y))
}




func (s *viamCheckers) Name() resource.Name {
	return s.name
}

func (s *viamCheckers) Close(ctx context.Context) error {
	s.cancelFunc()
	return nil
}

func (s *viamCheckers) DoCommand(ctx context.Context, cmdMap map[string]interface{}) (map[string]interface{}, error) {
	s.doCommandCount.Add(1)
	ctx, span := trace.StartSpan(ctx, "checkers::DoCommand")
	defer span.End()

	s.doCommandLock.Lock()
	defer s.doCommandLock.Unlock()

	return nil, nil
}
