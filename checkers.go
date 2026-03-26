package viamchess

import (
	"context"
	"fmt"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
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
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/vision/viscapture"
	"go.viam.com/utils/trace"
)

const grabZ = 175.0 // Where I wanted the arm to be
const gripperGrabZ = 25.0

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

type Move struct {
	From string
	To   string
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
	visData   viscapture.VisCapture

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

	s.logger.Infof("Camera named: %s", conf.Camera)
	s.logger.Infof("PieceFinder named: %s", conf.PieceFinder)
	s.logger.Infof("Arm named: %s", conf.Arm)
	s.logger.Infof("Gripper named: %s", conf.Gripper)
	s.logger.Infof("Pieces on black squares.")
	s.logger.Infof("Ready to begin!")

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

func readMoveCommand(cmdMap map[string]interface{}) (Move, error) {
	cmd, ok := cmdMap["move"].(string)
	if !ok {
		return Move{}, fmt.Errorf("missing 'move' field in command")
	}

	// Split by comma and trim whitespace
	parts := strings.Split(cmd, ",")
	if len(parts) != 2 {
		return Move{}, fmt.Errorf("move command must be in format 'from,to' (e.g., 'b6,a5'), got: %s", cmd)
	}

	from := strings.TrimSpace(parts[0])
	to := strings.TrimSpace(parts[1])

	if from == "" || to == "" {
		return Move{}, fmt.Errorf("from and to positions cannot be empty")
	}

	return Move{From: from, To: to}, nil
}

func (s *viamCheckers) MovePiece(ctx context.Context, move Move) error {

	defer func() {
		err := s.goToStart(ctx)
		if err != nil {
			s.logger.Errorf("error going to start after MovePiece: %v", err)
		}
	}()

	if !s.isValidMove(move){
		return s.logger.Errorf("invalid move: %s", move)
	}
	// Go to the "from" square
	s1Position, err := s.GoToSquare(ctx, move.From)
	if err != nil {
		return fmt.Errorf("could not go to square %s: %w", move.From, err)
	}
	time.Sleep(time.Millisecond * 1000)

	// Grab it
	grabbed, err := s.gripper.Grab(ctx, nil)
	if err != nil {
		return fmt.Errorf("could not grab piece: %w", err)
	}
	time.Sleep(time.Millisecond * 1500)
	s.logger.Infof("We grabbed the piece: %v", grabbed)

	// Move up a bit
	err = s.moveGripper(ctx, r3.Vector{X: s1Position.X, Y: s1Position.Y, Z: gripperGrabZ + 200})
	if err != nil {
		return fmt.Errorf("could not move up after grabbing: %w", err)
	}
	time.Sleep(time.Millisecond * 1000)

	// Move to the "to" square
	_, err = s.GoToSquare(ctx, move.To)
	if err != nil {
		return fmt.Errorf("could not go to square %s: %w", move.To, err)
	}
	time.Sleep(time.Millisecond * 1000)

	// Release it
	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return fmt.Errorf("could not release piece: %w", err)
	}
	time.Sleep(time.Millisecond * 1000)

	s.logger.Infof("Moved piece from %s to %s", move.From, move.To)
	s.gameState.update(move)	
	s.logger.Infof("Board after move: %s", printBoard(s.gameState))

	return nil
}

func (s *viamCheckers) GoToSquare(ctx context.Context, square string) (r3.Vector, error) {
	pos, err := s.findObjectCenter(s.visData, square)
	if err != nil {
		return r3.Vector{}, fmt.Errorf("could not find position for square %s: %w", square, err)
	}
	s.logger.Infof("Got position for square %s: %v", square, pos)
	err = s.moveGripper(ctx, pos)
	if err != nil {
		return r3.Vector{}, fmt.Errorf("could not move gripper to square %s at pos %v: %w", square, pos, err)
	}
	return pos, nil
}

func initializeGameState(onBlack bool) GameState {
	num := 0
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

// Expect the actual move to be input
func (g *GameState) update(move Move) error {

	_, ok := g.checkSquare(move.To)
	if ok { // There's something in the destination spot.  Not cool.
		return fmt.Errorf("can't update state. found something in destination")
	}
	pieceFrom, ok := g.checkSquare(move.From)
	if !ok {
		return fmt.Errorf("can't update state. found nothing in origin")
	}

	// Put the new piece in the square it's going to.
	g.Pieces[move.To] = pieceFrom
	// Remove the piece from the old square
	g.Pieces[move.From] = PieceInfo{}

	return nil

}

func (g *GameState) checkSquare(square string) (PieceInfo, bool) {
	piece, exists := g.Pieces[square]
	return piece, exists
}

func printBoard(state GameState) string {
	board := make([][]string, 8)
	for i := range board {
		board[i] = make([]string, 8)
		for j := range board[i] {
			board[i][j] = "."
		}
	}

	// Populate board with pieces
	for position, piece := range state.Pieces {
		if len(position) == 2 {
			col := int(position[0] - 'a')
			row := int(position[1] - '1')
			if row >= 0 && row < 8 && col >= 0 && col < 8 {
				symbol := "B"
				if piece.Color == "white" {
					symbol = "W"
				}
				board[7-row][col] = symbol
			}
		}
	}

	var b strings.Builder
	b.WriteString("  a b c d e f g h\n")
	for i := 7; i >= 0; i-- {
		row := fmt.Sprintf("%d ", i+1)
		for j := 0; j < 8; j++ {
			row += board[7-i][j] + " "
		}
		b.WriteString(row + "\n")
	}
	return strings.TrimRight(b.String(), "\n")
}

func coordToSquare(x, y int) string {
	return string(rune('a'+x)) + string(rune('1'+y))
}

func squareToCoord(square string) (int, int, error) {
	if len(square) != 2 {
		return 0, 0, fmt.Errorf("invalid square format: %s", square)
	}
	x := int(square[0] - 'a')
	y := int(square[1] - '1')
	if x < 0 || x > 7 || y < 0 || y > 7 {
		return 0, 0, fmt.Errorf("square out of bounds: %s", square)
	}
	return x, y, nil
}

func (s *viamCheckers) isValidMove(move Move) bool {

	// Convert square names to coordinates
	fromX, fromY, err := squareToCoord(move.From)
	if err != nil {
		return false
	}
	toX, toY, err := squareToCoord(move.To)
	if err != nil {
		return false
	}

	// Get the piece being moved
	piece, exists := s.gameState.checkSquare(move.From)
	if !exists {
		return false
	}

	// Determine valid direction based on color
	validMove := false
	if piece.Color == "black" && toY == fromY+1 {
		validMove = true
	} else if piece.Color == "white" && toY == fromY-1 {
		validMove = true
	}

	if !validMove {
		return false
	}

	// Check if it's a simple move (one square diagonally)
	if (toX-fromX) == 1 || (toX-fromX) == -1 {
		_, occupied := s.gameState.checkSquare(move.To)
		return !occupied // Valid if destination is empty
	}

	// Check if it's a capture (two squares diagonally)
	if (toX-fromX) == 2 || (toX-fromX) == -2 {
		// Get the middle square
		midX := (fromX + toX) / 2
		midY := (fromY + toY) / 2
		midSquare := coordToSquare(midX, midY)

		// Check if there's an opponent's piece to capture
		capturedPiece, occupied := s.gameState.checkSquare(midSquare)
		if !occupied || capturedPiece.Color == piece.Color {
			return false
		}

		// Check if destination is empty
		_, destOccupied := s.gameState.checkSquare(move.To)
		return !destOccupied
	}

	return false

}

func (s *viamCheckers) findObjectCenter(data viscapture.VisCapture, square string) (r3.Vector, error) {
	for _, o := range data.Objects {
		if strings.HasPrefix(o.Geometry.Label(), square) {
			md := o.MetaData()
			center := md.Center()
			return r3.Vector{
				X: center.X,
				Y: center.Y,
				Z: gripperGrabZ,
			}, nil

		}
	}
	return r3.Vector{}, fmt.Errorf("could not find object with label prefix: %s", square)
}

func (s *viamCheckers) moveGripper(ctx context.Context, p r3.Vector) error {
	ctx, span := trace.StartSpan(ctx, "moveGripper")
	defer span.End()

	orientation := &spatialmath.OrientationVectorDegrees{
		OZ:    -1,
		Theta: s.startPose.Pose().Orientation().OrientationVectorDegrees().Theta - 180,
	}

	s.logger.Infof("The Z value is: %f", p.Z)
	myPose := spatialmath.NewPose(p, orientation)
	des := referenceframe.NewPoseInFrame("world", myPose)
	s.logger.Infof("Moving gripper to pose: %v", des)
	_, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.conf.Gripper,
		Destination:   des,
	})
	if err != nil {
		return fmt.Errorf("can't move to %v: %w", myPose, err)
	}
	return nil
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

	move, err := readMoveCommand(cmdMap)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("Received command: %s", move.From+","+move.To)

	visData, err := s.pieceFinder.CaptureAllFromCamera(ctx, "", viscapture.CaptureOptions{}, nil)
	if err != nil {
		return nil, err
	}
	s.visData = visData

	s.MovePiece(ctx, move)

	defer func() {
		err := s.goToStart(ctx)
		if err != nil {
			s.logger.Errorf("error going to start after DoCommand: %v", err)
		}
	}()

	return nil, nil
}
