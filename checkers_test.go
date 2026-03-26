package viamchess

import (
	"testing"

	"go.viam.com/test"
)

func TestInitializeGameState(t *testing.T) {
	state := initializeGameState(true)

	// Verify the map is populated
	test.That(t, len(state.Pieces), test.ShouldBeGreaterThan, 0)
	test.That(t, state.WhiteToMove, test.ShouldBeTrue)

	// Print the board state
	t.Log("\n===== GameState Pieces =====")
	for position, piece := range state.Pieces {
		t.Logf("Position: %s | Type: %s | Color: %s", position, piece.Type, piece.Color)
	}

	// Pretty print the board
	t.Log("\n===== Board Layout =====")
	t.Log("\n" + printBoard(state))
}


