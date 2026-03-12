package viamchess

import (
	"fmt"
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
	printBoard(t, state)
}

// Helper function to visualize the board
func printBoard(t *testing.T, state GameState) {
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

	// Print board
	t.Log("  a b c d e f g h")
	for i := 7; i >= 0; i-- {
		row := fmt.Sprintf("%d ", i+1)
		for j := 0; j < 8; j++ {
			row += board[7-i][j] + " "
		}
		t.Log(row)
	}
}
