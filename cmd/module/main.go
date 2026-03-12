package main

import (
	"viamchess"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/vision"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: vision.API, Model: viamchess.PieceFinderModel},
		resource.APIModel{API: generic.API, Model: viamchess.ChessModel},
		resource.APIModel{API: generic.API, Model: viamchess.CheckersModel},
	)
}
