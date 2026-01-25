module ModelTest exposing (suite)

import Test exposing (..)
import Expect
import TestHelpers exposing (emptyModel)
import Types exposing (..)
import Types.Domain exposing (..)

suite : Test
suite =
    describe "Initial Model State"
        [ test "Das System startet im Simulations-Modus" <|
            \_ ->
                Expect.equal emptyModel.mode Simulation
        ]