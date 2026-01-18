module PlanningLogicTest exposing (suite)

import Test exposing (..)
import Expect
import Update.Planning as Planning
import TestHelpers exposing (emptyModel)

suite : Test
suite =
    describe "Planning Pure Logic"
        [ test "canPlan erkennt valide Koordinaten" <|
            \_ ->
                let
                    model = { emptyModel | pathStart = Just { x = 0, y = 0 }, pathGoal = Just { x = 1, y = 1 } }
                in
                Expect.equal (Planning.canPlan model) True
        ]