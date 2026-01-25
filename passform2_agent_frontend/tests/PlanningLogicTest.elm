module PlanningLogicTest exposing (suite)

import Test exposing (..)
import Expect
import Update.Planning as Planning
import TestHelpers exposing (emptyModel)

suite : Test
suite =
    describe "Planning Pure Logic"
        [ test "canPlan erkennt valide Koordinaten wenn online" <|
            \_ ->
                let
                    model = 
                        { emptyModel 
                            | pathStart = Just { x = 0, y = 0 }
                            , pathGoal = Just { x = 1, y = 1 }
                            , rosConnected = True  -- Wir simulieren ROS-Verbindung
                            , connected = True     -- Wir simulieren REST-Verbindung
                        }
                in
                Expect.equal (Planning.canPlan model) True

        , test "canPlan verweigert Planung wenn offline" <|
            \_ ->
                let
                    model = 
                        { emptyModel 
                            | pathStart = Just { x = 0, y = 0 }
                            , pathGoal = Just { x = 1, y = 1 }
                            , rosConnected = False -- System ist offline
                            , connected = False
                        }
                in
                Expect.equal (Planning.canPlan model) False
        ]