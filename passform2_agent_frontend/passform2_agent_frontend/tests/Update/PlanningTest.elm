module Update.PlanningTest exposing (suite)

import Test exposing (..)
import Expect
import Types exposing (..)
import Types.Domain exposing (..)
import Update.Planning as Planning
import TestHelpers exposing (emptyModel)

suite : Test
suite =
    describe "Domänen-Check: Planning Update & Logic"
        [ describe "Gewichts-Einstellungen"
            [ test "SetWeight aktualisiert 'execution_time_default' korrekt" <|
                \_ ->
                    let
                        ( updatedModel, _ ) = 
                            Planning.update (SetWeight "execution_time_default" "2.5") emptyModel
                    in
                    -- Wir erwarten 2.5 mit einer Toleranz von 0.0001
                    updatedModel.planningWeights.execution_time_default 
                        |> Expect.within (Expect.Absolute 0.0001) 2.5
            ]
        , describe "Missions-Validierung (canPlan)"
            [ test "canPlan ist False, wenn Start und Ziel fehlen" <|
                \_ ->
                    Planning.canPlan emptyModel 
                        |> Expect.equal False
            ]
        ]