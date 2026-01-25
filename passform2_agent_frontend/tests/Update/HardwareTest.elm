module Update.HardwareTest exposing (suite)

import Test exposing (..)
import Expect
import Types exposing (..)
import Types.Domain exposing (..)
import Update.Hardware as Hardware
import TestHelpers exposing (emptyModel)

suite : Test
suite =
    describe "Hardware Update Domain"
        [ test "DismissAlert setzt den Alert auf Nothing" <|
            \_ ->
                let
                    modelWithAlert = { emptyModel | alert = Just "Fehler" }
                    ( updatedModel, _ ) = Hardware.update DismissAlert modelWithAlert
                in
                Expect.equal updatedModel.alert Nothing

        , test "HandleRangerStatus aktualisiert die Batteriespannung" <|
            \_ ->
                let
                    -- Wir simulieren den Empfang einer Spannung von 24.5V
                    ( updatedModel, _ ) = Hardware.update (HandleRangerBattery 24.5) emptyModel
                in
                updatedModel.rangerBattery
                    |> Maybe.withDefault 0.0
                    |> Expect.within (Expect.Absolute 0.0001) 24.5
        ]