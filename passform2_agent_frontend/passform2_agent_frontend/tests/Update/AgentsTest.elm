module Update.AgentsTest exposing (suite)

import Test exposing (..)
import Expect
import Dict
import Types exposing (..)
import Types.Domain exposing (..)
import Update.Agents as Agents
import TestHelpers exposing (emptyModel)

suite : Test
suite =
    describe "Domänen-Check: Agents Update"
        [ test "RotateAgent dreht das Modul an Position (1,1) um 90 Grad" <|
            \_ ->
                let
                    pos = { x = 1, y = 1 }
                    agent = { agent_id = Just "A1", module_type = FTF, position = pos, orientation = 0, is_dynamic = True, payload = Nothing, signal_strength = 100 }
                    initialModel = { emptyModel | agents = Dict.insert (1, 1) agent Dict.empty }
                    ( updatedModel, _ ) = Agents.update (RotateAgent pos) initialModel
                in
                updatedModel.agents
                    |> Dict.get (1, 1)
                    |> Maybe.map .orientation
                    |> Expect.equal (Just 90)

        , test "RemoveAgent löscht das Modul an der angegebenen Position" <|
            \_ ->
                let
                    pos = { x = 2, y = 2 }
                    agent = { agent_id = Just "A2", module_type = Station, position = pos, orientation = 0, is_dynamic = False, payload = Nothing, signal_strength = 100 }
                    initialModel = { emptyModel | agents = Dict.insert (2, 2) agent Dict.empty }
                    ( updatedModel, _ ) = Agents.update (RemoveAgent pos) initialModel
                in
                -- Das Dict muss nach dem Löschen leer sein
                Dict.isEmpty updatedModel.agents
                    |> Expect.equal True
        ]