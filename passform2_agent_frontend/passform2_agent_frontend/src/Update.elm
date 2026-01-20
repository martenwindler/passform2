module Update exposing (update)

import Types exposing (..)
import Types.Domain exposing (..)
import Update.Planning as Planning
import Update.Hardware as Hardware
import Update.Agents as Agents
import Update.System as System

{-| 
  Zentraler Dispatcher: 
  Leitet die Nachrichten an die spezialisierten Module weiter.
  Da die Unter-Module bereits Cmd Msg zurückgeben, ist kein Cmd.map mehr nötig.
-}
update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        PlanningMsg subMsg ->
            Planning.update subMsg model

        HardwareMsg subMsg ->
            Hardware.update subMsg model

        AgentsMsg subMsg ->
            Agents.update subMsg model

        NoOp ->
            ( model, Cmd.none )

        OpenMenu x y ->
            ( { model | activeMenu = Just (SelectionMenu ( x, y )) }, Cmd.none )

        SystemMsg sMsg ->
            System.update sMsg model