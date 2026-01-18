module Update.Agents exposing (update)

import Types exposing (..)
import Types.Domain exposing (..)
import Dict
import Ports
import Decoders
import Json.Decode as Decode

{-| 
  Logik-Domäne: Teilnehmer & Gitter-Interaktion.
-}
update : AgentsMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        UpdateAgents rawJson ->
            if model.mode == Hardware then
                case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                    Ok newAgentsDict ->
                        let
                            criticalAgentId =
                                newAgentsDict |> Dict.values |> List.filter (\a -> a.signal_strength < 20)
                                    |> List.head |> Maybe.andThen .agent_id
                        in ( { model | agents = newAgentsDict, alert = criticalAgentId }, Cmd.none )
                    Err _ -> ( model, Cmd.none )
            else ( model, Cmd.none )

        ToggleMode ->
            let
                newMode = if model.mode == Simulation then Hardware else Simulation
                modeStr = if newMode == Simulation then "simulation" else "hardware"
                ( updatedAgents, logEntry ) =
                    case newMode of
                        Simulation -> ( model.savedDefault, { message = "Simulation aktiv.", level = Info } )
                        Hardware -> ( Dict.empty, { message = "Hardware aktiv: Sync...", level = Warning } )
            in
            ( { model | mode = newMode, agents = updatedAgents, logs = logEntry :: model.logs, currentPath = Nothing }
            , Ports.setMode modeStr
            )

        StartAgent moduleType cell ->
            let
                typeStr = Decoders.moduleTypeToString moduleType
                newId = typeStr ++ "-" ++ String.fromInt cell.x ++ String.fromInt cell.y
                newAgent = { agent_id = Just newId, module_type = moduleType, position = cell, orientation = 0, is_dynamic = (moduleType == FTF), payload = Nothing, signal_strength = 100 }
            in
            ( { model | agents = Dict.insert ( cell.x, cell.y ) newAgent model.agents, activeMenu = Nothing, currentPath = Nothing }, Cmd.none )

        RotateAgent cell ->
            let newAgents = Dict.update ( cell.x, cell.y ) (Maybe.map (\a -> { a | orientation = modBy 360 (a.orientation + 90) })) model.agents
            in ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )

        RemoveAgent cell ->
            ( { model | agents = Dict.remove ( cell.x, cell.y ) model.agents, activeMenu = Nothing, currentPath = Nothing }, Cmd.none )

        HandleGridClick cell ->
            case Dict.get ( cell.x, cell.y ) model.agents of
                Just agent -> ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )
                Nothing ->
                    if model.editing then ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none )
                    else ( model, Cmd.none )

        ToggleViewMode -> ( { model | is3D = not model.is3D }, Cmd.none )
        ToggleSidebar -> ( { model | sidebarOpen = not model.sidebarOpen }, Cmd.none )
        SwitchSidebarTab tab -> ( { model | activeSidebarTab = tab, sidebarOpen = True }, Cmd.none )
        CloseMenu -> ( { model | activeMenu = Nothing }, Cmd.none )
        _ -> ( model, Cmd.none )