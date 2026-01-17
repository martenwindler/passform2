module Update exposing (update)

import Dict
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)
import Decoders 

update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- --- SYSTEM-TAKT (SSoT) ---
        ChangeHz newHz ->
            ( { model | currentHz = newHz }
            , Ports.socketEmit "set_heartbeat_rate" (Encode.float newHz)
            )

        -- --- ALARM MANAGEMENT ---
        DismissAlert ->
            ( { model | alert = Nothing }, Cmd.none )

        -- --- AGENTEN UPDATES (SSoT Logik) ---
        UpdateAgents rawJson ->
            -- Nur im Hardware-Modus überschreibt das Backend unseren lokalen Zustand
            if model.mode == Hardware then
                case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                    Ok newAgentsDict ->
                        let
                            -- Check auf kritische Signalstärke (< 20%) für den Alarm-Toast
                            criticalAgentId =
                                newAgentsDict
                                    |> Dict.values
                                    |> List.filter (\a -> a.signal_strength < 20)
                                    |> List.head
                                    |> Maybe.andThen .agent_id
                        in
                        ( { model | agents = newAgentsDict, alert = criticalAgentId }, Cmd.none )

                    Err _ ->
                        ( model, Cmd.none )

            else
                ( model, Cmd.none )

        -- --- MODUS-STEUERUNG (Simulation vs. Hardware) ---
        ToggleMode ->
            let
                newMode =
                    if model.mode == Simulation then Hardware else Simulation

                modeStr =
                    if newMode == Simulation then "simulation" else "hardware"

                -- Beim Wechsel: Simulation lädt Defaults, Hardware startet leer (wartet auf Sync)
                ( updatedAgents, logEntry ) =
                    if newMode == Simulation then
                        ( model.savedDefault, { message = "Simulation aktiv: Lokale Bearbeitung möglich.", level = "info" } )
                    else
                        ( Dict.empty, { message = "Hardware aktiv: Synchronisiere mit ROS 2 Nodes...", level = "warning" } )
            in
            ( { model | mode = newMode, agents = updatedAgents, logs = logEntry :: model.logs, currentPath = Nothing }
            , Ports.setMode modeStr
            )

        -- --- AGENTEN MANAGEMENT (LOKAL) ---
        StartAgent moduleType cell ->
            let
                newId = moduleType ++ "-" ++ String.fromInt cell.x ++ String.fromInt cell.y
                newAgent =
                    { agent_id = Just newId
                    , module_type = moduleType
                    , position = cell
                    , orientation = 0
                    , is_dynamic = moduleType == "ftf"
                    , payload = Nothing
                    , signal_strength = 100
                    }
            in
            ( { model | agents = Dict.insert ( cell.x, cell.y ) newAgent model.agents, activeMenu = Nothing, currentPath = Nothing }
            , Cmd.none 
            )

        RotateAgent cell ->
            let
                newAgents =
                    Dict.update ( cell.x, cell.y ) (Maybe.map (\a -> { a | orientation = modBy 360 (a.orientation + 90) })) model.agents
            in
            ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )

        RemoveAgent cell ->
            ( { model | agents = Dict.remove ( cell.x, cell.y ) model.agents, activeMenu = Nothing, currentPath = Nothing }
            , Cmd.none 
            )

        MoveAgent { oldX, oldY, newX, newY } ->
            if Dict.member ( newX, newY ) model.agents then
                ( model, Cmd.none )
            else
                case Dict.get ( oldX, oldY ) model.agents of
                    Just agent ->
                        let
                            tempAgents = Dict.remove ( oldX, oldY ) model.agents
                            updatedAgent = { agent | position = { x = newX, y = newY } }
                            newAgents = Dict.insert ( newX, newY ) updatedAgent tempAgents
                        in
                        ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )
                    Nothing -> ( model, Cmd.none )

        -- --- HARDWARE INTERAKTION ---
        RequestNfcWrite content ->
            ( { model | waitingForNfc = True, logs = { message = "NFC: Sende Brennbefehl...", level = "info" } :: model.logs }
            , Ports.writeNfcTrigger content 
            )

        HandleNfcStatus result ->
            case result of
                Ok status -> ( { model | nfcStatus = status }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        -- --- SYSTEM UPDATES ---
        SetConnected status ->
            ( { model | connected = status }, Cmd.none )

        HandleSystemLog result ->
            case result of
                Ok logEntry ->
                    ( { model | logs = List.take 30 (logEntry :: model.logs), waitingForNfc = False }, Cmd.none )
                Err _ ->
                    ( model, Cmd.none )

        -- HARDWARE -- 
        HandleHardwareUpdate result ->
            case result of
                Ok hardwareList ->
                    ( { model | connectedHardware = hardwareList }, Cmd.none )

                Err error ->
                    -- Optional: Fehler loggen
                    ( model, Cmd.none )

        -- --- UI & PLANUNG ---
        HandleGridClick cell ->
            case Dict.get ( cell.x, cell.y ) model.agents of
                Just agent -> ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )
                Nothing ->
                    if model.editing then ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none )
                    else ( model, Cmd.none )

        SetWeight field val ->
            let
                newVal = String.toFloat val |> Maybe.withDefault 0.0
                oldWeights = model.planningWeights
                updatedWeights =
                    case field of
                        "execution_time_default" -> { oldWeights | execution_time_default = newVal }
                        "complex_module_time" -> { oldWeights | complex_module_time = newVal }
                        "human_extra_weight" -> { oldWeights | human_extra_weight = newVal }
                        "proximity_penalty" -> { oldWeights | proximity_penalty = newVal }
                        _ -> oldWeights
            in
            ( { model | planningWeights = updatedWeights }, Cmd.none )

        SaveWeights ->
            ( { model | logs = { message = "Planungsparameter aktualisiert.", level = "success" } :: model.logs }
            , Ports.savePlanningWeights (encodeWeights model.planningWeights) 
            )

        -- --- NAVIGATION & BOILERPLATE ---
        ToggleSidebar -> ( { model | sidebarOpen = not model.sidebarOpen }, Cmd.none )
        ToggleViewMode -> ( { model | is3D = not model.is3D }, Cmd.none )
        SetPathStart cell -> ( { model | pathStart = Just cell, activeMenu = Nothing }, Cmd.none )
        SetPathGoal cell -> ( { model | pathGoal = Just cell, activeMenu = Nothing }, Cmd.none )
        CloseMenu -> ( { model | activeMenu = Nothing }, Cmd.none )
        NoOp -> ( model, Cmd.none )
        _ -> ( model, Cmd.none )


-- HELPER


encodeWeights : PlanningWeights -> Encode.Value
encodeWeights w =
    Encode.object
        [ ( "execution_time_default", Encode.float w.execution_time_default )
        , ( "complex_module_time", Encode.float w.complex_module_time )
        , ( "human_extra_weight", Encode.float w.human_extra_weight )
        , ( "proximity_penalty", Encode.float w.proximity_penalty )
        ]