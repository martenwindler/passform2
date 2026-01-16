module Main exposing (main)

import Browser
import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Http
import Dict exposing (Dict)
import Json.Decode as Decode
import Json.Encode as Encode

import Types exposing (..)
import Ports
import Decoders
import View.Navbar as Navbar
import View.Sidebar as Sidebar

-- PROGRAM
main : Program { backendIP : String, savedConfig : Maybe String } Model Msg
main =
    Browser.element
        { init = init
        , view = view
        , update = update
        , subscriptions = subscriptions
        }

-- HARTE FALLBACK-KONFIGURATION
defaultAgents : Dict (Int, Int) AgentModule
defaultAgents =
    Dict.fromList
        [ ((3, 4), { agent_id = Just "Tisch-1", module_type = "tisch", position = {x=3, y=4}, orientation = 0 })
        , ((3, 5), { agent_id = Just "Greifer-1", module_type = "greifer", position = {x=3, y=5}, orientation = 0 })
        , ((4, 4), { agent_id = Just "Tisch-2", module_type = "tisch", position = {x=4, y=4}, orientation = 0 })
        , ((4, 5), { agent_id = Just "Tisch-3", module_type = "tisch", position = {x=4, y=5}, orientation = 0 })
        , ((5, 4), { agent_id = Just "Tisch-4", module_type = "tisch", position = {x=5, y=4}, orientation = 0 })
        , ((5, 5), { agent_id = Just "Rollen-1", module_type = "rollen_ns", position = {x=5, y=5}, orientation = 0 })
        , ((6, 4), { agent_id = Just "Tisch-5", module_type = "tisch", position = {x=6, y=4}, orientation = 0 })
        , ((6, 5), { agent_id = Just "Conveyeur-1", module_type = "conveyeur", position = {x=6, y=5}, orientation = 0 })
        ]

-- INIT
init : { backendIP : String, savedConfig : Maybe String } -> ( Model, Cmd Msg )
init flags =
    let
        initialAgents =
            case flags.savedConfig of
                Just json ->
                    case Decode.decodeString Decoders.agentMapDecoder json of
                        Ok agents -> agents
                        Err _ -> defaultAgents
                Nothing ->
                    defaultAgents
    in
    ( { mode = Simulation
      , backendIP = flags.backendIP
      , connected = True
      , agents = initialAgents
      , savedDefault = initialAgents
      , logs = [ { message = "System gestartet. Bereit.", level = "info" } ]
      , pathStart = Nothing
      , pathGoal = Nothing
      , currentPath = Nothing
      , hoveredCell = Nothing
      , sidebarOpen = False
      , gridWidth = 8 
      , gridHeight = 4
      , editing = True
      , is3D = False 
      , loading = False
      , activeMenu = Nothing
      }
    , Ports.connectToBackend flags.backendIP
    )

-- UPDATE
update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- AGENTEN MANAGEMENT
        RotateAgent cell ->
            let
                newAgents = Dict.update (cell.x, cell.y) (Maybe.map (\a -> { a | orientation = modBy 360 (a.orientation + 90) })) model.agents
            in
            ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )

        MoveAgent { oldX, oldY, newX, newY } ->
            let
                isOccupied = Dict.member (newX, newY) model.agents
            in
            if isOccupied then
                ( model, Cmd.none )
            else
                case Dict.get (oldX, oldY) model.agents of
                    Just agent ->
                        let
                            tempAgents = Dict.remove (oldX, oldY) model.agents
                            updatedAgent = { agent | position = { x = newX, y = newY } }
                            newAgents = Dict.insert (newX, newY) updatedAgent tempAgents
                        in
                        ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )
                    Nothing ->
                        ( model, Cmd.none )

        StartAgent moduleType cell ->
            let
                newId = (String.fromInt cell.x) ++ "-" ++ (String.fromInt cell.y)
                newAgent = { agent_id = Just newId, module_type = moduleType, position = cell, orientation = 0 }
            in
            ( { model | agents = Dict.insert (cell.x, cell.y) newAgent model.agents, activeMenu = Nothing, currentPath = Nothing }, Cmd.none )

        RemoveAgent cell ->
            ( { model | agents = Dict.remove (cell.x, cell.y) model.agents, activeMenu = Nothing, currentPath = Nothing }, Cmd.none )

        -- PERSISTENZ
        SetCurrentAsDefault ->
            let
                jsonString = model.agents |> Decoders.encodeAgentMap |> Encode.encode 0
                newLog = { message = "Standardkonfiguration lokal gesichert.", level = "success" }
            in
            ( { model | savedDefault = model.agents, logs = newLog :: model.logs }
            , Ports.saveToLocalStorage jsonString 
            )

        LoadDefaultConfig ->
            ( { model | agents = model.savedDefault, pathStart = Nothing, pathGoal = Nothing, currentPath = Nothing }, Cmd.none )

        ClearGrid ->
            ( { model | agents = Dict.empty, pathStart = Nothing, pathGoal = Nothing, currentPath = Nothing }, Cmd.none )

        -- PFADPLANUNG
        StartPlanning _ ->
            case ( model.pathStart, model.pathGoal ) of
                ( Just start, Just goal ) ->
                    let
                        payload =
                            Encode.object
                                [ ( "start", Encode.object [ ("x", Encode.int start.x), ("y", Encode.int start.y) ] )
                                , ( "goal", Encode.object [ ("x", Encode.int goal.x), ("y", Encode.int goal.y) ] )
                                , ( "agents", Decoders.encodeAgentMap model.agents )
                                ]
                    in
                    ( { model | loading = True, logs = { message = "Pfadplanung gestartet...", level = "info" } :: model.logs }
                    , Ports.triggerPlanning payload 
                    )
                _ -> ( model, Cmd.none )

        PlanningResultRaw rawJson ->
            case Decode.decodeValue Decoders.pathDecoder rawJson of 
                Ok path -> ( { model | currentPath = Just path, loading = False, logs = { message = "Pfad erfolgreich berechnet.", level = "success" } :: model.logs }, Cmd.none )
                Err _ -> ( { model | loading = False, logs = { message = "Fehler bei Pfadberechnung.", level = "warning" } :: model.logs }, Cmd.none )

        -- AGENTEN LIVE-UPDATE (Vom Backend/ROS)
        UpdateAgents rawJson ->
            case model.mode of
                Hardware ->
                    case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                        Ok newAgentsDict ->
                            ( { model | agents = newAgentsDict }, Cmd.none )
                        Err _ ->
                            ( model, Cmd.none )

                Simulation ->
                    ( model, Cmd.none )

        -- Die Verbindung zum Backend (Nur einmal!)
        SetConnected status ->
            ( { model | connected = status }, Cmd.none )

        -- SYSTEM-LOGS
        HandleSystemLog result ->
            case result of
                Ok logEntry ->
                    ( { model | logs = List.take 20 (logEntry :: model.logs) }, Cmd.none )
                Err err ->
                    ( model, Cmd.none )

        -- RFID SCANS
        HandleRfid result ->
            case result of
                Ok rfidId ->
                    let
                        newLog = { message = "RFID Scan: " ++ rfidId, level = "success" }
                    in
                    ( { model | logs = List.take 20 (newLog :: model.logs) }, Cmd.none )
                Err _ ->
                    ( model, Cmd.none )

        -- MODUS WECHSEL
        ToggleMode ->
            let
                newMode = if model.mode == Simulation then Hardware else Simulation
                modeStr = if newMode == Simulation then "simulation" else "hardware"
                
                ( updatedAgents, logEntry ) =
                    if newMode == Simulation then
                        ( model.savedDefault
                        , { message = "Wechsel zu Simulation: Standard geladen.", level = "info" }
                        )
                    else
                        ( Dict.empty
                        , { message = "Wechsel zu Hardware: Warte auf ROS-Agenten...", level = "warning" }
                        )
            in
            ( { model 
                | mode = newMode
                , agents = updatedAgents
                , logs = logEntry :: model.logs
                , currentPath = Nothing 
              }
            , Ports.setMode modeStr 
            )

        ConfigReceived jsonString ->
            case Decode.decodeString Decoders.agentMapDecoder jsonString of
                Ok newAgents -> ( { model | agents = newAgents, currentPath = Nothing }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        -- UI & NAVIGATION
        HandleGridClick cell ->
            case Dict.get (cell.x, cell.y) model.agents of
                Just agent -> ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )
                Nothing -> if model.editing then ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none ) else ( model, Cmd.none )

        ToggleSidebar -> ( { model | sidebarOpen = not model.sidebarOpen }, Cmd.none )
        ToggleViewMode -> ( { model | is3D = not model.is3D }, Cmd.none )
        SetPathStart cell -> ( { model | pathStart = Just cell, activeMenu = Nothing }, Cmd.none )
        SetPathGoal cell -> ( { model | pathGoal = Just cell, activeMenu = Nothing }, Cmd.none )
        CloseMenu -> ( { model | activeMenu = Nothing }, Cmd.none )
        SetGridWidth val -> ( { model | gridWidth = String.toInt val |> Maybe.withDefault 1 }, Cmd.none )
        SetGridHeight val -> ( { model | gridHeight = String.toInt val |> Maybe.withDefault 1 }, Cmd.none )
        
        NoOp -> ( model, Cmd.none )
        _ -> ( model, Cmd.none )

-- SUBSCRIPTIONS
subscriptions : Model -> Sub Msg
subscriptions model =
    Sub.batch
        [ -- 1. Status der Verbindung (Online/Offline)
          Ports.socketStatusReceiver SetConnected
        
        -- 2. Das Herzstück: Agenten-Updates vom Backend
        -- Empfängt das JSON {"agents": [...]}, das im 'update' decodiert wird
        , Ports.activeAgentsReceiver UpdateAgents 
        
        -- 3. Pfad-Ergebnisse vom neuen internen Planer
        , Ports.pathCompleteReceiver PlanningResultRaw 
        
        -- 4. Konfigurations-Daten (Import/Export)
        , Ports.configReceived ConfigReceived 
        
        -- 5. System-Logs für die Sidebar
        -- Wir decodieren hier direkt und schicken das Resultat an HandleSystemLog
        , Ports.systemLogReceiver (Decode.decodeValue Decoders.decodeSystemLog >> HandleSystemLog)
        
        -- 6. RFID/NFC-Events von den echten Sensoren
        , Ports.rfidReceiver (Decode.decodeValue Decoders.decodeRfid >> HandleRfid)
        ]

-- VIEW
view : Model -> Html Msg
view model =
    div [ class "app-layout" ]
        [ Navbar.view model
        , div [ class "content-area" ]
            [ view3D model
            , if model.sidebarOpen then Sidebar.view model else text ""
            ]
        , viewActiveMenu model.activeMenu
        ]

view3D : Model -> Html Msg
view3D model =
    node "three-grid-scene"
        [ attribute "agents" (model.agents |> Decoders.encodeAgentMap |> Encode.encode 0)
        , attribute "path" (Decoders.encodePath model.currentPath |> Encode.encode 0)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        , attribute "grid-width" (String.fromInt model.gridWidth)
        , attribute "grid-height" (String.fromInt model.gridHeight)
        , attribute "start-pos" (model.pathStart |> Maybe.map (\c -> Encode.object [("x", Encode.int c.x), ("y", Encode.int c.y)]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        , attribute "goal-pos" (model.pathGoal |> Maybe.map (\c -> Encode.object [("x", Encode.int c.x), ("y", Encode.int c.y)]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        , Html.Events.on "agent-moved" (Decode.map MoveAgent decodeAgentMove)
        , Html.Events.on "cell-clicked" (Decode.map HandleGridClick Decoders.decodeCellClick)
        ]
        []

decodeAgentMove : Decode.Decoder { oldX : Int, oldY : Int, newX : Int, newY : Int }
decodeAgentMove =
    Decode.at ["detail"] <|
        Decode.map4 (\ox oy nx ny -> { oldX = ox, oldY = oy, newX = nx, newY = ny })
            (Decode.field "oldX" Decode.int)
            (Decode.field "oldY" Decode.int)
            (Decode.field "newX" Decode.int)
            (Decode.field "newY" Decode.int)

viewActiveMenu : Maybe MenuType -> Html Msg
viewActiveMenu maybeMenu =
    case maybeMenu of
        Just (SelectionMenu cell) ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [] [ text "Agent hinzufügen" ]
                    , button [ onClick (StartAgent "rollen_ns" cell) ] [ text "Rollen Modul" ]
                    , button [ onClick (StartAgent "greifer" cell) ] [ text "Greifer Modul" ]
                    , button [ onClick (StartAgent "tisch" cell) ] [ text "Tisch Modul" ]
                    , button [ onClick (StartAgent "conveyeur" cell) ] [ text "Förderband" ]
                    , hr [] []
                    , button [ onClick CloseMenu, class "btn-close" ] [ text "Abbrechen" ]
                    ]
                ]
        Just (SettingsMenu cell agent) ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [] [ text ("Modul: " ++ (agent.module_type |> Sidebar.formatType)) ]
                    , button [ onClick (RotateAgent cell) ] [ text "Modul drehen (90°)" ]
                    , hr [] []
                    , button [ onClick (SetPathStart cell), class "btn-path-start" ] [ text "Als Start" ]
                    , button [ onClick (SetPathGoal cell), class "btn-path-goal" ] [ text "Als Ziel" ]
                    , hr [] []
                    , button [ onClick (RemoveAgent cell), class "btn-delete" ] [ text "Modul löschen" ]
                    , button [ onClick CloseMenu, class "btn-close" ] [ text "Schließen" ]
                    ]
                ]
        Nothing -> text ""