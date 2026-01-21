module Update.Agents exposing (update)

import Types exposing (..)
import Types.Domain exposing (..)
import Dict exposing (Dict)
import Ports
import Decoders
import Json.Decode as Decode

update : AgentsMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        UpdateAgents rawJson ->
            case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                Ok newAgentsDict ->
                    let
                        -- DEBUG: Wir loggen, was vom Server kommt
                        _ = Debug.log "✅ ELM: Server-Sync empfangen. Aktueller Drag-Status" model.isDragging
                        _ = Debug.log "📦 Anzahl Agenten vom Server" (Dict.size newAgentsDict)

                        -- 1. Dragging-Schutz: Lokale Daten behalten, wenn wir gerade schieben
                        finalAgents =
                            if model.isDragging then
                                model.agents
                            else
                                newAgentsDict

                        -- 2. Layout-Logik: Von Landing auf App umschalten
                        newLayout =
                            if model.activeLayout == LandingMode && not (Dict.isEmpty newAgentsDict) then
                                AppMode
                            else
                                model.activeLayout
                    in
                    ( { model 
                        | agents = finalAgents
                        , activeLayout = newLayout
                        , loading = False
                        , isDragging = False -- Quittung erhalten -> Sperre lösen
                      }
                    , Cmd.none
                    )

                Err error ->
                    let 
                        _ = Debug.log "❌ ELM: Decoder Fehler bei UpdateAgents" (Decode.errorToString error)
                    in
                    ( { model | loading = False }, Cmd.none )

        MoveAgent agentId newCell ->
            let
                _ = Debug.log "📥 ELM: MoveAgent Event für" agentId
                _ = Debug.log "📍 Ziel" newCell

                maybeAgent =
                    model.agents
                        |> Dict.values
                        |> List.filter (\a -> a.agent_id == Just agentId)
                        |> List.head
            in
            case maybeAgent of
                Just agent ->
                    let
                        -- Dictionary umbauen: Alten Platz löschen, neuen belegen
                        clearedAgents =
                            model.agents
                                |> Dict.toList
                                |> List.filter (\(_, a) -> a.agent_id /= Just agentId)
                                |> Dict.fromList

                        updatedAgents =
                            Dict.insert ( newCell.x, newCell.y ) { agent | position = newCell } clearedAgents

                        newModel = { model | agents = updatedAgents, isDragging = True }
                        
                        _ = Debug.log "🚀 ELM: Lokales Dict aktualisiert. Sende pushConfig. isDragging" newModel.isDragging
                    in
                    ( newModel
                    , Ports.pushConfig (Decoders.encodeFullConfig newModel)
                    )

                Nothing ->
                    let _ = Debug.log "⚠️ ELM: MoveAgent abgebrochen - Agent nicht gefunden" agentId in
                    ( model, Cmd.none )

        HandleGridClick cell ->
            let
                maybeAgent = Dict.get ( cell.x, cell.y ) model.agents
                
                debugInfo = 
                    { positionStr = "(" ++ String.fromInt cell.x ++ "," ++ String.fromInt cell.y ++ ")"
                    , agentGefunden = maybeAgent /= Nothing
                    , editingModus = model.editing
                    , dragging = model.isDragging
                    }
                
                _ = Debug.log "🔍 ANALYSE KLICK:" debugInfo
            in
            case maybeAgent of
                Just agent -> 
                    ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )

                Nothing ->
                    if model.editing then 
                        ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none )
                    else 
                        ( model, Cmd.none )

        StartAgent moduleType cell ->
            let
                typeStr = Decoders.moduleTypeToString moduleType
                newId = typeStr ++ "-" ++ String.fromInt cell.x ++ String.fromInt cell.y
                newAgent = { agent_id = Just newId, module_type = moduleType, position = cell, orientation = 0, is_dynamic = (moduleType == FTF), payload = Nothing, signal_strength = 100 }
                newModel = { model | agents = Dict.insert ( cell.x, cell.y ) newAgent model.agents, activeMenu = Nothing, currentPath = Nothing }
            in
            ( newModel, Ports.pushConfig (Decoders.encodeFullConfig newModel) )

        RotateAgent cell ->
            let 
                newAgents = Dict.update ( cell.x, cell.y ) (Maybe.map (\a -> { a | orientation = modBy 360 (a.orientation + 90) })) model.agents
                newModel = { model | agents = newAgents, currentPath = Nothing }
            in 
            ( newModel, Ports.pushConfig (Decoders.encodeFullConfig newModel) )

        RemoveAgent cell ->
            let
                newModel = { model | agents = Dict.remove ( cell.x, cell.y ) model.agents, activeMenu = Nothing, currentPath = Nothing }
            in
            ( newModel, Ports.pushConfig (Decoders.encodeFullConfig newModel) )

        ToggleMode ->
            let
                newMode = if model.mode == Simulation then Hardware else Simulation
                modeStr = if newMode == Simulation then "simulation" else "hardware"
                ( updatedAgents, logEntry ) =
                    case newMode of
                        Simulation -> 
                            ( model.savedDefault, { message = "Simulation aktiv.", level = Info } )
                        Hardware -> 
                            ( Dict.empty, { message = "Hardware aktiv: Sync...", level = Warning } )
            in
            ( { model | mode = newMode, agents = updatedAgents, logs = logEntry :: model.logs, currentPath = Nothing }
            , Ports.setMode modeStr
            )

        ToggleViewMode -> ( { model | is3D = not model.is3D }, Cmd.none )
        ToggleSidebar -> ( { model | sidebarOpen = not model.sidebarOpen }, Cmd.none )
        CloseMenu -> ( { model | activeMenu = Nothing }, Cmd.none )
        _ -> ( model, Cmd.none )

-- Hilfsfunktionen
getRequiredWidth : Dict (Int, Int) a -> Int
getRequiredWidth agents =
    agents |> Dict.keys |> List.map Tuple.first |> List.maximum |> Maybe.map (\x -> x + 1) |> Maybe.withDefault 10

getRequiredHeight : Dict (Int, Int) a -> Int
getRequiredHeight agents =
    agents |> Dict.keys |> List.map Tuple.second |> List.maximum |> Maybe.map (\y -> y + 1) |> Maybe.withDefault 10