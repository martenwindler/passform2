module Update.Agents exposing (update)

import Decoders
import Dict exposing (Dict)
import Json.Decode as Decode
import Ports
import Types exposing (..)
import Types.Domain exposing (..)


{-| Hilfsfunktion zum Abgleichen der Belegung basierend auf Agenten-Positionen.
Prüft primär Level 0, da Bays Boden-basiert sind.
-}
updateOccupancy : Dict (Int, Int, Int) AgentModule -> List Bay -> List Bay
updateOccupancy agents bays =
    List.map
        (\bay ->
            let
                -- floor sorgt dafür, dass 2.5 zu 2 wird (passend zum Int-Grid)
                posX = floor bay.origin.x
                posY = floor bay.origin.y

                -- Suche Agent auf Level 0 an dieser Stelle
                maybeAgent = Dict.get ( posX, posY, 0 ) agents
            in
            case maybeAgent of
                Just agent ->
                    -- Hier wird die ID des Agenten explizit in die Bay geschrieben
                    { bay | occupation = True, module_uuid = Maybe.withDefault "" agent.agent_id }

                Nothing ->
                    { bay | occupation = False, module_uuid = "" }
        )
        bays


update : AgentsMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        UpdateAgents rawJson ->
            case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                Ok newAgentsDict ->
                    let
                        finalAgents =
                            if model.isDragging then
                                model.agents

                            else
                                newAgentsDict

                        newLayout =
                            if model.activeLayout == LandingMode && not (Dict.isEmpty newAgentsDict) then
                                AppMode

                            else
                                model.activeLayout

                        newBays =
                            updateOccupancy finalAgents model.bays
                    in
                    ( { model
                        | agents = finalAgents
                        , bays = newBays
                        , activeLayout = newLayout
                        , loading = False
                        , isDragging = False
                      }
                    , Cmd.none
                    )

                Err error ->
                    let
                        _ =
                            Debug.log "❌ ELM: Decoder Fehler bei UpdateAgents" (Decode.errorToString error)
                    in
                    ( { model | loading = False }, Cmd.none )

        MoveAgent agentId targetCell ->
            let
                -- 1. Den Agenten anhand seiner ID finden (egal wo er im Dict steckt)
                maybeAgent =
                    model.agents
                        |> Dict.values
                        |> List.filter (\a -> a.agent_id == Just agentId)
                        |> List.head
            in
            case maybeAgent of
                Just agent ->
                    let
                        oldKey = ( agent.position.x, agent.position.y, agent.position.level )

                        -- 2. Level-Berechnung für das Stacking:
                        -- Wir schauen, wie viele Agenten (ohne den aktuell bewegten) schon an targetCell (x,y) stehen.
                        newLevel =
                            model.agents
                                |> Dict.filter (\(x, y, _) _ -> x == targetCell.x && y == targetCell.y)
                                |> Dict.filter (\key _ -> key /= oldKey) -- Den "Self-Count" verhindern
                                |> Dict.size

                        -- 3. Z-Koordinate für ROS/3D berechnen (400mm pro Etage)
                        newZ = newLevel * 400

                        updatedAgent =
                            { agent | position = 
                                { x = targetCell.x
                                , y = targetCell.y
                                , z = newZ
                                , level = newLevel 
                                } 
                            }

                        -- 4. Agenten-Map aktualisieren (Verschieben im Dict)
                        newAgents =
                            model.agents
                                |> Dict.remove oldKey
                                |> Dict.insert ( targetCell.x, targetCell.y, newLevel ) updatedAgent

                        -- 5. SPATIAL MAPPING: Buchten-Zustand berechnen
                        -- Wir iterieren über alle Buchten und prüfen, ob auf Level 0 ein Agent steht.
                        newBays =
                            model.bays
                                |> List.map (\bay ->
                                    let
                                        occupyingAgent =
                                            newAgents
                                                |> Dict.get ( round bay.origin.x, round bay.origin.y, 0 )

                                        isOccupied =
                                            case occupyingAgent of
                                                Just _ -> True
                                                Nothing -> False

                                        agentUuid =
                                            occupyingAgent 
                                                |> Maybe.andThen .agent_id 
                                                |> Maybe.withDefault ""
                                    in
                                    { bay | occupation = isOccupied, module_uuid = agentUuid }
                                )

                        updatedModel = { model | agents = newAgents, bays = newBays }
                    in
                    ( updatedModel
                    , Ports.pushConfig (Decoders.encodeFullConfig updatedModel) 
                    )

                Nothing ->
                    ( model, Cmd.none )

        HandleGridClick cell ->
            let
                -- Suche alle Agenten an dieser X/Y-Stelle
                agentsAtPos =
                    model.agents
                        |> Dict.toList
                        |> List.filter (\((x, y, l), _) -> x == cell.x && y == cell.y)
                        |> List.sortBy (\((_, _, l), _) -> l)
                        |> List.reverse -- Oberster Agent zuerst

                maybeTopEntry = List.head agentsAtPos
            in
            case maybeTopEntry of
                Just ((x, y, level), agent) ->
                    -- Wir öffnen das SettingsMenu für den obersten Agenten
                    ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )

                Nothing ->
                    -- Nichts da? Dann direkt zum Auswahlmenü (Ebene 0)
                    if model.editing then
                        ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none )
                    else
                        ( model, Cmd.none )

        StartAgent moduleType cell ->
            let
                -- 1. Zähle vorhandene Agenten an dieser Position für das Level
                currentLevel =
                    model.agents
                        |> Dict.keys
                        |> List.filter (\( x, y, _ ) -> x == cell.x && y == cell.y)
                        |> List.length

                -- 2. 400 Einheiten (mm) Höhe pro Level
                newZ =
                    currentLevel * 400

                -- 3. Eindeutige ID generieren (z.B. "Greifer-1-2-L1")
                newId =
                    formatModuleType moduleType 
                        ++ "-" ++ String.fromInt cell.x 
                        ++ "-" ++ String.fromInt cell.y 
                        ++ "-L" ++ String.fromInt currentLevel

                -- 4. Der vollständige Agent-Record (alle 8 Felder!)
                newAgent =
                    { agent_id = Just newId
                    , module_type = moduleType
                    , position = { x = cell.x, y = cell.y, z = newZ, level = currentLevel }
                    , orientation = 0
                    , is_dynamic = False
                    , payload = Nothing            -- NEU: Da vom Typ Maybe String
                    , signal_strength = 100        -- NEU: Startwert für die Anzeige
                    , status = Online              -- NEU: Da vom Typ HardwareStatus
                    }

                updatedAgents =
                    Dict.insert ( cell.x, cell.y, currentLevel ) newAgent model.agents
            in
            ( { model | agents = updatedAgents, activeMenu = Nothing }
            , Ports.pushConfig (Decoders.encodeFullConfig { model | agents = updatedAgents })
            )

        OpenSelectionMenu cell ->
            ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none )

        RotateAgent cell ->
            let
                -- Nutze cell.level, um den richtigen Agenten im Stapel zu drehen
                newAgents =
                    Dict.update ( cell.x, cell.y, cell.level )
                        (Maybe.map (\a -> { a | orientation = modBy 360 (a.orientation + 90) }))
                        model.agents

                newModel =
                    { model | agents = newAgents, currentPath = Nothing }
            in
            ( newModel, Ports.pushConfig (Decoders.encodeFullConfig newModel) )

        RemoveAgent cell ->
            let
                updatedAgents =
                    Dict.remove ( cell.x, cell.y, cell.level ) model.agents

                updatedBays =
                    updateOccupancy updatedAgents model.bays
                    
                newModel = 
                    { model 
                    | agents = updatedAgents
                    , bays = updatedBays
                    , activeMenu = Nothing 
                    }
            in
            ( newModel, Ports.pushConfig (Decoders.encodeFullConfig newModel) )

        ToggleMode ->
            let
                newMode = if model.mode == Simulation then Hardware else Simulation
                modeStr = if newMode == Simulation then "simulation" else "hardware"

                -- Wir stellen sicher, dass updatedAgents als 3D-Dict erkannt wird
                ( updatedAgents, logEntry ) =
                    case newMode of
                        Simulation ->
                            ( model.savedDefault, { message = "Simulation aktiv.", level = Info } )

                        Hardware ->
                            ( Dict.empty, { message = "Hardware aktiv: Sync...", level = Warning } )

                updatedBays =
                    updateOccupancy updatedAgents model.bays
            in
            ( { model | mode = newMode, agents = updatedAgents, bays = updatedBays, logs = logEntry :: model.logs, currentPath = Nothing }
            , Ports.setMode modeStr
            )

        ToggleViewMode ->
            ( { model | is3D = not model.is3D }, Cmd.none )

        CloseMenu ->
            ( { model | activeMenu = Nothing }, Cmd.none )

        _ ->
            ( model, Cmd.none )


-- --- HILFSFUNKTIONEN ---

getRequiredWidth : Dict ( Int, Int, Int ) a -> Int
getRequiredWidth agents =
    agents
        |> Dict.keys
        |> List.map (\( x, y, l ) -> x)
        |> List.maximum
        |> Maybe.map (\x -> x + 1)
        |> Maybe.withDefault 10


getRequiredHeight : Dict ( Int, Int, Int ) a -> Int
getRequiredHeight agents =
    agents
        |> Dict.keys
        |> List.map (\( x, y, l ) -> y)
        |> List.maximum
        |> Maybe.map (\y -> y + 1)
        |> Maybe.withDefault 10

formatModuleType : ModuleType -> String
formatModuleType mType =
    case mType of
        FTF -> "FTF"
        Conveyeur -> "Conveyeur"
        RollenModul -> "Rollen"
        Mensch -> "Mensch"
        Greifer -> "Greifer"
        Station -> "Station"
        UnknownModule name -> name