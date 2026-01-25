module Decoders exposing (..)

import Types exposing (..)
import Types.Domain exposing (..)
import Dict exposing (Dict)
import Json.Decode as Decode exposing (Decoder, bool, field, float, int, list, maybe, string)
import Json.Encode as Encode

-- --- TYPE-SAFE MAPPERS (Internal) ---


moduleTypeDecoder : Decoder ModuleType
moduleTypeDecoder =
    string
        |> Decode.map
            (\s ->
                case s of
                    "ftf" ->
                        FTF

                    "conveyeur" ->
                        Conveyeur

                    "rollen_ns" ->
                        RollenModul

                    "mensch" ->
                        Mensch

                    "greifer" ->
                        Greifer

                    "tisch" ->
                        Station

                    _ ->
                        UnknownModule s
            )


logLevelDecoder : Decoder LogLevel
logLevelDecoder =
    string
        |> Decode.map
            (\s ->
                case s of
                    "success" ->
                        Success

                    "warning" ->
                        Warning

                    "error" ->
                        Danger

                    _ ->
                        Info
            )


hardwareStatusDecoder : Decoder HardwareStatus
hardwareStatusDecoder =
    string
        |> Decode.map
            (\s ->
                case s of
                    "online" ->
                        Online

                    "missing" ->
                        Missing

                    "error" ->
                        Error

                    _ ->
                        UnknownStatus
            )


moduleTypeToString : ModuleType -> String
moduleTypeToString mt =
    case mt of
        FTF ->
            "ftf"

        Conveyeur ->
            "conveyeur"

        RollenModul ->
            "rollen_ns"

        Mensch ->
            "mensch"

        Greifer ->
            "greifer"

        Station ->
            "tisch"

        UnknownModule s ->
            s

-- --- SYSTEM DECODERS ---


decodeSystemLog : Decoder SystemLog
decodeSystemLog =
    Decode.map2 SystemLog
        (field "message" string)
        (field "level" logLevelDecoder)


decodeRfid : Decoder String
decodeRfid =
    Decode.oneOf
        [ field "rfid_id" string
        , string
        ]


-- --- AGENT DECODERS ---


gridCellDecoder : Decoder GridCell
gridCellDecoder =
    Decode.map2 GridCell
        (field "x" int)
        (field "y" int)


andMap : Decoder a -> Decoder (a -> b) -> Decoder b
andMap decoderA decoderFunction =
    Decode.map2 (\value function -> function value) decoderA decoderFunction


agentModuleDecoder : Decoder AgentModule
agentModuleDecoder =
    Decode.succeed AgentModule
        |> andMap (field "agent_id" (maybe string))
        |> andMap (field "module_type" moduleTypeDecoder)
        |> andMap (field "position" gridCellDecoder)
        |> andMap
            (Decode.oneOf
                [ field "orientation" int
                , field "orientation" float |> Decode.map round
                , Decode.succeed 0
                ]
            )
        |> andMap (Decode.oneOf [ field "is_dynamic" bool, Decode.succeed False ])
        |> andMap (Decode.oneOf [ field "payload" (maybe string), Decode.succeed Nothing ])
        |> andMap (Decode.oneOf [ field "signal_strength" int, Decode.succeed 100 ])

-- Ersetze deinen agentMapDecoder in Decoders.elm durch diesen:

agentMapDecoder : Decode.Decoder (Dict (Int, Int) AgentModule)
agentMapDecoder =
    -- Wir probieren drei Varianten, falls eine scheitert:
    Decode.oneOf
        [ -- 1. Das Ideal: Ein Objekt mit "agents": [...]
          Decode.field "agents" decodeAgentListAsDict
        , -- 2. Fallback: Eine direkte Liste von Agenten [...]
          decodeAgentListAsDict
        , -- 3. Fallback: Ein direktes Dictionary {"0,0": {...}}
          decodeAgentDictDirectly
        ]


-- HILFSFUNKTIONEN FÜR DEN DECODER --

decodeAgentListAsDict : Decode.Decoder (Dict (Int, Int) AgentModule)
decodeAgentListAsDict =
    Decode.list agentDecoder
        |> Decode.map (\list -> 
            list 
                |> List.map (\a -> ( (a.position.x, a.position.y), a ))
                |> Dict.fromList
        )

decodeAgentDictDirectly : Decode.Decoder (Dict (Int, Int) AgentModule)
decodeAgentDictDirectly =
    Decode.dict agentDecoder
        |> Decode.map (\dict ->
            dict
                |> Dict.values
                |> List.map (\a -> ( (a.position.x, a.position.y), a ))
                |> Dict.fromList
        )

agentDecoder : Decode.Decoder AgentModule
agentDecoder =
    Decode.succeed AgentModule
        |> andMap (Decode.maybe (Decode.field "agent_id" Decode.string))
        |> andMap (Decode.field "module_type" decodeModuleType)
        |> andMap decodeFlexiblePosition -- KORREKTUR: Sucht überall nach x und y
        |> andMap (Decode.oneOf [ Decode.field "orientation" Decode.int, Decode.succeed 0 ])
        |> andMap (Decode.oneOf [ Decode.field "is_dynamic" Decode.bool, Decode.succeed False ])
        |> andMap (Decode.oneOf [ Decode.field "payload" (Decode.maybe Decode.string), Decode.succeed Nothing ])
        |> andMap (Decode.oneOf [ Decode.field "signal_strength" Decode.int, Decode.succeed 100 ])

-- Hilfsfunktion: Versucht x/y flach ODER in einem position-Objekt zu finden
decodeFlexiblePosition : Decode.Decoder GridCell
decodeFlexiblePosition =
    Decode.oneOf
        [ -- Variante A: { "position": { "x": 1, "y": 1 } }
          Decode.field "position" (Decode.map2 GridCell (Decode.field "x" Decode.int) (Decode.field "y" Decode.int))
        , -- Variante B: { "x": 1, "y": 1 }
          Decode.map2 GridCell (Decode.field "x" Decode.int) (Decode.field "y" Decode.int)
        ]

-- Neue Hilfsfunktion, um x/y flach in ein GridCell zu verwandeln
decodePositionFromFlat : Decode.Decoder GridCell
decodePositionFromFlat =
    Decode.map2 GridCell
        (Decode.field "x" Decode.int)
        (Decode.field "y" Decode.int)

decodeGridCell : Decode.Decoder GridCell
decodeGridCell =
    Decode.map2 GridCell
        (Decode.field "x" Decode.int)
        (Decode.field "y" Decode.int)

-- Hilfsfunktion um "0,0" zu (0,0) zu machen
parseKey : String -> (Int, Int)
parseKey str =
    case String.split "," str of
        [ xStr, yStr ] ->
            ( String.toInt xStr |> Maybe.withDefault 0
            , String.toInt yStr |> Maybe.withDefault 0
            )
        _ ->
            ( 0, 0 )

decodeCell : Decode.Decoder GridCell
decodeCell =
    Decode.map2 GridCell
        (Decode.field "x" Decode.int)
        (Decode.field "y" Decode.int)

hardwareDeviceDecoder : Decode.Decoder HardwareDevice
hardwareDeviceDecoder =
    Decode.map3 HardwareDevice
        (Decode.field "pi_id" Decode.string)
        (Decode.field "rfid_status" hardwareStatusDecoder)
        (Decode.field "pi_exists" Decode.bool)


hardwareListDecoder : Decode.Decoder (List HardwareDevice)
hardwareListDecoder =
    Decode.list hardwareDeviceDecoder


decodeModuleType : Decoder ModuleType
decodeModuleType =
    Decode.string
        |> Decode.andThen
            (\str ->
                case String.toLower str of
                    "ftf" -> Decode.succeed FTF
                    "conveyeur" -> Decode.succeed Conveyeur
                    "rollenmodul" -> Decode.succeed RollenModul
                    "rollen_ns" -> Decode.succeed RollenModul -- Mapping für dein Backend
                    "mensch" -> Decode.succeed Mensch
                    "greifer" -> Decode.succeed Greifer
                    "station" -> Decode.succeed Station
                    _ -> Decode.succeed (UnknownModule str)
            )

-- --- PLANNING & PATH DECODERS ---

decodePathResult : Decoder Path
decodePathResult =
    Decode.map3 Path
        (Decode.oneOf [ field "status" int, Decode.succeed 200 ])
        (field "cost" float)
        (field "path" (list agentModuleDecoder))


planningWeightsDecoder : Decode.Decoder PlanningWeights
planningWeightsDecoder =
    Decode.map5 PlanningWeights
        (Decode.field "execution_time_default" Decode.float)
        (Decode.field "complex_module_time" Decode.float)
        (Decode.field "human_extra_weight" Decode.float)
        (Decode.field "proximity_penalty" Decode.float)
        (Decode.field "hardware_safety_factor" Decode.float)


pathDecoder : Decoder Path
pathDecoder =
    Decode.map3 Path
        (field "status" int)
        (field "cost" float)
        (field "path" (list agentModuleDecoder))


-- --- ENCODERS ---

{-| NEU: Erzeugt das JSON für eine CNP-Aufgabenankündigung.
    Inklusive Agents-Liste, damit das Backend die Weltkarte kennt.
-}
encodeCnpAnnouncement : Model -> Encode.Value
encodeCnpAnnouncement model =
    Encode.object
        [ ( "type", Encode.string "CNP_TASK_ANNOUNCEMENT" )
        , ( "payload"
          , Encode.object
                [ ( "start", model.pathStart |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "goal", model.pathGoal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "weights", encodeWeights model.planningWeights )
                , ( "agents", encodeAgentMap model.agents )
                ]
          )
        ]


encodeAgentMap : Dict (Int, Int) AgentModule -> Encode.Value
encodeAgentMap agents =
    -- WICHTIG: Hier muss eine LISTE erzeugt werden, kein Objekt!
    Encode.list encodeAgent (Dict.values agents)



encodeAgentModule : AgentModule -> Encode.Value
encodeAgentModule agent =
    Encode.object
        [ ( "agent_id", agent.agent_id |> Maybe.map Encode.string |> Maybe.withDefault Encode.null )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) )
        , ( "x", Encode.int agent.position.x )
        , ( "y", Encode.int agent.position.y )
        , ( "position", encodeGridCell agent.position )
        , ( "orientation", Encode.int agent.orientation )
        , ( "is_dynamic", Encode.bool agent.is_dynamic )
        , ( "payload", agent.payload |> Maybe.map Encode.string |> Maybe.withDefault Encode.null )
        , ( "signal_strength", Encode.int agent.signal_strength )
        ]


encodeGridCell : GridCell -> Encode.Value
encodeGridCell cell =
    Encode.object
        [ ( "x", Encode.int cell.x )
        , ( "y", Encode.int cell.y )
        ]


encodePath : Maybe Path -> Encode.Value
encodePath maybePath =
    case maybePath of
        Just p ->
            Encode.list encodeAgentModule p.path

        Nothing ->
            Encode.list identity []


encodeWeights : PlanningWeights -> Encode.Value
encodeWeights w =
    Encode.object
        [ ( "execution_time_default", Encode.float w.execution_time_default )
        , ( "complex_module_time", Encode.float w.complex_module_time )
        , ( "human_extra_weight", Encode.float w.human_extra_weight )
        , ( "proximity_penalty", Encode.float w.proximity_penalty )
        , ( "hardware_safety_factor", Encode.float w.hardware_safety_factor )
        ]


encodePlanningData : { start : Maybe GridCell, goal : Maybe GridCell, weights : PlanningWeights, isRanger : Bool } -> Encode.Value
encodePlanningData data =
    Encode.object
        [ ( "start", data.start |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "goal", data.goal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "weights", encodeWeights data.weights )
        , ( "isRanger", Encode.bool data.isRanger )
        ]

encodeFullConfig model =
    Encode.object
        [ ( "agents", encodeAgentMap model.agents )
        , ( "config"
          , Encode.object
                [ ( "grid"
                  , Encode.object
                        [ ( "width", Encode.int model.gridWidth )
                        , ( "height", Encode.int model.gridHeight )
                        ]
                  )
                ]
          )
        ]

encodeAgent : AgentModule -> Encode.Value
encodeAgent agent =
    Encode.object
        [ ( "agent_id", Encode.string (Maybe.withDefault "unknown" agent.agent_id) )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) )
        , ( "x", Encode.int agent.position.x )
        , ( "y", Encode.int agent.position.y )
        , ( "orientation", Encode.int agent.orientation )
        , ( "is_dynamic", Encode.bool agent.is_dynamic )
        , ( "signal_strength", Encode.int agent.signal_strength )
        ]