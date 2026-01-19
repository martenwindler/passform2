module Decoders exposing (..)

import Dict exposing (Dict)
import Json.Decode as Decode exposing (Decoder, bool, field, float, int, list, maybe, string)
import Json.Encode as Encode
import Types exposing (..)
import Types.Domain exposing (..)


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


agentMapDecoder : Decoder (Dict (Int, Int) AgentModule)
agentMapDecoder =
    field "agents" (list agentModuleDecoder)
        |> Decode.map
            (\agents ->
                agents
                    |> List.map (\a -> ( ( a.position.x, a.position.y ), a ))
                    |> Dict.fromList
            )


hardwareDeviceDecoder : Decode.Decoder HardwareDevice
hardwareDeviceDecoder =
    Decode.map3 HardwareDevice
        (Decode.field "pi_id" Decode.string)
        (Decode.field "rfid_status" hardwareStatusDecoder)
        (Decode.field "pi_exists" Decode.bool)


hardwareListDecoder : Decode.Decoder (List HardwareDevice)
hardwareListDecoder =
    Decode.list hardwareDeviceDecoder


-- --- PLANNING & PATH DECODERS ---

{-| 
Dekodiert das Ergebnis einer CNP-Ausschreibung (Zuschlag).
Wir nutzen hier den Typ 'Path', da dieser die benötigten Felder 
(cost und path) bereits bereitstellt.
-}
decodePathResult : Decoder Path
decodePathResult =
    Decode.map3 Path
        (Decode.succeed 200) -- Wir setzen den Status fest auf 200 (Success)
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


encodeAgentMap : Dict (Int, Int) AgentModule -> Encode.Value
encodeAgentMap agents =
    agents
        |> Dict.values
        |> Encode.list encodeAgentModule


encodeAgentModule : AgentModule -> Encode.Value
encodeAgentModule agent =
    Encode.object
        [ ( "agent_id", agent.agent_id |> Maybe.map Encode.string |> Maybe.withDefault Encode.null )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) )
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


{-| NEU: Erzeugt das JSON für eine CNP-Aufgabenankündigung (Ausschreibung). -}
encodeCnpAnnouncement : Model -> Encode.Value
encodeCnpAnnouncement model =
    Encode.object
        [ ( "type", Encode.string "CNP_TASK_ANNOUNCEMENT" )
        , ( "payload"
          , Encode.object
                [ ( "start", model.pathStart |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "goal", model.pathGoal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "weights", encodeWeights model.planningWeights )
                ]
          )
        ]


{-| Encodiert alle Daten für den triggerPlanning Port (Bestands-Logik). -}
encodePlanningData : { start : Maybe GridCell, goal : Maybe GridCell, weights : PlanningWeights, isRanger : Bool } -> Encode.Value
encodePlanningData data =
    Encode.object
        [ ( "start", data.start |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "goal", data.goal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "weights", encodeWeights data.weights )
        , ( "isRanger", Encode.bool data.isRanger )
        ]