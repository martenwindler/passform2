module Decoders exposing (..)

import Json.Decode as Decode exposing (Decoder, bool, field, float, int, list, maybe, string)
import Json.Encode as Encode
import Dict exposing (Dict)
import Types exposing (..)

-- --- TYPE-SAFE MAPPERS (Internal) ---

{-| Wandelt JSON-Strings in unsere typsicheren Elm-Typen um. -}
moduleTypeDecoder : Decoder ModuleType
moduleTypeDecoder =
    string |> Decode.map (\s ->
        case s of
            "ftf" -> FTF
            "conveyeur" -> Conveyeur
            "rollen_ns" -> RollenModul
            "mensch" -> Mensch
            "greifer" -> Greifer
            "tisch" -> Station
            _ -> UnknownModule s
    )

logLevelDecoder : Decoder LogLevel
logLevelDecoder =
    string |> Decode.map (\s ->
        case s of
            "success" -> Success
            "warning" -> Warning
            "error" -> Danger
            _ -> Info
    )

hardwareStatusDecoder : Decoder HardwareStatus
hardwareStatusDecoder =
    string |> Decode.map (\s ->
        case s of
            "online" -> Online
            "missing" -> Missing
            "error" -> Error
            _ -> UnknownStatus
    )

{-| Hilfsfunktion, um Elm-Typen für Ports/JSON wieder in Strings zu verwandeln. -}
moduleTypeToString : ModuleType -> String
moduleTypeToString mt =
    case mt of
        FTF -> "ftf"
        Conveyeur -> "conveyeur"
        RollenModul -> "rollen_ns"
        Mensch -> "mensch"
        Greifer -> "greifer"
        Station -> "tisch"
        UnknownModule s -> s

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

-- --- AGENT DECODERS (JSON -> Elm) ---

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
        |> andMap (field "module_type" moduleTypeDecoder) -- Typsicherer Decoder
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
        |> Decode.map (\agents -> 
            agents
                |> List.map (\a -> ((a.position.x, a.position.y), a))
                |> Dict.fromList
        )

hardwareDeviceDecoder : Decode.Decoder HardwareDevice
hardwareDeviceDecoder =
    Decode.map3 HardwareDevice
        (Decode.field "pi_id" Decode.string)
        (Decode.field "rfid_status" hardwareStatusDecoder) -- Typsicherer Decoder
        (Decode.field "pi_exists" Decode.bool)

hardwareListDecoder : Decode.Decoder (List HardwareDevice)
hardwareListDecoder =
    Decode.list hardwareDeviceDecoder

-- --- WEITERE DECODER ---

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

decodeCellClick : Decoder GridCell
decodeCellClick =
    Decode.at ["detail"] 
        (Decode.map2 GridCell
            (field "x" int)
            (field "y" int)
        )

-- --- ENCODERS (Elm -> JSON / Ports) ---

encodeAgentMap : Dict (Int, Int) AgentModule -> Encode.Value
encodeAgentMap agents =
    agents
        |> Dict.values
        |> Encode.list encodeAgentModule

encodeAgentModule : AgentModule -> Encode.Value
encodeAgentModule agent =
    Encode.object
        [ ( "agent_id"
          , case agent.agent_id of
                Just s -> Encode.string s
                Nothing -> Encode.null 
          )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) ) -- Zurück in String wandeln
        , ( "position"
          , Encode.object 
                [ ("x", Encode.int agent.position.x)
                , ("y", Encode.int agent.position.y) 
                ] 
          )
        , ( "orientation", Encode.int agent.orientation )
        , ( "is_dynamic", Encode.bool agent.is_dynamic )
        , ( "payload"
          , case agent.payload of
                Just p -> Encode.string p
                Nothing -> Encode.null
          )
        , ( "signal_strength", Encode.int agent.signal_strength )
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