module Decoders exposing (..)

import Json.Decode as Decode exposing (Decoder, field, int, string, list, maybe, float, bool)
import Json.Encode as Encode
import Dict exposing (Dict)
import Types exposing (..)

-- --- SYSTEM DECODERS ---

decodeSystemLog : Decoder SystemLog
decodeSystemLog =
    Decode.map2 SystemLog
        (field "message" string)
        (field "level" string)

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
        |> andMap (field "module_type" string)
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

-- Erwartet das Objekt {"agents": [...]}
agentMapDecoder : Decoder (Dict (Int, Int) AgentModule)
agentMapDecoder =
    field "agents" (list agentModuleDecoder)
        |> Decode.map (\agents -> 
            agents
                |> List.map (\a -> ((a.position.x, a.position.y), a))
                |> Dict.fromList
        )

-- --- WEITERE DECODER ---

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

-- --- ENCODERS (Elm -> JSON) ---

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
        , ( "module_type", Encode.string agent.module_type )
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
        ]

encodePath : Maybe Path -> Encode.Value
encodePath maybePath =
    case maybePath of
        Just p ->
            Encode.list encodeAgentModule p.path

        Nothing ->
            Encode.list identity []