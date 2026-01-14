module Decoders exposing (..)

import Json.Decode as Decode exposing (Decoder, field, int, string, list, maybe)
import Json.Encode as Encode
import Dict exposing (Dict)
import Types exposing (..)

-- DECODERS
gridCellDecoder : Decoder GridCell
gridCellDecoder =
    Decode.map2 GridCell
        (field "x" int)
        (field "y" int)

agentModuleDecoder : Decoder AgentModule
agentModuleDecoder =
    Decode.map3 AgentModule
        (maybe (field "agent_id" string))
        (field "module_type" string)
        (field "position" gridCellDecoder)

agentMapDecoder : Decoder (Dict (Int, Int) AgentModule)
agentMapDecoder =
    field "agents" (list agentModuleDecoder)
        |> Decode.map (\agents -> 
            List.map (\a -> ((a.position.x, a.position.y), a)) agents
                |> Dict.fromList
        )

decodeCellClick : Decoder GridCell
decodeCellClick =
    Decode.at ["detail"] 
        (Decode.map2 GridCell
            (field "x" int)
            (field "y" int)
        )

-- ENCODERS
encodeAgentMap : Dict (Int, Int) AgentModule -> String
encodeAgentMap agents =
    agents
        |> Dict.values
        |> Encode.list encodeAgentModule
        |> Encode.encode 0

encodeAgentModule : AgentModule -> Encode.Value
encodeAgentModule agent =
    Encode.object
        [ ( "agent_id", case agent.agent_id of
                            Just s -> Encode.string s
                            Nothing -> Encode.null )
        , ( "module_type", Encode.string agent.module_type )
        , ( "position", Encode.object [ ("x", Encode.int agent.position.x), ("y", Encode.int agent.position.y) ] )
        ]