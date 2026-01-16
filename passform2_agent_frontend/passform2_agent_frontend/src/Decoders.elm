module Decoders exposing (..)

import Json.Decode as Decode exposing (Decoder, field, int, string, list, maybe, float)
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

-- Liest x und y aus dem aktuellen Kontext
gridCellDecoder : Decoder GridCell
gridCellDecoder =
    Decode.map2 GridCell
        (field "x" int)
        (field "y" int)

-- --- AGENT DECODERS (JSON -> Elm) ---

-- KORREKTUR: Wir lesen orientation als float (sicherer) und runden dann
agentModuleDecoder : Decoder AgentModule
agentModuleDecoder =
    Decode.map4 AgentModule
        (field "agent_id" (maybe string))
        (field "module_type" string)
        gridCellDecoder
        -- Wir erzwingen hier die Umwandlung, egal ob Python Int oder Float schickt
        (Decode.oneOf 
            [ field "orientation" int
            , field "orientation" float |> Decode.map round
            , Decode.succeed 0 
            ]
        )

-- KORREKTUR: Die Hilfsfunktion für den Port/Update muss den richtigen Decoder nutzen!
decodeIncomingAgents : Decode.Value -> Msg
decodeIncomingAgents jsonValue =
    -- Wir nutzen hier den agentMapDecoder, da er das {"agents": [...]} Format beherrscht
    -- und direkt das fertige Dict für dein Model liefert.
    case Decode.decodeValue agentMapDecoder jsonValue of
        Ok agentsDict ->
            -- WICHTIG: Deine Msg in Types.elm muss dieses Dict aufnehmen können!
            -- Falls UpdateAgents nur Decode.Value nimmt, musst du dies in der Main.elm handeln.
            UpdateAgents jsonValue 

        Err err ->
            let
                _ = Debug.log "ELM DECODER ERROR" (Decode.errorToString err)
            in
            NoOp

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
        ]

encodePath : Maybe Path -> Encode.Value
encodePath maybePath =
    case maybePath of
        Just p ->
            Encode.list encodeAgentModule p.path

        Nothing ->
            Encode.list identity []