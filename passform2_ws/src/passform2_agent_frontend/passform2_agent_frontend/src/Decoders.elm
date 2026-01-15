module Decoders exposing (..)

import Json.Decode as Decode exposing (Decoder, field, int, string, list, maybe, float)
import Json.Encode as Encode
import Dict exposing (Dict)
import Types exposing (..)

-- --- DECODERS (JSON -> Elm) ---

-- Dekodiert eine einfache Gitter-Zelle {x: Int, y: Int}
gridCellDecoder : Decoder GridCell
gridCellDecoder =
    Decode.map2 GridCell
        (field "x" int)
        (field "y" int)

-- Dekodiert ein einzelnes Modul inklusive der Rotation
agentModuleDecoder : Decoder AgentModule
agentModuleDecoder =
    Decode.map4 AgentModule
        (maybe (field "agent_id" string))
        (field "module_type" string)
        (field "position" gridCellDecoder)
        -- Falls orientation fehlt (Abwärtskompatibilität), nehmen wir 0 als Standard
        (Decode.oneOf [ field "orientation" int, Decode.succeed 0 ])

-- Dekodiert die gesamte Liste von Agenten in ein Dict für das Model
agentMapDecoder : Decoder (Dict (Int, Int) AgentModule)
agentMapDecoder =
    list agentModuleDecoder
        |> Decode.map (\agents -> 
            List.map (\a -> ((a.position.x, a.position.y), a)) agents
                |> Dict.fromList
        )

-- Dekodiert das Ergebnis der Pfadplanung vom Backend
pathDecoder : Decoder Path
pathDecoder =
    Decode.map3 Path
        (field "status" int)
        (field "cost" float)
        (field "path" (list agentModuleDecoder))

-- Hilfs-Dekoder für Klicks aus der Three.js Web Component
decodeCellClick : Decoder GridCell
decodeCellClick =
    Decode.at ["detail"] 
        (Decode.map2 GridCell
            (field "x" int)
            (field "y" int)
        )


-- --- ENCODERS (Elm -> JSON) ---

-- Wandelt das Dict der Agenten zurück in eine JSON-Liste für Ports/Export
encodeAgentMap : Dict (Int, Int) AgentModule -> Encode.Value
encodeAgentMap agents =
    agents
        |> Dict.values
        |> Encode.list encodeAgentModule

-- Wandelt ein einzelnes Modul in JSON um (wichtig für die 3D-Szene)
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

-- Wandelt den berechneten Pfad in eine Liste für die 3D-Visualisierung um
encodePath : Maybe Path -> Encode.Value
encodePath maybePath =
    case maybePath of
        Just p ->
            Encode.list encodeAgentModule p.path

        Nothing ->
            Encode.list identity []