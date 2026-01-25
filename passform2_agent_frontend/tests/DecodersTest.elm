module DecodersTest exposing (suite)

import Test exposing (..)
import Expect
import Json.Decode as Decode
import Decoders
import Types exposing (..)
import Types.Domain exposing (..)
import Dict

suite : Test
suite =
    describe "Schnittstellen-Check: Inbound JSON"
        [ test "Agent-Map Decoder liest ROS-Daten korrekt ein" <|
            \_ ->
                let
                    json = """{ "agents": [ { "agent_id": "FTF-01", "module_type": "ftf", "position": {"x": 1, "y": 1}, "orientation": 0, "is_dynamic": true, "signal_strength": 80 } ] }"""
                    result = Decode.decodeString Decoders.agentMapDecoder json
                in
                case result of
                    Ok dict -> Expect.equal (Dict.size dict) 1
                    Err e -> Expect.fail (Decode.errorToString e)
        
        , test "ModuleType Decoder erkennt unbekannte Typen" <|
            \_ ->
                "\"laser-scanner\""
                    |> Decode.decodeString Decoders.moduleTypeDecoder
                    |> Expect.equal (Ok (UnknownModule "laser-scanner"))
        ]