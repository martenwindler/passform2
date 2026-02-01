module DecodersTest exposing (suite)

import Decoders
import Dict
import Expect
import Json.Decode as Decode
import Test exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)


suite : Test
suite =
    describe "Schnittstellen-Check: Inbound JSON"
        [ describe "Agent & Position Decoding"
            [ test "Agent-Map Decoder liest ROS-Daten (verschachtelt) korrekt ein" <|
                \_ ->
                    let
                        json =
                            """{ "agents": [ { "agent_id": "FTF-01", "module_type": "ftf", "position": {"x": 1, "y": 1}, "orientation": 0, "is_dynamic": true } ] }"""

                        result =
                            Decode.decodeString Decoders.agentMapDecoder json
                    in
                    case result of
                        Ok dict ->
                            Expect.equal (Dict.size dict) 1

                        Err e ->
                            Expect.fail (Decode.errorToString e)
            , test "Agent Decoder erkennt flache Koordinaten (Variante B)" <|
                \_ ->
                    let
                        -- Testet, ob x/y auch ohne das "position" Objekt erkannt werden
                        json =
                            """{ "agent_id": "FTF-02", "module_type": "ftf", "x": 5, "y": 3 }"""

                        result =
                            Decode.decodeString Decoders.agentDecoder json
                    in
                    case result of
                        Ok agent ->
                            Expect.all
                                [ \a -> Expect.equal a.position.x 5
                                , \a -> Expect.equal a.position.y 3
                                ]
                                agent

                        Err e ->
                            Expect.fail (Decode.errorToString e)
            ]
        , describe "Typ-Mapping (Enums)"
            [ test "ModuleType Decoder erkennt bekannte Typen und Fallbacks" <|
                \_ ->
                    "\"rollen_ns\""
                        |> Decode.decodeString Decoders.moduleTypeDecoder
                        |> Expect.equal (Ok RollenModul)
            , test "ModuleType Decoder fängt Unbekanntes ab" <|
                \_ ->
                    "\"laser-scanner\""
                        |> Decode.decodeString Decoders.moduleTypeDecoder
                        |> Expect.equal (Ok (UnknownModule "laser-scanner"))
            , test "HardwareStatus Decoder liest 'online' korrekt" <|
                \_ ->
                    "\"online\""
                        |> Decode.decodeString Decoders.hardwareStatusDecoder
                        |> Expect.equal (Ok Online)
            ]
        , describe "System-Log Decoding"
            [ test "SystemLog Decoder mappt Level 'error' auf Danger" <|
                \_ ->
                    let
                        json =
                            """{ "message": "Kritischer Fehler", "level": "error" }"""
                        
                        -- Hier nutzen wir Result.map, um in das fertige Ergebnis zu schauen
                        result = 
                            Decode.decodeString Decoders.decodeSystemLog json
                                |> Result.map .level
                    in
                    Expect.equal result (Ok Danger)
            ]
        ]