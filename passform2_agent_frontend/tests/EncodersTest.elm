module EncodersTest exposing (suite)

import Test exposing (..)
import Expect
import Json.Encode as Encode
import Decoders
import Types exposing (..)
import Types.Domain exposing (..)

suite : Test
suite =
    describe "Schnittstellen-Check: Outbound JSON"
        [ test "encodePlanningData bündelt alle Informationen für den Port" <|
            \_ ->
                let
                    data = 
                        { start = Just { x = 0, y = 0 }
                        , goal = Just { x = 5, y = 5 }
                        , weights = { execution_time_default = 1.0, complex_module_time = 1.0, human_extra_weight = 1.0, proximity_penalty = 1.0, hardware_safety_factor = 1.0 }
                        , isRanger = True 
                        }
                    json = Decoders.encodePlanningData data |> Encode.encode 0
                in
                Expect.all 
                    [ \s -> String.contains "\"x\":0" s |> Expect.equal True
                    , \s -> String.contains "\"isRanger\":true" s |> Expect.equal True
                    ] json
        ]