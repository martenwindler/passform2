module Domain.Representations.MissionTest exposing (all)

import Test exposing (..)
import Expect
import Json.Decode as Decode
import Domain.Representations.Mission as Mission exposing (MissionStatus(..))

all : Test
all =
    describe "Mission Domain"
        [ test "decodes a complex mission with steps" <|
            \_ ->
                let
                    json = """
                    {
                        "id": "m-42",
                        "name": "Pick and Place Sequence",
                        "status": "InProgress",
                        "steps": [
                            { "skill_name": "MoveArm", "parameters": {"x": 10, "y": 20} },
                            { "skill_name": "Grasp", "parameters": {} }
                        ]
                    }
                    """
                    result = Decode.decodeString Mission.decoder json
                in
                case result of
                    Ok mission ->
                        Expect.all
                            [ \m -> Expect.equal m.id "m-42"
                            , \m -> Expect.equal m.status InProgress
                            , \m -> Expect.equal (List.length m.steps) 2
                            ]
                            mission

                    Err err ->
                        Expect.fail (Decode.errorToString err)
        ]