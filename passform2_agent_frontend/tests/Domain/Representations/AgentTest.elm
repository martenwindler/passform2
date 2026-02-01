module Domain.Representations.AgentTest exposing (all)

import Test exposing (..)
import Expect
import Json.Decode as Decode
import Domain.Representations.Agent as Agent
import Domain.Representations.Skill exposing (SkillType(..))

all : Test
all =
    describe "Agent Domain"
        [ test "decodes an agent with a list of skills" <|
            \_ ->
                let
                    json = """
                    {
                        "id": "robot_01",
                        "name": "UR5e",
                        "status": "IDLE",
                        "skills": [
                            { "name": "Move", "type": "MoveArm" },
                            { "name": "Stop", "type": "MoveStop" }
                        ]
                    }
                    """
                    result = Decode.decodeString Agent.decoder json
                in
                case result of
                    Ok agent ->
                        Expect.all
                            [ \a -> Expect.equal a.id "robot_01"
                            , \a -> Expect.equal (List.length a.skills) 2
                            ]
                            agent
                    Err err ->
                        Expect.fail (Decode.errorToString err)
        ]