module Domain.Representations.SkillTest exposing (all)

import Test exposing (..)
import Expect
import Json.Decode as Decode
import Domain.Representations.Skill as Skill exposing (Skill, SkillType(..))

all : Test
all =
    describe "Skill Domain"
        [ describe "Decoder"
            [ test "decodes a simple primitive skill (MoveArm)" <|
                \_ ->
                    let
                        json = """{ "name": "Arm bewegen", "type": "MoveArm", "id_short": "MA01" }"""
                        result = Decode.decodeString Skill.decoder json
                    in
                    case result of
                        Ok skill ->
                            Expect.all
                                [ \s -> Expect.equal s.name "Arm bewegen"
                                , \s -> Expect.equal s.skillType MoveArm
                                , \s -> Expect.equal s.idShort (Just "MA01")
                                ]
                                skill
                        Err err ->
                            Expect.fail (Decode.errorToString err)

            , test "decodes a complex CUSTOM skill (PickAndPlace)" <|
                \_ ->
                    let
                        json = """{ "name": "Greifen", "type": "CUSTOM" }"""
                        result = Decode.decodeString Skill.decoder json
                    in
                    case result of
                        Ok skill ->
                            Expect.equal skill.skillType Custom
                        Err err ->
                            Expect.fail (Decode.errorToString err)
            ]
        ]