module Domain.Representations.Skill exposing (Skill, SkillType(..), decoder)

import Json.Decode as Decode exposing (Decoder)
import Json.Decode.Pipeline exposing (required, optional)

type SkillType
    = MoveArm
    | MoveConveyor
    | MoveEffector
    | MoveStop
    | ToolGet
    | ToolPut
    | ToolUse
    | RandomStatus
    | Custom -- Für Pick-and-place
    | Unknown String

type alias Skill =
    { name : String
    , skillType : SkillType
    , idShort : Maybe String
    , driverTopic : Maybe String
    -- Später erweitern wir dies um Properties/Parameter
    }

decoder : Decoder Skill
decoder =
    Decode.succeed Skill
        |> required "name" Decode.string
        |> required "type" skillTypeDecoder
        |> optional "id_short" (Decode.maybe Decode.string) Nothing
        |> optional "driver_topic" (Decode.maybe Decode.string) Nothing

skillTypeDecoder : Decoder SkillType
skillTypeDecoder =
    Decode.string
        |> Decode.map (\str ->
            case str of
                "MoveArm" -> MoveArm
                "MoveConveyor" -> MoveConveyor
                "MoveEffector" -> MoveEffector
                "MoveStop" -> MoveStop
                "ToolGet" -> ToolGet
                "ToolPut" -> ToolPut
                "ToolUse" -> ToolUse
                "RandomStatus" -> RandomStatus
                "CUSTOM" -> Custom
                _ -> Unknown str
        )