module Domain.Representations.Mission exposing (Mission, MissionStatus(..), MissionStep, decoder)

import Json.Decode as Decode exposing (Decoder)
import Json.Decode.Pipeline exposing (required, optional)
import Json.Encode as Encode

type MissionStatus
    = Pending
    | InProgress
    | Completed
    | Failed String

type alias MissionStep =
    { skill_name : String
    , parameters : Decode.Value -- Flexibel für verschiedene Skill-Parameter
    }

type alias Mission =
    { id : String
    , name : String
    , steps : List MissionStep
    , status : MissionStatus
    }

decoder : Decoder Mission
decoder =
    Decode.succeed Mission
        |> required "id" Decode.string
        |> required "name" Decode.string
        |> required "steps" (Decode.list stepDecoder)
        |> required "status" statusDecoder

stepDecoder : Decoder MissionStep
stepDecoder =
    Decode.succeed MissionStep
        |> required "skill_name" Decode.string
        |> optional "parameters" Decode.value Encode.null

statusDecoder : Decoder MissionStatus
statusDecoder =
    Decode.string
        |> Decode.map (\str ->
            case str of
                "Pending" -> Pending
                "InProgress" -> InProgress
                "Completed" -> Completed
                _ -> Failed str
        )