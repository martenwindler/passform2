module Domain.Representations.Agent exposing (Agent, decoder)

import Domain.Representations.Skill as Skill exposing (Skill)
import Json.Decode as Decode exposing (Decoder)
import Json.Decode.Pipeline exposing (required)

type alias Agent =
    { id : String
    , name : String
    , skills : List Skill
    , status : String -- z.B. "IDLE", "BUSY", "ERROR"
    }

-- Diesen Decoder nutzt du, wenn das Backend die "active_agents" schickt
decoder : Decoder Agent
decoder =
    Decode.succeed Agent
        |> required "id" Decode.string
        |> required "name" Decode.string
        |> required "skills" (Decode.list Skill.decoder)
        |> required "status" Decode.string