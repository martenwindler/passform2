module Domain.Representations.Grid exposing (Grid, GridCell, CellType(..), decoder, cellDecoder, isWithinBounds)

import Json.Decode as Decode exposing (Decoder)
import Json.Decode.Pipeline exposing (required, optional)

type CellType
    = Walkable
    | Obstacle
    | LoadingStation
    | AssemblyPoint

type alias GridCell =
    { x : Int
    , y : Int
    , cellType : CellType
    }

type alias Grid =
    { width : Int
    , height : Int
    , cells : List GridCell
    }

cellDecoder : Decoder GridCell
cellDecoder =
    Decode.succeed GridCell
        |> required "x" Decode.int
        |> required "y" Decode.int
        |> optional "type" typeDecoder Walkable

typeDecoder : Decoder CellType
typeDecoder =
    Decode.string
        |> Decode.map (\str ->
            case str of
                "Obstacle" -> Obstacle
                "Loading" -> LoadingStation
                "Assembly" -> AssemblyPoint
                _ -> Walkable
        )

decoder : Decoder Grid
decoder =
    Decode.succeed Grid
        |> required "width" Decode.int
        |> required "height" Decode.int
        |> required "cells" (Decode.list cellDecoder)

isWithinBounds : Grid -> { x : Int, y : Int } -> Bool
isWithinBounds grid pos =
    pos.x >= 0 && pos.x < grid.width && pos.y >= 0 && pos.y < grid.height