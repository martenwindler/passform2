module Domain.Representations.GridTest exposing (all)

import Test exposing (..)
import Expect
import Json.Decode as Decode
import Domain.Representations.Grid as Grid exposing (CellType(..))

all : Test
all =
    describe "Grid Domain"
        [ describe "Decoding"
            [ test "decodes a basic grid cell" <|
                \_ ->
                    let
                        json = """{ "x": 5, "y": 2, "type": "Obstacle" }"""
                        result = Decode.decodeString Grid.cellDecoder json
                    in
                    case result of
                        Ok cell ->
                            Expect.all
                                [ \c -> Expect.equal c.x 5
                                , \c -> Expect.equal c.y 2
                                , \c -> Expect.equal c.cellType Obstacle
                                ]
                                cell
                        Err err ->
                            Expect.fail (Decode.errorToString err)

            , test "defaults to Walkable if type is unknown" <|
                \_ ->
                    let
                        json = """{ "x": 0, "y": 0, "type": "Unknown" }"""
                        result = Decode.decodeString Grid.cellDecoder json
                    in
                    case result of
                        Ok cell ->
                            Expect.equal cell.cellType Walkable
                        Err _ ->
                            Expect.fail "Should have defaulted to Walkable"
            ]
        
        , describe "Logic"
            [ test "isWithinBounds returns True for valid coordinates" <|
                \_ ->
                    let
                        grid = { width = 10, height = 10, cells = [] }
                    in
                    Grid.isWithinBounds grid { x = 5, y = 5 }
                        |> Expect.equal True

            , test "isWithinBounds returns False for coordinates outside" <|
                \_ ->
                    let
                        grid = { width = 10, height = 10, cells = [] }
                    in
                    Grid.isWithinBounds grid { x = 11, y = 5 }
                        |> Expect.equal False
            ]
        ]