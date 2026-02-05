module View.Styleguide.Styleguide exposing (main)

import ElmBook exposing (Book, book)
import ElmBook.Chapter exposing (chapter, render, withStatefulComponent)
import Html exposing (Html, div, h3, text, br, node)
import Html.Attributes exposing (style, attribute)


type alias MyState =
    { zHeight : Int }


main : Book MyState
main =
    book "Passform 2.0 UI System"
        |> ElmBook.withChapters
            [ levelAtoms ]


levelAtoms : ElmBook.Chapter.Chapter MyState
levelAtoms =
    chapter "Atome: Höhen-Ebenen"
        -- 1. Zuerst die Komponente an den Builder binden
        |> withStatefulComponent
            (\state ->
                div [ style "padding" "20px" ]
                    [ node "sl-badge"
                        [ attribute "variant" (if state.zHeight >= 400 then "success" else "neutral") ]
                        [ text (if state.zHeight >= 400 then "Level 1: Table" else "Level 0: Floor") ]
                    , div [ style "margin-top" "10px", style "color" "gray" ] 
                        [ text ("Höhe im State: " ++ String.fromInt state.zHeight ++ "mm") ]
                    ]
            )
        -- 2. Ganz am Ende rendern (macht aus dem Builder ein Chapter)
        |> render """
### Status der Z-Ebenen
Hier testen wir die Visualisierung der verschiedenen Höhenstufen basierend auf der Z-Achse.

<component />
"""