module View.Molecules.FormGroup exposing (numberField, textField)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onInput)

{-| 
Erstellt eine Gruppe für numerische Eingaben.
Nutzt die CSS-Klasse .sidebar-section, damit die Styles für Labels 
und Number-Inputs aus deinem Stylesheet greifen.
-}
numberField : String -> Float -> Float -> (String -> msg) -> Html msg
numberField labelText val stepVal toMsg =
    div [ class "sidebar-section" ]
        [ label [] [ text labelText ]
        , input
            [ type_ "number"
            , step (String.fromFloat stepVal)
            , value (String.fromFloat val)
            , onInput toMsg
            ]
            []
        ]

{-| 
Erstellt eine Gruppe für Texteingaben.
Auch hier nutzen wir .sidebar-section für ein konsistentes Spacing in der Sidebar.
-}
textField : String -> String -> String -> (String -> msg) -> Html msg
textField labelText currentVal placeholderText toMsg =
    div [ class "sidebar-section" ]
        [ label [] [ text labelText ]
        , input
            [ type_ "text"
            , placeholder placeholderText
            , value currentVal
            , onInput toMsg
            ]
            []
        ]