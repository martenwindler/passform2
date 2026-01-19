module View.Atoms.Button exposing (primary, secondary, danger, iconDelete, ghost)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)

{-| 
Alle Buttons nehmen jetzt ein zusätzliches 'isFullWidth' Argument (Bool).
Nutzt classList, um die 'btn-full' Klasse bei Bedarf hinzuzufügen.
-}

primary : String -> msg -> Bool -> Bool -> Html msg
primary label msg isEnabled isFullWidth =
    button 
        [ class "btn-primary"
        , classList [ ("btn-full", isFullWidth) ]
        , onClick msg
        , disabled (not isEnabled) 
        ] 
        [ text label ]

secondary : String -> msg -> Bool -> Bool -> Html msg
secondary label msg isEnabled isFullWidth =
    button 
        [ class "btn-secondary"
        , classList [ ("btn-full", isFullWidth) ]
        , onClick msg
        , disabled (not isEnabled) 
        ] 
        [ text label ]

danger : String -> msg -> Bool -> Bool -> Html msg
danger label msg isEnabled isFullWidth =
    button 
        [ class "btn-danger"
        , classList [ ("btn-full", isFullWidth) ]
        , onClick msg
        , disabled (not isEnabled) 
        ] 
        [ text label ]

ghost : String -> msg -> Bool -> Bool -> Html msg
ghost label msg isEnabled isFullWidth =
    button 
        [ class "btn-icon-sidebar"
        , classList [ ("btn-full", isFullWidth) ]
        , onClick msg
        , disabled (not isEnabled) 
        ] 
        [ text label ]

{-| Icon-Buttons (wie das Lösch-X) bleiben meistens in ihrer festen Größe -}
iconDelete : msg -> Html msg
iconDelete msg =
    button [ class "btn-icon-delete", onClick msg ] [ text "×" ]