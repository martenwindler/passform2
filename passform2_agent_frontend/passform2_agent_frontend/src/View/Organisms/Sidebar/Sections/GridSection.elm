module View.Organisms.Sidebar.Sections.GridSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onInput)
import Types exposing (..)

{-| 
Diese Sektion steuert die physischen Ausmaße des Arbeitsbereichs.
Änderungen hier triggern eine Neuberechnung des Navigations-Gitters.
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section grid-config" ]
        [ h4 [] [ text "Gitter-Konfiguration" ]
        
        , -- Zweispaltiges Layout für Breite und Länge
          div [ class "grid grid-cols-2 gap-4" ]
            [ viewParamInput "Breite (B)" (String.fromInt model.gridWidth) (PlanningMsg << SetGridWidth)
            , viewParamInput "Länge (L)" (String.fromInt model.gridHeight) (PlanningMsg << SetGridHeight)
            ]
        ]

-- --- HELPER ---

{-| 
Rendert ein Standard-Eingabefeld für die Sidebar.
Nutzt die '.sidebar-section' Klasse für das konsistente Label-Styling.
-}
viewParamInput : String -> String -> (String -> Msg) -> Html Msg
viewParamInput labelText currentVal toMsg =
    div [ class "sidebar-section" ]
        [ label [] [ text labelText ]
        , input 
            [ type_ "number"
            , Html.Attributes.min "1" -- Verhindert ungültige Gitter-Größen
            , value currentVal
            , onInput toMsg
            , class "w-full bg-black/20 border border-white/10 p-2 text-white rounded-sm"
            ] []
        ]