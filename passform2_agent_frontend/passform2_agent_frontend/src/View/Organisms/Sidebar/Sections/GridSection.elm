module View.Organisms.Sidebar.Sections.GridSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onInput)
import Types exposing (..)

{-| 
Diese Sektion steuert die physischen Ausmaße des Arbeitsbereichs.
Bleibt im zweispaltigen Layout, begrenzt aber die Werte auf 0-100.
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section grid-config" ]
        [ h4 [] [ text "Gitter-Konfiguration" ]
        
        , -- Zweispaltiges Layout für Breite und Länge beibehalten
          div [ class "grid grid-cols-2 gap-4" ]
            [ viewParamInput "Breite (B)" (String.fromInt model.gridWidth) (PlanningMsg << SetGridWidth)
            , viewParamInput "Länge (L)" (String.fromInt model.gridHeight) (PlanningMsg << SetGridHeight)
            ]
        ]

-- --- HELPER ---

{-| 
Rendert ein Eingabefeld mit vertikalem Abstand (gap-1.5) zwischen Label und Input.
Neu: Begrenzung auf Ganzzahlen von 0 bis 100.
-}
viewParamInput : String -> String -> (String -> Msg) -> Html Msg
viewParamInput labelText currentVal toMsg =
    div [ class "sidebar-section flex flex-col gap-1.5" ] 
        [ label 
            [ class "text-[0.65rem] text-white/40 uppercase font-bold tracking-wider" ] 
            [ text labelText ]
        , input 
            [ type_ "number"
            , Html.Attributes.min "0"   -- Untergrenze 0
            , Html.Attributes.max "100" -- Obergrenze 100
            , Html.Attributes.step "1"  -- Nur Ganzzahlen (1er Schritte)
            , value currentVal
            , onInput toMsg
            , class "w-full bg-black/20 border border-white/10 p-2 text-white rounded-sm focus:border-info/50 outline-none transition-colors font-mono"
            ] []
        ]