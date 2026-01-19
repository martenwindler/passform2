module View.Organisms.Sidebar.Sections.WeightsSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Types exposing (..)

{-| 
Diese Sektion verwaltet die Gewichtungen für das Contract-Net-Protokoll (CNP).
Die Werte beeinflussen direkt die Kostenfunktion der Pfadplanung.
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section weights-config" ]
        [ h4 [] [ text "CNP-Kostenfunktion" ]
        
        , -- Das 4-Spalten-Raster für die Parameter
          div [ class "parameter-grid-row" ]
            [ viewWeightItem "Basis" "0.1" model.planningWeights.execution_time_default "execution_time_default"
            , viewWeightItem "Komplex" "0.1" model.planningWeights.complex_module_time "complex_module_time"
            , viewWeightItem "Mensch" "0.5" model.planningWeights.human_extra_weight "human_extra_weight"
            , viewWeightItem "Nähe" "0.1" model.planningWeights.proximity_penalty "proximity_penalty"
            ]
            
        , -- Button zum Absenden der neuen Gewichte an das Backend/System
          button 
            [ class "btn-secondary btn-full text-[0.7rem] py-2 mt-2"
            , onClick (PlanningMsg SaveWeights) 
            ] 
            [ text "Gewichte anwenden" ]
        ]

-- --- HELPER ---

{-| Rendert ein einzelnes Gewichts-Eingabefeld im Grid -}
viewWeightItem : String -> String -> Float -> String -> Html Msg
viewWeightItem labelTitle stepVal val key =
    div [ class "parameter-item" ]
        [ label [] [ text labelTitle ]
        , input 
            [ type_ "number"
            , step stepVal
            , value (String.fromFloat val)
            , onInput (PlanningMsg << SetWeight key)
            , class "focus:border-success" -- Highlight bei Fokus
            ] []
        ]