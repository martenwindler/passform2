module View.Organisms.Sidebar.Sections.WeightsSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Types exposing (..)

view : Model -> Html Msg
view model =
    div [ class "planning-section planning-weights-section" ]
        [ h4 [] [ text "CNP-Kostenparameter" ]
        
        , div [ class "weights-grid" ] -- Nutzt die Klasse aus _planning-weights.scss
            [ viewWeightItem "Basis" "0.1" model.planningWeights.execution_time_default "execution_time_default"
            , viewWeightItem "Komplex" "0.1" model.planningWeights.complex_module_time "complex_module_time"
            , viewWeightItem "Mensch" "0.5" model.planningWeights.human_extra_weight "human_extra_weight"
            , viewWeightItem "Nähe" "0.1" model.planningWeights.proximity_penalty "proximity_penalty"
            ]
            
        , button 
            [ class "btn-apply-weights" -- Spezielle Button-Klasse
            , onClick (PlanningMsg SaveWeights) 
            ] 
            [ text "Parameter anwenden" ]
        ]

viewWeightItem : String -> String -> Float -> String -> Html Msg
viewWeightItem labelTitle stepVal val key =
    div [ class "parameter-item" ]
        [ label [] [ text labelTitle ]
        , input 
            [ type_ "number", step stepVal, value (String.fromFloat val), onInput (PlanningMsg << SetWeight key) ] []
        ]