module View.Organisms.Sidebar.Sections.WeightsSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Types exposing (..)

view : Model -> Html Msg
view model =
    div [ class "planning-section planning-weights-section" ]
        [ h4 [] [ text "CNP-Kostenparameter" ]
        
        , div [ class "weights-grid" ]
            [ -- Basis-Zeit: 0.1s bis 5.0s
              viewWeightItem "Basis" "0.1" "0.1" "5.0" model.planningWeights.execution_time_default "execution_time_default"
              
            -- Komplex (Greifer): 1.0s bis 10.0s
            , viewWeightItem "Komplex" "0.1" "1.0" "10.0" model.planningWeights.complex_module_time "complex_module_time"
            
            -- Mensch-Zuschlag: 0.0s bis 20.0s
            , viewWeightItem "Mensch" "0.5" "0.0" "20.0" model.planningWeights.human_extra_weight "human_extra_weight"
            
            -- Nähe-Strafe: 0.0s bis 10.0s
            , viewWeightItem "Nähe" "0.1" "0.0" "10.0" model.planningWeights.proximity_penalty "proximity_penalty"
            ]
            
        , button 
            [ class "btn-apply-weights"
            , onClick (PlanningMsg SaveWeights) 
            ] 
            [ text "Parameter anwenden" ]
        ]

{-| Hilfsfunktion mit min/max Begrenzung -}
viewWeightItem : String -> String -> String -> String -> Float -> String -> Html Msg
viewWeightItem labelTitle stepVal minVal maxVal val key =
    div [ class "parameter-item" ]
        [ label [] [ text (labelTitle ++ " (s)") ] -- Einheit (s) zur Klarheit hinzugefügt
        , input 
            [ type_ "number"
            , step stepVal
            , Html.Attributes.min minVal
            , Html.Attributes.max maxVal
            , value (String.fromFloat val)
            , onInput (PlanningMsg << SetWeight key) 
            ] []
        ]