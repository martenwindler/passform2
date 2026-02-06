module View.Organisms.Sidebar.Sections.PathResultSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)

view : Model -> Html Msg
view model =
    div [ class "planning-section planning-result-section mt-4 animate-fade-in" ]
        [ h4 [ class "text-white/50 flex items-center gap-2 mb-3" ] 
            [ span [ class ("status-dot " ++ (if model.currentPath /= Nothing then "active" else "warning")) ] []
            , text "CNP-Ergebnis" 
            ]
        
        , case model.currentPath of
            Just pathResult -> 
                let 
                    steps = pathResult.path
                    displayCost = String.fromFloat (toFloat (round (pathResult.cost * 100)) / 100)
                in
                div [ class "result-card" ]
                    [ div [ class "flex justify-between text-[0.6rem] uppercase font-black text-white/30 mb-2 tracking-tighter" ]
                        [ span [] [ text "Etappe" ]
                        , span [] [ text "Position (X,Y)" ]
                        ]
                    
                    , div [ class "path-list custom-scrollbar" ]
                        (List.indexedMap viewPathStep steps)
                    
                    , div [ class "result-footer" ]
                        [ span [ class "footer-label" ] [ text "Kosten" ]
                        , span [ class "footer-value" ] [ text displayCost ]
                        ]
                    ]

            Nothing ->
                if canPlan model then
                    div [ class "status-box required" ]
                        [ div [ class "status-msg" ] [ text "⚠️ Route veraltet" ]
                        , div [ class "status-hint" ] [ text "Bitte 'Ausschreibung starten' klicken" ]
                        ]
                else
                    div [ class "status-box initial" ]
                        [ div [ class "status-msg" ] [ text "Bereit für Planung" ]
                        , div [ class "status-hint" ] [ text "Bitte Start- und Zielpunkt wählen" ]
                        ]
        ]

viewPathStep : Int -> GridCell -> Html Msg
viewPathStep index cell =
    div [ class "path-step" ]
        [ span [ class "step-index" ] [ text (String.fromInt (index + 1)) ]
        , span [ class "step-pos" ] 
            [ text (String.fromInt cell.x ++ "/" ++ String.fromInt cell.y ++ " (L" ++ String.fromInt cell.level ++ ")") ]
        ]
        
-- DIESER HELPER FEHLTE:
canPlan : Model -> Bool
canPlan model =
    case ( model.pathStart, model.pathGoal ) of
        ( Just _, Just _ ) -> True
        _ -> False