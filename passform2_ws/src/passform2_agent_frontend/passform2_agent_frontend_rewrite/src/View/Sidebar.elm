module View.Sidebar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..) -- Importiert Model, Msg, GridCell etc.

-- Die Hauptfunktion der Sidebar
view : Model -> Html Msg
view model =
    aside [ class "sidebar" ]
        [ h2 [] [ text "Agenten Steuerung" ]
        
        -- Sektion: Pfad-Konfiguration (Start/Ziel Status)
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Pfadplanung" ]
            , viewCoordinateStatus "Startpunkt" model.pathStart
            , viewCoordinateStatus "Zielpunkt" model.pathGoal
            ]

        -- Sektion: Aktionen
        , div [ class "sidebar-actions" ]
            [ button 
                [ class "btn-primary"
                , onClick (StartPlanning False) -- Löst Msg in Main.elm aus
                , disabled (model.loading || not (canPlan model))
                ] 
                [ text (if model.loading then "Plane..." else "Pfad planen") ]
            
            , button 
                [ class "btn-secondary"
                , onClick (StartPlanning True) -- Central-Planning
                , disabled (model.loading || not (canPlan model))
                ] 
                [ text "Zentral planen" ]
            ]

        -- Sektion: Pfad-Ergebnisse (wird nur angezeigt, wenn ein Pfad existiert)
        , viewPathResults model.currentPath
        ]

-- Hilfsfunktion: Zeigt an, ob Koordinate gesetzt ist
viewCoordinateStatus : String -> Maybe GridCell -> Html Msg
viewCoordinateStatus label maybeCell =
    div [ class "coord-status" ]
        [ span [] [ text (label ++ ": ") ]
        , case maybeCell of
            Just cell ->
                span [ class "text-green" ] 
                    [ text ("(" ++ String.fromInt cell.x ++ ", " ++ String.fromInt cell.y ++ ")") ]
            
            Nothing ->
                span [ class "text-red" ] [ text "Nicht gesetzt" ]
        ]

-- Hilfsfunktion: Details zum aktuellen Pfad
viewPathResults : Maybe Path -> Html Msg
viewPathResults maybePath =
    case maybePath of
        Just path ->
            div [ class "sidebar-section path-results" ]
                [ h3 [] [ text "Pfad Details" ]
                , ul []
                    [ li [] [ text ("Kosten: " ++ String.fromFloat path.cost) ]
                    , li [] [ text ("Segmente: " ++ (path.path |> List.length |> String.fromInt)) ]
                    , li [] [ text ("Status: " ++ String.fromInt path.status) ]
                    ]
                ]

        Nothing ->
            div [ class "sidebar-section hint" ] 
                [ text "Wähle einen Agenten auf dem Gitter aus, um Start oder Ziel zu setzen." ]

-- Validierung: Können wir planen?
canPlan : Model -> Bool
canPlan model =
    case (model.pathStart, model.pathGoal) of
        (Just _, Just _) -> True
        _ -> False