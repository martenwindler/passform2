module View.Sidebar exposing (view, formatType)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Dict 
import Types exposing (..)

view : Model -> Html Msg
view model =
    aside [ class "sidebar" ]
        [ h2 [] [ text "Gitter-Konfiguration" ]
        
        -- 1. System-Logs (Ganz oben)
        , div [ class "sidebar-section log-section" ]
            [ h3 [] [ text "System-Logs" ]
            , div [ class "log-container" ]
                (List.map (\log -> 
                    div [ class ("log-entry " ++ log.level) ] 
                        [ text log.message ]
                ) model.logs)
            ]

        -- 2. Gitter-Einstellungen (50/50 Layout)
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Dimensionen" ]
            , div [ class "input-row" ] 
                [ div [ class "input-group" ]
                    [ label [] [ text "Breite:" ]
                    , input [ type_ "number", value (String.fromInt model.gridWidth), onInput SetGridWidth ] []
                    ]
                , div [ class "input-group" ]
                    [ label [] [ text "Länge:" ]
                    , input [ type_ "number", value (String.fromInt model.gridHeight), onInput SetGridHeight ] []
                    ]
                ]
            ]

        -- 3. Aktionen (Wieder hinzugefügt für Import/Export/Reset)
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Aktionen" ]
            , div [ class "sidebar-actions-grid" ]
                [ button [ class "btn-secondary", onClick LoadDefaultConfig ] [ text "Standard laden" ]
                , button [ class "btn-primary", onClick SetCurrentAsDefault ] [ text "Standard setzen" ]
                , button [ class "btn-secondary", onClick ExportConfig ] [ text "Export JSON" ]
                , button [ class "btn-secondary", onClick ImportConfigTrigger ] [ text "Import JSON" ]
                , button [ class "btn-danger", onClick ClearGrid ] [ text "Gitter leeren" ]
                ]
            ]

        -- 4. Liste der Agenten
        , div [ class "sidebar-section" ]
            [ h3 [] [ text ("Module (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
            , if Dict.isEmpty model.agents then
                p [ class "sidebar-hint" ] [ text "Gitter ist leer." ]
              else
                ul [ class "agent-list" ] 
                    (model.agents |> Dict.values |> List.map viewAgentItem)
            ]

        -- 5. Pfadplanung Status
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Pfadplanung" ]
            , viewCoordinateStatus "Start" model.pathStart
            , viewCoordinateStatus "Ziel" model.pathGoal
            
            -- Dynamisches Feedback
            , if model.loading then
                div [ class "planning-status-box" ]
                    [ div [ class "spinner" ] [] -- CSS Spinner
                    , span [] [ text "Berechnung läuft..." ]
                    ]
            else
                text ""

            , button 
                [ class (if canPlan model && not model.loading then "btn-sidebar-primary" else "btn-disabled")
                , onClick (StartPlanning False)
                , disabled (not (canPlan model) || model.loading)
                ] 
                [ text (if model.loading then "Warten..." else "Pfad berechnen") ]
            ]
        ]

-- --- HILFSFUNKTIONEN ---

viewAgentItem : AgentModule -> Html Msg
viewAgentItem agent =
    li [ class "agent-item" ]
        [ div [ class "agent-info" ]
            [ span [ class "agent-label" ] [ text (formatType agent.module_type) ]
            , span [ class "agent-coords" ] 
                [ text ("(" ++ String.fromInt agent.position.x ++ "," ++ String.fromInt agent.position.y ++ ") | " ++ String.fromInt agent.orientation ++ "°") ]
            ]
        , button [ class "btn-icon-delete", onClick (RemoveAgent agent.position) ] [ text "×" ]
        ]

formatType : String -> String
formatType t =
    case t of
        "rollen_ns" -> "Rollen-Modul"
        "greifer" -> "Greifer"
        "tisch" -> "Tisch"
        "conveyeur" -> "Förderband"
        _ -> t

viewCoordinateStatus : String -> Maybe GridCell -> Html Msg
viewCoordinateStatus labelTitle maybeCell =
    div [ class "coord-item" ]
        [ span [ class "coord-label" ] [ text (labelTitle ++ ": ") ]
        , case maybeCell of
            Just cell -> span [ class "text-green" ] [ text ("(" ++ String.fromInt cell.x ++ "," ++ String.fromInt cell.y ++ ")") ]
            Nothing -> span [ class "text-red" ] [ text "Nicht gewählt" ]
        ]

canPlan : Model -> Bool
canPlan model =
    case (model.pathStart, model.pathGoal) of
        (Just _, Just _) -> True
        _ -> False