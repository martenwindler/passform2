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
        
        -- 1. System-Logs
        , div [ class "sidebar-section log-section" ]
            [ h3 [] [ text "System-Meldungen" ]
            , div [ id "log-container", class "log-container" ]
                (List.map (\log -> 
                    div [ class ("log-entry " ++ log.level) ] 
                        [ span [ class "log-dot" ] []
                        , text log.message 
                        ]
                ) model.logs)
            ]

        -- NEU: NFC Warte-Indikator (Erscheint nur während des Schreibvorgangs)
        , if model.waitingForNfc then
            div [ class "sidebar-section nfc-wait-section" ]
                [ div [ class "nfc-wait-indicator" ]
                    [ div [ class "pulse-ring" ] []
                    , span [] [ text "Bitte Chip an den Reader halten (5s)..." ]
                    ]
                ]
          else
            text ""

        -- 2. Gitter-Einstellungen
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Gitter-Dimensionen" ]
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

        -- 3. Globale Aktionen
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Aktionen" ]
            , div [ class "sidebar-actions-grid" ]
                [ button [ class "btn-secondary", onClick LoadDefaultConfig ] [ text "Standard laden" ]
                , button [ class "btn-primary", onClick SetCurrentAsDefault ] [ text "Layout speichern" ]
                , button [ class "btn-secondary", onClick ExportConfig ] [ text "Export JSON" ]
                , button [ class "btn-secondary", onClick ImportConfigTrigger ] [ text "Import JSON" ]
                , button [ class "btn-danger", onClick ClearGrid ] [ text "Gitter leeren" ]
                ]
            ]

        -- 4. Liste der aktiven Module
        , div [ class "sidebar-section" ]
            [ h3 [] [ text ("Aktive Module (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
            , if Dict.isEmpty model.agents then
                p [ class "sidebar-hint" ] [ text "Keine Module im Gitter." ]
              else
                ul [ class "agent-list" ] 
                    (model.agents |> Dict.values |> List.sortBy .module_type |> List.map viewAgentItem)
            ]

        -- 5. Missions-Status & Planung
        , div [ class "sidebar-section mission-section" ]
            [ h3 [] [ text "Missions-Planung" ]
            , viewCoordinateStatus "📦 Start" model.pathStart
            , viewCoordinateStatus "🏁 Ziel" model.pathGoal
            
            , if model.loading then
                div [ class "planning-status-box" ]
                    [ div [ class "spinner" ] []
                    , span [] [ text "Berechne Mission..." ]
                    ]
              else
                text ""

            , button 
                [ class (if canPlan model && not model.loading then "btn-sidebar-primary" else "btn-disabled")
                , onClick (StartPlanning False)
                , disabled (not (canPlan model) || model.loading)
                ] 
                [ text (if model.loading then "Warten..." else "Mission starten") ]
            ]
        ]

-- --- HILFSFUNKTIONEN ---

viewAgentItem : AgentModule -> Html Msg
viewAgentItem agent =
    li [ class ("agent-item " ++ (if agent.is_dynamic then "dynamic-agent" else "static-agent")) ]
        [ div [ class "agent-info" ]
            [ div [ class "agent-header" ] 
                [ span [ class "agent-label" ] [ text (formatType agent.module_type) ]
                , if agent.is_dynamic then 
                    span [ class "badge dynamic" ] [ text "Mobil" ]
                  else 
                    text ""
                ]
            , span [ class "agent-coords" ] 
                [ text ("(" ++ String.fromInt agent.position.x ++ "," ++ String.fromInt agent.position.y ++ ") | " ++ String.fromInt agent.orientation ++ "°") ]
            
            , case agent.payload of
                Just pId -> 
                    div [ class "agent-payload" ] [ text ("📦 Trägt: " ++ pId) ]
                Nothing -> 
                    text ""
            ]
        , button [ class "btn-icon-delete", onClick (RemoveAgent agent.position) ] [ text "×" ]
        ]

formatType : String -> String
formatType t =
    case t of
        "ftf" -> "Transport-Agent (FTF)"
        "rollen_ns" -> "Rollen-Modul"
        "greifer" -> "Greifer-Einheit"
        "tisch" -> "Arbeitstisch"
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