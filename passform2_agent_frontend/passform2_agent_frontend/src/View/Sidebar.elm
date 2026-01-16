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

        -- NFC Warte-Indikator
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
            , div [ class "actions-compact-grid" ]
                [ button [ class "btn-secondary btn-small", onClick LoadDefaultConfig ] 
                    [ span [] [ text "📂" ], text " Standard" ]
                , button [ class "btn-primary btn-small", onClick SetCurrentAsDefault ] 
                    [ span [] [ text "💾" ], text " Speichern" ]
                , button [ class "btn-secondary btn-small", onClick ExportConfig ] 
                    [ span [] [ text "📤" ], text " Export" ]
                , button [ class "btn-secondary btn-small", onClick ImportConfigTrigger ] 
                    [ span [] [ text "📥" ], text " Import" ]
                ]
            , button [ class "btn-danger btn-small btn-full-width", onClick ClearGrid ] 
                [ span [] [ text "🗑️" ], text " Gitter leeren" ]
            ]

        -- 4. Planungs-Parameter
        , viewPlanningSettings model

        -- 5. Liste der aktiven Module
        , div [ class "sidebar-section" ]
            [ h3 [] [ text ("Aktive Module (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
            , if Dict.isEmpty model.agents then
                p [ class "sidebar-hint" ] [ text "Keine Module im Gitter." ]
              else
                ul [ class "agent-list" ] 
                    (model.agents |> Dict.values |> List.sortBy .module_type |> List.map viewAgentItem)
            ]

        -- 6. Missions-Status & Planung
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

viewPlanningSettings : Model -> Html Msg
viewPlanningSettings model =
    let
        weights = model.planningWeights
    in
    div [ class "sidebar-section settings-section" ]
        [ h3 [] [ text "Planungs-Parameter" ]
        -- NEU: Ein Container für eine Raster-Zeile
        , div [ class "parameter-grid-row" ]
            [ -- Item 1
              div [ class "parameter-item" ]
                [ label [] [ text "Basis (s)" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.execution_time_default), onInput (SetWeight "execution_time_default") ] []
                ]
            -- Item 2
            , div [ class "parameter-item" ]
                [ label [] [ text "Komplex (s)" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.complex_module_time), onInput (SetWeight "complex_module_time") ] []
                ]
            -- Item 3
            , div [ class "parameter-item" ]
                [ label [] [ text "Mensch" ]
                , input [ type_ "number", step "0.5", value (String.fromFloat weights.human_extra_weight), onInput (SetWeight "human_extra_weight") ] []
                ]
            -- Item 4
            , div [ class "parameter-item" ]
                [ label [] [ text "Nähe" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.proximity_penalty), onInput (SetWeight "proximity_penalty") ] []
                ]
            ]
        , button [ class "btn-apply", onClick SaveWeights ] [ text "Parameter anwenden" ]
        ]

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
        "ftf" -> "FTF Transport"
        "conveyeur" -> "Conveyeur-Modul"
        "rollen_ns" -> "Rollen-Modul"
        "mensch" -> "Mensch (Bediener)"
        "greifer" -> "Greifer-Einheit"
        "tisch" -> "Arbeitsstation"
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