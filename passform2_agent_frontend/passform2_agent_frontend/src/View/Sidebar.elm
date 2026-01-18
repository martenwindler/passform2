module View.Sidebar exposing (view, formatType)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Dict 
import Types exposing (..)

view : Model -> Html Msg
view model =
    aside [ class "sidebar" ]
        [ h2 [] [ text "Konfiguration" ]
        
        -- 1. System-Logs (Immer ganz oben für Feedback)
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

        -- 3. Aktive Module (Die "Teilnehmer" am Contract-Net)
        , div [ class "sidebar-section" ]
            [ h3 [] [ text ("Aktive Module (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
            , if Dict.isEmpty model.agents then
                p [ class "sidebar-hint" ] [ text "Keine Module im Gitter." ]
              else
                ul [ class "agent-list" ] 
                    (model.agents |> Dict.values |> List.sortBy .module_type |> List.map viewAgentItem)
            ]

        -- 4. Hauptabschnitt Gitter (Bündelt Dimensionen & Konfiguration)
        , div [ class "sidebar-section grid-main-section" ]
            [ h3 [] [ text "Gitterkonfiguration" ]
            
            -- Unterelement A: Dimensionen
            , div [ class "nested-box" ]
                [ p [ class "sidebar-hint" ] [ text "Gitter-Dimensionen:" ]
                , div [ class "input-row" ] 
                    [ div [ class "input-group" ]
                        [ label [] [ text "B:" ]
                        , input [ type_ "number", value (String.fromInt model.gridWidth), onInput SetGridWidth ] []
                        ]
                    , div [ class "input-group" ]
                        [ label [] [ text "L:" ]
                        , input [ type_ "number", value (String.fromInt model.gridHeight), onInput SetGridHeight ] []
                        ]
                    ]
                ]

        -- Unterelement B: Konfiguration / Aktionen
        , div [ class "nested-box" ]
            [ p [ class "sidebar-hint" ] [ text "Konfigurations-Management:" ]
            , div [ class "actions-compact-grid" ]
                [ button [ class "btn-secondary btn-small", onClick LoadDefaultConfig ] [ text "📂 Default" ]
                , button [ class "btn-primary btn-small", onClick SetCurrentAsDefault ] [ text "💾 Save" ]
                , button [ class "btn-secondary btn-small", onClick ExportConfig ] [ text "📤 Export" ]
                , button [ class "btn-secondary btn-small", onClick ImportConfigTrigger ] [ text "📥 Import" ]
                ]
            , button [ class "btn-danger btn-small btn-full-width", style "margin-top" "8px", onClick ClearGrid ] 
                [ span [] [ text "🗑️" ], text " Gitter leeren" ]
            ]
        ]

        -- 2. Contract-Net-Protokoll (Dezentrale Logik nach Harlan)
        , div [ class "sidebar-section mission-section" ]
            [ h3 [] [ text "Contract-Net-Protokoll" ]
            , viewContractNetSettings model
            , hr [] []
            , viewCoordinateStatus "📍 Start-Position" model.pathStart
            , viewCoordinateStatus "🎯 Ziel-Position" model.pathGoal
            
            , if model.loading then
                div [ class "planning-status-box" ]
                    [ div [ class "spinner" ] []
                    , span [] [ text "Verhandle Verträge..." ]
                    ]
              else
                text ""

            , button 
                [ class (if canPlan model && not model.loading then "btn-sidebar-primary" else "btn-disabled")
                , onClick (StartPlanning False)
                , disabled (not (canPlan model) || model.loading)
                ] 
                [ text (if model.loading then "Ausschreibung läuft..." else "Mission starten") ]
            ]

        -- 5. Agile Robotics Ranger (Ganz unten als Hardware-Schnittstelle)
        , div [ class "sidebar-section ranger-section highlight-hardware" ]
            [ h3 [] [ text "Agile Robotics Ranger" ]
            
            -- Status & Telemetrie
            , div [ class "status-grid" ]
                [ div [ class "status-item" ] 
                    [ span [ class "label" ] [ text "CAN-Bus:" ]
                    , span [ class "value status-online" ] [ text " Verbunden" ] 
                    ]
                , div [ class "status-item" ] 
                    [ span [ class "label" ] [ text "Modus:" ]
                    , span [ class "value" ] [ text " Navigation" ] 
                    ]
                ]
            
            -- Ranger Pfadplanung (Zweiter Planner)
            , div [ class "nested-planning-box" ]
                [ p [ class "sidebar-hint" ] [ text "Hardware-Pfadplanung:" ]
                , viewCoordinateStatus "📍 Start-Position" model.pathStart
                , viewCoordinateStatus "🎯 Ziel-Position" model.pathGoal
                
                , button 
                    [ class "btn-sidebar-ranger btn-full-width"
                    , onClick (SetMode "Ranger_Execute_Path") 
                    , disabled (not (canPlan model))
                    ] 
                    [ text "Ranger Pfad berechnen & ausführen" ]
                ]
            ]
        ]

-- --- HILFSFUNKTIONEN ---

viewContractNetSettings : Model -> Html Msg
viewContractNetSettings model =
    let
        weights = model.planningWeights
    in
    div [ class "nested-settings" ]
        [ p [ class "sidebar-hint" ] [ text "Gebotsparameter (Kostenfunktion):" ]
        , div [ class "parameter-grid-row" ]
            [ div [ class "parameter-item" ]
                [ label [] [ text "Basis" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.execution_time_default), onInput (SetWeight "execution_time_default") ] []
                ]
            , div [ class "parameter-item" ]
                [ label [] [ text "Komplex" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.complex_module_time), onInput (SetWeight "complex_module_time") ] []
                ]
            , div [ class "parameter-item" ]
                [ label [] [ text "Mensch" ]
                , input [ type_ "number", step "0.5", value (String.fromFloat weights.human_extra_weight), onInput (SetWeight "human_extra_weight") ] []
                ]
            , div [ class "parameter-item" ]
                [ label [] [ text "Nähe" ]
                , input [ type_ "number", step "0.1", value (String.fromFloat weights.proximity_penalty), onInput (SetWeight "proximity_penalty") ] []
                ]
            ]
        , button [ class "btn-apply btn-very-small", onClick SaveWeights ] [ text "Gewichte anwenden" ]
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