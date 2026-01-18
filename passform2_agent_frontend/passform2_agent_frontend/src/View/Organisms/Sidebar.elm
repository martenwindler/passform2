module View.Organisms.Sidebar exposing (view, formatType)

import Dict
import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, onInput)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Icons as Icons
import View.Molecules.HardwareStatus exposing (viewStatusBadge)


view : Model -> Html Msg
view model =
    aside [ class "sidebar-container" ]
        [ -- 1. Navigation Rail
          div [ class "sidebar-rail" ]
            [ -- Der Toggle-Button nutzt AgentsMsg, also mappen wir ihn auf Msg
              Html.map AgentsMsg <|
                button [ class "rail-btn toggle-btn", onClick ToggleSidebar ]
                    [ text (if model.sidebarOpen then "«" else "»") ]
            
            -- Die Rail-Buttons liefern jetzt Html Msg (wegen internem Mapping)
            , railButton Icons.iconPlanning "Planung" TabPlanning model
            , railButton Icons.iconAgents "Module" TabAgents model
            , railButton Icons.iconHardware "Hardware" TabHardware model
            , railButton Icons.iconLogs "Logs" TabLogs model
            
            , div [ class "rail-spacer" ] []
            ]
        
        -- 2. Sidebar Content (Mapping passiert in viewActiveTab / viewPlanningTab etc.)
        , if model.sidebarOpen then
            div [ class "sidebar-content" ]
                [ viewTabHeader model
                , div [ class "tab-body" ] [ viewActiveTab model ]
                ]
          else
            text ""
        ]


-- --- RAIL HELPERS ---

{-| 
  Korrektur: Wir nehmen ein generisches Icon (Html msg) und mappen es 
  intern auf AgentsMsg, damit der Button-Inhalt zum onClick-Handler passt.
-}
railButton : Html msg -> String -> SidebarTab -> Model -> Html Msg
railButton icon label tab model =
    -- Wir mappen das Ergebnis direkt auf Msg, damit es in der Rail-Liste (view) passt
    Html.map AgentsMsg <|
        button
            [ classList [ ( "rail-btn", True ), ( "active", model.activeSidebarTab == tab ) ]
            , title label
            , onClick (SwitchSidebarTab tab)
            ]
            -- Das Icon wird hier "gecastet", damit es in die Liste der AgentsMsg-Buttons passt
            [ span [ class "rail-icon" ] [ Html.map (always (SwitchSidebarTab tab)) icon ] ]


-- --- TAB ROUTING ---

viewActiveTab : Model -> Html Msg
viewActiveTab model =
    case model.activeSidebarTab of
        TabPlanning -> viewPlanningTab model
        TabAgents -> viewAgentsTab model
        TabHardware -> viewHardwareTab model
        TabLogs -> viewLogsTab model


-- --- TAB: PLANNING ---

viewPlanningTab : Model -> Html Msg
viewPlanningTab model =
    div []
        [ div [ class "sidebar-section" ]
            [ h3 [] [ text "Gitterdimensionen" ]
            , Html.map PlanningMsg <|
                div [ class "nested-box" ]
                    [ div [ class "input-row" ]
                        [ div [ class "input-group" ]
                            [ label [] [ text "B:" ], input [ type_ "number", value (String.fromInt model.gridWidth), onInput SetGridWidth ] [] ]
                        , div [ class "input-group" ]
                            [ label [] [ text "L:" ], input [ type_ "number", value (String.fromInt model.gridHeight), onInput SetGridHeight ] [] ]
                        ]
                    ]
            ]
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Contract-Net (Harlan)" ]
            , Html.map PlanningMsg (viewContractNetSettings model)
            , hr [] []
            , viewCoordinateStatus Icons.iconGoal "Start" "status-start" model.pathStart
            , viewCoordinateStatus Icons.iconGoal "Ziel" "status-goal" model.pathGoal
            , if model.loading then
                div [ class "planning-status-box" ] [ div [ class "spinner" ] [], text "Verhandle..." ]
              else
                text ""
            , Html.map PlanningMsg <|
                button
                    [ class (if canPlan model && not model.loading then "btn-sidebar-primary" else "btn-disabled")
                    , onClick (StartPlanning False)
                    , disabled (not (canPlan model) || model.loading)
                    ]
                    [ text "Mission starten" ]
            ]
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Management" ]
            , Html.map AgentsMsg <|
                div [ class "actions-compact-grid" ]
                    [ button [ class "btn-secondary btn-small", onClick LoadDefaultConfig ] [ text "📂 Default" ]
                    , button [ class "btn-primary btn-small", onClick SetCurrentAsDefault ] [ text "💾 Save" ]
                    , button [ class "btn-secondary btn-small", onClick ExportConfig ] [ text "📤 Export" ]
                    , button [ class "btn-secondary btn-small", onClick ImportConfigTrigger ] [ text "📥 Import" ]
                    , button [ class "btn-danger btn-small", onClick ClearGrid ]
                        [ Icons.iconTrash, span [] [ text " Gitter löschen" ] ]
                    ]
            ]
        ]


-- --- TAB: AGENTS ---

viewAgentsTab : Model -> Html Msg
viewAgentsTab model =
    div [ class "sidebar-section" ]
        [ h3 [] [ text ("Teilnehmer (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
        , if Dict.isEmpty model.agents then
            p [ class "sidebar-hint" ] [ text "Keine Module im Gitter." ]
          else
            Html.map AgentsMsg <|
                ul [ class "agent-list" ]
                    (model.agents
                        |> Dict.values
                        |> List.sortBy (\a -> formatType a.module_type)
                        |> List.map viewAgentItem
                    )
        ]


-- --- TAB: HARDWARE ---

viewHardwareTab : Model -> Html Msg
viewHardwareTab model =
    div []
        [ div [ class "sidebar-section highlight-hardware" ]
            [ h3 [] [ text "Agile Robotics Ranger" ]
            , div [ class "status-grid" ]
                [ div [ class "status-item" ]
                    [ span [ class "label" ] [ text "CAN-Bus: " ]
                    , span [ class "value online" ] [ text "Verbunden" ]
                    ]
                , div [ class "status-item" ]
                    [ span [ class "label" ] [ text "Batterie: " ]
                    , span [ class "value" ] [ text (model.rangerBattery |> Maybe.map (\v -> String.fromFloat v ++ "V") |> Maybe.withDefault "---") ]
                    ]
                ]
            , div [ class "nested-planning-box" ]
                [ p [ class "sidebar-hint" ] [ text "Hardware-Pfadplanung:" ]
                , Html.map HardwareMsg <|
                    button [ class "btn-sidebar-ranger", onClick (SetMode "Ranger_Execute_Path"), disabled (not (canPlan model)) ]
                        [ text "Pfad an Ranger senden" ]
                ]
            ]
        , div [ class "sidebar-section" ]
            [ h3 [] [ text "Raspberry Pi Nodes" ]
            , div [ class "hardware-list" ] 
                (List.map (\hw -> Html.map HardwareMsg (viewHardwareItem hw)) model.connectedHardware)
            ]
        ]


-- --- TAB: LOGS ---

viewLogsTab : Model -> Html Msg
viewLogsTab model =
    div [ class "sidebar-section fill" ]
        [ h3 [] [ text "System-Historie" ]
        , div [ id "log-container", class "log-container" ]
            (List.map
                (\log ->
                    div [ class ("log-entry " ++ logLevelToClass log.level) ]
                        [ span [ class "log-dot" ] []
                        , text log.message
                        ]
                )
                model.logs
            )
        ]


-- --- HILFSFUNKTIONEN (Mit spezifischen Typen) ---

viewHardwareItem : HardwareDevice -> Html HardwareMsg
viewHardwareItem hw =
    div [ class "nested-box hw-item" ]
        [ div [ class "hw-header" ] [ b [] [ text hw.pi_id ] ]
        , div [ class "hw-details" ]
            -- viewStatusBadge liefert HardwareMsg
            [ viewStatusBadge "Node" (if hw.pi_exists then "online" else "missing") (if hw.pi_exists then "Online" else "Offline")
            , viewStatusBadge "RFID" (statusToClass hw.rfid_status) (statusToText hw.rfid_status)
            ]
        ]


viewContractNetSettings : Model -> Html PlanningMsg
viewContractNetSettings model =
    let weights = model.planningWeights in
    div [ class "nested-settings" ]
        [ div [ class "parameter-grid-row" ]
            [ paramInput "Basis" weights.execution_time_default "execution_time_default"
            , paramInput "Komp." weights.complex_module_time "complex_module_time"
            , paramInput "Mensch" weights.human_extra_weight "human_extra_weight"
            , paramInput "Nähe" weights.proximity_penalty "proximity_penalty"
            ]
        , button [ class "btn-secondary btn-very-small", onClick SaveWeights ] [ text "Anwenden" ]
        ]


paramInput : String -> Float -> String -> Html PlanningMsg
paramInput lbl val field =
    div [ class "parameter-item" ]
        [ label [] [ text lbl ]
        , input [ type_ "number", step "0.1", value (String.fromFloat val), onInput (SetWeight field) ] []
        ]


viewAgentItem : AgentModule -> Html AgentsMsg
viewAgentItem agent =
    li [ class ("agent-item " ++ (if agent.is_dynamic then "dynamic-agent" else "static-agent")) ]
        [ div [ class "agent-info" ]
            [ div [ class "agent-header" ]
                [ span [ class "agent-label" ] [ text (formatType agent.module_type) ]
                , if agent.is_dynamic then span [ class "badge dynamic" ] [ text "Mobil" ] else text ""
                ]
            , span [ class "agent-coords" ] [ text ("(" ++ String.fromInt agent.position.x ++ "," ++ String.fromInt agent.position.y ++ ")") ]
            ]
        , button [ class "btn-icon-delete", onClick (RemoveAgent agent.position) ] [ text "×" ]
        ]


-- --- STATISCHE HELPERS (Html msg / Html Msg) ---

viewTabHeader : Model -> Html msg
viewTabHeader model =
    let
        titleText =
            case model.activeSidebarTab of
                TabPlanning -> "Planung & Gewichte"
                TabAgents -> "Aktive Module"
                TabHardware -> "Hardware-Status"
                TabLogs -> "System-Historie"
    in
    div [ class "sidebar-tab-header" ] [ h2 [] [ text titleText ] ]

formatType : ModuleType -> String
formatType t =
    case t of
        FTF -> "FTF Transport"
        Conveyeur -> "Conveyeur"
        RollenModul -> "Rollen-Modul"
        Mensch -> "Mensch"
        Greifer -> "Greifer"
        Station -> "Station"
        UnknownModule s -> s

viewCoordinateStatus : Html msg -> String -> String -> Maybe GridCell -> Html msg
viewCoordinateStatus icon label statusClass maybeCell =
    div [ class ("coord-item " ++ statusClass) ]
        [ span [ class "icon-container" ] [ icon ]
        , span [ class "coord-label" ] [ text (label ++ ": ") ]
        , case maybeCell of
            Just cell ->
                span [ class "value text-highlight" ]
                    [ text ("(" ++ String.fromInt cell.x ++ "," ++ String.fromInt cell.y ++ ")") ]
            Nothing ->
                span [ class "value text-muted" ] [ text "---" ]
        ]

logLevelToClass : LogLevel -> String
logLevelToClass level =
    case level of
        Success -> "success"
        Info -> "info"
        Warning -> "warning"
        Danger -> "error"

statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning"
        Missing -> "missing"
        Error -> "error"
        UnknownStatus -> "unknown"

statusToText : HardwareStatus -> String
statusToText status =
    case status of
        Online -> "Online"
        Standby -> "Standby"
        Missing -> "Fehlt"
        Error -> "Fehler"
        UnknownStatus -> "Unbekannt"

canPlan : Model -> Bool
canPlan model =
    case ( model.pathStart, model.pathGoal ) of
        ( Just _, Just _ ) -> True
        _ -> False