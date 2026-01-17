module View.Modal exposing (viewActiveMenu)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import View.HardwareStatus exposing (viewStatusBadge, viewSignalStrength)
import View.Sidebar as Sidebar


viewActiveMenu : Model -> Maybe MenuType -> Html Msg
viewActiveMenu model maybeMenu =
    case maybeMenu of
        Just (SelectionMenu cell) ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [ style "color" "#fff" ] [ text "Modul hinzufügen" ]
                    , button [ onClick (StartAgent "ftf" cell), class "btn-ftf" ] [ text "FTF (Transport)" ]
                    , button [ onClick (StartAgent "conveyeur" cell) ] [ text "Conveyeur Modul" ]
                    , button [ onClick (StartAgent "rollen_ns" cell) ] [ text "Rollen Modul" ]
                    , button [ onClick (StartAgent "greifer" cell) ] [ text "Greifer Modul" ]
                    , button [ onClick (StartAgent "tisch" cell) ] [ text "Tisch Modul" ]
                    , button [ onClick (StartAgent "mensch" cell) ] [ text "Mensch (Bediener)" ]
                    , hr [] []
                    , button [ onClick CloseMenu, class "btn-close" ] [ text "Abbrechen" ]
                    ]
                ]

        Just (SettingsMenu cell agent) ->
            let
                aid =
                    agent.agent_id |> Maybe.withDefault "unnamed"

                -- 1. Suchen des Raspberry Pi in der Hardware-Registry
                maybePi =
                    model.connectedHardware
                        |> List.filter (\hw -> hw.pi_id == "PassForM2-Pi5-Client")
                        |> List.head

                -- 2. Status für den Raspberry Pi bestimmen
                ( piStat, piText ) =
                    case maybePi of
                        Just pi ->
                            if pi.pi_exists then
                                ( "online", "Verbunden" )
                            else
                                ( "missing", "Fehlt" )

                        Nothing ->
                            ( "missing", "Offline" )

                -- 3. Status für den RFID-Sensor (RC522) am Pi bestimmen
                ( nfcStat, nfcText ) =
                    case maybePi of
                        Just pi ->
                            if pi.rfid_status == "online" then
                                ( "online", "Bereit" )
                            else
                                ( "missing", "Fehlt" )

                        Nothing ->
                            ( "unknown", "Kein Pi" )

                -- 4. Übergeordnete Online-Logik (Kombination aus Socket und Pi-Status)
                isHardwareReady =
                    case maybePi of
                        Just pi -> pi.pi_exists && pi.rfid_status == "online"
                        Nothing -> False
                
                -- Signalstärke für das UI (basiert weiterhin auf dem Agent-Heartbeat)
                agentIsActive =
                    agent.signal_strength > 0
            in
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [ style "color" "#fff" ] [ text (Sidebar.formatType agent.module_type) ]
                    , div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]

                    -- Hardware-Status-Reihe: Hier siehst du jetzt direkt den Pi-Status
                    , div [ class "hardware-status-row" ]
                        [ viewStatusBadge "RC522 Sensor" nfcStat nfcText
                        , viewStatusBadge "RPI 5 Node" piStat piText
                        ]

                    -- Signalstärke des Agenten (Kommunikation zum Controller)
                    , viewSignalStrength agentIsActive agent.signal_strength

                    , hr [] []
                    
                    -- Der Brenn-Button ist nur aktiv, wenn Pi UND Sensor online sind
                    , button
                        [ onClick (RequestNfcWrite aid)
                        , class "btn-nfc-write"
                        , disabled (not isHardwareReady)
                        , style "opacity" (if isHardwareReady then "1" else "0.5")
                        ]
                        [ text "ID auf Chip brennen" ]

                    , hr [] []
                    , button [ onClick (RotateAgent cell) ] [ text "Drehen" ]
                    , button [ onClick (SetPathStart cell), class "btn-path-start" ] [ text "Start" ]
                    , button [ onClick (SetPathGoal cell), class "btn-path-goal" ] [ text "Ziel" ]
                    , hr [] []
                    , button [ onClick (RemoveAgent cell), class "btn-delete" ] [ text "Löschen" ]
                    , button [ onClick CloseMenu, class "btn-close" ] [ text "Schließen" ]
                    ]
                ]

        Nothing ->
            text ""