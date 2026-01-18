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

                -- 4. Übergeordnete Online-Logik (Pi UND Sensor müssen da sein)
                isHardwareReady =
                    case maybePi of
                        Just pi -> pi.pi_exists && pi.rfid_status == "online"
                        Nothing -> False
                
                -- 5. Signalstärke-Mapping (Kopie des Pi-Status)
                mappedSignalStrength =
                    case maybePi of
                        Just pi -> if pi.pi_exists then 100 else 0
                        Nothing -> 0

                agentIsActive =
                    mappedSignalStrength > 0

                -- 6. NEU: Anzeige der zuletzt geschriebenen RFID ID
                -- Erscheint im Erfolgsfall als grünes Badge
                lastWrittenView =
                    case model.lastWrittenId of
                        Just lastId ->
                            div [ class "last-written-badge"
                                , style "margin-top" "10px"
                                , style "padding" "8px"
                                , style "background" "rgba(72, 187, 120, 0.15)"
                                , style "border" "1px solid #48bb78"
                                , style "border-radius" "4px"
                                , style "color" "#48bb78"
                                , style "text-align" "center"
                                , style "font-size" "0.85em"
                                ]
                                [ text ("Zuletzt auf Chip geschrieben: " ++ lastId) ]
                        Nothing ->
                            text ""
            in
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [ style "color" "#fff" ] [ text (Sidebar.formatType agent.module_type) ]
                    , div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]

                    -- Hardware-Status-Reihe
                    , div [ class "hardware-status-row" ]
                        [ viewStatusBadge "RC522 Sensor" nfcStat nfcText
                        , viewStatusBadge "RPI 5 Node" piStat piText
                        ]

                    -- Signalstärke (Mapping von Hardware-Status)
                    , viewSignalStrength agentIsActive mappedSignalStrength

                    , hr [] []
                    
                    -- NFC Schreib-Sektion
                    , button
                        [ onClick (RequestNfcWrite aid)
                        , class "btn-nfc-write"
                        , disabled (not isHardwareReady)
                        , style "opacity" (if isHardwareReady then "1" else "0.5")
                        ]
                        [ text "ID auf Chip brennen" ]
                    
                    -- HIER WIRD DIE ID EINGEBLENDET
                    , lastWrittenView

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