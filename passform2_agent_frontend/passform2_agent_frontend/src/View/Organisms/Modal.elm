module View.Organisms.Modal exposing (viewActiveMenu)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
-- Importe auf die neue Atomic-Struktur angepasst:
import View.Molecules.HardwareStatus exposing (viewStatusBadge, viewSignalStrength)
import View.Organisms.Sidebar as Sidebar


-- --- HELPERS FÜR STATUS-MAPPING ---

statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning" -- Mappt auf Gelb (für Simulation/Wartezustand)
        Missing -> "danger"
        Error -> "danger"
        UnknownStatus -> "unknown"

statusToText : HardwareStatus -> String
statusToText status =
    case status of
        Online -> "Bereit"
        Standby -> "Standby"
        Missing -> "Fehlt"
        Error -> "Fehler"
        UnknownStatus -> "Offline"


{-| Organismus: Das zentrale Modal-System.
Steuert sowohl das Auswahlmenü (Selection) als auch die Agenten-Einstellungen (Settings).
-}
viewActiveMenu : Model -> Maybe MenuType -> Html Msg
viewActiveMenu model maybeMenu =
    case maybeMenu of
        Just (SelectionMenu cell) ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [] [ text "Modul hinzufügen" ]
                    , button [ onClick (StartAgent FTF cell), class "btn-ftf" ] [ text "FTF (Transport)" ]
                    , button [ onClick (StartAgent Conveyeur cell) ] [ text "Conveyeur Modul" ]
                    , button [ onClick (StartAgent RollenModul cell) ] [ text "Rollen Modul" ]
                    , button [ onClick (StartAgent Greifer cell) ] [ text "Greifer Modul" ]
                    , button [ onClick (StartAgent Station cell) ] [ text "Tisch Modul" ]
                    , button [ onClick (StartAgent Mensch cell) ] [ text "Mensch (Bediener)" ]
                    , hr [] []
                    , button [ onClick CloseMenu, class "btn-close" ] [ text "Abbrechen" ]
                    ]
                ]

        Just (SettingsMenu cell agent) ->
            let
                aid = agent.agent_id |> Maybe.withDefault "unnamed"

                maybePi = model.connectedHardware
                        |> List.filter (\hw -> hw.pi_id == "PassForM2-Pi5-Client")
                        |> List.head

                ( piStat, piText ) =
                    case maybePi of
                        Just pi -> 
                            if pi.pi_exists then ("online", "Verbunden") else ("missing", "Fehlt")
                        Nothing -> 
                            ("missing", "Offline")

                ( nfcStat, nfcText ) =
                    case maybePi of
                        Just pi -> 
                            ( statusToClass pi.rfid_status, statusToText pi.rfid_status )
                        Nothing -> 
                            ("unknown", "Kein Pi")

                isHardwareReady =
                    case maybePi of
                        Just pi -> 
                            pi.pi_exists && (pi.rfid_status == Online || pi.rfid_status == Standby)
                        Nothing -> 
                            False
                
                mappedSignalStrength =
                    case maybePi of
                        Just pi -> if pi.pi_exists then 100 else 0
                        Nothing -> 0

                agentIsActive = mappedSignalStrength > 0

                lastWrittenView =
                    case model.lastWrittenId of
                        Just lastId ->
                            div [ class "last-written-info" ]
                                [ text ("Chip-ID: " ++ lastId) ]
                        Nothing ->
                            text ""
            in
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [] [ text (Sidebar.formatType agent.module_type) ]
                    , div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]

                    , div [ class "hardware-status-row" ]
                        [ viewStatusBadge "NFC Sensor" nfcStat nfcText
                        , viewStatusBadge "Node" piStat piText
                        ]

                    , viewSignalStrength agentIsActive mappedSignalStrength

                    , hr [] []
                    
                    , button
                        [ onClick (RequestNfcWrite aid)
                        , class "btn-nfc-write"
                        , disabled (not isHardwareReady)
                        ]
                        [ text "ID auf Chip brennen" ]
                    
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