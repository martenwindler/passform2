module View.Organisms.Modal exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button


{-| Haupt-View des Modals -}
view : Model -> Html Msg
view model =
    case model.activeMenu of
        Nothing ->
            text ""

        Just menuType ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ viewHeader menuType
                    , div [ class "modal-body" ] [ viewBody model menuType ]
                    , viewFooter
                    ]
                ]


viewHeader : MenuType -> Html Msg
viewHeader menuType =
    h3 []
        [ text <|
            case menuType of
                SelectionMenu _ ->
                    "Modul hinzufügen"

                SettingsMenu _ agent ->
                    formatModuleType agent.module_type
        ]


viewBody : Model -> MenuType -> Html Msg
viewBody model menuType =
    case menuType of
        SelectionMenu cell ->
            div [ class "settings-form" ]
                [ button [ onClick (AgentsMsg (StartAgent FTF cell)), class "btn-ftf btn-full" ] [ text "FTF (Transport)" ]
                , button [ onClick (AgentsMsg (StartAgent Conveyeur cell)), class "btn-secondary btn-full" ] [ text "Conveyeur Modul" ]
                , button [ onClick (AgentsMsg (StartAgent RollenModul cell)), class "btn-secondary btn-full" ] [ text "Rollen Modul" ]
                , button [ onClick (AgentsMsg (StartAgent Greifer cell)), class "btn-secondary btn-full" ] [ text "Greifer Modul" ]
                , button [ onClick (AgentsMsg (StartAgent Station cell)), class "btn-secondary btn-full" ] [ text "Station / Tisch" ]
                , button [ onClick (AgentsMsg (StartAgent Mensch cell)), class "btn-secondary btn-full" ] [ text "Mensch (Bediener)" ]
                ]

        SettingsMenu cell agent ->
            let
                aid =
                    agent.agent_id |> Maybe.withDefault "unnamed"

                maybePi =
                    model.connectedHardware
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
                            if pi.rfid_status == Online then ("online", "Bereit") else ("missing", "Fehlt")
                        Nothing ->
                            ("unknown", "Kein Pi")

                isHardwareReady =
                    case maybePi of
                        Just pi ->
                            pi.pi_exists && pi.rfid_status == Online
                        Nothing ->
                            False

                mappedSignal =
                    if isHardwareReady then 100 else 0
            in
            div [ class "settings-form" ]
                [ -- 1. Identifikation
                  div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]
                
                , -- 2. Hardware Status (Weiße Pillen)
                  div [ class "hardware-status-row" ]
                    [ viewOldStatusPill "RC522 Sensor" nfcStat nfcText
                    , viewOldStatusPill "RPI 5 Node" piStat piText
                    ]

                , -- 3. Signalstärke
                  viewSignalBar mappedSignal

                , -- 4. NFC Brenn-Aktion
                  button
                    [ onClick (HardwareMsg (RequestNfcWrite aid))
                    , class "btn-nfc-write btn-full"
                    , disabled (not isHardwareReady)
                    ]
                    [ text "ID auf Chip brennen" ]

                , -- 5. Modul-Ausrichtung
                  button [ class "btn-secondary btn-full", onClick (AgentsMsg (RotateAgent cell)) ] [ text "Modul Drehen" ]

                , -- 6. Modul entfernen
                  Button.danger "Modul entfernen" (AgentsMsg (RemoveAgent cell)) True True
                ]


-- --- HELPER ---

formatModuleType : ModuleType -> String
formatModuleType mType =
    case mType of
        FTF -> "FTF (Transport)"
        Conveyeur -> "Conveyeur Modul"
        RollenModul -> "Rollen Modul"
        Mensch -> "Mensch (Bediener)"
        Greifer -> "Greifer Modul"
        Station -> "Station / Tisch"
        UnknownModule name -> name

viewOldStatusPill : String -> String -> String -> Html msg
viewOldStatusPill label stat txt =
    div [ class "status-column" ]
        [ span [ class "status-label" ] [ text label ]
        , div [ class "status-indicator" ]
            [ div [ class ("status-dot status-" ++ stat) ] []
            , span [ class "status-text" ] [ text txt ]
            ]
        ]

viewSignalBar : Int -> Html msg
viewSignalBar val =
    div [ class "signal-wrapper" ]
        [ span [ class "signal-label-main" ] [ text "Signal" ]
        , div [ class "signal-bar-container" ]
            [ div 
                [ class "signal-bar-fill"
                , style "width" (String.fromInt val ++ "%")
                , style "background-color" (if val > 0 then "#48bb78" else "#f56565") 
                ] [] 
            ]
        , span [ class "signal-value" ] [ text (String.fromInt val ++ "%") ]
        ]

viewFooter : Html Msg
viewFooter =
    button [ class "btn-close btn-full", onClick (AgentsMsg CloseMenu) ] [ text "Abbrechen" ]