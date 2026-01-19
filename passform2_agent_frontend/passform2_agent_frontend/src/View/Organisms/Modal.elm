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
                    "Zell-Konfiguration"

                SettingsMenu _ agent ->
                    formatModuleType agent.module_type
        ]


viewBody : Model -> MenuType -> Html Msg
viewBody model menuType =
    case menuType of
        SelectionMenu cell ->
            div [ class "settings-form" ]
                [ -- 1. Modul-Auswahl (Primäre Gruppe)
                  div [ class "flex flex-col gap-2 mb-4" ]
                    [ label [ class "text-[0.6rem] uppercase text-white/30 font-bold mb-2 block" ] [ text "Modul hinzufügen" ]
                    , button [ onClick (AgentsMsg (StartAgent FTF cell)), class "btn-ftf btn-full" ] [ text "FTF (Transport)" ]
                    , button [ onClick (AgentsMsg (StartAgent Conveyeur cell)), class "btn-secondary btn-full" ] [ text "Conveyeur Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent RollenModul cell)), class "btn-secondary btn-full" ] [ text "Rollen Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Greifer cell)), class "btn-secondary btn-full" ] [ text "Greifer Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Station cell)), class "btn-secondary btn-full" ] [ text "Station / Tisch" ]
                    , button [ onClick (AgentsMsg (StartAgent Mensch cell)), class "btn-secondary btn-full" ] [ text "Mensch (Bediener)" ]
                    ]
                
                , -- 2. Navigations-Optionen (Sekundäre Gruppe am Ende, ohne Label)
                  div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button [ onClick (PlanningMsg (SetPathStart cell)), class "btn-secondary btn-full text-info border-info/20" ] [ text "⊕ Als Startpunkt setzen" ]
                    , button [ onClick (PlanningMsg (SetPathGoal cell)), class "btn-secondary btn-full text-success border-success/20" ] [ text "⚑ Als Zielpunkt setzen" ]
                    ]
                ]

        SettingsMenu cell agent ->
            let
                aid = agent.agent_id |> Maybe.withDefault "unnamed"
                maybePi = model.connectedHardware |> List.filter (\hw -> hw.pi_id == "PassForM2-Pi5-Client") |> List.head

                ( piStat, piText ) =
                    case maybePi of
                        Just pi -> if pi.pi_exists then ("online", "Verbunden") else ("missing", "Fehlt")
                        Nothing -> ("missing", "Offline")

                ( nfcStat, nfcText ) =
                    case maybePi of
                        Just pi -> if pi.rfid_status == Online then ("online", "Bereit") else ("missing", "Fehlt")
                        Nothing -> ("unknown", "Kein Pi")

                isHardwareReady =
                    case maybePi of
                        Just pi -> pi.pi_exists && pi.rfid_status == Online
                        Nothing -> False

                mappedSignal = if isHardwareReady then 100 else 0
            in
            div [ class "settings-form" ]
                [ -- Identifikation & Hardware-Status
                  div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]
                , div [ class "hardware-status-row" ]
                    [ viewOldStatusPill "RC522 Sensor" nfcStat nfcText
                    , viewOldStatusPill "RPI 5 Node" piStat piText
                    ]
                , viewSignalBar mappedSignal
                
                , -- Primäre Aktionen
                  div [ class "flex flex-col gap-2 mt-4" ]
                    [ button
                        [ onClick (HardwareMsg (RequestNfcWrite aid))
                        , class "btn-nfc-write btn-full mb-2"
                        , disabled (not isHardwareReady)
                        ]
                        [ text "ID auf Chip brennen" ]
                    , button [ class "btn-secondary btn-full", onClick (AgentsMsg (RotateAgent cell)) ] [ text "Modul Drehen" ]
                    , Button.danger "Modul entfernen" (AgentsMsg (RemoveAgent cell)) True True
                    ]

                , -- Navigations-Optionen (Sekundäre Gruppe am Ende, ohne Label)
                  div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button [ onClick (PlanningMsg (SetPathStart cell)), class "btn-secondary btn-full text-info border-info/20" ] [ text "⊕ Als Startpunkt setzen" ]
                    , button [ onClick (PlanningMsg (SetPathGoal cell)), class "btn-secondary btn-full text-success border-success/20" ] [ text "⚑ Als Zielpunkt setzen" ]
                    ]
                ]


-- --- HELPER (unverändert) ---

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