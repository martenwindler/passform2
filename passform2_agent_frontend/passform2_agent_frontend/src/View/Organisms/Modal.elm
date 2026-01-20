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
    h3 [ class "mb-4" ] 
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
                [ div [ class "flex flex-col gap-2 mb-4" ]
                    [ label [ class "text-label mb-2 block" ] [ text "Modul hinzufügen" ]
                    , button [ onClick (AgentsMsg (StartAgent FTF cell)), class "btn-ftf btn-full text-button" ] [ text "FTF (Transport)" ]
                    , button [ onClick (AgentsMsg (StartAgent Conveyeur cell)), class "btn-secondary btn-full text-button" ] [ text "Conveyeur Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent RollenModul cell)), class "btn-secondary btn-full text-button" ] [ text "Rollen Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Greifer cell)), class "btn-secondary btn-full text-button" ] [ text "Greifer Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Station cell)), class "btn-secondary btn-full text-button" ] [ text "Station / Tisch" ]
                    , button [ onClick (AgentsMsg (StartAgent Mensch cell)), class "btn-secondary btn-full text-button" ] [ text "Mensch (Bediener)" ]
                    ]
                
                , div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button [ onClick (PlanningMsg (SetPathStart cell)), class "btn-secondary btn-full text-info border-info/20 text-button" ] [ text "⊕ Als Startpunkt setzen" ]
                    , button [ onClick (PlanningMsg (SetPathGoal cell)), class "btn-secondary btn-full text-success border-success/20 text-button" ] [ text "⚑ Als Zielpunkt setzen" ]
                    ]
                ]

        SettingsMenu cell agent ->
            let
                aid = agent.agent_id |> Maybe.withDefault "unnamed"
                maybePi = model.connectedHardware |> List.filter (\hw -> hw.pi_id == "PassForM2-Pi5-Client") |> List.head

                ( piStat, piText, piGlow ) =
                    case maybePi of
                        Just pi -> if pi.pi_exists then ("online", "Verbunden", "text-glow-success") else ("missing", "Fehlt", "text-glow-danger")
                        Nothing -> ("missing", "Offline", "text-glow-danger")

                ( nfcStat, nfcText, nfcGlow ) =
                    case maybePi of
                        Just pi -> if pi.rfid_status == Online then ("online", "Bereit", "text-glow-success") else ("missing", "Fehlt", "text-glow-danger")
                        Nothing -> ("unknown", "Kein Pi", "")

                isHardwareReady =
                    case maybePi of
                        Just pi -> pi.pi_exists && pi.rfid_status == Online
                        Nothing -> False

                mappedSignal = if isHardwareReady then 100 else 0
            in
            div [ class "settings-form" ]
                [ div [ class "agent-id-badge text-data mb-4" ] [ text ("ID: " ++ aid) ]
                , div [ class "hardware-status-row" ]
                    [ viewOldStatusPill "RC522 Sensor" nfcStat nfcText nfcGlow
                    , viewOldStatusPill "RPI 5 Node" piStat piText piGlow
                    ]
                , viewSignalBar mappedSignal
                
                , div [ class "flex flex-col gap-2 mt-4" ]
                    [ button
                        [ onClick (HardwareMsg (RequestNfcWrite aid))
                        , class "btn-nfc-write btn-full mb-2 text-button"
                        , disabled (not isHardwareReady)
                        ]
                        [ text "ID auf Chip brennen" ]
                    , button [ class "btn-secondary btn-full text-button", onClick (AgentsMsg (RotateAgent cell)) ] [ text "Modul Drehen" ]
                    , Button.danger "Modul entfernen" (AgentsMsg (RemoveAgent cell)) True True
                    ]

                , div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button [ onClick (PlanningMsg (SetPathStart cell)), class "btn-secondary btn-full text-info border-info/20 text-button" ] [ text "⊕ Als Startpunkt setzen" ]
                    , button [ onClick (PlanningMsg (SetPathGoal cell)), class "btn-secondary btn-full text-success border-success/20 text-button" ] [ text "⚑ Als Zielpunkt setzen" ]
                    ]
                ]


-- --- HELPER ---

viewOldStatusPill : String -> String -> String -> String -> Html msg
viewOldStatusPill label stat txt glowClass =
    div [ class "status-column" ]
        [ span [ class "text-label block mb-1 text-force-black font-bold" ] [ text label ]
        , div [ class "status-indicator" ]
            [ div [ class ("status-dot status-" ++ stat) ] []
            , span [ class ("text-caption font-bold text-force-black " ++ glowClass) ] [ text txt ]
            ]
        ]

viewSignalBar : Int -> Html msg
viewSignalBar val =
    let
        displayValue =
            if val > 0 then String.fromInt val ++ "%" else "---%"
        barColor =
            if val > 0 then "#48bb78" else "#f56565"
    in
    div [ class "signal-wrapper mt-6 w-full" ] 
        [ span [ class "text-label text-force-black font-bold mb-2 block" ] [ text "Signalstärke" ] 
        , div [ class "signal-bar-row" ] 
            [ div [ class "signal-bar-container" ]
                [ div 
                    [ class "signal-bar-fill"
                    , style "width" (String.fromInt val ++ "%")
                    , style "background-color" barColor
                    ] [] 
                ]
            , span [ class "signal-value" ] [ text displayValue ] 
            ]
        ]

viewFooter : Html Msg
viewFooter =
    div [ class "mt-6" ]
        [ button [ class "btn-close btn-full text-button", onClick (AgentsMsg CloseMenu) ] [ text "Abbrechen" ]
        ]

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