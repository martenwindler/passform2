module View.Organisms.Modal exposing (view)

import Dict
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
                    [ viewHeader model menuType 
                    , div [ class "modal-body" ] [ viewBody model menuType ]
                    , viewFooter
                    ]
                ]


{-| Header mit dynamischer Titelei für Stacking -}
viewHeader : Model -> MenuType -> Html Msg
viewHeader model menuType =
    h3 [ class "mb-4" ] 
        [ text <|
            case menuType of
                SelectionMenu cell ->
                    let
                        currentStack = 
                            model.agents 
                                |> Dict.keys 
                                |> List.filter (\(x, y, l) -> x == cell.x && y == cell.y) 
                                |> List.length
                    in
                    if currentStack > 0 then
                        "Modul stapeln (Ebene " ++ String.fromInt currentStack ++ ")"
                    else
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
                    , button 
                        [ class "btn-ftf btn-full text-button disabled"
                        , disabled True 
                        ] 
                        [ text "FTF (Transport)" ]
                    , button [ onClick (AgentsMsg (StartAgent Conveyeur cell)), class "btn-secondary btn-full text-button" ] [ text "Conveyeur Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent RollenModul cell)), class "btn-secondary btn-full text-button" ] [ text "Rollen Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Greifer cell)), class "btn-secondary btn-full text-button" ] [ text "Greifer Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Station cell)), class "btn-secondary btn-full text-button" ] [ text "Station / Tisch" ]
                    , button [ onClick (AgentsMsg (StartAgent Mensch cell)), class "btn-secondary btn-full text-button" ] [ text "Mensch (Bediener)" ]
                    ]
                
                , div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button 
                        [ class "btn-secondary btn-full text-info border-info/20 text-button disabled"
                        , disabled True 
                        ] 
                        [ text "⊕ Als Startpunkt setzen" ]
                    , button 
                        [ class "btn-secondary btn-full text-success border-success/20 text-button disabled"
                        , disabled True 
                        ] 
                        [ text "⚑ Als Zielpunkt setzen" ]
                    ]
                ]

        SettingsMenu cell agent ->
            let
                aid = agent.agent_id |> Maybe.withDefault "unnamed"
                posText = "Position: " ++ String.fromInt cell.x ++ " / " ++ String.fromInt cell.y
                
                stackHeight =
                    model.agents
                        |> Dict.keys
                        |> List.filter (\( x, y, l ) -> x == cell.x && y == cell.y)
                        |> List.length

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
                [ div [ class "flex justify-between items-center mb-4" ]
                    [ div [ class "agent-id-badge text-data" ] [ text ("ID: " ++ aid) ]
                    , div [ class "text-caption font-bold opacity-60" ] [ text posText ]
                    ]
                
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
                    , button [ class "btn-secondary btn-full text-button", onClick (AgentsMsg (RotateAgent agent.position)) ] [ text "Modul Drehen" ]
                    , Button.danger "Modul entfernen" (AgentsMsg (RemoveAgent agent.position)) True True
                    ]

                -- STAPEL OPTION INNERHALB DER SETTINGS
                , if model.editing && stackHeight < 4 then
                    div [ class "mt-6 pt-6 border-t border-cyan-500/30" ]
                        [ div [ class "bg-cyan-500/10 p-4 rounded-sm border border-cyan-500/20" ]
                            [ div [ class "flex justify-between items-center mb-3" ]
                                [ span [ class "text-[0.7rem] font-black text-cyan-400 uppercase tracking-widest" ] [ text "Stapel-Option" ]
                                , span [ class "text-[0.7rem] text-white/40" ] [ text ("Nächste Ebene: " ++ String.fromInt stackHeight) ]
                                ]
                            , button 
                                [ class "w-full py-2 bg-cyan-500 hover:bg-cyan-400 text-black text-[0.8rem] font-black rounded-xs transition-all"
                                , onClick (AgentsMsg (OpenSelectionMenu cell)) 
                                ] 
                                [ text "+ Modul hinzufügen" ]
                            ]
                        ]
                  else
                    text ""

                , div [ class "flex flex-col gap-2 mt-6 pt-4 border-t border-white/10" ]
                    [ button 
                        [ class "btn-secondary btn-full text-info border-info/20 text-button disabled"
                        , disabled True 
                        ] 
                        [ text "⊕ Als Startpunkt setzen" ]
                    , button 
                        [ class "btn-secondary btn-full text-success border-success/20 text-button disabled"
                        , disabled True 
                        ] 
                        [ text "⚑ Als Zielpunkt setzen" ]
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