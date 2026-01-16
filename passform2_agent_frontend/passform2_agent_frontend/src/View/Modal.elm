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

                ( nfcStat, nfcText ) =
                    case model.nfcStatus of
                        "online" ->
                            ( "online", "Bereit" )

                        "missing" ->
                            ( "missing", "Fehlt" )

                        _ ->
                            ( "unknown", "Prüfe..." )

                piIsOnline =
                    model.connected && agent.signal_strength > 0

                ( piStat, piText ) =
                    if piIsOnline then
                        ( "online", "Online" )

                    else
                        ( "missing", "Offline" )
            in
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ h3 [ style "color" "#fff" ] [ text (Sidebar.formatType agent.module_type) ]
                    , div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]

                    -- Hardware-Status-Reihe (Die Funktionen erzeugen die weißen Pillen)
                    , div [ class "hardware-status-row" ]
                        [ viewStatusBadge "NFC-Reader" nfcStat nfcText
                        , viewStatusBadge "Raspberry Pi" piStat piText
                        ]

                    -- Signalstärke (Ebenfalls im Pillen-Look)
                    , viewSignalStrength piIsOnline agent.signal_strength

                    , hr [] []
                    , button
                        [ onClick (RequestNfcWrite aid)
                        , class "btn-nfc-write"
                        , disabled (model.nfcStatus == "missing" || not piIsOnline)
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