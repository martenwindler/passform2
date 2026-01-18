module View.Organisms.Modal exposing (viewActiveMenu)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Json.Decode as Decode
import Types exposing (..)
import Types.Domain exposing (..)
import View.Molecules.HardwareStatus exposing (viewStatusBadge, viewSignalStrength)
import View.Organisms.Sidebar as Sidebar


{-| Organismus: Das aktive Modal / Menü.
Gibt Html Msg zurück und mappt alle internen Nachrichten explizit.
-}
viewActiveMenu : Model -> Maybe MenuType -> Html Msg
viewActiveMenu model maybeMenu =
    case maybeMenu of
        Just (SelectionMenu cell) ->
            -- Wir mappen das onClick-Event direkt auf die globale Msg
            div [ class "modal-overlay", onClick (AgentsMsg CloseMenu) ]
                [ div [ class "modal-content", stopPropagationOnClick ]
                    [ h3 [] [ text "Modul hinzufügen" ]
                    , button [ onClick (AgentsMsg (StartAgent FTF cell)), class "btn-ftf" ] [ text "FTF (Transport)" ]
                    , button [ onClick (AgentsMsg (StartAgent Conveyeur cell)) ] [ text "Conveyeur Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent RollenModul cell)) ] [ text "Rollen Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Greifer cell)) ] [ text "Greifer Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Station cell)) ] [ text "Tisch Modul" ]
                    , button [ onClick (AgentsMsg (StartAgent Mensch cell)) ] [ text "Mensch (Bediener)" ]
                    , hr [] []
                    , button [ onClick (AgentsMsg CloseMenu), class "btn-close" ] [ text "Abbrechen" ]
                    ]
                ]

        Just (SettingsMenu cell agent) ->
            let
                aid = Maybe.withDefault "---" agent.agent_id
                
                ( nfcStat, nfcText ) = case model.nfcStatus of
                    Online -> ( "online", "Bereit" )
                    Error -> ( "error", "Fehler" )
                    _ -> ( "unknown", "Warten..." )

                ( piStat, piText ) = ( "online", "Verbunden" )
                
                agentIsActive = agent.signal_strength > 0
                mappedSignalStrength = agent.signal_strength
            in
            -- Auch hier: Das Overlay-Event wird auf die globale Msg gemappt
            div [ class "modal-overlay", onClick (AgentsMsg CloseMenu) ]
                [ div [ class "modal-content settings-modal", stopPropagationOnClick ]
                    [ h3 [] [ text (Sidebar.formatType agent.module_type) ]
                    , div [ class "agent-id-badge" ] [ text ("ID: " ++ aid) ]

                    , -- Hardware Sektion (Mappt auf HardwareMsg)
                      Html.map HardwareMsg <|
                        div []
                            [ div [ class "hardware-status-row" ]
                                [ viewStatusBadge "NFC Sensor" nfcStat nfcText
                                , viewStatusBadge "Node" piStat piText
                                ]
                            , viewSignalStrength agentIsActive mappedSignalStrength
                            , hr [] []
                            , button
                                [ onClick (RequestNfcWrite aid)
                                , class "btn-nfc-write"
                                , disabled model.waitingForNfc
                                ]
                                [ text (if model.waitingForNfc then "Schreibe..." else "ID auf Chip brennen") ]
                            ]

                    , hr [] []

                    , -- Planungs Sektion (Mappt auf PlanningMsg)
                      Html.map PlanningMsg <|
                        div [ class "modal-action-row" ]
                            [ button [ onClick (SetPathStart cell), class "btn-path-start" ] [ text "Start" ]
                            , button [ onClick (SetPathGoal cell), class "btn-path-goal" ] [ text "Ziel" ]
                            ]

                    , hr [] []

                    , -- Agenten Sektion (Mappt auf AgentsMsg)
                      Html.map AgentsMsg <|
                        div [ class "modal-action-row column" ]
                            [ button [ onClick (RotateAgent cell), class "btn-rotate" ] [ text "Drehen" ]
                            , button [ onClick (RemoveAgent cell), class "btn-delete" ] [ text "Löschen" ]
                            , button [ onClick CloseMenu, class "btn-close" ] [ text "Schließen" ]
                            ]
                    ]
                ]

        Nothing ->
            text ""


-- --- HELPER ---

{-| Korrigierte Typ-Annotation: Muss Attribute Msg sein, da NoOp eine globale Msg ist. -}
stopPropagationOnClick : Attribute Msg
stopPropagationOnClick =
    Html.Events.stopPropagationOn "click" (Decode.succeed ( NoOp, True ))