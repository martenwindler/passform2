module View.Organisms.Modal exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button
import View.Molecules.FormGroup as FormGroup
import View.Molecules.HardwareStatus as StatusDisplay

{-| Haupt-View des Modals - Alle Buttons nutzen jetzt 'btn-full' -}
view : Model -> Html Msg
view model =
    case model.activeMenu of
        Nothing ->
            text ""

        Just menuType ->
            div [ class "modal-overlay" ]
                [ div [ class "modal-content" ]
                    [ viewHeader menuType
                    , viewBody model menuType
                    , viewFooter
                    ]
                ]

viewHeader : MenuType -> Html Msg
viewHeader menuType =
    h3 [] 
        [ text <|
            case menuType of
                SelectionMenu _ -> "Modul-Typ wählen"
                SettingsMenu _ _ -> "Agenten konfigurieren"
        ]

viewBody : Model -> MenuType -> Html Msg
viewBody model menuType =
    case menuType of
        SelectionMenu pos ->
            -- Liste aller 7 Agententypen untereinander
            div [ class "settings-form" ]
                [ button [ class "btn-ftf btn-full", onClick (AgentsMsg (StartAgent FTF pos)) ] [ text "FTF Transport" ]
                , button [ class "btn-path-goal btn-full", onClick (AgentsMsg (StartAgent Conveyeur pos)) ] [ text "Förderer" ]
                , button [ class "btn-secondary btn-full", onClick (AgentsMsg (StartAgent RollenModul pos)) ] [ text "Rollenmodul" ]
                , button [ class "btn-secondary btn-full", onClick (AgentsMsg (StartAgent Mensch pos)) ] [ text "Mensch (Interaktion)" ]
                , button [ class "btn-secondary btn-full", onClick (AgentsMsg (StartAgent Greifer pos)) ] [ text "Greifer-Einheit" ]
                , button [ class "btn-path-start btn-full", onClick (AgentsMsg (StartAgent Station pos)) ] [ text "Station" ]
                , button [ class "btn-secondary btn-full", onClick (AgentsMsg (StartAgent (UnknownModule "Scanner") pos)) ] [ text "Scanner / Sonstige" ]
                ]

        SettingsMenu pos agent ->
            div [ class "settings-form" ]
                [ div [ class "agent-id-badge" ] [ text (Maybe.withDefault "Keine ID" agent.agent_id) ]
                
                , FormGroup.textField "ID ändern" (Maybe.withDefault "" agent.agent_id) "A-01" 
                    (\val -> AgentsMsg (UpdateAgent pos { agent | agent_id = Just val }))

                , div [ class "hardware-status-row" ]
                    [ div [ class "hardware-pill" ] 
                        [ div [ class "status-dot online" ] []
                        , span [ class "status-text" ] [ text "Verbunden" ] 
                        ]
                    , div [ class "hardware-pill" ] 
                        [ span [ class "signal-value" ] [ text (String.fromInt agent.signal_strength ++ "%") ]
                        ]
                    ]
                
                -- NFC Button (Full Width)
                , button 
                    [ class "btn-nfc-write btn-full"
                    , disabled model.waitingForNfc
                    , onClick (HardwareMsg (RequestNfcWrite (Maybe.withDefault "" agent.agent_id))) 
                    ] 
                    [ text (if model.waitingForNfc then "Schreibe..." else "NFC ID Schreiben") ]
                
                , if model.waitingForNfc then 
                    div [ class "last-written-info" ] [ text "NFC Kontakt halten..." ]
                  else text ""

                -- Löschen Button (Full Width) nutzt das danger-Atom mit isFullWidth=True
                , Button.danger "Agent Löschen" (AgentsMsg (RemoveAgent pos)) True True
                ]

viewFooter : Html Msg
viewFooter =
    button [ class "btn-close btn-full", onClick (AgentsMsg CloseMenu) ] [ text "Schließen" ]