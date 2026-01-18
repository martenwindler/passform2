module View.Organisms.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)

{-| Organismus: Die Haupt-Navigationsleiste.
Wir nutzen hier Html AgentsMsg, da die Steuerung (Rechts) diese Nachrichten produziert.
-}
view : Model -> Html AgentsMsg
view model =
    nav [ class "navbar blue-look" ]
        [ -- 1. Links: Logo und Titel (Statische Elemente passen sich dem Typ an)
          div [ class "navbar-section" ]
            [ img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title" ] [ text "Passform 2.0" ]
            ]
        
        -- 2. Mitte: Status-Pille (Monitoring)
        , div [ class "navbar-center" ]
            [ div [ class "status-pill combined-status" ]
                [ -- REST API Verbindung
                  statusIndicator 
                    (if model.connected then Online else Error) 
                    "REST"
                
                , span [ class "status-divider" ] []

                -- ROS Bridge Verbindung
                , statusIndicator 
                    (if model.rosConnected then Online else Error) 
                    "ROS"

                , span [ class "status-divider" ] []

                -- System Modus (Hardware vs Simulation)
                , statusIndicator 
                    (case model.mode of
                        Hardware -> Online
                        Simulation -> Standby
                    ) 
                    (case model.mode of
                        Simulation -> "Simulation"
                        Hardware -> "Hardware"
                    )
                ]
            ]

        -- 3. Rechts: Ansichts-Steuerung (Nutzt AgentsMsg Konstruktoren)
        , div [ class "navbar-actions" ]
            [ button [ class "btn-mode-switch", onClick ToggleMode ] 
                [ text (if model.mode == Simulation then "Hardware Modus" else "Simulation") ]
            , button [ class "btn-view-toggle", onClick ToggleViewMode ] 
                [ text (if model.is3D then "2D Ansicht" else "3D Ansicht") ]
            ]
        ]


-- --- HELPER (Intern für diesen Organismus) ---

{-| Mappt den typsicheren HardwareStatus auf die CSS-Klassen. -}
statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning"
        Error -> "offline"
        Missing -> "offline"
        UnknownStatus -> "offline"

{-| Erstellt einen Status-Indikator (Molekül-Struktur). 
Gibt jetzt Html AgentsMsg zurück, um mit der Navbar kompatibel zu sein.
-}
statusIndicator : HardwareStatus -> String -> Html AgentsMsg
statusIndicator status label =
    span [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        , span [ class "indicator-label" ] [ text label ]
        ]