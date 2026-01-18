module View.Organisms.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

{-| Organismus: Die Haupt-Navigationsleiste.
Besteht aus Logo (Atom), Status-Pille (Molekül-Gruppe) und Steuerung (Atoms).
-}
view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- 1. Links: Logo und Titel
          div [ class "navbar-section" ]
            [ img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title" ] [ text "Passform 2.0" ]
            ]
        
        -- 2. Mitte: Status-Pille (Zentralisiertes Monitoring)
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

        -- 3. Rechts: Ansichts-Steuerung
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

{-| Erstellt einen Status-Indikator (Molekül-Struktur). -}
statusIndicator : HardwareStatus -> String -> Html Msg
statusIndicator status label =
    span [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        , span [ class "indicator-label" ] [ text label ]
        ]