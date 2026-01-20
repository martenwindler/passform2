module View.Organisms.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)

view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- 1. Links: Logo und Titel
          div [ class "navbar-section" ]
            [ img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            -- NEU: text-h3 statt manueller Font-Größe
            , span [ class "navbar-title text-h3 hidden sm:inline ml-3" ] [ text "Passform 2.0" ]
            ]
        
        , -- 2. Mitte: Status-Pille (zentriert)
          div [ class "navbar-center hidden md:flex" ]
            [ div [ class "status-pill combined-status" ]
                [ viewIndicator "REST" (if model.connected then Online else Error)
                , div [ class "status-divider" ] [] 
                , viewIndicator "ROS" (if model.rosConnected then Online else Error)
                , div [ class "status-divider" ] []
                , viewIndicator 
                    (case model.mode of
                        Simulation -> "SIM"
                        Hardware -> "HW"
                    )
                    (case model.mode of
                        Hardware -> Online
                        Simulation -> Standby
                    )
                ]
            ]

        , -- 3. Rechts: Actions
          div [ class "navbar-actions" ]
            [ button 
                [ class "btn-mode-switch text-button"
                , onClick (AgentsMsg ToggleMode) 
                ] 
                [ span [ class "hidden lg:inline" ] 
                    [ text (if model.mode == Simulation then "Hardware-Modus" else "Simulations-Modus") ]
                ]
            , button 
                [ class "btn-view-toggle text-button"
                , onClick (AgentsMsg ToggleViewMode) 
                ] 
                [ text (if model.is3D then "2D" else "3D") ]
            
            , a 
                [ class "btn-view-toggle link-pill text-button"
                , href "https://passform.biba.uni-bremen.de/"
                , target "_blank"
                , rel "noopener noreferrer"
                ] 
                [ text "Projekt" ]
            ]
        ]

-- --- HELPER ---

viewIndicator : String -> HardwareStatus -> Html Msg
viewIndicator label status =
    div [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        -- NEU: text-label für die Status-Beschriftung
        , span [ class "indicator-label text-label" ] [ text label ]
        ]

statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning"
        Error -> "offline"
        _ -> "offline"