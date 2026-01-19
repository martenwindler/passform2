module View.Organisms.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)

view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- 1. Links: Logo und Titel (Pfad wieder auf src/... korrigiert)
          div [ class "navbar-section" ]
            [ img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title hidden sm:inline" ] [ text "Passform 2.0" ]
            ]
        
        , -- 2. Mitte: Status-Pille
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
                [ class "btn-mode-switch"
                , onClick (AgentsMsg ToggleMode) 
                ] 
                [ span [ class "hidden lg:inline" ] 
                    [ text (if model.mode == Simulation then "Hardware-Modus" else "Simulations-Modus") ]
                ]
            , button 
                [ class "btn-view-toggle"
                , onClick (AgentsMsg ToggleViewMode) 
                ] 
                [ text (if model.is3D then "2D" else "3D") ]
            ]
        ]

-- --- HELPER ---

viewIndicator : String -> HardwareStatus -> Html Msg
viewIndicator label status =
    -- Geändert von span zu div für stabileres Flex-Layout
    div [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        , span [ class "indicator-label" ] [ text label ]
        ]

statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning"
        Error -> "offline"
        _ -> "offline"