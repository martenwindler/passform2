module View.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- Links: Logo und Toggle-Button
          div [ class "navbar-section" ]
            [ button [ class "btn-icon-sidebar", onClick ToggleSidebar ] 
                [ text "☰" ]
            , img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title" ] [ text "Passform 2.0" ]
            ]
        
        -- Mitte: Status-Pille (Drei Stati kombiniert)
        , div [ class "navbar-section navbar-center" ]
            [ div [ class "status-pill combined-status" ]
                [ -- 1. REST Status (Backend 8000)
                  statusIndicator 
                    (if model.connected then "#48bb78" else "#f56565") 
                    "REST"
                
                , span [ class "status-divider" ] [ text "|" ]

                -- 2. ROS Status (Bridge 5000)
                -- Hinweis: model.rosConnected muss in Types/Update definiert sein
                , statusIndicator 
                    (if model.connected then "#48bb78" else "#f56565") -- Temporär model.connected, falls rosConnected noch fehlt
                    "ROS"

                , span [ class "status-divider" ] [ text "|" ]

                -- 3. System Modus (Grün für Hardware, Gelb für Simulation)
                , statusIndicator 
                    (if model.mode == Hardware then "#48bb78" else "#ecc94b") 
                    (if model.mode == Simulation then "Simulation" else "Hardware")
                ]
            ]

        -- Rechts: Ansichts-Steuerung
        , div [ class "navbar-section" ]
            [ button [ class "btn-mode-switch", onClick ToggleMode ] 
                [ text (if model.mode == Simulation then "Hardware Modus" else "Simulation") ]
            , button [ class "btn-view-toggle", onClick ToggleViewMode ] 
                [ text (if model.is3D then "2D Ansicht" else "3D Ansicht") ]
            ]
        ]

-- Hilfsfunktion für die Indikatoren innerhalb der Pille
statusIndicator : String -> String -> Html Msg
statusIndicator color label =
    span [ class "indicator-group" ]
        [ div 
            [ class "status-dot-small"
            , style "background" color
            , style "box-shadow" ("0 0 5px " ++ color) -- Erzeugt einen leichten "Glüh-Effekt"
            ] []
        , span [ class "indicator-label" ] [ text label ]
        ]