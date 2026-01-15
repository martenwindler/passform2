module View.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- Links: Logo und Toggle-Button (Icon fixiert auf Hamburger)
          div [ class "navbar-section" ]
            [ button [ class "btn-icon-sidebar", onClick ToggleSidebar ] 
                [ text "☰" ] -- Fixiert: Zeigt immer das Hamburger-Menü
            , img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title" ] [ text "Passform 2.0" ]
            ]
        
        -- Mitte: Status (Zentriert)
        , div [ class "navbar-section navbar-center" ]
            [ div [ class "status-pill" ]
                [ div 
                    [ class "status-dot"
                    , style "background" (if model.connected then "#48bb78" else "#f56565")
                    ] []
                , text 
                    (if model.connected then 
                        "Verbunden (" ++ (if model.mode == Simulation then "Simulation" else "Hardware") ++ ")"
                     else 
                        "Getrennt"
                    )
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