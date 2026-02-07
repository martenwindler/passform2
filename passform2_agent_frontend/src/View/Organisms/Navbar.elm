module View.Organisms.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)

{-|
    Die Navbar ist die zentrale Steuerung am oberen Bildschirmrand.
    Das Logo führt nun zurück zur Landing Page.
-}
view : Model -> Html Msg
view model =
    nav [ class "navbar blue-look" ]
        [ -- 1. Links: Logo und Titel (Klickbar für Rückkehr zur Landing Page)
          div 
            [ class "navbar-section cursor-pointer"
            , onClick (SystemMsg ResetToLanding) -- Hier wird der Reset ausgelöst
            ]
            [ img [ src "src/assets/images/logo.png", class "navbar-logo", alt "Logo" ] []
            , span [ class "navbar-title text-h3 hidden sm:inline ml-3 text-white" ] [ text "Passform 2.0" ]
            ]
        
        , -- 2. Mitte: Status-Anzeigen (Platzhalter für spätere Integration)
          div [ class "navbar-center hidden md:flex" ]
            [ -- Hier können später Status-Pillen wieder aktiviert werden
            ]

        , -- 3. Rechts: Aktionen (Kontextabhängig)
          viewActions model
        ]


{-| Zeigt Aktions-Buttons nur an, wenn wir uns im AppMode befinden. -}
viewActions : Model -> Html Msg
viewActions model =
    case model.activeLayout of
        LandingMode ->
            -- Auf der Startseite: Nur der BIBA Projekt-Link
            div [ class "navbar-actions" ]
                [ viewProjectLink ]

        AppMode ->
            -- In der App: Volle Action-Bar inklusive Modus-Switch
            div [ class "navbar-actions" ]
                [ button 
                    [ class "btn-mode-switch text-button"
                    , onClick (AgentsMsg ToggleMode) 
                    ] 
                    [ span [ class "hidden lg:inline text-white" ] 
                        [ text (if model.mode == Simulation then "Hardware-Modus" else "Simulations-Modus") ]
                    ]
                , button 
                    [ class "btn-view-toggle text-button text-white"
                    , onClick (AgentsMsg ToggleViewMode) 
                    ] 
                    [ text (if model.is3D then "2D" else "3D") ]
                
                , viewProjectLink
                ]


-- --- HELPER ---

{-| Kleine Status-Indikatoren für REST, ROS etc. -}
viewIndicator : String -> HardwareStatus -> Html Msg
viewIndicator label status =
    div [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        , span [ class "indicator-label text-label text-white" ] [ text label ]
        ]


{-| Mapping von Status-Typen auf CSS-Farben -}
statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "online"
        Standby -> "warning"
        Error -> "offline"
        _ -> "offline"


{-| Externer Link zur BIBA Projektseite -}
viewProjectLink : Html Msg
viewProjectLink =
    a 
        [ class "btn-view-toggle link-pill text-button text-white"
        , href "https://passform.biba.uni-bremen.de/"
        , target "_blank"
        , rel "noopener noreferrer"
        ] 
        [ text "Projekt" ]