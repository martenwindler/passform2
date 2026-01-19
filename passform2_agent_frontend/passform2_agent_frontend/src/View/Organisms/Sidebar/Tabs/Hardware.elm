module View.Organisms.Sidebar.Tabs.Hardware exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..) 
import View.Molecules.HardwareStatus as StatusDisplay

{-| 
Hardware-Tab: Zeigt System-Verbindungen und Edge-Devices.
Nutzt das neue Grid-System für die Kacheln oben.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content" ]
        [ h3 [] [ text "System-Hardware" ]
        
        , -- Globaler Status (Backend & ROS) im 2-spaltigen Grid
          div [ class "status-grid" ]
            [ div [ class "status-card" ]
                [ label [] [ text "Backend API" ]
                , div [ class "mt-1" ] [ StatusDisplay.view (if model.connected then Online else Error) ]
                ]
            , div [ class "status-card" ]
                [ label [] [ text "ROS 2 Bridge" ]
                , div [ class "mt-1" ] [ StatusDisplay.view (if model.rosConnected then Online else Error) ]
                ]
            ]

        , -- Liste der Pis
          div [ class "planning-section mt-6" ]
            [ h4 [] [ text "Verbundene Einheiten" ]
            , if List.isEmpty model.connectedHardware then
                p [ class "text-muted italic text-[0.8rem] px-2" ] 
                    [ text "Suche nach aktiven Edge-Devices..." ]
              else
                div [ class "device-list flex flex-col gap-2" ] 
                    (List.map viewDevice model.connectedHardware)
            ]
        ]

{-| 
Darstellung eines einzelnen Pis.
Wir orientieren uns am agent-item Design für Konsistenz.
-}
viewDevice : HardwareDevice -> Html Msg
viewDevice device =
    div [ class "device-item flex justify-between items-center bg-white/5 border border-white/10 p-3 rounded-industrial-sm" ]
        [ div [ class "flex flex-col gap-1" ]
            [ span [ class "font-bold text-sm tracking-wide text-white" ] [ text device.pi_id ]
            , span [ class "text-[0.7rem] text-white/40 font-mono" ] [ text "ID: EDGE-PI-NODE" ]
            ]
        , -- Status-LED des Pis
          StatusDisplay.view device.rfid_status
        ]