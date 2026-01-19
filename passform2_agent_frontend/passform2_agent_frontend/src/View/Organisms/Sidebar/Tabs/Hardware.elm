module View.Organisms.Sidebar.Tabs.Hardware exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..) 
import View.Molecules.HardwareStatus as StatusDisplay
-- NEU: Import der Management-Sektion
import View.Organisms.Sidebar.Sections.FileConfigSection as FileConfigSection

{-| 
Hardware-Tab: Zeigt System-Verbindungen, Edge-Devices und System-Management.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content scrollbar-hide flex flex-col gap-6" ]
        [ h3 [] [ text "System-Hardware" ]
        
        , -- Globaler Status (Backend & ROS)
          div [ class "status-grid grid grid-cols-2 gap-3" ]
            [ div [ class "status-card bg-white/5 border border-white/10 p-3 rounded-industrial-sm" ]
                [ label [ class "text-[0.6rem] uppercase text-white/40 font-bold mb-1 block" ] [ text "Backend API" ]
                , div [ class "flex items-center" ] [ StatusDisplay.view (if model.connected then Online else Error) ]
                ]
            , div [ class "status-card bg-white/5 border border-white/10 p-3 rounded-industrial-sm" ]
                [ label [ class "text-[0.6rem] uppercase text-white/40 font-bold mb-1 block" ] [ text "ROS 2 Bridge" ]
                , div [ class "flex items-center" ] [ StatusDisplay.view (if model.rosConnected then Online else Error) ]
                ]
            ]

        , -- Liste der Pis
          div [ class "planning-section" ]
            [ h4 [ class "mb-3" ] [ text "Verbundene Einheiten" ]
            , if List.isEmpty model.connectedHardware then
                div [ class "p-4 border border-dashed border-white/10 rounded-sm text-center" ]
                    [ p [ class "text-white/20 italic text-[0.75rem]" ] 
                        [ text "Suche nach aktiven Edge-Devices..." ] 
                    ]
              else
                div [ class "device-list flex flex-col gap-2" ] 
                    (List.map viewDevice model.connectedHardware)
            ]

        , -- NEU: Datei-Management & System-Reset (Save/Load/Export/Clear)
          -- Hier sitzt jetzt das System-Management
          FileConfigSection.view model
        ]

viewDevice : HardwareDevice -> Html Msg
viewDevice device =
    div [ class "device-item flex justify-between items-center bg-white/5 border border-white/10 p-3 rounded-industrial-sm" ]
        [ div [ class "flex flex-col gap-1" ]
            [ span [ class "font-bold text-sm tracking-wide text-white" ] [ text device.pi_id ]
            , span [ class "text-[0.7rem] text-white/40 font-mono" ] [ text "ID: EDGE-PI-NODE" ]
            ]
        , StatusDisplay.view device.rfid_status
        ]