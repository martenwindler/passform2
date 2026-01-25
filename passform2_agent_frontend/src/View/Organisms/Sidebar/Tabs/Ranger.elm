module View.Organisms.Sidebar.Tabs.Ranger exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Organisms.Sidebar.Sections.RangerStatusSection as RangerStatus

{-| 
Dieser Tab ist die dedizierte Kommandozentrale für den Agile Robotics Ranger.
Hier laufen alle Live-Daten der physischen Einheit zusammen.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content h-full flex flex-col gap-6" ]
        [ h3 [] [ text "Ranger-Überwachung" ]
        
        , -- 1. Die Telemetrie-Sektion (Signalstärke, CAN-Bus, Status-Pillen)
          RangerStatus.view model
          
        , -- 2. Ein dynamischer Footer für den Verbindungsstatus
          div [ class "mt-auto" ]
            [ div [ class "p-4 bg-info/5 border border-info/10 rounded-industrial-sm" ]
                [ div [ class "flex items-center gap-2 mb-2" ]
                    [ div [ class "w-1.5 h-1.5 rounded-full bg-info animate-pulse" ] []
                    , span [ class "text-[0.7rem] font-bold uppercase tracking-widest text-info" ] 
                        [ text "Live-Datenstrom" ]
                    ]
                , p [ class "text-[0.65rem] text-white/40 leading-relaxed italic" ]
                    [ text "Synchronisiere Telemetrie über Ranger-Gateway... Alle Systeme innerhalb der Betriebsparameter." ]
                ]
            ]
        ]