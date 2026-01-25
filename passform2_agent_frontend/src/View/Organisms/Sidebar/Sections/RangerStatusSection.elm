module View.Organisms.Sidebar.Sections.RangerStatusSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Atoms.Icons as Icons

{-| 
Sektion für die Vitalwerte des Agile Robotics Rangers.
Hier werden Konnektivität und Systemstatus visualisiert.
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section ranger-telemetry" ]
        [ h4 [ class "mb-4 text-white/50 uppercase text-[0.7rem] tracking-widest" ] 
            [ text "Agile Robotics Ranger" ]
        
        , div [ class "flex flex-col gap-4 p-4 bg-black/20 border border-white/5 rounded-industrial-sm" ]
            [ -- 1. Status-Pillen (Weißer Kontrast-Look für Lesbarkeit)
              div [ class "flex justify-between gap-3" ]
                [ viewStatusPill "CAN-Bus" "Online" "bg-success"
                , viewStatusPill "System" "Active" "bg-info"
                ]
            
            , -- 2. Signalstärke-Indikator
              div [ class "signal-wrapper bg-white/5 p-3 rounded-sm border border-white/5" ]
                [ div [ class "flex justify-between items-center mb-2" ]
                    [ span [ class "text-[0.6rem] uppercase font-black text-white/30" ] [ text "Signalstärke" ]
                    , span [ class "text-xs font-mono text-success font-bold" ] [ text "94%" ]
                    ]
                , -- Der Fortschrittsbalken
                  div [ class "h-1.5 w-full bg-white/10 rounded-full overflow-hidden" ]
                    [ div 
                        [ class "h-full bg-success transition-all duration-500 shadow-[0_0_10px_rgba(72,187,120,0.5)]" 
                        , style "width" "94%" 
                        ] [] 
                    ]
                ]
            ]
        ]

-- --- HELPER ---

{-| 
Rendert eine auffällige Status-Pille im "Industrial White" Look.
Bricht das dunkle Design auf, um Aufmerksamkeit auf Hardware-Zustände zu lenken.
-}
viewStatusPill : String -> String -> String -> Html Msg
viewStatusPill labelTitle statusText colorClass =
    div [ class "flex-1 bg-white px-3 py-2 rounded-full flex flex-col items-center shadow-lg transform hover:scale-[1.02] transition-transform" ]
        [ span [ class "text-[0.55rem] font-black uppercase text-gray-400 leading-none mb-1" ] 
            [ text labelTitle ]
        , div [ class "flex items-center gap-1.5" ]
            [ div [ class ("w-2 h-2 rounded-full " ++ colorClass) ] []
            , span [ class "text-xs font-bold text-black" ] 
                [ text statusText ]
            ]
        ]