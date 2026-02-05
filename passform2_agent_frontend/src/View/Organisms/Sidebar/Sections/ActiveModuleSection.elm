module View.Organisms.Sidebar.Sections.ActiveModuleSection exposing (view, formatModuleType)

import Dict
import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button

{-| 
Zusammengeführte Sektion für Buchten (Bays) und aktive Agenten.
-}
view : Model -> Html Msg
view model =
    div [ class "active-module-section flex flex-col h-full space-y-6" ]
        [ -- --- SEKTION 1: BUCHTEN (BAYS) ---
          div [ class "flex flex-col min-h-0" ]
            [ h3 [ class "text-info text-[0.8rem] uppercase tracking-widest mb-3 border-b border-white/10 pb-2 flex justify-between" ] 
                [ text "Anlagen-Layout (Bays)"
                , span [ class "opacity-50" ] [ text (String.fromInt (List.length model.bays)) ]
                ]
            , if List.isEmpty model.bays then
                p [ class "text-white/20 italic text-center py-4 text-xs" ] [ text "Keine Layout-Daten geladen." ]
              else
                div [ class "bay-list space-y-2 overflow-y-auto max-h-[250px] scrollbar-hide" ]
                    (List.map viewBayItem model.bays)
            ]

        -- --- SEKTION 2: AGENTEN ---
        , div [ class "flex flex-col flex-1 min-h-0" ]
            [ h3 [ class "text-warning text-[0.8rem] uppercase tracking-widest mb-3 border-b border-white/10 pb-2 flex justify-between" ] 
                [ text "Aktive Agenten"
                , span [ class "opacity-50" ] [ text (String.fromInt (Dict.size model.agents)) ]
                ]
            , if Dict.isEmpty model.agents then
                p [ class "text-white/20 italic text-center mt-10 text-sm" ] [ text "Keine Agenten im System." ]
              else
                div [ class "agent-list flex-1 overflow-y-auto pr-2 scrollbar-hide" ]
                    (model.agents 
                        |> Dict.values 
                        |> List.sortBy (\a -> formatModuleType a.module_type) 
                        |> List.map viewAgentItem
                    )
            ]
        ]

{-| Einzelne Bucht (Bay) mit Belegungsanzeige -}
viewBayItem : Bay -> Html Msg
viewBayItem bay =
    div [ class ("p-2 rounded border transition-all duration-300 " ++ 
            (if bay.occupation then "bg-cyan-500/20 border-cyan-500 shadow-[0_0_10px_rgba(0,242,255,0.2)]" else "bg-white/5 border-white/5")) ]
        [ div [ class "flex justify-between items-center" ]
            [ span [ class "text-[0.75rem] font-bold text-white uppercase" ] [ text bay.name ]
            , if bay.occupation then
                span [ class "text-[9px] bg-cyan-500 text-black px-1 font-black rounded animate-pulse" ] [ text "OCCUPIED" ]
              else
                span [ class "text-[9px] text-white/20 font-bold" ] [ text "VACANT" ]
            ]
        -- Zeige die ID des Agenten an, wenn belegt
        , if bay.occupation then
            div [ class "text-[0.65rem] text-cyan-400 mt-1 font-mono italic" ] [ text ("ID: " ++ bay.module_uuid) ]
          else
            div [ class "text-[0.65rem] text-white/20 mt-1 font-mono" ] 
                [ text (String.fromFloat bay.origin.x ++ " / " ++ String.fromFloat bay.origin.y) ]
        ]
        
{-| Einzelner Agent -}
viewAgentItem : AgentModule -> Html Msg
viewAgentItem agent =
    div [ class "agent-item group flex justify-between items-center p-3 mb-2 bg-white/5 border border-white/5 rounded-sm transition-all hover:bg-white/10" ]
        [ div [ class "flex flex-col gap-1" ]
            [ div [ class "flex items-center gap-2" ]
                [ span [ class "font-black text-white text-sm uppercase tracking-tight" ] 
                    [ text (formatModuleType agent.module_type) ]
                , if agent.is_dynamic then 
                    span [ class "text-[9px] bg-warning text-black px-1 font-black rounded-xs" ] [ text "MOBIL" ] 
                  else text ""
                ]
            , span [ class "agent-coords font-mono text-white/40 text-[0.7rem] font-bold" ] 
                [ text ("LOC: " ++ String.fromInt agent.position.x ++ " / " ++ String.fromInt agent.position.y) ]
            ]
        , div [ class "opacity-0 group-hover:opacity-100 transition-opacity duration-200" ]
            [ Button.iconDelete (AgentsMsg (RemoveAgent agent.position)) ]
        ]

{-| Formatiert den Modultyp -}
formatModuleType : ModuleType -> String
formatModuleType mType =
    case mType of
        FTF -> "Ranger"
        Conveyeur -> "Förderer"
        RollenModul -> "Rollenmodul"
        Mensch -> "Operator"
        Greifer -> "Greifer"
        Station -> "Station"
        UnknownModule name -> name