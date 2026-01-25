module View.Organisms.Sidebar.Sections.ActiveModuleSection exposing (view, formatModuleType)

import Dict
import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button

{-| 
Diese Sektion rendert die Liste aller Module im Gitter. 
Sie ist optimiert für die Anzeige im Agents-Tab der Sidebar.
-}
view : Model -> Html Msg
view model =
    div [ class "active-module-section flex flex-col h-full" ]
        [ h3 [ class "text-info text-[0.9rem] uppercase tracking-wider mb-4 border-b border-white/10 pb-2" ] 
            [ text ("Aktive Module (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
        
        , if Dict.isEmpty model.agents then
            p [ class "text-white/20 italic text-center mt-10 text-sm" ] 
                [ text "Keine Module im Gitter." ]
          else
            div [ class "agent-list flex-1 overflow-y-auto pr-2 scrollbar-hide" ]
                (model.agents 
                    |> Dict.values 
                    |> List.sortBy (\a -> formatModuleType a.module_type) 
                    |> List.map viewAgentItem
                )
        ]

{-| Einzelnes Listen-Element für einen Agenten -}
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
            
            , case agent.payload of
                Just pId -> 
                    div [ class "mt-1 text-[0.7rem] text-info font-black italic" ] 
                        [ text ("📦 " ++ pId) ]
                Nothing -> 
                    text ""
            ]
        
        , -- Löschen-Button (Erscheint nur beim Hovern)
          div [ class "opacity-0 group-hover:opacity-100 transition-opacity duration-200" ]
            [ Button.iconDelete (AgentsMsg (RemoveAgent agent.position)) ]
        ]

{-| Formatiert den Modultyp für die Anzeige -}
formatModuleType : ModuleType -> String
formatModuleType mType =
    case mType of
        FTF -> "FTF"
        Conveyeur -> "Förderer"
        RollenModul -> "Rollenmodul"
        Mensch -> "Mensch"
        Greifer -> "Greifer"
        Station -> "Station"
        UnknownModule name -> "Unbekannt: " ++ name