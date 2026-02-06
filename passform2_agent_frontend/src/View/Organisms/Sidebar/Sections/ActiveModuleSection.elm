module View.Organisms.Sidebar.Sections.ActiveModuleSection exposing (view, formatModuleType)

import Dict
import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Keyed as Keyed
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button

{-| 
Zusammengeführte Sektion für Buchten (Bays) und aktive Agenten.
Nutzt Keyed-Nodes für Stabilität gegen Virtual-DOM-Fehler.
-}
view : Model -> Html Msg
view model =
    div [ class "active-module-section flex flex-col h-full space-y-6" ]
        [ -- --- SEKTION 1: BUCHTEN (BAYS) ---
          div [ class "sidebar-section flex flex-col min-h-0" ]
            [ h3 [ class "text-info text-[0.8rem] uppercase tracking-widest mb-3 border-b border-white/10 pb-2 flex justify-between" ] 
                [ text "Anlagen-Layout (Bays)"
                , span [ class "opacity-50" ] [ text (" (" ++ String.fromInt (List.length model.bays) ++ ")") ]
                ]
            , div [ class "bay-list" ] 
                (if List.isEmpty model.bays then
                    [ p [ class "text-white/20 italic text-center py-4 text-xs" ] [ text "Keine Layout-Daten geladen." ] ]
                 else
                    List.map viewBayItem model.bays
                )
            ]

        -- --- SEKTION 2: AGENTEN ---
        , div [ class "sidebar-section flex flex-col min-h-0" ]
            [ h3 [ class "text-warning text-[0.8rem] uppercase tracking-widest mb-3 border-b border-white/10 pb-2 flex justify-between" ] 
                [ text "Aktive Agenten"
                , span [ class "opacity-50" ] [ text (" (" ++ String.fromInt (Dict.size model.agents) ++ ")") ]
                ]
            , if Dict.isEmpty model.agents then
                p [ class "text-white/20 italic text-center mt-10 text-sm" ] [ text "Keine Agenten im System." ]
              else
                Keyed.node "div" [ class "agent-list" ]
                    (model.agents 
                        |> Dict.toList 
                        |> List.sortBy (\( _, a ) -> formatModuleType a.module_type) 
                        |> List.map viewKeyedAgentItem
                    )
            ]
        ]

{-| Hilfsfunktion für Keyed Rendering: Erzeugt (Key, Html) Tupel -}
viewKeyedAgentItem : ( ( Int, Int, Int ), AgentModule ) -> ( String, Html Msg )
viewKeyedAgentItem ( ( x, y, l ), agent ) =
    let
        agentId = Maybe.withDefault "Unknown" agent.agent_id
        -- Eindeutiger Key für Elm: kombiniert ID, Position und Ebene
        idKey = agentId ++ "-" ++ String.fromInt x ++ "-" ++ String.fromInt y ++ "-" ++ String.fromInt l
    in
    ( idKey
    , div [ class "agent-item group flex justify-between items-center p-3 mb-2 bg-white/5 border border-white/5 rounded-sm transition-all hover:bg-white/10" ]
        [ div [ class "flex flex-col gap-1" ]
            [ div [ class "flex items-center gap-2" ]
                [ span [ class "font-black text-white text-sm uppercase tracking-tight" ] 
                    [ text (formatModuleType agent.module_type) ]
                , if l > 0 then
                    span [ class "text-[9px] bg-cyan-500 text-black px-1 font-black rounded-xs" ] [ text ("LVL " ++ String.fromInt l) ]
                  else
                    text ""
                , if agent.is_dynamic then 
                    span [ class "text-[9px] bg-warning text-black px-1 font-black rounded-xs" ] [ text "MOBIL" ] 
                  else text ""
                ]
            , span [ class "agent-coords font-mono text-white/40 text-[0.7rem] font-bold" ] 
                [ text (agentId ++ " @ " ++ String.fromInt x ++ "/" ++ String.fromInt y) ]
            ]
        , div [ class "opacity-0 group-hover:opacity-100 transition-opacity duration-200" ]
            [ Button.iconDelete (AgentsMsg (RemoveAgent agent.position)) ]
        ]
    )

{-| Einzelne Bucht (Bay) im sauberen Look -}
viewBayItem : Bay -> Html Msg
viewBayItem bay =
    let
        displayName =
            if String.startsWith "BayBay_" bay.name then
                String.dropLeft 6 bay.name -- Korrigiert auf 6 für "BayBay_"
            else
                bay.name
    in
    div 
        [ class "agent-item group flex justify-between items-center p-3 mb-2 bg-white/5 border border-white/5 rounded-sm transition-all hover:bg-white/10" 
        , classList [ ( "border-cyan-500/50 bg-cyan-500/5", bay.occupation ) ]
        ]
        [ div [ class "flex flex-col gap-1" ]
            [ div [ class "flex items-center gap-2" ]
                [ -- Der statische Name aus Rust (z.B. "Bucht 1")
                span [ class "font-black text-white text-sm uppercase tracking-tight" ] 
                    [ text bay.name ] 
                , if bay.occupation then
                    span [ class "text-[9px] bg-cyan-500 text-black px-1.5 py-0.5 font-black rounded-xs animate-pulse" ] 
                        [ text "\u{00A0}BELEGT" ] 
                else
                    span [ class "text-[9px] text-white/20 font-bold border border-white/10 px-1.5 py-0.5 rounded-xs" ] 
                        [ text "\u{00A0}FREI" ]
                ]
            , if bay.occupation then
                -- Die dynamische ID des Agenten (enthält Typ & Position)
                span [ class "agent-coords font-mono text-cyan-400/60 text-[0.7rem] font-bold italic" ] 
                    [ text ("ID: " ++ bay.module_uuid) ] 
            else
                -- Physische Position der Bucht im Raum
                span [ class "agent-coords font-mono text-white/20 text-[0.7rem] font-bold" ] 
                    [ text ("POS: " ++ String.fromFloat bay.origin.x ++ " / " ++ String.fromFloat bay.origin.y) ]
            ]
        , -- Platzhalter für Interaktions-Buttons (z.B. ein Mülleimer-Icon zum Entfernen)
        div [ class "w-6 h-6 flex items-center justify-center" ] [] 
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