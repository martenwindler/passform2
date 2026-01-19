module View.Organisms.Sidebar.Tabs.Agents exposing (view, formatModuleType)

import Html exposing (..)
import Html.Attributes exposing (..)
import Dict
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Button as Button

view : Model -> Html Msg
view model =
    -- Der Wrapper garantiert Weiß als Textfarbe und das richtige Padding
    div [ class "sidebar-tab-content h-full flex flex-col" ]
        [ h3 [] [ text "Aktive Module" ]
        
        , -- Die Liste der Agenten (scrollbar innerhalb des Bereichs)
          div [ class "agent-list flex-1 overflow-y-auto pr-2" ]
            (model.agents 
                |> Dict.values 
                |> List.map viewAgentItem
            )
        
        , -- Action-Bereich am Ende des Tabs
          div [ class "mt-auto pt-4 border-t border-white/5" ]
            [ Button.danger "Gitter leeren" (AgentsMsg ClearGrid) True True ]
        ]

viewAgentItem : AgentModule -> Html Msg
viewAgentItem agent =
    -- .agent-item nutzt jetzt justify-between für Label (links) und Icon (rechts)
    div [ class "agent-item group flex justify-between items-center" ]
        [ div [ class "flex flex-col gap-0.5" ]
            [ span [ class "agent-label" ] [ text (formatModuleType agent.module_type) ]
            , span [ class "agent-coords font-mono opacity-60 text-[0.75rem]" ] 
                [ text ("LOC: " ++ String.fromInt agent.position.x ++ " / " ++ String.fromInt agent.position.y) ]
            ]
        , -- Der Löschen-Button wird im SCSS als .btn-icon-delete angesprochen
          div [ class "opacity-0 group-hover:opacity-100 transition-opacity" ]
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