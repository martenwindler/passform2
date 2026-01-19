module View.Organisms.Sidebar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Icons as Icons
import View.Organisms.Sidebar.Tabs.Planning as PlanningTab
import View.Organisms.Sidebar.Tabs.Agents as AgentsTab
import View.Organisms.Sidebar.Tabs.Hardware as HardwareTab
import View.Organisms.Sidebar.Tabs.Logs as LogsTab

view : Model -> Html Msg
view model =
    aside 
        [ class "sidebar-container"
        , classList [ ( "sidebar-open", model.sidebarOpen ) ]
        ]
        [ viewRail model   -- 1. RAIL (Links im Sidebar-Verbund)
        , viewDrawer model -- 2. DRAWER (Rechts, bündig zum Bildschirmrand)
        ]

viewDrawer : Model -> Html Msg
viewDrawer model =
    div [ class "sidebar-content" ]
        [ div [ class "sidebar-tab-header" ] 
            [ h2 [] [ text (getTabTitle model.activeSidebarTab) ] ]
        , div [ class "tab-body" ] 
            [ viewTabContent model ]
        ]

viewRail : Model -> Html Msg
viewRail model =
    nav [ class "sidebar-rail" ]
        [ railButton Icons.iconPlanning TabPlanning model
        , railButton Icons.iconAgents TabAgents model
        , railButton Icons.iconHardware TabHardware model
        , railButton Icons.iconLogs TabLogs model
        , span [ class "flex-grow" ] []
        , button 
            [ class "rail-btn toggle-btn"
            , onClick (AgentsMsg ToggleSidebar) 
            ] 
            [ text (if model.sidebarOpen then "»" else "«") ]
        ]

railButton : Html Msg -> SidebarTab -> Model -> Html Msg
railButton icon tab model =
    button 
        [ class "rail-btn"
        , classList [ ("active", model.activeSidebarTab == tab) ]
        , onClick (AgentsMsg (SwitchSidebarTab tab))
        ] 
        [ icon ]

viewTabContent : Model -> Html Msg
viewTabContent model =
    case model.activeSidebarTab of
        TabPlanning -> PlanningTab.view model
        TabAgents -> AgentsTab.view model
        TabHardware -> HardwareTab.view model
        TabLogs -> LogsTab.view model

getTabTitle : SidebarTab -> String
getTabTitle tab =
    case tab of
        TabPlanning -> "Missions-Planung"
        TabAgents -> "Aktive Module"
        TabHardware -> "Hardware-Status"
        TabLogs -> "System-Logs"