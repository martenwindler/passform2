module View.Organisms.Sidebar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Icons as Icons
import View.Organisms.Sidebar.Tabs.Planning as PlanningTab
import View.Organisms.Sidebar.Tabs.Grid as GridTab -- Neu: Gitter-Tab importiert
import View.Organisms.Sidebar.Tabs.Agents as AgentsTab
import View.Organisms.Sidebar.Tabs.Hardware as HardwareTab
import View.Organisms.Sidebar.Tabs.Ranger as RangerTab
import View.Organisms.Sidebar.Tabs.Logs as LogsTab

{-|
Die Sidebar-Komponente: Orchestriert das Rail (Navigation) 
und den Drawer (Inhalt der Tabs).
-}
view : Model -> Html Msg
view model =
    aside 
        [ class "sidebar-container"
        , classList [ ( "sidebar-open", model.sidebarOpen ) ]
        ]
        [ viewRail model   -- 1. RAIL (Links: Schnellwahl-Icons)
        , viewDrawer model -- 2. DRAWER (Rechts: Detail-Ansichten)
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
        [ -- Navigations-Reihenfolge: Strategie -> Raum -> Module -> Roboter -> Hardware -> Log
          railButton Icons.iconPlanning TabPlanning model
        , railButton Icons.iconGrid TabGrid model         -- NEU: Gitter-Dimensionen
        , railButton Icons.iconAgents TabAgents model
        , railButton Icons.iconRanger TabRanger model     -- Telemetrie Rover
        , railButton Icons.iconHardware TabHardware model -- System-Hardware & NFC
        , railButton Icons.iconLogs TabLogs model
        
        , span [ class "flex-grow" ] [] -- Schiebt den Toggle-Button nach unten
        
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
        TabGrid -> GridTab.view model         -- Routing zum Gitter-Tab
        TabAgents -> AgentsTab.view model
        TabHardware -> HardwareTab.view model
        TabRanger -> RangerTab.view model
        TabLogs -> LogsTab.view model

getTabTitle : SidebarTab -> String
getTabTitle tab =
    case tab of
        TabPlanning -> "Missions-Planung"
        TabGrid -> "Gitter-Umgebung"          -- Titel für Infrastruktur
        TabAgents -> "Aktive Module"
        TabHardware -> "System-Diagnose"
        TabRanger -> "Ranger-Status"
        TabLogs -> "System-Logs"