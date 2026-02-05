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
    div [ class "sidebar-content h-full flex flex-col" ] 
        [ div [ class "sidebar-tab-header" ] 
            [ h2 [ class "text-h3" ] [ text (getTabTitle model.activeSidebarTab) ] ]
        , div 
            [ class "tab-body flex-1 overflow-y-auto custom-scrollbar" -- flex-col und min-h-0 entfernt!
            , id ("tab-container-" ++ tabToIdString model.activeSidebarTab)
            ] 
            [ viewTabContent model ]
        ]
-- Hilfsfunktion am Ende der Datei
tabToIdString : SidebarTab -> String
tabToIdString tab =
    case tab of
        TabPlanning -> "planning"
        TabGrid -> "grid"
        TabAgents -> "agents"
        TabHardware -> "hardware"
        TabRanger -> "ranger"
        TabLogs -> "logs"

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
        
        , -- Beim Toggle-Button unten ebenfalls:
            button 
                [ class "rail-btn toggle-btn"
                , onClick ToggleSidebar -- Jetzt direkt!
                ] 
                [ text (if model.sidebarOpen then "»" else "«") ]
        ]

-- In View.Organisms.Sidebar.elm

railButton : Html Msg -> SidebarTab -> Model -> Html Msg
railButton icon tab model =
    let
        -- Liste der aktuell deaktivierten Module
        isDisabled =
            List.member tab [ TabPlanning, TabGrid, TabRanger, TabHardware ]
            
        -- Wir definieren die Klick-Message nur, wenn der Button NICHT disabled ist
        clickAttribute =
            if isDisabled then
                [] -- Keine Message, kein Klick
            else
                [ onClick (SwitchSidebarTab tab) ]
                
        titleText =
            if isDisabled then
                getTabTitle tab ++ " (FUTURE)"
            else
                getTabTitle tab
    in
    button 
        ([ class "rail-btn"
         , classList 
            [ ("active", model.activeSidebarTab == tab)
            , ("disabled", isDisabled) -- Neue CSS-Klasse
            ]
         , title titleText
         ] ++ clickAttribute -- Nur klickbar wenn erlaubt
        ) 
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