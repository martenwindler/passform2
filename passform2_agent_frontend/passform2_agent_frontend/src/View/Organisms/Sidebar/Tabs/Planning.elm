module View.Organisms.Sidebar.Tabs.Planning exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Organisms.Sidebar.Sections.GridSection as GridSection
import View.Organisms.Sidebar.Sections.WeightsSection as WeightsSection
import View.Organisms.Sidebar.Sections.StationAgentMissionSection as MissionSection
import View.Organisms.Sidebar.Sections.FileConfigSection as FileConfigSection

{-| 
Das Planning-Tab ist jetzt ein reiner Orchestrator. 
Jede Sektion ist in ein eigenes Modul im Ordner 'Sections' ausgelagert.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content scrollbar-hide flex flex-col gap-6" ]
        [ h3 [] [ text "Missions-Planung" ]
        
        , -- A: Gitter-Dimensionen (B/L)
          GridSection.view model

        , -- B: Contract-Net Gewichte (Kostenfunktion)
          WeightsSection.view model

        , -- C: Navigation, Status & Start-Button (Mit SVG-Icons)
          MissionSection.view model

        , -- D: Datei-Management (Save/Load/Export/Clear)
          FileConfigSection.view model
        ]