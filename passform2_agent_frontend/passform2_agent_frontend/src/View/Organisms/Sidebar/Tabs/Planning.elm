module View.Organisms.Sidebar.Tabs.Planning exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Organisms.Sidebar.Sections.WeightsSection as WeightsSection
import View.Organisms.Sidebar.Sections.StationAgentMissionSection as MissionSection
import View.Organisms.Sidebar.Sections.PathResultSection as PathResultSection

view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content scrollbar-hide flex flex-col gap-6" ]
        [ -- REBRANDING: Haupttitel angepasst
          h3 [] [ text "Contract Net Protocol" ]

        , -- Navigation, Status & Start-Button
          MissionSection.view model
        , -- NEU: Das Ergebnis der Pfadsuche (erscheint nur bei Erfolg)
          PathResultSection.view model
        , -- CNP Gewichte (Kostenfunktion)
          WeightsSection.view model
        ]