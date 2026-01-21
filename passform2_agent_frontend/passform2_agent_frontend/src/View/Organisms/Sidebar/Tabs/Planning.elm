module View.Organisms.Sidebar.Tabs.Planning exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Organisms.Sidebar.Sections.WeightsSection as WeightsSection
import View.Organisms.Sidebar.Sections.StationAgentMissionSection as MissionSection
import View.Organisms.Sidebar.Sections.PathResultSection as PathResultSection

view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content tab-planning flex flex-col gap-6" ] -- Neu: tab-planning
        [ h3 [] [ text "Contract Net Protocol" ]
        , MissionSection.view model
        , PathResultSection.view model
        , WeightsSection.view model
        ]