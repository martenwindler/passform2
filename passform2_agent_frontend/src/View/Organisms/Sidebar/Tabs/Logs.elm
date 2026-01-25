module View.Organisms.Sidebar.Tabs.Logs exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import View.Organisms.Sidebar.Sections.LogSection as LogSection

{-| 
Organismus-Teil: System-Logs.
Orchestriert die LogSection innerhalb des Sidebar-Layouts.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content tab-logs h-full flex flex-col" ] -- Neu: tab-logs
        [ h3 [ class "shrink-0 mb-4" ] [ text "System-Historie" ]
        , LogSection.view model
        ]