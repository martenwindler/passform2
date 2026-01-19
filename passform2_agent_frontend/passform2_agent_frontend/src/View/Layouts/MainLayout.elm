module View.Layouts.MainLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import View.Organisms.Navbar as Navbar
import View.Organisms.Sidebar as Sidebar
import View.Organisms.Modal as Modal
import Types exposing (..)

view : Model -> Html Msg -> Html Msg
view model gridContent =
    div [ class "app-layout flex flex-col h-screen w-screen overflow-hidden bg-bg-main" ]
        [ -- NAVBAR (Immer oben, fixe Höhe)
          Navbar.view model
        
        , -- CONTENT AREA (Das Grid-System)
          div [ class "content-area" ] 
            [ -- GRID VIEWPORT
              div [ class "grid-viewport" ] 
                [ gridContent ] 
            
              -- SIDEBAR (Rail links, Drawer rechts)
            , Sidebar.view model 
            ]
        ]