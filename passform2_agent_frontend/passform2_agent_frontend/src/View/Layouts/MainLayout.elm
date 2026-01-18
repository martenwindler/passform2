module View.Layouts.MainLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)

-- Import der Organismen
import View.Organisms.Navbar as Navbar
import View.Organisms.Sidebar as Sidebar
import View.Organisms.Modal as Modal

-- Import der Moleküle
import View.Molecules.HardwareStatus as HardwareStatus


{-| Layout: Das Grundgerüst der Anwendung.
Hier führen wir die modularisierten Views zusammen.
-}
view : Model -> Html Msg -> Html Msg
view model centralContent =
    div [ class "app-layout" ]
        [ -- 1. Navbar: Liefert Html AgentsMsg -> Muss gemappt werden
          Html.map AgentsMsg (Navbar.view model)
        
        , -- 2. Hauptbereich
          div [ class "content-area" ]
            [ -- Content: Kommt von Main.elm bereits als Html Msg
              centralContent 
            
            , -- Sidebar: Liefert JETZT Html Msg (Mapping passiert intern in Sidebar.elm)
              Sidebar.view model 
            
            , -- Alert: Liefert Html HardwareMsg -> Muss gemappt werden
              Html.map HardwareMsg (HardwareStatus.viewAlertOverlay model.alert) 
            ]
        
        , -- 3. Modals: Liefert JETZT Html Msg (Mapping passiert intern in Modal.elm)
          Modal.viewActiveMenu model model.activeMenu 
        ]