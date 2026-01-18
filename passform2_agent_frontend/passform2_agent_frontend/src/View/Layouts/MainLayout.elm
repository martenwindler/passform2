module View.Layouts.MainLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)

-- Import der Organismen
import View.Organisms.Navbar as Navbar
import View.Organisms.Sidebar as Sidebar
import View.Organisms.Modal as Modal

-- Import der Moleküle
import View.Molecules.HardwareStatus as HardwareStatus


{-| Layout: Das Grundgerüst der Anwendung.
Es definiert, wie Navbar, Sidebar und Content angeordnet sind.
-}
view : Model -> Html Msg -> Html Msg
view model centralContent =
    div [ class "app-layout" ]
        [ -- 1. Die Kopfzeile (Organismus)
          Navbar.view model
        
        , -- 2. Der Hauptbereich (Flex-Container für Content + Sidebar)
          div [ class "content-area" ]
            [ -- Der eigentliche Inhalt (z.B. die 3D-Szene)
              centralContent 
            
            , -- Die Sidebar (Organismus)
              Sidebar.view model 
            
            , -- Einblendbare Alarme (Molekül)
              HardwareStatus.viewAlertOverlay model.alert 
            ]
        
        , -- 3. Modals / Overlays (Organismus)
          -- Wird absolut über das Layout gelegt, wenn aktiv
          Modal.viewActiveMenu model model.activeMenu 
        ]