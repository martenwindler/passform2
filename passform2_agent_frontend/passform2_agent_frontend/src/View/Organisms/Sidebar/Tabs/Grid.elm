module View.Organisms.Sidebar.Tabs.Grid exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..) -- Das hier hat gefehlt!
import Types exposing (..)
import View.Organisms.Sidebar.Sections.GridSection as GridSection

{-| 
Spezialisierter Tab für die Gitter-Infrastruktur.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content h-full flex flex-col gap-6" ]
        [ h3 [] [ text "Gitter & Umgebung" ]
        
        , -- Die Gitter-Sektion (Breite/Länge)
          GridSection.view model
          
        , -- Kleiner Info-Text zur Gitter-Logik
          div [ class "mt-4 p-4 bg-white/5 border border-white/5 rounded text-[0.65rem] text-white/40 leading-relaxed" ]
            [ text "Hinweis: Änderungen an den Gitter-Dimensionen setzen das aktuelle Pathfinding zurück und erfordern eine Neuausrichtung der Agenten." ]
        ]