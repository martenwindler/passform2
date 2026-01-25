module View.Layouts.MainLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (on) -- Hinzugefügt für das Custom Event
import Json.Decode as Decode -- Hinzugefügt für die Daten-Extraktion
import View.Organisms.Navbar as Navbar
import View.Organisms.Sidebar as Sidebar
import View.Organisms.Modal as Modal
import Types exposing (..)

view : Model -> Html Msg -> Html Msg
view model gridContent =
    div [ class "app-layout flex flex-col h-screen w-screen overflow-hidden bg-bg-main" ]
        [ -- 1. NAVBAR (Immer oben, fixe Höhe)
          Navbar.view model
        
        , -- 2. CONTENT AREA (Zentrales Grid/Flex System)
          div [ class "content-area" ] 
            [ -- GRID VIEWPORT
              -- Hier fangen wir das "cell-clicked" Event aus der ThreeGridScene.ts ab
              div 
                [ class "grid-viewport" 
                , on "cell-clicked" decodeCellClick
                ] 
                [ gridContent ] 
            
              -- SIDEBAR (Rail links, Drawer rechts innerhalb des Containers)
            , Sidebar.view model 
            ]

        , -- 3. MODAL (Overlay wird nur gerendert, wenn ein Menü aktiv ist)
          Modal.view model
        ]

-- --- HELPER ---

{-| 
Extrahiert die Koordinaten aus dem CustomEvent-Detail der WebComponent.
JS-Struktur: { detail: { x: number, y: number } }
-}
decodeCellClick : Decode.Decoder Msg
decodeCellClick =
    Decode.at [ "detail" ] 
        (Decode.map2 (\x y -> AgentsMsg (OpenMenu x y))
            (Decode.field "x" Decode.int)
            (Decode.field "y" Decode.int)
        )