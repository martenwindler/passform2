module View.Organisms.Sidebar.Tabs.Planning exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)

view : Model -> Html Msg
view model =
    -- Der Hauptcontainer: Erbt 'text-white' und setzt das Padding
    div [ class "sidebar-tab-content" ]
        [ h3 [] [ text "Missions-Planung" ]
        
        , -- Sektion 1: Gewichtungen
          div [ class "planning-section" ]
            [ h4 [] [ text "Gewichtungen" ]
            , viewParamInput "Standard-Zeit (s)" "0.1"
            , viewParamInput "Komplexe Module" "0.1"
            , viewParamInput "Mensch-Faktor" "0.1"
            ]
        
        , -- Sektion 2: Navigation
          div [ class "planning-section" ]
            [ h4 [] [ text "Navigation" ]
            , div [ class "coord-display" ]
                [ label [] [ text "Startpunkt:" ]
                , span [] [ text "Nicht gesetzt" ]
                ]
            , div [ class "coord-display" ]
                [ label [] [ text "Zielpunkt:" ]
                , span [] [ text "Nicht gesetzt" ]
                ]
            ]
            
        , -- Der große Action-Button (btn-primary btn-full nutzen die Stile von btn-apply)
          button 
            [ class "btn-primary btn-full"
            , disabled True -- Logik hier einfügen, ob Button aktiv sein darf
            ] 
            [ text "Optimierung starten" ]
        ]

-- --- HELPER ---

viewParamInput : String -> String -> Html Msg
viewParamInput labelText stepVal =
    -- sidebar-section sorgt für 70% weißes Label und gestylten Input
    div [ class "sidebar-section" ]
        [ label [] [ text labelText ]
        , input 
            [ type_ "number"
            , step stepVal
            , value "1.0" -- Beispielwert
            ] []
        ]