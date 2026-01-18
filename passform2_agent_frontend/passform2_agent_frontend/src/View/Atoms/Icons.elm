module View.Atoms.Icons exposing (..)

import Html exposing (Html)
import Svg exposing (Svg)
import Svg.Attributes as Attr

{-| 
  Atom: Basis-Konfiguration für alle Icons.
  Zentralisiert das Sizing und die Stroke-Attribute für einen konsistenten Look.
-}
iconBase : List (Svg.Attribute msg) -> List (Svg.Svg msg) -> Html msg
iconBase attrs children =
    Svg.svg
        ([ Attr.width "20"
         , Attr.height "20"
         , Attr.viewBox "0 0 24 24"
         , Attr.fill "none"
         , Attr.stroke "currentColor"
         , Attr.strokeWidth "2"
         , Attr.strokeLinecap "round"
         , Attr.strokeLinejoin "round"
         , Attr.class "icon-svg" 
         ] ++ attrs)
        children

-- --- SIDEBAR NAVIGATION ICONS ---

{-| Icon: Planning (Calendar Style) -}
iconPlanning : Html msg
iconPlanning = 
    iconBase [] 
        [ Svg.rect [ Attr.x "3", Attr.y "4", Attr.width "18", Attr.height "18", Attr.rx "2", Attr.ry "2" ] []
        , Svg.line [ Attr.x1 "16", Attr.y1 "2", Attr.x2 "16", Attr.y2 "6" ] []
        , Svg.line [ Attr.x1 "8", Attr.y1 "2", Attr.x2 "8", Attr.y2 "6" ] []
        , Svg.line [ Attr.x1 "3", Attr.y1 "10", Attr.x2 "21", Attr.y2 "10" ] []
        ]

{-| Icon: Module (Isometric Cube Style) -}
iconAgents : Html msg
iconAgents = 
    iconBase [] 
        [ Svg.path [ Attr.d "M12 2L20 7V17L12 22L4 17V7L12 2Z" ] []
        , Svg.path [ Attr.d "M12 22V12" ] []
        , Svg.path [ Attr.d "M20 7L12 12L4 7" ] []
        ]

{-| Icon: Hardware (Microchip Style) -}
iconHardware : Html msg
iconHardware = 
    iconBase [] 
        [ Svg.rect [ Attr.x "4", Attr.y "4", Attr.width "16", Attr.height "16", Attr.rx "2" ] []
        , Svg.line [ Attr.x1 "9", Attr.y1 "4", Attr.x2 "9", Attr.y2 "2" ] []
        , Svg.line [ Attr.x1 "15", Attr.y1 "4", Attr.x2 "15", Attr.y2 "2" ] []
        , Svg.line [ Attr.x1 "9", Attr.y1 "20", Attr.x2 "9", Attr.y2 "22" ] []
        , Svg.line [ Attr.x1 "15", Attr.y1 "20", Attr.x2 "15", Attr.y2 "22" ] []
        , Svg.line [ Attr.x1 "20", Attr.y1 "9", Attr.x2 "22", Attr.y2 "9" ] []
        , Svg.line [ Attr.x1 "20", Attr.y1 "15", Attr.x2 "22", Attr.y2 "15" ] []
        , Svg.line [ Attr.x1 "4", Attr.y1 "9", Attr.x2 "2", Attr.y2 "9" ] []
        , Svg.line [ Attr.x1 "4", Attr.y1 "15", Attr.x2 "2", Attr.y2 "15" ] []
        ]

{-| Icon: Logs (Scroll/List Style) -}
iconLogs : Html msg
iconLogs = 
    iconBase [] 
        [ Svg.path [ Attr.d "M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" ] []
        , Svg.polyline [ Attr.points "14 2 14 8 20 8" ] []
        ]

-- --- ACTION ICONS ---

{-| Icon: Rotate (Refresh Arrow) -}
iconRotate : Html msg
iconRotate = 
    iconBase [] 
        [ Svg.path [ Attr.d "M23 4v6h-6" ] []
        , Svg.path [ Attr.d "M20.49 15a9 9 0 1 1-2.12-9.36L23 10" ] []
        ]

{-| Icon: Trash (Delete Style) -}
iconTrash : Html msg
iconTrash = 
    iconBase [] 
        [ Svg.polyline [ Attr.points "3 6 5 6 21 6" ] []
        , Svg.path [ Attr.d "M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" ] []
        ]

-- --- NAVIGATION / ROUTING ICONS ---

{-| Icon: Start (Play Circle) -}
iconStart : Html msg
iconStart = 
    iconBase [] 
        [ Svg.circle [ Attr.cx "12", Attr.cy "12", Attr.r "10" ] []
        , Svg.path [ Attr.d "M10 8l6 4-6 4V8z" ] []
        ]

{-| Icon: Goal (Flag Style) -}
iconGoal : Html msg
iconGoal = 
    iconBase [] 
        [ Svg.path [ Attr.d "M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z" ] []
        , Svg.line [ Attr.x1 "4", Attr.y1 "22", Attr.x2 "4", Attr.y2 "15" ] []
        ]