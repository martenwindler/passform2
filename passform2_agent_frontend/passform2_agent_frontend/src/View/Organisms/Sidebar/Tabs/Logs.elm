module View.Organisms.Sidebar.Tabs.Logs exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)

{-| 
Organismus-Teil: System-Logs.
Nutzt die 'fill'-Logik aus der sidebar.scss für ein echtes Terminal-Scrolling.
-}
view : Model -> Html Msg
view model =
    -- h-full sorgt für 100% Höhe des Tab-Bereichs
    div [ class "sidebar-tab-content h-full flex flex-col" ]
        [ h3 [ class "shrink-0 mb-4" ] [ text "System-Historie" ]
        
        , -- Dieser Container "frisst" den restlichen Platz
          div [ class "sidebar-section fill flex-1 min-h-0 flex flex-col" ]
            [ div [ class "log-container flex-1" ]
                (if List.isEmpty model.logs then
                    [ div [ class "p-lg opacity-30 italic text-sm" ] [ text "Warte auf System-Input..." ] ]
                 else
                    List.map viewLogEntry model.logs
                )
            ]
        ]

viewLogEntry : SystemLog -> Html Msg
viewLogEntry log =
    -- log-entry nutzt die Dot-Logik und die farblichen Akzente aus dem SCSS
    div [ class ("log-entry " ++ levelToClass log.level) ]
        [ div [ class "log-dot" ] [] -- Der kleine farbige Punkt links
        , div [ class "flex flex-col gap-0.5" ]
            [ -- Falls du einen Zeitstempel im Modell hast, hier einfügen
              -- span [ class "opacity-30 text-[0.6rem] font-mono" ] [ text "10:24:01" ]
              span [ class "log-message" ] [ text log.message ]
            ]
        ]

levelToClass : LogLevel -> String
levelToClass level =
    case level of
        Success -> "success"
        Info -> "info"
        Warning -> "warning"
        _ -> ""