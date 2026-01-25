module View.Organisms.Sidebar.Sections.LogSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..)

{-| 
Diese Sektion rendert die System-Nachrichten ohne redundante Pattern-Matches.
-}
view : Model -> Html Msg
view model =
    div [ class "sidebar-section fill flex-1 min-h-0 flex flex-col" ]
        [ div [ class "log-container flex-1 overflow-y-auto scrollbar-hide" ]
            (if List.isEmpty model.logs then
                [ div [ class "p-lg opacity-30 italic text-sm text-center mt-10" ] 
                    [ text "Warte auf System-Input..." ] 
                ]
             else
                List.map viewLogEntry model.logs
            )
        ]

viewLogEntry : SystemLog -> Html Msg
viewLogEntry log =
    div [ class ("log-entry " ++ levelToClass log.level) ]
        [ div [ class "log-dot" ] [] 
        , div [ class "flex flex-col gap-0.5" ]
            [ span [ class "log-message font-mono text-[0.85rem]" ] [ text log.message ] ]
        ]

{-| 
Hier haben wir den Wildcard-Zweig entfernt, da alle Log-Level 
explizit abgedeckt sind.
-}
levelToClass : LogLevel -> String
levelToClass level =
    case level of
        Success -> "success"
        Info -> "info"
        Warning -> "warning"
        Danger -> "danger"