module View.Molecules.HardwareStatus exposing (viewStatusBadge, viewSignalStrength, viewAlertOverlay)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)

{-| Molekül: Ein Status-Badge (Kombination aus Label, Dot und Text).
Gibt jetzt Html HardwareMsg zurück, um typsicher mit der Hardware-Domäne zu sein.
-}
viewStatusBadge : String -> String -> String -> Html HardwareMsg
viewStatusBadge label status text_ =
    let
        statusClass =
            case status of
                "online" -> "online"
                "missing" -> "danger"
                _ -> "unknown"
    in
    div [ class "status-pill" ]
        [ span [ class "status-label" ] [ text label ]
        , div [ class "status-indicator" ]
            [ span [ class ("status-dot " ++ statusClass) ] [] 
            , span [ class "status-text" ] [ text text_ ]
            ]
        ]

{-| Molekül: Die Signalstärke-Anzeige (Label + Progress-Bar + Wert).
-}
viewSignalStrength : Bool -> Int -> Html HardwareMsg
viewSignalStrength isOnline strength =
    let
        ( strengthClass, labelText ) =
            if not isOnline then
                ( "signal-none", "---" )
            else
                let
                    sClass =
                        if strength > 75 then "signal-strong"
                        else if strength > 30 then "signal-fair"
                        else "signal-weak"
                in
                ( sClass, String.fromInt strength ++ "%" )
        
        widthStyle = 
            if isOnline then String.fromInt strength ++ "%" else "0%"
    in
    div [ class "signal-wrapper" ]
        [ span [ class "signal-label-main" ] [ text "Signal:" ]
        , div [ class "signal-bar-container" ]
            [ div
                [ class ("signal-bar-fill " ++ strengthClass)
                , style "width" widthStyle 
                ]
                []
            ]
        , span [ class "signal-value" ] [ text labelText ]
        ]

{-| Molekül: Ein Alarm-Overlay (Toast).
Nutzt DismissAlert, welches ein Konstruktor von HardwareMsg ist.
-}
viewAlertOverlay : Maybe String -> Html HardwareMsg
viewAlertOverlay maybeId =
    case maybeId of
        Just id ->
            div [ class "alert-toast" ]
                [ span [] [ text ("⚠️ KRITISCH: Modul " ++ id ++ " verliert Verbindung!") ]
                , button [ onClick DismissAlert, class "alert-close" ] [ text "OK" ]
                ]

        Nothing ->
            text ""