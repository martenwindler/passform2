module View.HardwareStatus exposing (viewStatusBadge, viewSignalStrength, viewAlertOverlay)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)


-- --- HARDWARE BADGES (Pillen-Design) ---


viewStatusBadge : String -> String -> String -> Html Msg
viewStatusBadge label status text_ =
    let
        statusClass =
            case status of
                "online" ->
                    "status-online"

                "missing" ->
                    "status-missing"

                _ ->
                    "status-unknown"
    in
    div [ class "status-pill" ] -- Das weiße Underlay
        [ span [ class "status-label" ] [ text label ]
        , div [ class "status-indicator" ]
            [ span [ class ("status-dot " ++ statusClass) ] []
            , span [ class "status-text" ] [ text text_ ]
            ]
        ]


-- --- SIGNALSTÄRKE ---


viewSignalStrength : Bool -> Int -> Html Msg
viewSignalStrength isOnline strength =
    let
        ( displayWidth, color, labelText ) =
            if not isOnline then
                ( "0%", "#bdc3c7", "---" )

            else
                let
                    c =
                        if strength > 75 then "#4caf50"
                        else if strength > 30 then "#ffc107"
                        else "#f44336"
                in
                ( String.fromInt strength ++ "%", c, String.fromInt strength ++ "%" )
    in
    div [ class "signal-wrapper" ]
        [ span [ class "signal-label-main" ] [ text "Verbindung:" ]
        , div [ class "signal-bar-container" ]
            [ div
                [ class "signal-bar-fill"
                , style "width" displayWidth
                , style "background-color" color
                ]
                []
            ]
        , span [ class "signal-value" ] [ text labelText ]
        ]


-- --- ALARM OVERLAY ---


viewAlertOverlay : Maybe String -> Html Msg
viewAlertOverlay maybeId =
    case maybeId of
        Just id ->
            div [ class "alert-toast" ]
                [ span [] [ text ("⚠️ KRITISCH: Modul " ++ id ++ " verliert Verbindung!") ]
                , button [ onClick DismissAlert, class "alert-close" ] [ text "OK" ]
                ]

        Nothing ->
            text ""