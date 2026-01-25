module View.Molecules.HardwareStatus exposing (view, viewIndicator)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types.Domain exposing (HardwareStatus(..))

{-| Der Punkt für die Hardware-Liste -}
view : HardwareStatus -> Html msg
view status =
    div [ class ("status-dot " ++ statusToClass status) ] []

{-| Die "Pille" für die Navbar -}
viewIndicator : String -> HardwareStatus -> Html msg
viewIndicator label status =
    span [ class "indicator-group" ]
        [ div [ class ("status-dot-small " ++ statusToClass status) ] []
        , span [ class "indicator-label" ] [ text label ]
        ]

statusToClass : HardwareStatus -> String
statusToClass status =
    case status of
        Online -> "status-online"
        Error -> "status-missing"
        Standby -> "status-unknown"
        _ -> "status-unknown"