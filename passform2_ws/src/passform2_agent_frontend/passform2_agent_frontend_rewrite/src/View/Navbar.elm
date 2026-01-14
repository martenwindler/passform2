module View.Navbar exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

view : Model -> Html Msg
view model =
    nav [ class "navbar" ]
        [ div [ class "logo" ] [ text "Passform 2.0" ]
        , div [ class "status" ] 
            [ text (if model.connected then "Online" else "Offline") ]
        , button [ onClick ToggleMode ] 
            [ text (if model.mode == Simulation then "Zu Hardware wechseln" else "Zu Simulation wechseln") ]
        ]