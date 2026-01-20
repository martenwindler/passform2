module View.Layouts.LandingLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, preventDefaultOn)
import Json.Decode as Decode
import Types exposing (..)

view : Model -> Html Msg
view model =
    main_ [ class "landing-layout-content" ]
        [ -- Der zentrale Wrapper (Flex-Container)
          div [ class "landing-wrapper" ]
            [ -- Zone 1: Hero (Überschrift)
              div [ class "landing-hero" ]
                [ h1 [ class "text-h1" ] [ text "Industrieller Gitter-Planer" ]
                , p [ class "text-label" ] [ text "Starten Sie ein neues Projekt oder laden Sie eine Konfiguration." ]
                ]

            , -- Zone 2: Actions (Horizontaler Flex-Container innerhalb des vertikalen Wrappers)
              div [ class "landing-actions" ]
                [ button [ class "landing-action-card bhover", onClick (SystemMsg NewProject) ]
                    [ div [ class "card-icon" ] [ text "⊕" ]
                    , div [ class "card-text" ] [ span [ class "text-button block" ] [ text "Neues Projekt" ] ]
                    ]
                , label [ class "landing-action-card bhover cursor-pointer", for "json-upload" ]
                    [ div [ class "card-icon" ] [ text "📂" ]
                    , div [ class "card-text" ] [ span [ class "text-button block" ] [ text "Vom Computer öffnen" ] ]
                    , input [ type_ "file", id "json-upload", class "sr-only", accept ".json", onFileChange (SystemMsg << FileSelected) ] []
                    ]
                ]

            , -- Zone 3: Massive Drop-Area (Grid/Flex Wrapper für Zentrierung)
              div 
                [ class "landing-drop-area"
                , onDragOver DragOver
                , onDrop FileDropped
                ]
                [ div [ class "drop-zone-inner" ]
                    [ div [ class "drop-icon" ] [ text "⤓" ]
                    , span [ class "text-label" ] [ text "Fügen Sie Ihre .json-Konfiguration an einer beliebigen Stelle in diesem Feld ein." ]
                    ]
                ]
            ]
        ]


-- --- EVENT HELPER ---

{-| Erkennt Dateiauswahl über den Button/Dialog -}
onFileChange : (Decode.Value -> Msg) -> Attribute Msg
onFileChange toMsg =
    Html.Events.on "change" (Decode.map toMsg Decode.value)


{-| Verhindert Browser-Default beim Dragover -}
onDragOver : SystemMsg -> Attribute Msg
onDragOver sMsg =
    preventDefaultOn "dragover" (Decode.succeed ( SystemMsg sMsg, True ))


{-| Extrahiert die Datei beim Drop und schickt sie an SystemMsg -}
onDrop : (Decode.Value -> SystemMsg) -> Attribute Msg
onDrop toSMsg =
    let
        fileDecoder =
            Decode.at [ "dataTransfer", "files", "0" ] Decode.value
    in
    preventDefaultOn "drop" 
        (Decode.map (\val -> ( SystemMsg (toSMsg val), True )) fileDecoder)