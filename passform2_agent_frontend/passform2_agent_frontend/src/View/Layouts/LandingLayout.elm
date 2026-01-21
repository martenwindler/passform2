module View.Layouts.LandingLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, preventDefaultOn)
import Json.Decode as Decode
import Types exposing (..)

view : Model -> Html Msg
view model =
    main_ [ class "landing-layout-content" ]
        [ div [ class "landing-wrapper" ]
            [ div [ class "landing-hero" ]
                [ h1 [ class "text-h1" ] [ text "Industrieller Gitter-Planer" ]
                , p [ class "text-label" ] [ text "Starten Sie ein neues Projekt oder laden Sie eine Konfiguration." ]
                ]

            , div [ class "landing-actions" ]
                [ -- 1. Neues Projekt
                  button 
                    [ class "landing-action-card bhover"
                    , onClick (SystemMsg NewProject) 
                    ]
                    [ div [ class "card-icon" ] [ text "⊕" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Neues Projekt" ] ]
                    ]

                -- 2. Vorlagen
                , button 
                    [ class "landing-action-card bhover"
                    , onClick NoOp 
                    ]
                    [ div [ class "card-icon" ] [ text "📋" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Vorlagen" ] ]
                    ]
                ]

            , div 
                [ class "landing-drop-area cursor-pointer" 
                , onDragOver DragOver
                , onDrop FileDropped
                , onClick (SystemMsg OpenFileBrowser) -- Triggert den JS-Dialog
                ]
                [ div [ class "drop-zone-inner" ]
                    [ div [ class "drop-icon" ] [ text "⤓" ]
                    , span [ class "text-label" ] [ text "Drop your .json or click here to browse" ] 
                    ]
                ]
            ]
        ]


-- --- EVENT HELPER ---

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