module View.Layouts.LandingLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, preventDefaultOn)
import Json.Decode as Decode
import Types exposing (..)


{-| Die Landing Page ist der Einstiegspunkt der App.
Hier wählt der Nutzer zwischen einem leeren Projekt, einer Vorlage oder dem Import.
-}
view : Model -> Html Msg
view model =
    main_ [ class "landing-layout-content" ]
        [ div [ class "landing-wrapper" ]
            [ div [ class "landing-hero" ]
                [ h1 [ class "text-h1" ] [ text "Industrieller Gitter-Planer" ]
                , p [ class "text-label" ] [ text "Starten Sie ein neues Projekt oder laden Sie eine Konfiguration." ]
                ]

            , div [ class "landing-actions" ]
                [ -- 1. Fortsetzen: Öffnet den Editor mit dem Stand der aktuellen session.json (oder initial__00.json)
                  button
                    [ class "landing-action-card bhover continue-card"
                    , onClick (SystemMsg EnterAppMode) 
                    ]
                    [ div [ class "card-icon" ] [ text "🚀" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Session fortsetzen" ] ]
                    ]

                -- 2. Neues Projekt: Triggert das Löschen der session.json im Backend via Socket
                , button
                    [ class "landing-action-card bhover"
                    , onClick (SystemMsg NewProject)
                    ]
                    [ div [ class "card-icon" ] [ text "⊕" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Neues Projekt" ] ]
                    ]

                -- 3. Vorlage laden: Kopiert die demo_layout_biba.json im Backend in die aktive Session
                , button
                    [ class "landing-action-card bhover"
                    , onClick (SystemMsg (SelectTemplate "demo_layout_biba"))
                    ]
                    [ div [ class "card-icon" ] [ text "📋" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Vorlage laden" ] ]
                    ]
                ]

            -- 4. Drag & Drop Area / File Browser
            -- Ermöglicht das Hochladen einer externen JSON, die dann zur neuen Session wird.
            {- 
            , div
                [ class "landing-drop-area cursor-pointer"
                , onDragOver DragOver
                , onDrop FileDropped
                , onClick (SystemMsg OpenFileBrowser) -- Triggert den JS-Dialog via Ports
                ]
                [ div [ class "drop-zone-inner" ]
                    [ div [ class "drop-icon" ] [ text "⤓" ]
                    , span [ class "text-label" ] [ text "JSON hierher ziehen oder klicken zum Durchsuchen" ]
                    ]
                ]
            -}



            ]
        ]


-- --- EVENT HELPER ---

{-| Verhindert Browser-Default (z.B. Datei im Tab öffnen) beim Dragover -}
onDragOver : SystemMsg -> Attribute Msg
onDragOver sMsg =
    preventDefaultOn "dragover" (Decode.succeed ( SystemMsg sMsg, True ))


{-| Extrahiert die Datei beim Drop und schickt sie an SystemMsg FileDropped -}
onDrop : (Decode.Value -> SystemMsg) -> Attribute Msg
onDrop toSMsg =
    let
        fileDecoder =
            Decode.at [ "dataTransfer", "files", "0" ] Decode.value
    in
    preventDefaultOn "drop"
        (Decode.map (\val -> ( SystemMsg (toSMsg val), True )) fileDecoder)