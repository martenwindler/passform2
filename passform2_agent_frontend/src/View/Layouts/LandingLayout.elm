module View.Layouts.LandingLayout exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick, preventDefaultOn)
import Json.Decode as Decode
import Types exposing (..)


view : Model -> Html Msg
view model =
    main_ [ class "landing-layout-content" ]
        [ -- 1. Hintergrund-Ebene (Neu)
          div [ class "gradient-background" ]
            [ div [ class "gradient-sphere sphere-1" ] []
            , div [ class "gradient-sphere sphere-2" ] []
            , div [ class "gradient-sphere sphere-3" ] []
            , div [ class "glow" ] []
            , div [ class "grid-overlay" ] []
            , div [ class "noise-overlay" ] []
            , div [ class "particles-container", id "particles-container" ] []
            ]
        
        -- 2. Content-Ebene (Bestehend, leicht angepasst)
        , div [ class "landing-wrapper" ]
            [ div [ class "landing-hero" ]
                [ h1 [] [ text "Industrieller Gitter-Planer" ]
                , p [] [ text "Starten Sie ein neues Projekt oder laden Sie eine Konfiguration." ]
                ]

            , div [ class "landing-actions" ]
                [ button
                    [ class "landing-action-card continue-card"
                    , onClick (SystemMsg EnterAppMode) 
                    ]
                    [ div [ class "card-icon" ] [ text "🚀" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Session fortsetzen" ] ]
                    ]

                , button
                    [ class "landing-action-card"
                    , onClick (SystemMsg NewProject)
                    ]
                    [ div [ class "card-icon" ] [ text "⊕" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Neues Projekt" ] ]
                    ]

                , button
                    [ class "landing-action-card"
                    , onClick (SystemMsg (SelectTemplate "demo_layout_biba"))
                    ]
                    [ div [ class "card-icon" ] [ text "📋" ]
                    , div [ class "card-text" ] [ span [ class "text-button" ] [ text "Vorlage laden" ] ]
                    ]
                ]
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
