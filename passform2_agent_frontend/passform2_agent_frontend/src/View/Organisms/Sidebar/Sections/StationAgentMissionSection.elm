module View.Organisms.Sidebar.Sections.StationAgentMissionSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Icons as Icons

view : Model -> Html Msg
view model =
    div [ class "planning-section planning-planer-section" ]
        [ h4 [] [ text "CNP-Ausschreibung" ]
        
        , div [ class "planer-coords-container" ]
            [ viewCoord Icons.iconStart "Startpunkt" model.pathStart
            , viewCoord Icons.iconGoal "Zielpunkt" model.pathGoal
            ]
        
        , if model.loading then
            div [ class "flex items-center gap-3 p-3 bg-info/10 border border-info/20 rounded-sm mt-2 animate-pulse" ]
                [ div [ class "text-info" ] [ Icons.iconPlanning ]
                , span [ class "text-xs font-bold text-info uppercase tracking-wider" ] 
                    [ text "Verhandle Verträge..." ] 
                ]
          else 
            text ""
        
        , button 
            [ classList 
                [ ("btn-start-optimization", True)
                , ("active", canPlan model && not model.loading)
                , ("disabled", not (canPlan model) || model.loading)
                ]
            , onClick (PlanningMsg (StartPlanning False))
            , disabled (not (canPlan model) || model.loading)
            ] 
            [ text (if model.loading then "Ausschreibung läuft..." else "Ausschreibung starten") ]
        ]

viewCoord : Html Msg -> String -> Maybe GridCell -> Html Msg
viewCoord icon labelTitle maybeCell =
    div [ class "coord-display" ]
        [ div [ class "coord-label flex items-center gap-2" ] 
            [ div [ class "text-info opacity-80 scale-90" ] [ icon ]
            , label [ class "text-white/50 text-[0.75rem] font-medium m-0" ] [ text (labelTitle ++ ":") ]
            ]
        , case maybeCell of
            Just cell -> 
                span [ class "coord-value is-set text-sm font-mono text-success font-bold" ] 
                    -- KORREKTUR: cell.x und cell.y direkt ansprechen
                    [ text ("(" ++ String.fromInt cell.x ++ "," ++ String.fromInt cell.y ++ ")") ]
            Nothing -> 
                span [ class "coord-value is-empty text-danger/60 italic text-[0.7rem]" ] [ text "Nicht gewählt" ]
        ]

canPlan : Model -> Bool
canPlan model =
    case (model.pathStart, model.pathGoal) of
        (Just _, Just _) -> True
        _ -> False