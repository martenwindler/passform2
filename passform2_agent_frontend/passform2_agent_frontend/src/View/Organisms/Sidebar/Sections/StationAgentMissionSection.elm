module View.Organisms.Sidebar.Sections.StationAgentMissionSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)
import Types.Domain exposing (..)
import View.Atoms.Icons as Icons

{-| 
Zentrale Sektion für die Missions-Parameter. 
Nutzt die hochwertigen SVG-Icons für Start und Ziel.
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section mission-control" ]
        [ h4 [] [ text "Navigation & Status" ]
        
        , -- Icons werden direkt als Atome übergeben
          viewCoord Icons.iconStart "Startpunkt" model.pathStart
        , viewCoord Icons.iconGoal "Zielpunkt" model.pathGoal
        
        , -- WICHTIG: Visuelles Feedback während 'model.loading'
          if model.loading then
            div [ class "flex items-center gap-3 p-3 bg-info/10 border border-info/20 rounded-sm mt-2 animate-pulse" ]
                [ div [ class "text-info" ] [ Icons.iconPlanning ] -- Nutzt das Kalender/Planungs-Icon als Spinner-Ersatz
                , span [ class "text-xs font-bold text-info uppercase tracking-wider" ] 
                    [ text "Verhandle Verträge..." ] 
                ]
          else 
            text ""
        
        , -- Action Button
          button 
            [ class (if canPlan model && not model.loading then "btn-primary btn-full mt-4" else "btn-disabled btn-full mt-4")
            , onClick (PlanningMsg (StartPlanning False))
            , disabled (not (canPlan model) || model.loading)
            ] 
            [ text (if model.loading then "Ausschreibung läuft..." else "Optimierung starten") ]
        ]

viewCoord : Html Msg -> String -> Maybe GridCell -> Html Msg
viewCoord icon labelTitle maybeCell =
    div [ class "coord-display flex justify-between items-center py-2 border-b border-white/5" ]
        [ div [ class "flex items-center gap-2" ] 
            [ div [ class "text-info opacity-80 scale-90" ] [ icon ] -- Icons etwas dezenter skaliert
            , label [ class "text-white/50 text-[0.75rem] font-medium m-0" ] [ text (labelTitle ++ ":") ]
            ]
        , case maybeCell of
            Just cell -> 
                span [ class "text-sm font-mono text-success font-bold" ] 
                    [ text ("(" ++ String.fromInt cell.x ++ "," ++ String.fromInt cell.y ++ ")") ]
            Nothing -> 
                span [ class "text-danger/60 italic text-[0.7rem]" ] [ text "Nicht gewählt" ]
        ]

canPlan : Model -> Bool
canPlan model =
    case (model.pathStart, model.pathGoal) of
        (Just _, Just _) -> True
        _ -> False