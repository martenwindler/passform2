module Update.Planning exposing (update, canPlan)

import Types exposing (..)
import Types.Domain exposing (..)
import Ports
import Decoders
import Json.Decode as Decode
import Json.Encode as Encode

{-| 
    Logik-Domäne: Planung & Gitter-Konfiguration.
    Sichert Gitter-Dimensionen ab und steuert den Lebenszyklus von Pfad-Resultaten.
-}
update : PlanningMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        SetGridWidth val ->
            let
                newWidth = 
                    String.toInt val 
                        |> Maybe.withDefault model.gridWidth 
                        |> clamp 0 100
            in
            ( { model | gridWidth = newWidth, currentPath = Nothing }, Cmd.none )

        SetGridHeight val ->
            let
                newHeight = 
                    String.toInt val 
                        |> Maybe.withDefault model.gridHeight 
                        |> clamp 0 100
            in
            ( { model | gridHeight = newHeight, currentPath = Nothing }, Cmd.none )

        SetPathStart cell ->
            -- Pfad zurücksetzen, da sich die Route durch neuen Start ändert
            ( { model | pathStart = Just cell, currentPath = Nothing, activeMenu = Nothing }, Cmd.none )

        SetPathGoal cell ->
            -- Pfad zurücksetzen, da sich die Route durch neues Ziel ändert
            ( { model | pathGoal = Just cell, currentPath = Nothing, activeMenu = Nothing }, Cmd.none )

        SetWeight field value ->
            let
                newVal = String.toFloat value |> Maybe.withDefault 0.0
                oldWeights = model.planningWeights
                newWeights =
                    case field of
                        "execution_time_default" -> { oldWeights | execution_time_default = newVal }
                        "complex_module_time" -> { oldWeights | complex_module_time = newVal }
                        "human_extra_weight" -> { oldWeights | human_extra_weight = newVal }
                        "proximity_penalty" -> { oldWeights | proximity_penalty = newVal }
                        "hardware_safety_factor" -> { oldWeights | hardware_safety_factor = newVal }
                        _ -> oldWeights
            in
            -- Auch bei Gewichtsänderungen setzen wir den Pfad zurück, da die Kosten neu berechnet werden müssen
            ( { model | planningWeights = newWeights, currentPath = Nothing }, Cmd.none )

        SaveWeights ->
            ( { model | logs = { message = "Planungsparameter aktualisiert.", level = Success } :: model.logs }
            , Ports.savePlanningWeights (Decoders.encodeWeights model.planningWeights) 
            )

        StartPlanning isRanger ->
            if canPlan model then
                ( { model | loading = True, currentPath = Nothing }
                , Ports.triggerPlanning 
                    (Decoders.encodePlanningData 
                        { start = model.pathStart
                        , goal = model.pathGoal
                        , weights = model.planningWeights
                        , isRanger = isRanger 
                        }
                    )
                )
            else
                ( model, Cmd.none )

        PlanningResultRaw rawJson ->
            case Decode.decodeValue Decoders.pathDecoder rawJson of
                Ok path ->
                    ( { model | currentPath = Just path, loading = False }, Cmd.none )
                Err _ ->
                    ( { model | loading = False, currentPath = Nothing }, Cmd.none )

        ConfigReceived _ ->
            ( model, Cmd.none )


-- --- HELPERS ---

{-| Prüft, ob alle Voraussetzungen für eine Pfadplanung erfüllt sind. -}
canPlan : { a | pathStart : Maybe GridCell, pathGoal : Maybe GridCell } -> Bool
canPlan model =
    case ( model.pathStart, model.pathGoal ) of
        ( Just _, Just _ ) -> True
        _ -> False