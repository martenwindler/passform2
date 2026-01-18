module Update.Planning exposing (update, canPlan)

import Types exposing (..)
import Types.Domain exposing (..)
import Ports
import Decoders
import Json.Decode as Decode
import Json.Encode as Encode

{-| 
    Logik-Domäne: Planung & Gitter-Konfiguration.
    Zuständig für Pfadsuche, Gitterdimensionen und Contract-Net Gewichte.
-}
update : PlanningMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        SetGridWidth val ->
            ( { model | gridWidth = String.toInt val |> Maybe.withDefault model.gridWidth }, Cmd.none )

        SetGridHeight val ->
            ( { model | gridHeight = String.toInt val |> Maybe.withDefault model.gridHeight }, Cmd.none )

        SetPathStart cell ->
            ( { model | pathStart = Just cell, activeMenu = Nothing }, Cmd.none )

        SetPathGoal cell ->
            ( { model | pathGoal = Just cell, activeMenu = Nothing }, Cmd.none )

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
            ( { model | planningWeights = newWeights }, Cmd.none )

        SaveWeights ->
            ( { model | logs = { message = "Planungsparameter aktualisiert.", level = Success } :: model.logs }
            , Ports.savePlanningWeights (Decoders.encodeWeights model.planningWeights) 
            )

        StartPlanning isRanger ->
            if canPlan model then
                ( { model | loading = True }
                , -- FIX: Nutzt triggerPlanning (wie in Ports.elm definiert) 
                  -- und encodiert die Daten zu JSON (Decode.Value)
                  Ports.triggerPlanning 
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
            -- FIX: Decode ist jetzt durch den Import verfügbar
            case Decode.decodeValue Decoders.pathDecoder rawJson of
                Ok path ->
                    ( { model | currentPath = Just path, loading = False }, Cmd.none )
                Err _ ->
                    ( { model | loading = False }, Cmd.none )

        ConfigReceived _ ->
            ( model, Cmd.none )


-- --- HELPERS ---

{-| Prüft, ob alle Voraussetzungen für eine Pfadplanung erfüllt sind. -}
canPlan : { a | pathStart : Maybe GridCell, pathGoal : Maybe GridCell } -> Bool
canPlan model =
    case ( model.pathStart, model.pathGoal ) of
        ( Just _, Just _ ) -> True
        _ -> False