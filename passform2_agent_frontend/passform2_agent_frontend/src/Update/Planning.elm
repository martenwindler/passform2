module Update.Planning exposing (update, canPlan)

import Decoders
import Json.Decode as Decode
import Ports
import Types exposing (..)
import Types.Domain exposing (..)

update : PlanningMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        SetGridWidth val ->
            let
                newWidth = String.toInt val |> Maybe.withDefault model.gridWidth |> clamp 0 100
            in
            ( { model | gridWidth = newWidth, currentPath = Nothing }, Cmd.none )

        SetGridHeight val ->
            let
                newHeight = String.toInt val |> Maybe.withDefault model.gridHeight |> clamp 0 100
            in
            ( { model | gridHeight = newHeight, currentPath = Nothing }, Cmd.none )

        SetPathStart cell ->
            ( { model | pathStart = Just cell, currentPath = Nothing, activeMenu = Nothing }, Cmd.none )

        SetPathGoal cell ->
            ( { model | pathGoal = Just cell, currentPath = Nothing, activeMenu = Nothing }, Cmd.none )

        SetWeight field value ->
            let
                -- Wir wandeln den String in einen Float um, Default 0.0 bei Fehlern
                rawVal = String.toFloat value |> Maybe.withDefault 0.0
                
                -- HIER: Die harten Grenzen nach Harlan's Modell
                newVal =
                    case field of
                        "execution_time_default" ->
                            clamp 0.1 5.0 rawVal

                        "complex_module_time" ->
                            clamp 1.0 10.0 rawVal

                        "human_extra_weight" ->
                            clamp 0.0 20.0 rawVal

                        "proximity_penalty" ->
                            clamp 0.0 10.0 rawVal

                        "hardware_safety_factor" ->
                            clamp 1.0 2.0 rawVal

                        _ ->
                            rawVal

                oldWeights = model.planningWeights
                
                -- Update des entsprechenden Feldes im Weights-Record
                newWeights =
                    case field of
                        "execution_time_default" -> { oldWeights | execution_time_default = newVal }
                        "complex_module_time" -> { oldWeights | complex_module_time = newVal }
                        "human_extra_weight" -> { oldWeights | human_extra_weight = newVal }
                        "proximity_penalty" -> { oldWeights | proximity_penalty = newVal }
                        "hardware_safety_factor" -> { oldWeights | hardware_safety_factor = newVal }
                        _ -> oldWeights
            in
            -- Wir setzen den Pfad auf Nothing, da sich die Kostenbasis geändert hat
            ( { model | planningWeights = newWeights, currentPath = Nothing }, Cmd.none )

        SaveWeights ->
            ( { model | logs = { message = "Planungsparameter aktualisiert.", level = Success } :: model.logs }
            , Ports.savePlanningWeights (Decoders.encodeWeights model.planningWeights)
            )

        StartPlanning _ ->
            if canPlan model then
                ( { model | loading = True, currentPath = Nothing }
                , Ports.sendCnpRequest (Decoders.encodeCnpAnnouncement model)
                )
            else
                ( model, Cmd.none )

        SetPathResult path ->
            ( { model | currentPath = Just path, loading = False }, Cmd.none )

        PlanningResultRaw rawJson ->
            case Decode.decodeValue Decoders.pathDecoder rawJson of
                Ok path ->
                    ( { model | currentPath = Just path, loading = False }, Cmd.none )
                Err _ ->
                    ( { model | loading = False, currentPath = Nothing }, Cmd.none )

        ConfigReceived _ ->
            ( model, Cmd.none )

canPlan : Model -> Bool
canPlan model =
    let
        hasPoints =
            case ( model.pathStart, model.pathGoal ) of
                ( Just _, Just _ ) ->
                    True

                _ ->
                    False
    in
    -- Wir haben 'model.rosConnected' hier entfernt, 
    -- damit du auch ohne ROS-Bridge planen kannst.
    hasPoints && model.connected