module Update.System exposing (update)

import Types exposing (..)
import Dict
import Ports
import Json.Decode as Decode

{-| 
    Logik-Domäne: System-Aktionen & Projektverwaltung.
    Behandelt Layout-Wechsel, Datei-Operationen und den globalen Weltzustand.
-}
update : SystemMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        NewProject ->
            ( { model 
                | activeLayout = AppMode
                , editing = True
                , agents = Dict.empty 
              }
            , Cmd.none 
            )

        DragOver ->
            ( model, Cmd.none )

        FileDropped value ->
            ( { model | loading = True }
            , Ports.requestFileRead value 
            )

        OpenFileBrowser ->
            ( { model | loading = True }
            , Ports.importConfigTrigger () 
            )

        FileSelected value ->
            ( { model | loading = True }
            , Ports.requestFileRead value 
            )

        ResetToLanding ->
            ( { model | activeLayout = LandingMode }, Cmd.none )

        -- NEU: Verarbeitet den Weltzustand vom Backend für den Planner
        HandleWorldState value ->
            -- Wir speichern das JSON-Value im Dictionary, um später darauf zugreifen zu können
            ( { model | worldState = Dict.insert "current" value model.worldState }
            , Cmd.none 
            )