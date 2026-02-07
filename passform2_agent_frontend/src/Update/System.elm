module Update.System exposing (update)

import Decoders
import Dict
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)


{-|
    Logik-Domäne: System-Aktionen & Projektverwaltung.
    Behandelt das Sandbox-Management (Session vs. Initial), Layout-Wechsel und Kamera-Steuerung.
-}
update : SystemMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- 1. NEUES PROJEKT
        NewProject ->
            let
                newModel = { model | activeLayout = AppMode, editing = True, agents = Dict.empty }
            in
            ( newModel, Ports.socketEmit "reset_session" Encode.null )

        -- 2. VORLAGEN
        SelectTemplate templateName ->
            ( { model | loading = True, activeLayout = AppMode }
            , Ports.socketEmit "load_template" (Encode.string templateName)
            )

        -- 3. LAYOUT-WECHSEL
        EnterAppMode ->
            ( { model | activeLayout = AppMode }, Cmd.none )

        ResetToLanding ->
            ( { model | activeLayout = LandingMode }
            , Ports.persistToMaster (Decoders.encodeFullConfig model)
            )

        -- 4. DATEI-HANDLING
        DragOver -> ( model, Cmd.none )

        FileDropped value ->
            ( { model | loading = True, activeLayout = AppMode }, Ports.requestFileRead value )

        OpenFileBrowser ->
            ( { model | loading = True }, Ports.importConfigTrigger () )

        FileSelected value ->
            ( { model | loading = True, activeLayout = AppMode }, Ports.requestFileRead value )

        -- 5. BACKEND-DATEN
        HandleWorldState value ->
            ( { model | worldState = Dict.insert "current" value model.worldState }, Cmd.none )

        -- 6. KAMERA-STEUERUNG
        RotateCamera angle ->
            let
                -- Das hier erzeugt eine Ausgabe in der Browser-Konsole
                _ = Debug.log "Elm: Sende Rotation an Port" angle
            in
            ( model, Ports.rotateCamera angle )