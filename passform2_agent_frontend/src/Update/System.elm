module Update.System exposing (update)

import Decoders
import Dict
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)


{-|
    Logik-Domäne: System-Aktionen & Projektverwaltung.
    Behandelt das Sandbox-Management (Session vs. Initial) und Layout-Wechsel.
-}
update : SystemMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- 1. NEUES PROJEKT: Arbeitskopie (Session) im Backend löschen und lokal leeren.
        -- Rust lädt beim nächsten 'Fortsetzen' oder Start die initial__00.json.
        NewProject ->
            let
                newModel =
                    { model
                        | activeLayout = AppMode
                        , editing = True
                        , agents = Dict.empty
                    }
            in
            ( newModel
            , Ports.socketEmit "reset_session" Encode.null
            )

        -- 2. VORLAGEN: Lädt ein Template, schreibt es in die session.json und wechselt die Ansicht.
        SelectTemplate templateName ->
            ( { model 
                | loading = True 
                , activeLayout = AppMode 
              }
            , Ports.socketEmit "load_template" (Encode.string templateName)
            )

        -- 3. LAYOUT-WECHSEL
        EnterAppMode ->
            -- "Fortsetzen": Der Editor wird geöffnet. 
            -- Das Backend schickt die Daten (Session oder Initial) automatisch via activeAgentsReceiver.
            ( { model | activeLayout = AppMode }, Cmd.none )

        ResetToLanding ->
            -- Zurück zum Hauptmenü: Hier findet der REWRITE statt.
            -- Wir schicken den aktuellen Stand der Session an den Golden Master (initial).
            ( { model | activeLayout = LandingMode }
            , Ports.persistToMaster (Decoders.encodeFullConfig model)
            )

        -- 4. DATEI-HANDLING (Manueller Import / Drag & Drop)
        DragOver ->
            ( model, Cmd.none )

        FileDropped value ->
            -- Datei reinziehen springt ebenfalls direkt in den Editor
            ( { model | loading = True, activeLayout = AppMode }
            , Ports.requestFileRead value
            )

        OpenFileBrowser ->
            ( { model | loading = True }
            , Ports.importConfigTrigger ()
            )

        FileSelected value ->
            ( { model | loading = True, activeLayout = AppMode }
            , Ports.requestFileRead value
            )

        -- 5. BACKEND-DATEN
        HandleWorldState value ->
            ( { model | worldState = Dict.insert "current" value model.worldState }
            , Cmd.none
            )