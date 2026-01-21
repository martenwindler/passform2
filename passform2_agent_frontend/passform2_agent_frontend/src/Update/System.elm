module Update.System exposing (update)

import Types exposing (..)
import Dict
import Ports
import Json.Decode as Decode

{-|
Logik für App-weite System-Events:
- Drag & Drop Handling
- Datei-Import via Browser
- Layout-Umschaltung
-}
update : SystemMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        NewProject ->
            ( { model 
                | activeLayout = AppMode
                , editing = True  -- Das muss hier stehen!
                , agents = Dict.empty 
            }
            , Cmd.none 
            )
            
        OpenFileBrowser ->
            -- Dieser Call wird meist indirekt über das Label/Input im View gelöst,
            -- aber wir lassen ihn als Platzhalter für manuelle Trigger.
            ( model, Cmd.none )

        DragOver ->
            -- Notwendig, um das Standardverhalten des Browsers zu unterdrücken
            ( model, Cmd.none )

        FileDropped value ->
            -- Wir schicken das JS-Value (DataTransfer Objekt) an den Port
            ( { model | loading = True }
            , Ports.requestFileRead value 
            )

        FileSelected value ->
            -- Wir schicken das JS-Value (File Objekt vom Input) an den Port
            ( { model | loading = True }
            , Ports.requestFileRead value 
            )

        ResetToLanding ->
            -- Zurück zum Photopea-Startbildschirm
            ( { model | activeLayout = LandingMode }, Cmd.none )