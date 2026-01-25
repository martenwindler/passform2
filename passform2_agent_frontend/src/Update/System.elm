module Update.System exposing (update)

import Types exposing (..)
import Dict
import Ports

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
            ( { model | loading = True } -- Auch beim Klick auf "Browse" setzen
            , Ports.importConfigTrigger () 
            )

        FileSelected value ->
            ( { model | loading = True }
            , Ports.requestFileRead value 
            )

        ResetToLanding ->
            ( { model | activeLayout = LandingMode }, Cmd.none )