module Update.Hardware exposing (update)

import Types exposing (..)
import Types.Domain exposing (..)
import Ports
import Json.Encode as Encode

{-| 
    Logik-Domäne: Hardware & Kommunikation.
    Zuständig für die Interaktion mit physischen Sensoren, ROS 2 Nodes 
    und die Verwaltung der System-Frequenz (Heartbeat).
-}
update : HardwareMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- --- VERBINDUNGSSTATUS ---
        SetConnected status ->
            ( { model | connected = status }, Cmd.none )

        SetRosConnected status ->
            ( { model | rosConnected = status }, Cmd.none )

        -- --- SENSORIK & DATA ---
        HandleSystemLog result ->
            case result of
                Ok logEntry ->
                    -- Maximal 30 Einträge speichern
                    ( { model | logs = List.take 30 (logEntry :: model.logs), waitingForNfc = False }
                    , Cmd.none 
                    )
                Err _ -> ( model, Cmd.none )

        HandleRfid result ->
            case result of
                Ok tagId -> 
                    ( { model | lastWrittenId = Just tagId }, Cmd.none )
                Err _ -> 
                    ( model, Cmd.none )

        HandleHardwareUpdate result ->
            case result of
                Ok hardwareList -> 
                    ( { model | connectedHardware = hardwareList }, Cmd.none )
                Err _ -> 
                    ( model, Cmd.none )

        -- --- RANGER SPEZIFISCH ---
        HandleRangerBattery voltage ->
            ( { model | rangerBattery = Just voltage }, Cmd.none )

        -- --- NFC LOGIK ---
        RequestNfcWrite content ->
            ( { model | waitingForNfc = True, logs = { message = "NFC: Sende Brennbefehl...", level = Info } :: model.logs }
            , Ports.writeNfcTrigger content 
            )

        HandleNfcStatus result ->
            case result of
                Ok status -> 
                    if String.startsWith "Success:" status then
                        let 
                            idOnly = String.dropLeft 8 status
                        in 
                        ( { model | nfcStatus = Online, lastWrittenId = Just idOnly, waitingForNfc = False }, Cmd.none )
                    else
                        ( { model | nfcStatus = Error, waitingForNfc = False }, Cmd.none )
                Err _ -> 
                    ( { model | nfcStatus = Error, waitingForNfc = False }, Cmd.none )

        -- --- SYSTEM-TAKT & MODI ---
        ChangeHz newHz ->
            ( { model | currentHz = newHz }
            , Ports.socketEmit "set_heartbeat_rate" (Encode.float newHz)
            )

        SetMode modeStr ->
            ( model, Ports.setMode modeStr )

        DismissAlert ->
            ( { model | alert = Nothing }, Cmd.none )