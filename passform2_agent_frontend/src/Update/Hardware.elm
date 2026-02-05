module Update.Hardware exposing (update)

import Decoders
import Json.Decode as Decode
import Ports
import Types exposing (..)
import Types.Domain exposing (..)
import Json.Encode as Encode

{-| 
    Logik-Domäne: Hardware & Kommunikation.
    Zuständig für die Interaktion mit physischen Sensoren, ROS 2 Nodes,
    den Digitalen Zwilling (Bays) und die Materialverwaltung (Inventory).
-}
update : HardwareMsg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        -- --- VERBINDUNGSSTATUS ---
        SetConnected status ->
            ( { model | connected = status }, Cmd.none )

        SetRosConnected status ->
            ( { model | rosConnected = status }, Cmd.none )

        -- --- DIGITAL TWIN: BUCHTEN (BAYS) ---
        HandleInitialBays result ->
            case result of
                Ok bayList ->
                    ( { model | bays = bayList }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        HandleBayUpdate result ->
            case result of
                Ok updatedBay ->
                    -- Wir suchen in der Liste nach der Bucht und tauschen sie aus
                    let
                        newBays =
                            List.map 
                                (\bay -> 
                                    if bay.unique_id == updatedBay.unique_id then 
                                        -- Hier übernehmen wir den neuen Status vom Backend
                                        { bay | occupation = updatedBay.occupation
                                            , module_uuid = updatedBay.module_uuid 
                                        } 
                                    else 
                                        bay
                                ) 
                                model.bays
                        
                        -- Optional: Einen Log-Eintrag erstellen, wenn ein Tisch belegt wird
                        newLogs = 
                            if updatedBay.occupation then
                                { message = updatedBay.name ++ " belegt durch " ++ updatedBay.module_uuid
                                , level = Info 
                                } :: model.logs
                            else
                                model.logs
                    in
                    ( { model | bays = newBays, logs = List.take 30 newLogs }, Cmd.none )

                Err _ -> 
                    ( model, Cmd.none )

        -- --- LOGISTIK: INVENTAR ---
        HandleInventoryUpdate result ->
            case result of
                Ok itemList ->
                    ( { model | inventory = itemList }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        -- --- HARDWARE SPECS & ROS INTERFACES ---
        HandleSpecsUpdate rawValue ->
            -- Hier könnten wir die Hardware-Spezifikationen im Modell ablegen
            ( model, Cmd.none )

        HandleRosInterfaces rawValue ->
            -- Speichert die verfügbaren ROS Messages/Services für die UI
            ( model, Cmd.none )

        -- --- SENSORIK & DATA ---
        HandleSystemLog result ->
            case result of
                Ok logEntry ->
                    ( { model | logs = List.take 30 (logEntry :: model.logs), waitingForNfc = False }
                    , Cmd.none 
                    )
                Err _ -> ( model, Cmd.none )

        HandleRfid result ->
            case result of
                Ok tagId -> 
                    ( { model | lastWrittenId = Just tagId }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        HandleHardwareUpdate result ->
            case result of
                Ok hardwareList -> 
                    ( { model | connectedHardware = hardwareList }, Cmd.none )
                Err _ -> ( model, Cmd.none )

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
            , Ports.socketEmitPort ( "set_heartbeat_rate", Encode.float newHz )
            )

        SetMode modeStr ->
            ( model, Ports.setMode modeStr )

        DismissAlert ->
            ( { model | alert = Nothing }, Cmd.none )