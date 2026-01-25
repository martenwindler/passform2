module TestHelpers exposing (emptyModel)

import Types exposing (..)
import Types.Domain exposing (..)
import Dict

emptyModel : Model
emptyModel =
    { activeLayout = LandingMode -- HIER DER FIX: Standardwert für Tests
    , mode = Simulation
    , backendIP = "127.0.0.1"
    , connected = False
    , rosConnected = False
    , canConnected = False
    , rangerBattery = Nothing
    , agents = Dict.empty
    , savedDefault = Dict.empty
    , logs = []
    , pathStart = Nothing
    , pathGoal = Nothing
    , currentPath = Nothing
    , hoveredCell = Nothing
    , editing = True
    , isDragging = False
    , is3D = False
    , loading = False
    , activeMenu = Nothing
    , sidebarOpen = False
    , activeSidebarTab = TabAgents
    , gridWidth = 6
    , gridHeight = 4
    , waitingForNfc = False
    , nfcStatus = UnknownStatus
    , planningWeights = 
        { execution_time_default = 1.0
        , complex_module_time = 3.5
        , human_extra_weight = 1.0
        , proximity_penalty = 0.5
        , hardware_safety_factor = 1.2 
        }
    , currentHz = 1.0
    , alert = Nothing
    , connectedHardware = []
    , lastWrittenId = Nothing
    }