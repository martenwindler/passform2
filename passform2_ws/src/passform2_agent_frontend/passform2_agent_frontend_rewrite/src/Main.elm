module Main exposing (main)

import Browser
import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Http
import Dict
import Json.Decode as Decode
import Json.Encode as Encode

import Types exposing (..)
import Ports
import Decoders
import View.Navbar as Navbar
import View.Sidebar as Sidebar

main : Program { backendIP : String } Model Msg
main =
    Browser.element
        { init = init
        , view = view
        , update = update
        , subscriptions = subscriptions
        }

init : { backendIP : String } -> ( Model, Cmd Msg )
init flags =
    ( { mode = Simulation
      , backendIP = flags.backendIP
      , connected = True -- MOCK: Wir tun so, als wären wir verbunden
      , agents = Dict.empty
      , pathStart = Nothing
      , pathGoal = Nothing
      , currentPath = Nothing
      , hoveredCell = Nothing
      , editing = True
      , is3D = False
      , loading = false
      , activeMenu = Nothing
      }
    , Cmd.none
    )

update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        ToggleMode ->
            let
                newMode = if model.mode == Simulation then Hardware else Simulation
            in
            ( { model | mode = newMode }, Cmd.none )

        SetConnected status ->
            ( { model | connected = status }, Cmd.none )

        UpdateAgents rawJson ->
            case Decode.decodeValue Decoders.agentMapDecoder rawJson of
                Ok newAgents -> ( { model | agents = newAgents }, Cmd.none )
                Err _ -> ( model, Cmd.none )

        HandleGridClick cell ->
            case Dict.get (cell.x, cell.y) model.agents of
                Just agent -> 
                    ( { model | activeMenu = Just (SettingsMenu cell agent) }, Cmd.none )
                Nothing -> 
                    if model.editing then 
                        ( { model | activeMenu = Just (SelectionMenu cell) }, Cmd.none ) 
                    else 
                        ( model, Cmd.none )

        CloseMenu -> 
            ( { model | activeMenu = Nothing }, Cmd.none )

        -- MOCK: Agent lokal hinzufügen
        StartAgent moduleType cell ->
            let
                newAgent = 
                    { agent_id = Just "mock-id"
                    , module_type = moduleType
                    , position = cell
                    }
                newAgents = Dict.insert (cell.x, cell.y) newAgent model.agents
            in
            ( { model | agents = newAgents, activeMenu = Nothing }, Cmd.none )

        -- MOCK: Agent lokal entfernen
        RemoveAgent cell ->
            let
                newAgents = Dict.remove (cell.x, cell.y) model.agents
            in
            ( { model | agents = newAgents, activeMenu = Nothing }, Cmd.none )

        SetPathStart cell -> 
            ( { model | pathStart = Just cell, activeMenu = Nothing }, Cmd.none )
        
        SetPathGoal cell -> 
            ( { model | pathGoal = Just cell, activeMenu = Nothing }, Cmd.none )

        StartPlanning _ ->
            ( { model | loading = true }, Cmd.none )

        PlanningResult result ->
            case result of
                Ok path -> ( { model | currentPath = Just path, loading = false }, Cmd.none )
                Err _ -> ( { model | loading = false }, Cmd.none )
        
        NoOp -> ( model, Cmd.none )
        _ -> ( model, Cmd.none )

subscriptions : Model -> Sub Msg
subscriptions _ =
    Sub.batch
        [ Ports.socketStatusReceiver SetConnected
        , Ports.activeAgentsReceiver UpdateAgents
        ]

view : Model -> Html Msg
view model =
    div [ class "app-layout" ]
        [ Navbar.view model
        , div [ class "content-area" ]
            [ view3D model
            , Sidebar.view model
            ]
        , viewActiveMenu model.activeMenu
        ]

view3D : Model -> Html Msg
view3D model =
    node "three-grid-scene"
        [ Html.Events.on "cell-clicked" (Decode.map HandleGridClick Decoders.decodeCellClick)
        , attribute "agents" (Decoders.encodeAgentMap model.agents)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        ]
        []

viewActiveMenu : Maybe MenuType -> Html Msg
viewActiveMenu maybeMenu =
    case maybeMenu of
        Just (SelectionMenu cell) ->
            div [ class "modal-overlay" ]
                [ h3 [] [ text "Agent auswählen" ]
                , button [ onClick (StartAgent "rollen_ns" cell) ] [ text "Rollen Modul" ]
                , button [ onClick (StartAgent "greifer" cell) ] [ text "Roboter Greifer" ]
                , button [ onClick CloseMenu ] [ text "Abbrechen" ]
                ]
        Just (SettingsMenu cell _) ->
            div [ class "modal-overlay" ]
                [ button [ onClick (SetPathStart cell) ] [ text "Als Start setzen" ]
                , button [ onClick (SetPathGoal cell) ] [ text "Als Ziel setzen" ]
                , button [ onClick (RemoveAgent cell), style "background" "#e53e3e" ] [ text "Löschen" ]
                , button [ onClick CloseMenu ] [ text "Schließen" ]
                ]
        Nothing -> text ""