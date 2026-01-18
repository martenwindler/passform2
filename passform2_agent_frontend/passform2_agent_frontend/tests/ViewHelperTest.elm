module ViewHelperTest exposing (suite)

import Test exposing (..)
import Expect
import Types exposing (..)
import Types.Domain exposing (..)
import View.Organisms.Sidebar as Sidebar

suite : Test
suite =
    describe "UI-Helper Check"
        [ test "formatType wandelt ModuleType in lesbaren Text" <|
            \_ ->
                Sidebar.formatType FTF |> Expect.equal "FTF Transport"
        
        , test "formatType zeigt Fallback für unbekannte Module" <|
            \_ ->
                Sidebar.formatType (UnknownModule "Scanner") |> Expect.equal "Scanner"
        ]