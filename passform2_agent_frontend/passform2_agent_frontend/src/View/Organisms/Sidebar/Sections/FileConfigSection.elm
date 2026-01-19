module View.Organisms.Sidebar.Sections.FileConfigSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

{-| 
Diese Sektion kümmert sich um das Laden, Speichern und Exportieren 
von Gitter-Konfigurationen (JSON/Default).
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section system-management mt-8 pt-4 border-t border-white/5" ]
        [ h4 [ class "mb-3 text-white/50" ] [ text "Konfigurations-Management" ]
        
        , -- Grid für die Datei-Operationen (2 Spalten)
          div [ class "grid grid-cols-2 gap-2" ]
            [ button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem] flex items-center justify-center gap-2"
                , onClick (AgentsMsg LoadDefaultConfig) 
                ] 
                [ text "📂 Default" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem] flex items-center justify-center gap-2"
                , onClick (AgentsMsg SetCurrentAsDefault) 
                ] 
                [ text "💾 Save" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem] flex items-center justify-center gap-2"
                , onClick (AgentsMsg ExportConfig) 
                ] 
                [ text "📤 Export" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem] flex items-center justify-center gap-2"
                , onClick (AgentsMsg ImportConfigTrigger) 
                ] 
                [ text "📥 Import" ]
            ]
            
        , div [ class "mt-3 text-[0.6rem] text-white/20 italic text-center" ]
            [ text "JSON-Konfigurationen synchronisiert mit LocalStorage" ]
        ]