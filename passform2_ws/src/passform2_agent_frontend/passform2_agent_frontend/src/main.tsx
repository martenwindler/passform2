import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { GridScene } from "./pages/home/components/grid_scene/GridScene.tsx";
import { Provider } from "./components/ui/provider.tsx";
import { Box } from "@chakra-ui/react";
import HomePage from "./pages/home/HomePage.tsx";
import { socketManager } from "./socket/SocketManager";
import "./index.css";

socketManager;

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <Provider>
      <Box
        style={{ width: "100%", height: "100%" }}
        alignItems={"center"}
        justifyContent={"center"}
        justifyItems={"center"}
      >
        <HomePage></HomePage>
      </Box>
    </Provider>
  </StrictMode>
);
