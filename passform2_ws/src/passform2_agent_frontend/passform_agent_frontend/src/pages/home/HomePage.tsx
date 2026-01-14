import { Box, Flex, Heading, HStack } from "@chakra-ui/react";
import React from "react";
import { GridScene } from "./components/grid_scene/GridScene";
import { Navbar } from "./components/NavBar";
import { Sidebar } from "./components/sidebar/Sidebar";

const HomePage = () => {
  return (
    <Box width={"100%"} height={"100%"} bg={"gray.900"}>
      <Navbar />
      <HStack width={"100%"} height={"100%"} align={"start"}>
        <GridScene></GridScene>
        <Sidebar></Sidebar>
      </HStack>
    </Box>
  );
};

export default HomePage;
