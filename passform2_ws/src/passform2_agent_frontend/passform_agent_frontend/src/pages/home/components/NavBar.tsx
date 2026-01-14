import {
  Box,
  Flex,
  Heading,
  Spacer,
  HStack,
  Circle,
  Text,
  Icon,
  Image,
  Button,
} from "@chakra-ui/react";
import { MdCheckCircle, MdError } from "react-icons/md";
import React from "react";
import Logo from "@/assets/images/logo.png";
import { useStore } from "@/store";

export const Navbar: React.FC = () => {
  const connected = useStore((state) => state.connected);
  const mode = useStore((state) => state.mode);
  const setMode = useStore((state) => state.setMode);
  const backendIP = useStore((state) => state.backendIP);
  const [loading, setLoading] = React.useState(false);

  const handleToggleMode = async () => {
    const newMode = mode === "simulation" ? "hardware" : "simulation";
    setLoading(true);
    try {
      const response = await fetch(`http://${backendIP}:8000/api/system/mode`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: newMode }),
      });
      const data = await response.json();
      if (!response.ok)
        throw new Error(data.detail || "Fehler beim Umschalten");
    } catch (err) {
      alert("Fehler beim Umschalten des Modus: " + (err as Error).message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <HStack
      as="nav"
      width="100%"
      height="60px"
      align="center"
      bg="gray.800"
      color="white"
      boxShadow="md"
    >
      <Flex align="center">
        <Image src={Logo} alt="Logo" h={"10"} mr={2} />
        <Heading size="xl">PassForm</Heading>
      </Flex>
      <Spacer />
      <HStack pr={4} gap={2} align="center">
        <Text fontSize="sm" fontWeight="bold">
          {connected ? "Verbunden" : "Getrennt"}
        </Text>
        <Icon
          as={connected ? MdCheckCircle : MdError}
          color={connected ? "green.400" : "red.400"}
        />
        <Button
          size="sm"
          colorScheme={mode === "hardware" ? "red" : "teal"}
          variant={mode === "hardware" ? "solid" : "outline"}
          disabled={loading}
          onClick={handleToggleMode}
        >
          {mode === "simulation"
            ? "Wechsle zu Hardware"
            : "Wechsle zu Simulation"}
        </Button>
      </HStack>
    </HStack>
  );
};
