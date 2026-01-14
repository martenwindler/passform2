import React from "react";
import { Box, Button, Flex, Text, Badge } from "@chakra-ui/react";
import { useStore } from "@/store";
import { v4 as uuidv4 } from "uuid";

export const Sidebar: React.FC = () => {
  const currentPath = useStore((state) => state.currentPath);

  const handleStartPlanning = async (central: boolean = false) => {
    // Simulate planning logic
    try {
      const backendIP = useStore.getState().backendIP;
      const start = useStore.getState().pathStart;
      const goal = useStore.getState().pathGoal;

      if (!start || !goal) {
        console.error("❌ Start or end point is not set.");
        return;
      }

      const requestId = uuidv4();

      const request = {
        request_id: requestId,
        start: { x: start.x, y: start.y },
        goal: { x: goal.x, y: goal.y },
        central: central,
      };
      const response = await fetch(
        "http://" + backendIP + ":8000/api/path/plan_path",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(request),
        }
      );

      const data = await response.json();

      if (!response.ok) {
        throw new Error(`Fehler: ${data.detail || response.statusText}`);
      }

      console.log("✅ Pfadplanung gestartet:", data.message);
      return data;
    } catch (error) {
      console.error("❌ Fehler beim Pfadplanungs-Request:", error);
      throw error;
    }
  };

  return (
    <Box
      width="30%"
      height="100vh"
      bg="gray.700"
      color="white"
      p={"10px"}
      boxShadow="md"
    >
      {" "}
      <Flex direction="column" gap={2}>
        <Button colorScheme="teal" onClick={() => handleStartPlanning(false)}>
          Planung starten
        </Button>{" "}
        <Button colorScheme="teal" onClick={() => handleStartPlanning(true)}>
          Planung central starten
        </Button>{" "}
        <Box mt={4} p={3} bg="gray.600" borderRadius="md">
          <Text fontWeight="bold" mb={3}>
            Letzter geplanter Pfad:
          </Text>
          {currentPath ? (
            <Flex direction="column" gap={2}>
              <Flex align="center" gap={2}>
                <Text fontSize="sm" color="gray.300">
                  Status:
                </Text>
                <Badge
                  color={currentPath.status === 0 ? "green" : "red"}
                  variant="solid"
                >
                  {currentPath.status === 0 ? "Erfolgreich" : "Fehler"}
                </Badge>
              </Flex>

              <Flex align="center" gap={2}>
                <Text fontSize="sm" color="gray.300">
                  Anfrage-ID:
                </Text>
                <Text fontSize="sm" fontFamily="mono">
                  {currentPath.request_id}
                </Text>
              </Flex>

              <Flex align="center" gap={2}>
                <Text fontSize="sm" color="gray.300">
                  Pfadlänge:
                </Text>
                <Text fontSize="sm">{currentPath.path.length} Segmente</Text>
              </Flex>

              <Flex align="center" gap={2}>
                <Text fontSize="sm" color="gray.300">
                  Plannungs dauer:
                </Text>
                <Text fontSize="sm">
                  {currentPath.planning_time.sec +
                    currentPath.planning_time.nanosec / 1e9}{" "}
                  Sekunden
                </Text>
              </Flex>

              <Flex align="center" gap={2}>
                <Text fontSize="sm" color="gray.300">
                  Gesamt Kosten:
                </Text>
                <Text fontSize="sm">{currentPath.cost} </Text>
              </Flex>

              {currentPath.path.length > 0 && (
                <Box mt={2} p={2} bg="gray.700" borderRadius="sm" w="100%">
                  <Text fontSize="xs" color="gray.400" mb={1}>
                    Pfaddetails:
                  </Text>
                  <Flex direction="column" gap={1}>
                    {currentPath.path.slice(0, 20).map((segment, index) => (
                      <Flex key={index} fontSize="xs" gap={2}>
                        <Text color="gray.300">{index + 1}:</Text>
                        <Text color="blue.300">{segment.module_type}</Text>
                        <Text color="gray.400">
                          ({segment.position.x}, {segment.position.y})
                        </Text>
                      </Flex>
                    ))}
                    {currentPath.path.length > 20 && (
                      <Text fontSize="xs" color="gray.400">
                        ... und {currentPath.path.length - 3} weitere Segmente
                      </Text>
                    )}
                  </Flex>
                </Box>
              )}
            </Flex>
          ) : (
            <Text color="gray.400">Noch kein Pfad geplant.</Text>
          )}
        </Box>
      </Flex>
    </Box>
  );
};
