import GridCell from "./grid_cell/GridCell";

export const GridLayout = () => {
  const range = (from: number, to: number) =>
    Array.from({ length: to - from + 1 }, (_, i) => i + from);

  return (
    <>
      {range(-16, 16).map((x) =>
        range(-16, 16).map((y) => <GridCell key={`${x}-${y}`} x={x} y={y} />)
      )}
    </>
  );
};
