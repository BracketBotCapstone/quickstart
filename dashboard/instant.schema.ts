// BracketBot
// https://instantdb.com/dash?s=main&t=home&app=f8690ab7-9a6e-4062-a809-1edbec33d960

import { i } from "@instantdb/core";

const graph = i.graph(
  {
    "$users": i.entity({
      "email": i.any().unique().indexed(),
    }),
    "todos": i.entity({
      "completed": i.any(),
      "createdAt": i.any(),
      "text": i.any(),
    }),
  },
  {}
);

export default graph;
