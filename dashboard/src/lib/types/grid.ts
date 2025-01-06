export type GridData = {
  width: number; // 400
  flattened_grid_list: boolean[]; // list of 160000 true/false -> fill/empty values of the current 400 width view
  timestamp: number;
};