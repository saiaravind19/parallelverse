# Layout CSV rules

These rules apply to every layout CSV used by `sim2d`.

1. **File format:** plain CSV, one line per grid row, top row first. Each
   column is 1 m. Commas inside a field must be inside double quotes.
2. **Drivable cell:** a field written as a 4-part tuple, for example
   `"(-,v,>,-)"`. Each part is one of `-`, `^`, `v`, `<`, `>`. The sim
   currently ignores the tuple values; they are kept for future rules.
3. **Empty cell:** a blank field. Empty cells are outside the layout and are
   walls. Because of this, the empty cells around the drivable area form the
   border.
4. **Outside the file:** anything beyond the grid is a wall.
5. **Grid extent:** the grid is the smallest block of columns and rows that
   holds any drivable or legacy cell. Text to the right of or below the grid
   (for example the legend labels) is ignored.
6. **Legacy format (still accepted):** `0` = drivable, `1` = wall.
7. **Coordinates:** world position `(x, y)` is in metres from the bottom-left
   corner of the grid. Cell `(col, row)` in the file covers
   `x in [col, col+1)`, `y in [height-1-row, height-row)`.

Check a layout with `sim2d_map_check <file.csv>`, which prints `#` for walls
and `.` for drivable cells.
