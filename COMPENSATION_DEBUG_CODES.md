# Cutter Compensation Debug Codes (v7)

## Single-Character Progress Tracking

These compact codes show the execution flow through the compensation system:

### Buffer Operations
- **`B#`** - Buffered move (# = current buffer size)
  - Example: `B1`, `B2`, `B3` shows buffer filling up
- **`Z`** - Zero-length move skipped (updates Z position only)

### Compensation Processing
- **`C`** - Compensating move (starting lookahead processing)
- **`K`** - Corner detected (angle < 0.9999 threshold)
- **`I`** - Intersection applied (corner intersection successful)
- **`L`** - coLlinear moves (simple perpendicular offset used)
- **`S`** - Simple offset (next move is arc or invalid)
- **`A`** - Arc compensation (scaling I,J,K offsets)
- **`X`** - eXecuting compensated move (leaving preprocessor)

### Move Execution (Robot)
- **`P`** - Processing parsed move (applying WCS transforms)
- **`M`** - Move executed (append_line/append_arc completed)
- **`F#`** - Flushing # buffered moves (during G40 disable)

## Error Codes

Critical errors that prevent operation:

- **`E01`** - Invalid move (ParsedMove.valid = false)
- **`E02`** - Invalid compensation side (not NONE/LEFT/RIGHT)
- **`E03`** - Arc has no I,J offsets
- **`E04`** - Arc has zero radius
- **`E05`** - Arc compensation creates negative radius
- **`E06`** - Bad compensation side in Robot
- **`E07`** - Invalid parsed move in Robot

## Warning Codes

Non-fatal conditions:

- **`W01`** - Corner calculation failed (using simple offset instead)
- **`W02`** - Potential gouge (inside corner intersection behind start)
- **`W03`** - Singularity detected (very sharp inside corner)

## Typical Execution Patterns

### Successful Move with Corner
```
B1          # First move buffered
B2          # Second move buffered
B3          # Third move buffered, lookahead ready
C           # Compensating first move
K           # Corner detected with next move
I           # Intersection calculated and applied
X           # Compensation complete
P           # Robot processing move
M           # Move executed
```

### Zero-Length Move Filtered
```
B1
Z           # Zero-length move skipped
B2
```

### Collinear Moves (No Corner)
```
C
L           # Collinear - simple offset
X
P
M
```

### G40 Disable (Flush Buffer)
```
F2          # Flushing 2 remaining moves
P
M
P
M
```

## Debugging Tips

1. **Last code before crash** - Shows which operation failed
2. **Missing expected code** - Operation never reached
3. **Repeated patterns** - Count successful moves before failure
4. **Buffer size progression** - Watch B1→B2→B3 pattern

## Example: Analyzing Crash Output

```
B1 Z B2 B3
C K I X P M    # Move 1 OK
B3
C K I X P M    # Move 2 OK
B3
C K I X P M    # Move 3 OK
ALARM: Timeout, Connection lost!
```

Analysis:
- Moves 1-3 executed successfully (all show C→K→I→X→P→M)
- Buffer refilled after each move (B3 appears between moves)
- Crash occurs when attempting to buffer 4th move or process 5th move
- No error codes printed, suggesting system-level issue (not compensation logic)
