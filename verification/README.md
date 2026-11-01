# Search pattern check

`search_pattern_test.cpp` exercises the search path used by `MegaBallSearchNoTurn`. It checks both search-side choices and leg boundaries, confirms that the pattern finishes after its configured duration, and verifies that no search leg requests backward travel or rotation.

From the repository root, compile and run it with a C++11 host compiler:

```sh
g++ -std=c++11 -Wall -Wextra -Werror \
  -I sketches/MegaBallSearchNoTurn \
  verification/search_pattern_test.cpp -o /tmp/search_pattern_test
/tmp/search_pattern_test
```

Compile the Arduino sketches separately for `arduino:avr:mega`. Passing software checks does not verify the motor wiring, gyro sign, IR signal, or physical trajectory.
