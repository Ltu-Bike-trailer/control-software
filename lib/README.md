# Lib


This crate adds a few helpers such as encoding/decoding CAN frames, PID/ Gain scheduled PID controllers.

The reason that this is a seperate crate is simply that this allows us to run cargo test without any extra headache.
Moreover we could re-use the same parsing logic in other projects in the future if we so desire.
