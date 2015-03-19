This class is a beast. At its heart is just an extended Kalman filter which accepts measurements from the vision algorithms. This problem lies in the asynchronous processing of all the data. Since vision measurements have a significant delay between the time the image is taken and the time that the measurement is actually ready, naive application of the measurement is suboptimal.

Instead, I maintain a recent history of the state estimates and sensor measurements. When a new measurement comes in I rewind the state history to the appropriate point in time, apply the new measurement, and then apply all subsequent time updates and measurements to get back to the current time.

Easily said, but implementing it has been a nightmare since there are so many opportunities for bugs. This whole class might benefit from a scratch rewrite.

TODO
add equations for assumed system dynamics