/*
*   Compile with:
*       "g++ ScienceTest.cpp -std=c++11 -lkrpc -lprotobuf -pthread"
*
*
*/

#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>

int main()
{
  krpc::Client conn = krpc::connect("Korian 1", "192.168.1.102", 50000, 50001);
  krpc::services::SpaceCenter space_center(&conn);
  auto vessel = space_center.active_vessel();

  // Run Experiment
  for (auto experiment : vessel.parts().experiments())
  {
    experiment.run();
  }

}
