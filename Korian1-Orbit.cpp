/*
*   Compile with:
*       "g++ Korian1-Orbit.cpp -std=c++11 -lkrpc -lprotobuf -pthread"
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
  krpc::Client conn = krpc::connect("Korian 1 Orbit", "192.168.1.102", 50000, 50001);
  krpc::services::SpaceCenter space_center(&conn);
  auto vessel = space_center.active_vessel();

  float turn_start_altitude = 250;
  float turn_end_altitude = 45000;
  float target_altitude = 150000;

  // Set up streams for telemetry
  auto ut = space_center.ut_stream();
  auto altitude = vessel.flight().mean_altitude_stream();
  auto apoapsis = vessel.orbit().apoapsis_altitude_stream();
  auto stage_5_resources = vessel.resources_in_decouple_stage(4, false);
  auto stage_4_resources = vessel.resources_in_decouple_stage(3, false);
  auto stage_1_resources = vessel.resources_in_decouple_stage(1, false);
  auto srb_fuel = stage_5_resources.amount_stream("SolidFuel");
  auto lqd_fuel = stage_4_resources.amount_stream("LiquidFuel");
  auto lqd_fuel1 = stage_1_resources.amount_stream("LiquidFuel");

  // Pre-launch setup
  vessel.control().set_sas(false);
  vessel.control().set_rcs(false);
  vessel.control().set_throttle(1);

  // Countdown...
  std::cout << "3..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "2..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "1..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "Launch!" << std::endl;

  // Activate the first stage
  vessel.control().activate_next_stage();
  vessel.auto_pilot().engage();
  vessel.auto_pilot().target_pitch_and_heading(90, 90);

  // Main ascent loop
  bool srbs_separated = false;
  bool stage_2_separated = false;
  double turn_angle = 0;
  while (true)
  {
    // Gravity turn
    if (altitude() > turn_start_altitude && altitude() < turn_end_altitude)
    {
      double frac = (altitude() - turn_start_altitude)
                    / (turn_end_altitude - turn_start_altitude);
      double new_turn_angle = frac * 90.0;
      if (std::abs(new_turn_angle - turn_angle) > 0.5)
      {
        turn_angle = new_turn_angle;
        vessel.auto_pilot().target_pitch_and_heading(90.0 - turn_angle, 90.0);
      }
    }

    // Separate SRBs when finished
    if (!srbs_separated)
    {
      if (srb_fuel() < 0.1)
      {
        vessel.control().activate_next_stage();
        srbs_separated = true;
        std::cout << "SRBs separated" << std::endl;
      }
    }


    // Separate Ascent Stage When out of fuel
    if (!stage_2_separated)
    {
      if (lqd_fuel() <= 0)
      {
        vessel.control().activate_next_stage();
        vessel.control().activate_next_stage();
        stage_2_separated = true;
      }
    }

    // Decrease throttle when approaching target apoapsis
    if (apoapsis() > target_altitude * 0.9)
    {
      std::cout << "Approaching target apoapsis" << std::endl;
      break;
    }
  }

  // Disable engines when target apoapsis is reached
  vessel.control().set_throttle(0.25);
  while (apoapsis() < target_altitude)
  {
  }

  std::cout << "Target apoapsis reached" << std::endl;
  vessel.control().set_throttle(0);

  // Wait until out of atmosphere
  std::cout << "Coasting out of atmosphere" << std::endl;
  while (altitude() < 70500)
  {
  }

  // Decoupling Ascent Stage Before Circularization Burn
  if(!stage_2_separated)
  {
    std::cout << "Decoupling Ascent Stage" << std::endl;
    vessel.control().activate_next_stage();
    vessel.control().activate_next_stage();
    stage_2_separated = true;
  }

  // Plan circularization burn (using vis-viva equation)
  std::cout << "Planning circularization burn" << std::endl;
  double mu = vessel.orbit().body().gravitational_parameter();
  double r = vessel.orbit().apoapsis();
  double a1 = vessel.orbit().semi_major_axis();
  double a2 = r;
  double v1 = std::sqrt(mu * ((2.0 / r) - (1.0 / a1)));
  double v2 = std::sqrt(mu * ((2.0 / r) - (1.0 / a2)));
  double delta_v = v2 - v1;
  auto node = vessel.control().add_node(ut() + vessel.orbit().time_to_apoapsis(), delta_v);

  // Calculate burn time (using rocket equation)
  double F = vessel.available_thrust();
  double Isp = vessel.specific_impulse() * 9.82;
  double m0 = vessel.mass();
  double m1 = m0 / std::exp(delta_v / Isp);
  double flow_rate = F / Isp;
  double burn_time = (m0 - m1) / flow_rate;

  // Orientate ship
  std::cout << "Orientating ship for circularization burn" << std::endl;
  vessel.auto_pilot().set_reference_frame(node.reference_frame());
  vessel.auto_pilot().set_target_direction(std::make_tuple(0.0, 1.0, 0.0));
  vessel.auto_pilot().wait();

  // Wait until burn
  std::cout << "Waiting until circularization burn" << std::endl;
  double burn_ut = ut() + vessel.orbit().time_to_apoapsis() - (burn_time / 2.0);
  double lead_time = 5;
  space_center.warp_to(burn_ut - lead_time);

  // Execute burn
  std::cout << "Ready to execute burn" << std::endl;
  auto time_to_apoapsis = vessel.orbit().time_to_apoapsis_stream();
  while (time_to_apoapsis() - (burn_time / 2.0) > 0)
  {
  }

  std::cout << "Executing burn" << std::endl;
  vessel.control().set_throttle(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((burn_time - 0.1) * 1000)));
  std::cout << "Fine tuning" << std::endl;
  vessel.control().set_throttle(0.05);
  auto remaining_burn = node.remaining_burn_vector_stream(node.reference_frame());
  while (std::get<0>(remaining_burn()) > 0)
  {
  }

  vessel.control().set_throttle(0);
  node.remove();

  std::cout << "Launch complete" << std::endl;

  // Collect Science
  std::cout << "Running Experiments" << std::endl;
  for (auto experiment : vessel.parts().experiments())
  {
    experiment.run();
  }

  // Prepair for Retrograde Burn
  std::cout << "Orientating for Retrograde Deorbit burn" << std::endl;
  vessel.auto_pilot().set_reference_frame(vessel.surface_velocity_reference_frame());
  vessel.auto_pilot().set_target_direction(std::make_tuple(0.0, -1.0, 0.0));
  vessel.auto_pilot().wait();

  // Perform Retrograde Burn
  vessel.control().set_throttle(1);
  while (lqd_fuel1() > 0)
  {
  }

  // Decouple Orbit Stage and deploy Parachutes
  vessel.control().activate_next_stage();
  for (auto parachute : vessel.parts().parachutes())
  {
    parachute.deploy();
  }

  // Maintain Control Retrograde Loop
  vessel.auto_pilot().set_reference_frame(vessel.surface_velocity_reference_frame());
  while (altitude() > 1000)
  {
    vessel.auto_pilot().set_target_direction(std::make_tuple(0.0, -1.0, 0.0));
    vessel.auto_pilot().wait();
  }

  // Check For Touchdown
  bool Touchdown = false;
  double Mission_Time;
  while (!Touchdown)
  {
    if (vessel.flight().speed() < 0.1 )
    {
      std::cout << "Touchdown" << std::endl;
      Mission_Time = vessel.met();
      Touchdown = true;
    }
  }

  // Calculate Mission Time and Break it Into Seconds, Minutes, Hours, and Days
  int seconds = 0;
  int minutes = 0;
  int hours = 0;
  int days = 0;
  for (int Seconds; Mission_Time > 1.0; Seconds++)
  {
    for (int Minutes; Mission_Time > 60.0; Minutes++)
    {
      for (int Hours; Mission_Time > 3600.0; Hours++)
      {
        for (int Days; Mission_Time > 86400.0; Days++)
        {
          Mission_Time -= 86400.0;
          days = Days;
        }
        Mission_Time -= 3600.0;
        hours = Hours;
      }
      Mission_Time -= 60.0;
      minutes = Minutes;
    }
    Mission_Time -= 1.0;
    seconds = Seconds;
  }
  std::cout << "Mission time of: " << days << " Days, " << hours << " Hours, " << minutes << " Minutes, and " << seconds << "Seconds!" << std::endl;

}
