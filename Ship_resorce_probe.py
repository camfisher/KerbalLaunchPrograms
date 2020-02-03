import math
import time
import krpc

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

stage_1_resources = vessel.resources_in_decouple_stage(stage=0, cumulative=False)
stg1_fuel = conn.add_stream(stage_1_resources.amount, 'LiquidFuel')
stg1_sldfuel = conn.add_stream(stage_1_resources.amount, 'SolidFuel')

stage_2_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)
stg2_fuel = conn.add_stream(stage_2_resources.amount, 'LiquidFuel')
stg2_sldfuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')

stage_3_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
stg3_fuel = conn.add_stream(stage_3_resources.amount, 'LiquidFuel')
stg3_sldfuel = conn.add_stream(stage_3_resources.amount, 'SolidFuel')

print('stage 1(0) resources')
print(stg1_fuel())
print(stg1_sldfuel())
print()
print('stage 2(1) resources')
print(stg2_fuel())
print(stg2_sldfuel())
print()
print('stage 3(2) resources')
print(stg3_fuel())
print(stg3_sldfuel())
